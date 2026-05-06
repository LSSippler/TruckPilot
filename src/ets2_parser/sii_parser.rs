//! SII (SiiNunit) text format parser.
//!
//! Parses ETS2 definition files (`.sii`) into a structured AST.
//! The SII format is used for road definitions, prefab descriptors,
//! economy data, and more.

/// A parsed SII unit (block).
#[derive(Debug, Clone, PartialEq)]
pub struct SiiUnit {
    /// Unit type (e.g. "road_look", "traffic_rule").
    pub unit_type: String,
    /// Unit instance name (e.g. ".road_0", ".traffic.traffic_rule.a").
    pub unit_name: String,
    /// Key–value properties.
    pub properties: Vec<(String, SiiValue)>,
    /// Nested child units (rare, used in some compound types).
    pub children: Vec<SiiUnit>,
}

/// Possible values in SII properties.
#[derive(Debug, Clone, PartialEq)]
pub enum SiiValue {
    /// Quoted or unquoted string.
    String(String),
    /// Floating-point number.
    Float(f64),
    /// Boolean.
    Bool(bool),
    /// Hexadecimal UID (e.g. 0x002935DE00004D04).
    Uid(u64),
    /// List of hexadecimal UIDs.
    UidList(Vec<u64>),
    /// Float-2 tuple (x, y).
    Float2(f64, f64),
    /// Float-3 tuple (x, y, z).
    Float3(f64, f64, f64),
    /// Generic token (unrecognized or passthrough).
    Token(String),
}

/// Parse a complete SII content string into a list of top-level units.
///
/// Returns an error string if the content contains syntax errors.
pub fn parse_sii(content: &str) -> Result<Vec<SiiUnit>, String> {
    let cleaned = preprocess(content);
    let tokens = tokenize(&cleaned)?;
    let mut parser = Parser::new(tokens);
    parser.parse()
}

// ---------------------------------------------------------------------------
// Preprocessing
// ---------------------------------------------------------------------------

fn preprocess(input: &str) -> String {
    let mut out = String::with_capacity(input.len());
    let chars: Vec<char> = input.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        // Skip line comments.
        if i + 1 < chars.len() && chars[i] == '/' && chars[i + 1] == '/' {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
            continue;
        }
        // Skip block comments.
        if i + 1 < chars.len() && chars[i] == '/' && chars[i + 1] == '*' {
            i += 2;
            while i + 1 < chars.len() && !(chars[i] == '*' && chars[i + 1] == '/') {
                i += 1;
            }
            if i + 1 < chars.len() {
                i += 2; // skip */
            }
            continue;
        }
        // Skip C++ style attribute / include lines.
        if chars[i] == '@' {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
            continue;
        }
        // Skip hash includes (#include).
        if chars[i] == '#' {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
            continue;
        }
        out.push(chars[i]);
        i += 1;
    }
    out
}

// ---------------------------------------------------------------------------
// Tokenizer
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Token {
    /// A word / identifier.
    Ident(String),
    /// A quoted string (including the quotes stripped).
    StringLit(String),
    /// A bare numeric literal (possibly a float).
    Number(String),
    /// Opening brace.
    BraceOpen,
    /// Closing brace.
    BraceClose,
    /// Colon.
    Colon,
    /// Comma.
    Comma,
    /// Opening parenthesis.
    ParenOpen,
    /// Closing parenthesis.
    ParenClose,
    /// An ampersand token used as unit name prefix.
    Ampersand,
    /// End of input.
    Eof,
}

fn tokenize(input: &str) -> Result<Vec<Token>, String> {
    let mut tokens = Vec::new();
    let chars: Vec<char> = input.chars().collect();
    let mut i = 0;

    while i < chars.len() {
        let c = chars[i];

        // Whitespace.
        if c.is_whitespace() {
            i += 1;
            continue;
        }

        // Single-character punctuation.
        match c {
            '{' => {
                tokens.push(Token::BraceOpen);
                i += 1;
                continue;
            }
            '}' => {
                tokens.push(Token::BraceClose);
                i += 1;
                continue;
            }
            ':' => {
                tokens.push(Token::Colon);
                i += 1;
                continue;
            }
            ',' => {
                tokens.push(Token::Comma);
                i += 1;
                continue;
            }
            '(' => {
                tokens.push(Token::ParenOpen);
                i += 1;
                continue;
            }
            ')' => {
                tokens.push(Token::ParenClose);
                i += 1;
                continue;
            }
            '&' => {
                tokens.push(Token::Ampersand);
                i += 1;
                continue;
            }
            _ => {}
        }

        // String literal.
        if c == '"' {
            i += 1;
            let mut s = String::new();
            while i < chars.len() && chars[i] != '"' {
                if chars[i] == '\\' && i + 1 < chars.len() {
                    i += 1;
                    match chars[i] {
                        'n' => s.push('\n'),
                        't' => s.push('\t'),
                        '\\' => s.push('\\'),
                        '"' => s.push('"'),
                        other => {
                            s.push('\\');
                            s.push(other);
                        }
                    }
                } else {
                    s.push(chars[i]);
                }
                i += 1;
            }
            if i >= chars.len() {
                return Err("unterminated string literal".into());
            }
            i += 1; // skip closing quote
            tokens.push(Token::StringLit(s));
            continue;
        }

        // Number (including 0x hex and floats).
        if c.is_ascii_digit() || (c == '.' && i + 1 < chars.len() && chars[i + 1].is_ascii_digit())
        {
            let start = i;
            if c == '0' && i + 1 < chars.len() && chars[i + 1] == 'x' {
                i += 2;
                while i < chars.len() && chars[i].is_ascii_hexdigit() {
                    i += 1;
                }
            } else {
                i += 1;
                while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                    i += 1;
                }
            }
            let num: String = chars[start..i].iter().collect();
            tokens.push(Token::Number(num));
            continue;
        }

        // Dot-prefixed unit name or negative number.
        if c == '.' {
            let start = i;
            i += 1;
            while i < chars.len()
                && (chars[i].is_alphanumeric() || chars[i] == '.' || chars[i] == '_')
            {
                i += 1;
            }
            let ident: String = chars[start..i].iter().collect();
            tokens.push(Token::Ident(ident));
            continue;
        }

        // Negative number.
        if c == '-' && i + 1 < chars.len() && (chars[i + 1].is_ascii_digit() || chars[i + 1] == '.')
        {
            let start = i;
            i += 1;
            while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                i += 1;
            }
            let num: String = chars[start..i].iter().collect();
            tokens.push(Token::Number(num));
            continue;
        }

        // Identifier.
        if c.is_alphanumeric() || c == '_' {
            let start = i;
            while i < chars.len()
                && (chars[i].is_alphanumeric() || chars[i] == '_' || chars[i] == '.')
            {
                i += 1;
            }
            let ident: String = chars[start..i].iter().collect();
            tokens.push(Token::Ident(ident));
            continue;
        }

        return Err(format!("unexpected character '{}' at position {}", c, i));
    }

    tokens.push(Token::Eof);
    Ok(tokens)
}

// ---------------------------------------------------------------------------
// Recursive-descent parser
// ---------------------------------------------------------------------------

struct Parser {
    tokens: Vec<Token>,
    pos: usize,
}

impl Parser {
    fn new(tokens: Vec<Token>) -> Self {
        Parser { tokens, pos: 0 }
    }

    fn peek(&self) -> &Token {
        &self.tokens[self.pos]
    }

    fn advance(&mut self) -> &Token {
        let tok = &self.tokens[self.pos];
        self.pos += 1;
        tok
    }

    fn expect(&mut self, expected: &Token) -> Result<(), String> {
        let tok = self.advance();
        if std::mem::discriminant(tok) != std::mem::discriminant(expected) {
            return Err(format!("expected {:?}, got {:?}", expected, tok));
        }
        Ok(())
    }

    fn expect_ident(&mut self) -> Result<String, String> {
        match self.advance() {
            Token::Ident(s) => Ok(s.clone()),
            Token::Ampersand => {
                // & followed by ident
                match self.advance() {
                    Token::Ident(s) => Ok(format!("&{s}")),
                    other => Err(format!("expected ident after &, got {other:?}")),
                }
            }
            other => Err(format!("expected ident, got {other:?}")),
        }
    }

    fn parse(&mut self) -> Result<Vec<SiiUnit>, String> {
        // Expect SiiNunit { ... }
        match self.peek() {
            Token::Ident(s) if s == "SiiNunit" => {
                self.advance();
                self.expect(&Token::BraceOpen)?;
                let mut units = Vec::new();
                while !matches!(self.peek(), Token::BraceClose | Token::Eof) {
                    units.push(self.parse_unit()?);
                }
                self.expect(&Token::BraceClose)?;
                Ok(units)
            }
            Token::Ident(_) => {
                // No SiiNunit wrapper — just parse units directly.
                let mut units = Vec::new();
                while !matches!(self.peek(), Token::Eof) {
                    units.push(self.parse_unit()?);
                }
                Ok(units)
            }
            _ => Ok(vec![]),
        }
    }

    fn parse_unit(&mut self) -> Result<SiiUnit, String> {
        // type_name : unit_name { ... }
        let unit_type = self.expect_ident()?;
        self.expect(&Token::Colon)?;
        let unit_name = self.expect_ident()?;

        let mut properties = Vec::new();
        let mut children = Vec::new();

        match self.peek() {
            Token::BraceOpen => {
                self.advance();
                while !matches!(self.peek(), Token::BraceClose | Token::Eof) {
                    // Could be a property or a nested unit.
                    // Properties are: key: value
                    // Nested units start with a type name followed by colon.
                    if let Token::Ident(key) = self.peek() {
                        // Look ahead to see if this is a property or nested unit.
                        if self.pos + 1 < self.tokens.len()
                            && self.tokens[self.pos + 1] == Token::Colon
                        {
                            let key = key.clone();
                            self.advance(); // key
                            self.advance(); // colon
                            let value = self.parse_value()?;
                            properties.push((key, value));
                            continue;
                        }
                    }
                    // Try parsing as nested unit.
                    if let Token::Ident(_name) = self.peek() {
                        // Check if next next token is colon (unit header pattern)
                        if self.pos + 1 < self.tokens.len()
                            && matches!(&self.tokens[self.pos + 1], Token::Colon)
                        {
                            children.push(self.parse_unit()?);
                            continue;
                        }
                    }
                    // Unknown — skip.
                    self.advance();
                }
                self.expect(&Token::BraceClose)?;
            }
            _ => {
                // Inline properties: type : name { key: val, key: val }
                // We already consumed the header, nothing more to do.
            }
        }

        Ok(SiiUnit {
            unit_type,
            unit_name,
            properties,
            children,
        })
    }

    fn parse_value(&mut self) -> Result<SiiValue, String> {
        match self.advance().clone() {
            Token::StringLit(s) => Ok(SiiValue::String(s)),
            Token::Number(s) => {
                if s.starts_with("0x") || s.starts_with("0X") {
                    let uid = u64::from_str_radix(&s[2..], 16)
                        .map_err(|e| format!("invalid hex UID '{s}': {e}"))?;
                    Ok(SiiValue::Uid(uid))
                } else {
                    let f: f64 = s
                        .parse()
                        .map_err(|e| format!("invalid number '{s}': {e}"))?;
                    Ok(SiiValue::Float(f))
                }
            }
            Token::Ident(s) => match s.as_str() {
                "true" => Ok(SiiValue::Bool(true)),
                "false" => Ok(SiiValue::Bool(false)),
                _ => Ok(SiiValue::Token(s)),
            },
            Token::ParenOpen => {
                // Parse a tuple/list: (val1, val2, ...)
                let mut uids = Vec::new();
                let mut floats = Vec::new();
                loop {
                    match self.peek() {
                        Token::ParenClose | Token::Eof => break,
                        Token::Number(s) => {
                            let s = s.clone();
                            self.advance();
                            if s.starts_with("0x") || s.starts_with("0X") {
                                let uid = u64::from_str_radix(&s[2..], 16)
                                    .map_err(|e| format!("invalid hex UID: {e}"))?;
                                uids.push(uid);
                            } else {
                                let f: f64 =
                                    s.parse().map_err(|e| format!("invalid float: {e}"))?;
                                floats.push(f);
                            }
                            // Skip comma if present.
                            if matches!(self.peek(), Token::Comma) {
                                self.advance();
                            }
                        }
                        _ => {
                            self.advance();
                        }
                    }
                }
                self.expect(&Token::ParenClose)?;
                if !uids.is_empty() {
                    Ok(SiiValue::UidList(uids))
                } else if floats.len() == 2 {
                    Ok(SiiValue::Float2(floats[0], floats[1]))
                } else if floats.len() == 3 {
                    Ok(SiiValue::Float3(floats[0], floats[1], floats[2]))
                } else {
                    Ok(SiiValue::Token("()".into()))
                }
            }
            Token::Comma => {
                // Trailing comma — try again.
                self.parse_value()
            }
            other => Err(format!("unexpected token in value: {other:?}")),
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_unit() {
        let input = r#"
SiiNunit {
    road_look : .look0 {
        name: "Asphalt"
        speed_limit: 80.0
        lanes: 2
    }
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(units.len(), 1);
        assert_eq!(units[0].unit_type, "road_look");
        assert_eq!(units[0].unit_name, ".look0");
        assert_eq!(units[0].properties.len(), 3);
        assert_eq!(
            units[0].properties[0],
            ("name".to_string(), SiiValue::String("Asphalt".into()))
        );
        assert_eq!(
            units[0].properties[2],
            ("lanes".to_string(), SiiValue::Float(2.0))
        );
    }

    #[test]
    fn test_parse_multiple_units() {
        let input = r#"
SiiNunit {
    road_look : .a { name: "a" }
    road_look : .b { name: "b" }
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(units.len(), 2);
    }

    #[test]
    fn test_parse_boolean() {
        let input = r#"
SiiNunit {
    test : .t { enabled: true disabled: false }
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(
            units[0].properties[0],
            ("enabled".into(), SiiValue::Bool(true))
        );
        assert_eq!(
            units[0].properties[1],
            ("disabled".into(), SiiValue::Bool(false))
        );
    }

    #[test]
    fn test_parse_hex_uid() {
        let input = r#"
SiiNunit {
    node : .n0 { uid: 0x002935DE00004D04 }
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(
            units[0].properties[0],
            ("uid".into(), SiiValue::Uid(0x002935DE00004D04))
        );
    }

    #[test]
    fn test_parse_uid_list() {
        let input = r#"
SiiNunit {
    road : .r0 { nodes: (0x01, 0x02, 0x03) }
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(
            units[0].properties[0],
            ("nodes".into(), SiiValue::UidList(vec![0x01, 0x02, 0x03]))
        );
    }

    #[test]
    fn test_parse_float3() {
        let input = r#"
SiiNunit {
    node : .n0 { position: (1.0, 2.0, 3.0) }
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(
            units[0].properties[0],
            ("position".into(), SiiValue::Float3(1.0, 2.0, 3.0))
        );
    }

    #[test]
    fn test_comments_ignored() {
        let input = r#"
SiiNunit {
    // this is a comment
    road_look : .look0 {
        name: "Asphalt" /* inline comment */
        speed_limit: 80.0
    }
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(units.len(), 1);
        assert_eq!(units[0].properties.len(), 2);
    }

    #[test]
    fn test_no_siinunit_wrapper() {
        let input = r#"
road_look : .look0 {
    name: "test"
}
"#;
        let units = parse_sii(input).unwrap();
        assert_eq!(units.len(), 1);
    }

    #[test]
    fn test_syntax_error() {
        let input = r#"SiiNunit { road_look : { broken }"#;
        let result = parse_sii(input);
        assert!(result.is_err());
    }

    #[test]
    fn test_unterminated_string_literal() {
        let input = r#"
SiiNunit {
    road_look : .look0 {
        name: "Asphalt
    }
}
"#;
        let result = parse_sii(input);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            err.contains("unterminated string literal"),
            "expected unterminated string error, got: {}",
            err
        );
    }
}
