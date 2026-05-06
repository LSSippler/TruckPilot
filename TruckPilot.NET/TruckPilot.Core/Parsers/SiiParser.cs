using System.Text;

namespace TruckPilot.Core.Parsers;

/// <summary>SII (SiiNunit) text format parser for ETS2 definition files.</summary>
public static class SiiParser
{
    public sealed record SiiUnit(string Type, string Name, List<(string Key, string Value)> Properties);

    public static List<SiiUnit> Parse(string content)
    {
        var units = new List<SiiUnit>();
        int pos = SkipWhitespace(content, 0);
        if (Match(content, pos, "SiiNunit")) pos += 8;

        pos = SkipWhitespace(content, pos);
        if (pos < content.Length && content[pos] == '{') pos++;

        while (pos < content.Length)
        {
            pos = SkipWhitespaceAndComments(content, pos);
            if (pos >= content.Length || content[pos] == '}') break;
            var (unit, newPos) = ParseUnit(content, pos);
            if (unit != null) units.Add(unit);
            pos = newPos;
        }
        return units;
    }

    private static (SiiUnit?, int) ParseUnit(string s, int pos)
    {
        pos = SkipWhitespaceAndComments(s, pos);
        int typeStart = pos;
        while (pos < s.Length && (char.IsLetterOrDigit(s[pos]) || s[pos] == '_')) pos++;
        string type = s[typeStart..pos];

        pos = SkipWhitespace(s, pos);
        if (pos < s.Length && s[pos] == ':') pos++;
        pos = SkipWhitespace(s, pos);

        int nameStart = pos;
        while (pos < s.Length && (char.IsLetterOrDigit(s[pos]) || s[pos] is '_' or '.')) pos++;
        string name = s[nameStart..pos];

        var props = new List<(string, string)>();
        pos = SkipWhitespace(s, pos);
        if (pos < s.Length && s[pos] == '{')
        {
            pos++;
            while (pos < s.Length)
            {
                pos = SkipWhitespaceAndComments(s, pos);
                if (pos >= s.Length || s[pos] == '}') { pos++; break; }

                int keyStart = pos;
                while (pos < s.Length && (char.IsLetterOrDigit(s[pos]) || s[pos] == '_')) pos++;
                string key = s[keyStart..pos];

                pos = SkipWhitespace(s, pos);
                if (pos < s.Length && s[pos] == ':') pos++;
                pos = SkipWhitespace(s, pos);

                var (value, newPos) = ParseValue(s, pos);
                if (key.Length > 0) props.Add((key, value));
                pos = newPos;
            }
        }

        return (new SiiUnit(type, name, props), pos);
    }

    private static (string, int) ParseValue(string s, int pos)
    {
        pos = SkipWhitespace(s, pos);
        if (pos >= s.Length) return ("", pos);

        if (s[pos] == '"')
        {
            int end = s.IndexOf('"', pos + 1);
            return end >= 0 ? (s[(pos + 1)..end], end + 1) : ("", pos + 1);
        }
        if (s[pos] == '(')
        {
            int end = s.IndexOf(')', pos);
            return end >= 0 ? (s[pos..(end + 1)], end + 1) : ("", pos);
        }

        int valEnd = pos;
        while (valEnd < s.Length && !char.IsWhiteSpace(s[valEnd]) && s[valEnd] != ',' && s[valEnd] != '}')
            valEnd++;
        return (s[pos..valEnd], valEnd);
    }

    private static int SkipWhitespaceAndComments(string s, int pos)
    {
        while (pos < s.Length)
        {
            pos = SkipWhitespace(s, pos);
            if (pos + 1 < s.Length && s[pos] == '/' && s[pos + 1] == '/')
            {
                pos = s.IndexOf('\n', pos);
                if (pos < 0) return s.Length;
            }
            else if (pos + 1 < s.Length && s[pos] == '/' && s[pos + 1] == '*')
            {
                pos = s.IndexOf("*/", pos + 2, StringComparison.Ordinal);
                if (pos < 0) return s.Length;
                pos += 2;
            }
            else break;
        }
        return pos;
    }

    private static int SkipWhitespace(string s, int pos)
    {
        while (pos < s.Length && char.IsWhiteSpace(s[pos])) pos++;
        return pos;
    }

    private static bool Match(string s, int pos, string pattern)
    {
        return pos + pattern.Length <= s.Length && s[pos..(pos + pattern.Length)] == pattern;
    }
}
