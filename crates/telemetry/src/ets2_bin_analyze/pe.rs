//! Minimal PE32+ reader for offline file analysis (no process memory).

use std::fs;
use std::path::{Path, PathBuf};

/// One PE section mapped from on-disk layout.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeSection {
    pub name: String,
    pub rva: usize,
    pub virtual_size: usize,
    pub raw_offset: usize,
    pub raw_size: usize,
}

/// Parsed PE file loaded from disk.
#[derive(Debug, Clone)]
pub struct PeFile {
    pub path: PathBuf,
    pub data: Vec<u8>,
    pub image_base: u64,
    pub sections: Vec<PeSection>,
}

impl PeFile {
    /// Read and parse a PE executable from disk.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let data = fs::read(path).map_err(|e| format!("read {}: {e}", path.display()))?;
        parse_pe(path, &data)
    }

    pub fn section(&self, name: &str) -> Option<&PeSection> {
        self.sections.iter().find(|s| s.name == name)
    }

    pub fn section_data<'a>(&'a self, name: &str) -> Option<&'a [u8]> {
        let sec = self.section(name)?;
        let end = sec.raw_offset.checked_add(sec.raw_size)?;
        self.data.get(sec.raw_offset..end)
    }

    /// Map RVA to file offset using section headers.
    pub fn rva_to_offset(&self, rva: usize) -> Option<usize> {
        for sec in &self.sections {
            let span = sec.virtual_size.max(sec.raw_size);
            if rva >= sec.rva && rva < sec.rva.saturating_add(span) {
                let delta = rva - sec.rva;
                return sec.raw_offset.checked_add(delta).filter(|&off| off < self.data.len());
            }
        }
        None
    }

    pub fn read_at_rva(&self, rva: usize, len: usize) -> Option<&[u8]> {
        let off = self.rva_to_offset(rva)?;
        self.data.get(off..off.checked_add(len)?)
    }

    pub fn read_i32_at_rva(&self, rva: usize) -> Option<i32> {
        let b = self.read_at_rva(rva, 4)?;
        Some(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    pub fn read_u32_at_rva(&self, rva: usize) -> Option<u32> {
        let b = self.read_at_rva(rva, 4)?;
        Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// Extract a `.text` byte window around `center_rva`.
    pub fn text_window(
        &self,
        center_rva: usize,
        before: usize,
        after: usize,
    ) -> Option<(Vec<u8>, usize)> {
        let sec = self.section(".text")?;
        let data = self.section_data(".text")?;
        let rel = center_rva.checked_sub(sec.rva)?;
        if rel >= data.len() {
            return None;
        }
        let start_rel = rel.saturating_sub(before);
        let end_rel = (rel + after).min(data.len());
        Some((data[start_rel..end_rel].to_vec(), sec.rva + start_rel))
    }

    /// Read a contiguous `.text` slice `[start_rva, end_rva)`.
    pub fn read_rva_range(&self, start_rva: usize, end_rva: usize) -> Option<Vec<u8>> {
        if end_rva < start_rva {
            return None;
        }
        let sec = self.section(".text")?;
        let data = self.section_data(".text")?;
        let start_rel = start_rva.checked_sub(sec.rva)?;
        let end_rel = end_rva.checked_sub(sec.rva)?.min(data.len());
        if start_rel >= data.len() || start_rel > end_rel {
            return None;
        }
        Some(data[start_rel..end_rel].to_vec())
    }

    /// True when `rva` falls inside the named section virtual span.
    pub fn rva_in_section_name(&self, rva: usize, name: &str) -> bool {
        self.section(name)
            .map(|s| {
                let span = s.virtual_size.max(s.raw_size);
                rva >= s.rva && rva < s.rva.saturating_add(span)
            })
            .unwrap_or(false)
    }
    /// Test helper: parse in-memory PE bytes.
    #[cfg(test)]
    pub fn from_bytes_for_test(path: &Path, data: Vec<u8>) -> Result<Self, String> {
        parse_pe(path, &data)
    }
}

pub(crate) fn parse_pe(path: &Path, data: &[u8]) -> Result<PeFile, String> {
    if data.len() < 0x40 {
        return Err("file too small for DOS header".into());
    }
    if &data[0..2] != b"MZ" {
        return Err("missing MZ DOS signature".into());
    }
    let e_lfanew = i32::from_le_bytes(data[0x3C..0x40].try_into().unwrap()) as usize;
    if e_lfanew + 0x18 > data.len() {
        return Err("invalid e_lfanew".into());
    }
    let pe = &data[e_lfanew..];
    if pe.len() < 24 || &pe[0..4] != b"PE\0\0" {
        return Err("missing PE signature".into());
    }
    let num_sections = u16::from_le_bytes(pe[6..8].try_into().unwrap()) as usize;
    let opt_size = u16::from_le_bytes(pe[20..22].try_into().unwrap()) as usize;
    let opt = pe.get(24..24 + opt_size).ok_or("optional header truncated")?;
    if opt.len() < 2 {
        return Err("optional header too small".into());
    }
    let magic = u16::from_le_bytes(opt[0..2].try_into().unwrap());
    let image_base = match magic {
        0x20B => {
            if opt.len() < 32 {
                return Err("PE32+ optional header too small".into());
            }
            u64::from_le_bytes(opt[24..32].try_into().unwrap())
        }
        0x10B => {
            if opt.len() < 28 {
                return Err("PE32 optional header too small".into());
            }
            u32::from_le_bytes(opt[28..32].try_into().unwrap()) as u64
        }
        _ => return Err(format!("unknown optional header magic 0x{magic:X}")),
    };

    let sect_off = 24 + opt_size;
    if pe.len() < sect_off + num_sections * 40 {
        return Err("section table truncated".into());
    }

    let mut sections = Vec::with_capacity(num_sections);
    for i in 0..num_sections {
        let s = &pe[sect_off + i * 40..sect_off + (i + 1) * 40];
        let mut name_bytes = [0u8; 8];
        name_bytes.copy_from_slice(&s[0..8]);
        let name = std::str::from_utf8(&name_bytes)
            .unwrap_or("")
            .trim_end_matches('\0')
            .to_string();
        let virtual_size = u32::from_le_bytes(s[8..12].try_into().unwrap()) as usize;
        let rva = u32::from_le_bytes(s[12..16].try_into().unwrap()) as usize;
        let raw_size = u32::from_le_bytes(s[16..20].try_into().unwrap()) as usize;
        let raw_offset = u32::from_le_bytes(s[20..24].try_into().unwrap()) as usize;
        sections.push(PeSection {
            name,
            rva,
            virtual_size,
            raw_offset,
            raw_size,
        });
    }

    Ok(PeFile {
        path: path.to_path_buf(),
        data: data.to_vec(),
        image_base,
        sections,
    })
}

#[cfg(test)]
pub fn build_minimal_pe64(text_bytes: &[u8], rdata_bytes: &[u8]) -> Vec<u8> {
    let file_align = 0x200u32;
    let sect_align = 0x1000u32;
    let headers_size = 0x200usize;
    let text_raw_off = headers_size;
    let text_rva = sect_align as usize;
    let text_size = ((text_bytes.len() + file_align as usize - 1) / file_align as usize)
        * file_align as usize;
    let rdata_raw_off = text_raw_off + text_size;
    let rdata_rva = text_rva + sect_align as usize;
    let rdata_size = ((rdata_bytes.len() + file_align as usize - 1) / file_align as usize)
        * file_align as usize;
    let file_size = rdata_raw_off + rdata_size;

    let mut buf = vec![0u8; file_size];
    buf[0..2].copy_from_slice(b"MZ");
    buf[0x3C..0x40].copy_from_slice(&0x80u32.to_le_bytes());

    let pe_off = 0x80usize;
    buf[pe_off..pe_off + 4].copy_from_slice(b"PE\0\0");
    buf[pe_off + 4..pe_off + 6].copy_from_slice(&0x8664u16.to_le_bytes()); // AMD64
    buf[pe_off + 6..pe_off + 8].copy_from_slice(&2u16.to_le_bytes()); // 2 sections
    buf[pe_off + 20..pe_off + 22].copy_from_slice(&0xF0u16.to_le_bytes()); // opt hdr size

    let opt = pe_off + 24;
    buf[opt..opt + 2].copy_from_slice(&0x20Bu16.to_le_bytes());
    buf[opt + 16..opt + 20].copy_from_slice(&file_align.to_le_bytes());
    buf[opt + 20..opt + 24].copy_from_slice(&0x100u32.to_le_bytes()); // major linker
    buf[opt + 32..opt + 36].copy_from_slice(&sect_align.to_le_bytes());
    buf[opt + 36..opt + 40].copy_from_slice(&file_align.to_le_bytes());
    let size_of_image = rdata_rva + rdata_size.max(rdata_bytes.len());
    buf[opt + 56..opt + 60].copy_from_slice(&(size_of_image as u32).to_le_bytes());
    let image_base = 0x1_4000_0000u64;
    buf[opt + 24..opt + 32].copy_from_slice(&image_base.to_le_bytes());

    let sect = opt + 0xF0;
    write_section(&mut buf, sect, b".text\0\0\0", text_rva, text_bytes.len(), text_raw_off, text_size);
    write_section(
        &mut buf,
        sect + 40,
        b".rdata\0\0",
        rdata_rva,
        rdata_bytes.len(),
        rdata_raw_off,
        rdata_size,
    );

    buf[text_raw_off..text_raw_off + text_bytes.len()].copy_from_slice(text_bytes);
    buf[rdata_raw_off..rdata_raw_off + rdata_bytes.len()].copy_from_slice(rdata_bytes);
    buf
}

#[cfg(test)]
fn write_section(
    buf: &mut [u8],
    off: usize,
    name: &[u8; 8],
    rva: usize,
    vsize: usize,
    raw_off: usize,
    raw_size: usize,
) {
    buf[off..off + 8].copy_from_slice(name);
    buf[off + 8..off + 12].copy_from_slice(&(vsize as u32).to_le_bytes());
    buf[off + 12..off + 16].copy_from_slice(&(rva as u32).to_le_bytes());
    buf[off + 16..off + 20].copy_from_slice(&(raw_size as u32).to_le_bytes());
    buf[off + 20..off + 24].copy_from_slice(&(raw_off as u32).to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn minimal_pe_maps_text_and_rdata() {
        let text = [0xCCu8; 32];
        let rdata = b"simple_route_test\0";
        let bytes = build_minimal_pe64(&text, rdata);
        let pe = parse_pe(Path::new("test.exe"), &bytes).unwrap();
        assert_eq!(pe.image_base, 0x1_4000_0000);
        assert!(pe.section(".text").is_some());
        assert!(pe.section(".rdata").is_some());
        assert_eq!(pe.section_data(".text").unwrap().len(), 512);
        assert_eq!(&pe.section_data(".text").unwrap()[..32], &text[..32]);
        assert!(pe.section_data(".rdata").unwrap().starts_with(b"simple_route"));
    }
}
