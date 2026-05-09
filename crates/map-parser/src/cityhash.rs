//! CityHash64 — the variant used by SCS HashFS.
//!
//! Ported 1:1 from `TruckLib.HashFs/CityHash.cs` (sk-zk), which itself is a
//! port of `cityhash-c` by Alexander Nusov, which is a port of Google's
//! original CityHash 1.0.x. **This is intentionally NOT Google's CityHash
//! 1.1.x**, which is what most third-party Rust/Python crates ship — they
//! produce different hashes for non-empty input and do **not** match SCS
//! archive entry hashes.
//!
//! The empirical proof lives in `tests/real_archive_test.rs`: the hash of
//! `"automat"` computed here matches a real entry in ETS2 1.55 `base.scs`,
//! while the previous Google-1.1.x-style implementation matched none of
//! three known top-level paths.
//!
//! ## Verified test vectors (all from TruckLib's `CityHash.CityHash64`)
//! ```text
//! cityhash64(b"")            = 0x9AE16A3B2F90404F  (== K2)
//! cityhash64(b"abc")         = 0x3A912F483A4ECE31
//! cityhash64(b"automat")     = 0x56BC42EECBC73F2F  ← present in base.scs
//! cityhash64(b"manifest.sii")= 0xB97FFF7CE7377C95
//! ```

// Some primes between 2^63 and 2^64 for various uses.
const K0: u64 = 0xc3a5c85c97cb3127;
const K1: u64 = 0xb492b66fbe98f273;
const K2: u64 = 0x9ae16a3b2f90404f;
const K3: u64 = 0xc949d7c7509e6557;

/// Multiplier for `Hash128To64` — Murmur-inspired.
const K_MUL: u64 = 0x9ddfea08eb382d69;

/// Compute CityHash64 (1.0.x variant, as used by SCS HashFS).
pub fn cityhash64(data: &[u8]) -> u64 {
    let len = data.len();
    if len <= 16 {
        return hash_len_0_to_16(data);
    }
    if len <= 32 {
        return hash_len_17_to_32(data);
    }
    if len <= 64 {
        return hash_len_33_to_64(data);
    }
    hash_len_65_plus(data)
}

// ── helpers ──────────────────────────────────────────────────────────

fn fetch32(s: &[u8], pos: usize) -> u64 {
    u32::from_le_bytes(s[pos..pos + 4].try_into().unwrap()) as u64
}

fn fetch64(s: &[u8], pos: usize) -> u64 {
    u64::from_le_bytes(s[pos..pos + 8].try_into().unwrap())
}

/// Bitwise right rotate. `shift` is in bits (0..=63).
fn rotate(val: u64, shift: u32) -> u64 {
    if shift == 0 {
        val
    } else {
        val.rotate_right(shift)
    }
}

/// Right rotate by **at least one** bit. Caller guarantees `shift >= 1`.
fn rotate_by_at_least_1(val: u64, shift: u32) -> u64 {
    val.rotate_right(shift)
}

fn shift_mix(val: u64) -> u64 {
    val ^ (val >> 47)
}

/// Hash 128 input bits down to 64. Murmur-inspired.
fn hash128to64(lo: u64, hi: u64) -> u64 {
    let mut a = (lo ^ hi).wrapping_mul(K_MUL);
    a ^= a >> 47;
    let mut b = (hi ^ a).wrapping_mul(K_MUL);
    b ^= b >> 47;
    b.wrapping_mul(K_MUL)
}

fn hash_len_16(u: u64, v: u64) -> u64 {
    hash128to64(u, v)
}

// ── length-specific sub-hashers ─────────────────────────────────────

fn hash_len_0_to_16(s: &[u8]) -> u64 {
    let len = s.len();
    if len > 8 {
        let a = fetch64(s, 0);
        let b = fetch64(s, len - 8);
        return hash_len_16(
            a,
            rotate_by_at_least_1(b.wrapping_add(len as u64), len as u32),
        ) ^ b;
    }
    if len >= 4 {
        let a = fetch32(s, 0);
        return hash_len_16(
            (len as u64).wrapping_add(a << 3),
            fetch32(s, len - 4),
        );
    }
    if len > 0 {
        let a = s[0] as u64;
        let b = s[len >> 1] as u64;
        let c = s[len - 1] as u64;
        let y = a.wrapping_add(b << 8);
        let z = (len as u64).wrapping_add(c << 2);
        return shift_mix(y.wrapping_mul(K2) ^ z.wrapping_mul(K3)).wrapping_mul(K2);
    }
    K2
}

fn hash_len_17_to_32(s: &[u8]) -> u64 {
    let len = s.len();
    let a = fetch64(s, 0).wrapping_mul(K1);
    let b = fetch64(s, 8);
    let c = fetch64(s, len - 8).wrapping_mul(K2);
    let d = fetch64(s, len - 16).wrapping_mul(K0);
    hash_len_16(
        rotate(a.wrapping_sub(b), 43)
            .wrapping_add(rotate(c, 30))
            .wrapping_add(d),
        a.wrapping_add(rotate(b ^ K3, 20))
            .wrapping_sub(c)
            .wrapping_add(len as u64),
    )
}

fn hash_len_33_to_64(s: &[u8]) -> u64 {
    let len = s.len();
    let mut z = fetch64(s, 24);
    let mut a = fetch64(s, 0).wrapping_add(
        ((len as u64).wrapping_add(fetch64(s, len - 16))).wrapping_mul(K0),
    );
    let mut b = rotate(a.wrapping_add(z), 52);
    let mut c = rotate(a, 37);
    a = a.wrapping_add(fetch64(s, 8));
    c = c.wrapping_add(rotate(a, 7));
    a = a.wrapping_add(fetch64(s, 16));
    let vf = a.wrapping_add(z);
    let vs = b.wrapping_add(rotate(a, 31)).wrapping_add(c);
    a = fetch64(s, 16).wrapping_add(fetch64(s, len - 32));
    z = fetch64(s, len - 8);
    b = rotate(a.wrapping_add(z), 52);
    c = rotate(a, 37);
    a = a.wrapping_add(fetch64(s, len - 24));
    c = c.wrapping_add(rotate(a, 7));
    a = a.wrapping_add(fetch64(s, len - 16));
    let wf = a.wrapping_add(z);
    let ws = b.wrapping_add(rotate(a, 31)).wrapping_add(c);
    let r = shift_mix(
        vf.wrapping_add(ws)
            .wrapping_mul(K2)
            .wrapping_add(wf.wrapping_add(vs).wrapping_mul(K0)),
    );
    shift_mix(r.wrapping_mul(K0).wrapping_add(vs)).wrapping_mul(K2)
}

/// Returns `(first, second)` of the C# `Uint128`.
fn weak_hash_len_32_with_seeds_raw(
    w: u64,
    x: u64,
    y: u64,
    z: u64,
    a0: u64,
    b0: u64,
) -> (u64, u64) {
    let mut a = a0.wrapping_add(w);
    let mut b = rotate(b0.wrapping_add(a).wrapping_add(z), 21);
    let c = a;
    a = a.wrapping_add(x);
    a = a.wrapping_add(y);
    b = b.wrapping_add(rotate(a, 44));
    (a.wrapping_add(z), b.wrapping_add(c))
}

fn weak_hash_len_32_with_seeds(s: &[u8], pos: usize, a: u64, b: u64) -> (u64, u64) {
    weak_hash_len_32_with_seeds_raw(
        fetch64(s, pos),
        fetch64(s, pos + 8),
        fetch64(s, pos + 16),
        fetch64(s, pos + 24),
        a,
        b,
    )
}

fn hash_len_65_plus(s: &[u8]) -> u64 {
    let len = s.len();
    // Hash the end first, then loop with 56 bytes of state (v, w, x, y, z).
    let mut x = fetch64(s, len - 40);
    let mut y = fetch64(s, len - 16).wrapping_add(fetch64(s, len - 56));
    let mut z = hash_len_16(
        fetch64(s, len - 48).wrapping_add(len as u64),
        fetch64(s, len - 24),
    );
    let mut v = weak_hash_len_32_with_seeds(s, len - 64, len as u64, z);
    let mut w = weak_hash_len_32_with_seeds(s, len - 32, y.wrapping_add(K1), x);
    x = x.wrapping_mul(K1).wrapping_add(fetch64(s, 0));

    // Decrease len to nearest multiple of 64 and process 64-byte chunks.
    let mut remaining = (len - 1) & !63usize;
    let mut pos = 0usize;
    loop {
        x = rotate(
            x.wrapping_add(y)
                .wrapping_add(v.0)
                .wrapping_add(fetch64(s, pos + 8)),
            37,
        )
        .wrapping_mul(K1);
        y = rotate(
            y.wrapping_add(v.1).wrapping_add(fetch64(s, pos + 48)),
            42,
        )
        .wrapping_mul(K1);
        x ^= w.1;
        y = y.wrapping_add(v.0).wrapping_add(fetch64(s, pos + 40));
        z = rotate(z.wrapping_add(w.0), 33).wrapping_mul(K1);
        v = weak_hash_len_32_with_seeds(
            s,
            pos,
            v.1.wrapping_mul(K1),
            x.wrapping_add(w.0),
        );
        w = weak_hash_len_32_with_seeds(
            s,
            pos + 32,
            z.wrapping_add(w.1),
            y.wrapping_add(fetch64(s, pos + 16)),
        );
        std::mem::swap(&mut z, &mut x);

        pos += 64;
        remaining = remaining.saturating_sub(64);
        if remaining == 0 {
            break;
        }
    }

    hash_len_16(
        hash_len_16(v.0, w.0)
            .wrapping_add(shift_mix(y).wrapping_mul(K1))
            .wrapping_add(z),
        hash_len_16(v.1, w.1).wrapping_add(x),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Empty input → K2.
    #[test]
    fn empty_string_returns_k2() {
        assert_eq!(cityhash64(b""), 0x9AE16A3B2F90404F);
    }

    /// Verified against TruckLib `CityHash.CityHash64` on 2026-05-09 via dotnet.
    #[test]
    fn trucklib_vector_abc() {
        assert_eq!(cityhash64(b"abc"), 0x3A912F483A4ECE31);
    }

    /// Verified against TruckLib **and** confirmed present in
    /// ETS2 1.55 `base.scs` index1 — the empirical regression sentinel.
    #[test]
    fn trucklib_vector_automat() {
        assert_eq!(cityhash64(b"automat"), 0x56BC42EECBC73F2F);
    }

    /// Verified against TruckLib `CityHash.CityHash64`.
    #[test]
    fn trucklib_vector_manifest_sii() {
        assert_eq!(cityhash64(b"manifest.sii"), 0xB97FFF7CE7377C95);
    }

    /// Verified against TruckLib (string length 11 → 17_to_32 path). Was a
    /// useful boundary check while porting.
    #[test]
    fn trucklib_vector_version_sii() {
        assert_eq!(cityhash64(b"version.sii"), 0x02D338506339918D);
    }

    /// Length 18 → 17_to_32 path, longer ASCII path.
    #[test]
    fn trucklib_vector_road_sii() {
        assert_eq!(
            cityhash64(b"def/world/road.sii"),
            0x97E6A16838335F87,
        );
    }

    /// Length 29 → 17_to_32 path edge.
    #[test]
    fn trucklib_vector_sector_base() {
        assert_eq!(
            cityhash64(b"map/europe/sec+0000+0000.base"),
            0xC6C0364995A4439A,
        );
    }

    #[test]
    fn deterministic() {
        let a = cityhash64(b"def/world/road.sii");
        let b = cityhash64(b"def/world/road.sii");
        assert_eq!(a, b);
    }

    #[test]
    fn different_paths_differ() {
        assert_ne!(cityhash64(b"foo"), cityhash64(b"bar"));
    }
}
