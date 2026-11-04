//! 77-bit FT8/FT4 messages to text. The encoding (call signs in 28 bits, grids in 15,
//! hashed call signs, free text in base 42) follows WSJT-X's packjt77.f90.

use std::collections::HashMap;

const NTOKENS: u32 = 2_063_592;
const MAX22: u32 = 4_194_304;
const MAXGRID4: u32 = 32_400;
const A1: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const A2: &[u8] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const A3: &[u8] = b"0123456789";
const A4: &[u8] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ";
/// Characters of a hashed call sign.
const HASH_CHARS: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";
/// Characters of free text.
const TEXT_CHARS: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?";
/// ARRL sections (field day messages).
const SECTIONS: [&str; 86] = [
    "AB", "AK", "AL", "AR", "AZ", "BC", "CO", "CT", "DE", "EB", "EMA", "ENY", "EPA", "EWA", "GA", "GH", "IA", "ID", "IL", "IN",
    "KS", "KY", "LA", "LAX", "NS", "MB", "MDC", "ME", "MI", "MN", "MO", "MS", "MT", "NC", "ND", "NE", "NFL", "NH", "NL", "NLI",
    "NM", "NNJ", "NNY", "TER", "NTX", "NV", "OH", "OK", "ONE", "ONN", "ONS", "OR", "ORG", "PAC", "PR", "QC", "RI", "SB", "SC",
    "SCV", "SD", "SDG", "SF", "SFL", "SJV", "SK", "SNJ", "STX", "SV", "TN", "UT", "VA", "VI", "VT", "WCF", "WI", "WMA", "WNY",
    "WPA", "WTX", "WV", "WWA", "WY", "DX", "PE", "NB",
];

/// Hash of a call sign as sent in `m` bits.
pub fn hash_call(call: &str, m: u32) -> u32 {
    let padded: Vec<u8> = call.bytes().chain(std::iter::repeat(b' ')).take(11).collect();
    let mut n: u64 = 0;
    for c in padded {
        let j = HASH_CHARS.iter().position(|&h| h == c).unwrap_or(0) as u64;
        n = n.wrapping_mul(38).wrapping_add(j);
    }
    (47_055_833_459u64.wrapping_mul(n) >> (64 - m)) as u32
}

/// Call signs heard, by their 10-, 12- and 22-bit hashes, to show hashed calls.
#[derive(Default)]
pub struct CallBook {
    h10: HashMap<u32, String>,
    h12: HashMap<u32, String>,
    h22: HashMap<u32, String>,
}

impl CallBook {
    pub fn remember(&mut self, call: &str) {
        let call = call.trim().trim_start_matches('<').trim_end_matches('>');
        if call.len() < 3 || call.starts_with("...") || call.contains(' ') {
            return;
        }
        self.h10.insert(hash_call(call, 10), call.to_string());
        self.h12.insert(hash_call(call, 12), call.to_string());
        self.h22.insert(hash_call(call, 22), call.to_string());
    }

    fn lookup(&self, table: &HashMap<u32, String>, h: u32) -> String {
        table.get(&h).map_or_else(|| "<...>".to_string(), |c| format!("<{c}>"))
    }
}

fn field(bits: &[u8], start: usize, len: usize) -> u64 {
    bits[start..start + len].iter().fold(0u64, |v, &b| (v << 1) | b as u64)
}

/// As `field`, for fields longer than 64 bits (free text and telemetry use 71).
fn wide_field(bits: &[u8], start: usize, len: usize) -> u128 {
    bits[start..start + len].iter().fold(0u128, |v, &b| (v << 1) | b as u128)
}

/// A 28-bit call sign field; None if malformed.
fn unpack28(n28: u32, book: &CallBook) -> Option<String> {
    if n28 < NTOKENS {
        return Some(match n28 {
            0 => "DE".into(),
            1 => "QRZ".into(),
            2 => "CQ".into(),
            3..=1002 => format!("CQ {:03}", n28 - 3),
            1003..=532_443 => {
                let mut n = n28 - 1003;
                let mut s = String::new();
                for d in [27 * 27 * 27, 27 * 27, 27, 1] {
                    s.push(A4[(n / d) as usize] as char);
                    n %= d;
                }
                format!("CQ {}", s.trim())
            }
            _ => return None,
        });
    }
    let n = n28 - NTOKENS;
    if n < MAX22 {
        return Some(book.lookup(&book.h22, n));
    }
    let mut n = n - MAX22;
    let mut s = String::new();
    for (alphabet, d) in [(A1, 36 * 10 * 27 * 27 * 27), (A2, 10 * 27 * 27 * 27), (A3, 27 * 27 * 27), (A4, 27 * 27), (A4, 27), (A4, 1)] {
        let i = (n / d) as usize;
        s.push(*alphabet.get(i)? as char);
        n %= d;
    }
    let s = s.trim().to_string();
    // A blank inside the call sign means the field was not a call sign.
    (!s.contains(' ') && !s.is_empty()).then_some(s)
}

fn grid4(mut n: u32) -> Option<String> {
    let j1 = n / (18 * 10 * 10);
    n -= j1 * 18 * 10 * 10;
    let j2 = n / 100;
    n -= j2 * 100;
    let (j3, j4) = (n / 10, n % 10);
    (j1 < 18 && j2 < 18).then(|| format!("{}{}{}{}", (b'A' + j1 as u8) as char, (b'A' + j2 as u8) as char, j3, j4))
}

fn grid6(mut n: u32) -> Option<String> {
    let mut out = String::new();
    for (d, base, letter) in [(18 * 10 * 10 * 24 * 24, 18, true), (10 * 10 * 24 * 24, 18, true), (10 * 24 * 24, 10, false), (24 * 24, 10, false), (24, 24, true), (1, 24, true)] {
        let j = n / d;
        if j >= base {
            return None;
        }
        n -= j * d;
        out.push(if letter { (b'A' + j as u8) as char } else { (b'0' + j as u8) as char });
    }
    Some(out)
}

/// Signal report text, e.g. -9 → "-09", 5 → "+05".
fn report(r: i32) -> String {
    if r >= 0 { format!("+{r:02}") } else { format!("-{:02}", -r) }
}

/// Text of a 77-bit message (bits as 0/1 bytes); None for undefined or malformed ones.
/// Call signs seen are added to `book`, so later hashed references can be shown.
pub fn unpack(bits: &[u8], book: &mut CallBook) -> Option<String> {
    let n3 = field(bits, 71, 3) as u32;
    let i3 = field(bits, 74, 3) as u32;
    let text = match (i3, n3) {
        (0, 0) => {
            // Free text: 71 bits, 13 characters in base 42.
            let mut v = wide_field(bits, 0, 71);
            let mut chars = [b' '; 13];
            for c in chars.iter_mut().rev() {
                *c = TEXT_CHARS[(v % 42) as usize];
                v /= 42;
            }
            let s = String::from_utf8_lossy(&chars).trim().to_string();
            if s.is_empty() {
                return None;
            }
            s
        }
        (0, 1) => {
            // DXpedition: K1ABC RR73; W9XYZ <KH1/KH7Z> -11
            let a = unpack28(field(bits, 0, 28) as u32, book)?;
            let b = unpack28(field(bits, 28, 28) as u32, book)?;
            let h = book.lookup(&book.h10, field(bits, 56, 10) as u32);
            let r = 2 * field(bits, 66, 5) as i32 - 30;
            format!("{a} RR73; {b} {h} {}", report(r))
        }
        (0, 3) | (0, 4) => {
            // ARRL Field Day: W9XYZ K1ABC R 6A WI
            let a = unpack28(field(bits, 0, 28) as u32, book)?;
            let b = unpack28(field(bits, 28, 28) as u32, book)?;
            let r = bits[56] == 1;
            let ntx = field(bits, 57, 4) as u32 + 1 + if n3 == 4 { 16 } else { 0 };
            let class = (b'A' + field(bits, 61, 3) as u8) as char;
            let section = SECTIONS.get((field(bits, 64, 7) as usize).checked_sub(1)?)?;
            format!("{a} {b} {}{ntx}{class} {section}", if r { "R " } else { "" })
        }
        (0, 5) => {
            // Telemetry: 71 bits as 18 hex digits.
            let v = wide_field(bits, 0, 71);
            format!("{v:018X}").trim_start_matches('0').to_string()
        }
        (1, _) | (2, _) => {
            // Standard message: call call grid/report.
            let n28a = field(bits, 0, 28) as u32;
            let n28b = field(bits, 29, 28) as u32;
            let mut a = unpack28(n28a, book)?;
            let mut b = unpack28(n28b, book)?;
            let suffix = if i3 == 1 { "/R" } else { "/P" };
            if bits[28] == 1 && !a.starts_with('<') && a.len() >= 3 {
                a.push_str(suffix);
            }
            if bits[57] == 1 && !b.starts_with('<') && b.len() >= 3 {
                b.push_str(suffix);
            }
            let r = bits[58] == 1;
            let g = field(bits, 59, 15) as u32;
            if !b.starts_with('<') {
                book.remember(&b);
            }
            if !a.starts_with('<') && !a.starts_with("CQ") && a != "DE" && a != "QRZ" {
                book.remember(&a);
            }
            if g <= MAXGRID4 {
                let grid = grid4(g)?;
                if a.starts_with("CQ") && r {
                    return None;
                }
                if r { format!("{a} {b} R {grid}") } else { format!("{a} {b} {grid}") }
            } else {
                let irpt = g - MAXGRID4;
                let tail = match irpt {
                    1 => String::new(),
                    2 => " RRR".into(),
                    3 => " RR73".into(),
                    4 => " 73".into(),
                    _ => {
                        let mut snr = irpt as i32 - 35;
                        if snr > 50 {
                            snr -= 101;
                        }
                        format!(" {}{}", if r { "R" } else { "" }, report(snr))
                    }
                };
                if a.starts_with("CQ") && irpt >= 2 {
                    return None;
                }
                format!("{a} {b}{tail}")
            }
        }
        (3, _) => {
            // ARRL RTTY contest: TU; K1ABC W9XYZ R 579 MA
            let tu = bits[0] == 1;
            let a = unpack28(field(bits, 1, 28) as u32, book)?;
            let b = unpack28(field(bits, 29, 28) as u32, book)?;
            let r = bits[57] == 1;
            let rst = format!("5{}9", field(bits, 58, 3) + 2);
            let exch = field(bits, 61, 13) as u32;
            let x = if exch > 8000 { format!("#{}", exch - 8000) } else { format!("{exch:04}") };
            format!("{}{a} {b} {}{rst} {x}", if tu { "TU; " } else { "" }, if r { "R " } else { "" })
        }
        (4, _) => {
            // One non-standard call (11 characters) and one hashed call.
            let h = book.lookup(&book.h12, field(bits, 0, 12) as u32);
            let mut n58 = field(bits, 12, 58);
            let mut c11 = [b' '; 11];
            for c in c11.iter_mut().rev() {
                *c = HASH_CHARS[(n58 % 38) as usize];
                n58 /= 38;
            }
            let call = String::from_utf8_lossy(&c11).trim().to_string();
            book.remember(&call);
            let flip = bits[70] == 1;
            let nrpt = field(bits, 71, 2);
            let cq = bits[73] == 1;
            if cq {
                format!("CQ {call}")
            } else {
                let (a, b) = if flip { (call, h) } else { (h, call) };
                let tail = ["", " RRR", " RR73", " 73"][nrpt as usize];
                format!("{a} {b}{tail}")
            }
        }
        (5, _) => {
            // EU VHF contest: <PA3XYZ> <G4ABC/P> R 590003 IO91NP
            let a = book.lookup(&book.h12, field(bits, 0, 12) as u32);
            let b = book.lookup(&book.h22, field(bits, 12, 22) as u32);
            let r = bits[34] == 1;
            let rs = 52 + field(bits, 35, 3);
            let serial = field(bits, 38, 11);
            let g = grid6(field(bits, 49, 25) as u32)?;
            format!("{a} {b} {}{rs}{serial:04} {g}", if r { "R " } else { "" })
        }
        _ => return None,
    };
    (!text.starts_with("CQ <")).then_some(text)
}

/// A call sign as 28 bits (standard calls and the CQ/DE/QRZ tokens); for tests and
/// later transmitting.
pub fn pack28(call: &str) -> Option<u32> {
    match call {
        "DE" => return Some(0),
        "QRZ" => return Some(1),
        "CQ" => return Some(2),
        _ => {}
    }
    let c = call.as_bytes();
    // Call area digit: the last digit, at position 2 or 3 (1-based).
    let area = c.iter().rposition(|b| b.is_ascii_digit())? + 1;
    if !(2..=3).contains(&area) || c.len() > 6 {
        return None;
    }
    let six: Vec<u8> = if area == 2 { std::iter::once(b' ').chain(c.iter().copied()).collect() } else { c.to_vec() };
    let six: Vec<u8> = six.into_iter().chain(std::iter::repeat(b' ')).take(6).collect();
    let idx = |alphabet: &[u8], ch: u8| alphabet.iter().position(|&a| a == ch).map(|p| p as u32);
    let n = idx(A1, six[0])? * 36 * 10 * 27 * 27 * 27
        + idx(A2, six[1])? * 10 * 27 * 27 * 27
        + idx(A3, six[2])? * 27 * 27 * 27
        + idx(A4, six[3])? * 27 * 27
        + idx(A4, six[4])? * 27
        + idx(A4, six[5])?;
    Some(n + NTOKENS + MAX22)
}

/// A standard message (type 1) as 77 bits: "CQ K1ABC FN42", "K1ABC W9XYZ -11",
/// "K1ABC W9XYZ R-09", "K1ABC W9XYZ RR73", "K1ABC W9XYZ 73", "K1ABC W9XYZ RRR".
pub fn pack_standard(text: &str) -> Option<Vec<u8>> {
    let words: Vec<&str> = text.split_whitespace().collect();
    let (a, b, rest) = match words.as_slice() {
        [a, b] => (*a, *b, ""),
        [a, b, r] => (*a, *b, *r),
        _ => return None,
    };
    let (mut r, mut tail) = (false, rest);
    if let Some(t) = tail.strip_prefix('R').filter(|t| t.starts_with('-') || t.starts_with('+')) {
        r = true;
        tail = t;
    }
    let g = match tail {
        "" => MAXGRID4 + 1,
        "RRR" => MAXGRID4 + 2,
        "RR73" => MAXGRID4 + 3,
        "73" => MAXGRID4 + 4,
        t if t.starts_with('-') || t.starts_with('+') => {
            let snr: i32 = t.parse().ok()?;
            let irpt = if snr < -50 { snr + 101 } else { snr } + 35;
            MAXGRID4 + irpt as u32
        }
        t if t.len() == 4 => {
            let c = t.as_bytes();
            (c[0] - b'A') as u32 * 1800 + (c[1] - b'A') as u32 * 100 + (c[2] - b'0') as u32 * 10 + (c[3] - b'0') as u32
        }
        _ => return None,
    };
    let mut bits = Vec::with_capacity(77);
    let mut push = |v: u64, n: usize| bits.extend((0..n).rev().map(|i| ((v >> i) & 1) as u8));
    push(pack28(a)? as u64, 28);
    push(0, 1);
    push(pack28(b)? as u64, 28);
    push(0, 1);
    push(r as u64, 1);
    push(g as u64, 15);
    push(1, 3);
    Some(bits)
}

/// Free text (up to 13 characters of TEXT_CHARS) as 77 bits.
pub fn pack_text(text: &str) -> Option<Vec<u8>> {
    let t = text.to_ascii_uppercase();
    if t.len() > 13 {
        return None;
    }
    let mut v: u128 = 0;
    for c in format!("{t:>13}").bytes() {
        v = v * 42 + TEXT_CHARS.iter().position(|&x| x == c)? as u128;
    }
    let mut bits: Vec<u8> = (0..71).rev().map(|i| ((v >> i) & 1) as u8).collect();
    bits.extend([0, 0, 0, 0, 0, 0]);
    Some(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_messages_round_trip() {
        let mut book = CallBook::default();
        for text in [
            "CQ K1ABC FN42",
            "K1ABC W9XYZ EN37",
            "W9XYZ K1ABC -11",
            "K1ABC W9XYZ R-09",
            "W9XYZ K1ABC RRR",
            "K1ABC W9XYZ 73",
            "K1ABC W9XYZ RR73",
            "CQ TA4SO KM46",
            "TA4SO DL1ABC +05",
        ] {
            let bits = pack_standard(text).expect(text);
            assert_eq!(bits.len(), 77);
            assert_eq!(unpack(&bits, &mut book).as_deref(), Some(text));
        }
    }

    #[test]
    fn free_text_round_trips() {
        let mut book = CallBook::default();
        let bits = pack_text("TNX QSO 73").unwrap();
        assert_eq!(unpack(&bits, &mut book).as_deref(), Some("TNX QSO 73"));
        // All 13 characters used: the value needs all 71 bits (once lost above 64 bits).
        let bits = pack_text("ZZZZZZZZZZZZZ").unwrap();
        assert_eq!(unpack(&bits, &mut book).as_deref(), Some("ZZZZZZZZZZZZZ"));
    }

    #[test]
    fn hashes_are_stable_and_in_range() {
        let h = hash_call("KH1/KH7Z", 22);
        assert!(h < 1 << 22);
        assert_eq!(h, hash_call("KH1/KH7Z", 22));
        assert_ne!(h, hash_call("KH1/KH7Y", 22));
    }
}
