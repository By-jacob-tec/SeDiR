//! FT8 and FT4 reception: slots of FSK with Costas sync arrays, LDPC (174,91) coding
//! and 77-bit messages, as defined by WSJT-X (K1JT et al.). The decoder here is written
//! for SeDiR; the code tables, the sync patterns, the scrambling vector and the message
//! encoding constants come from WSJT-X.

pub mod decode;
pub mod ldpc;
pub mod message;
pub mod tables;
pub mod worker;

/// What differs between FT8 and FT4; the decoder works from this.
#[derive(Debug)]
pub struct Protocol {
    pub name: &'static str,
    pub tones: usize,
    pub bits_per_symbol: usize,
    /// Sync and data symbols (FT4's ramp symbols at either end are not counted).
    pub symbols: usize,
    /// Sample rate of the baseband the decoder works on, and samples per symbol there.
    pub rate: f64,
    pub sps: usize,
    /// Samples per symbol after a candidate is brought down on its own.
    pub fine_sps: usize,
    /// Sync blocks: first symbol and tone pattern.
    pub sync: &'static [(usize, &'static [usize])],
    /// Bits (as a number, first bit most significant) → tone.
    pub gray: &'static [usize],
    /// Data symbol j → its place in the transmission.
    pub data_symbol: fn(usize) -> usize,
    /// Bits XORed into the 77-bit message before encoding (FT4).
    pub scramble: Option<&'static [u8; 77]>,
    pub slot_s: f64,
    /// Nominal start of sync symbol 0 after the slot start.
    pub start_s: f64,
    /// The samples given begin this long before the slot (senders whose clocks run
    /// ahead), and the latest start searched after the slot start.
    pub early_s: f64,
    pub latest_start_s: f64,
    /// A slot is decoded when this much of it (from the slot start) has arrived.
    pub decode_at_s: f64,
}

impl Protocol {
    pub fn tone_spacing(&self) -> f64 {
        self.rate / self.sps as f64
    }

    pub fn data_symbols(&self) -> usize {
        174 / self.bits_per_symbol
    }

    /// Tones of a 174-bit codeword.
    pub fn tones_of(&self, codeword: &[u8; ldpc::N]) -> Vec<usize> {
        let mut t = vec![0usize; self.symbols];
        for &(start, pattern) in self.sync {
            t[start..start + pattern.len()].copy_from_slice(pattern);
        }
        let b = self.bits_per_symbol;
        for j in 0..self.data_symbols() {
            let index = codeword[b * j..b * j + b].iter().fold(0usize, |v, &bit| (v << 1) | bit as usize);
            t[(self.data_symbol)(j)] = self.gray[index];
        }
        t
    }
}

pub static FT8: Protocol = Protocol {
    name: "FT8",
    tones: 8,
    bits_per_symbol: 3,
    symbols: 79,
    rate: 6400.0,
    sps: 1024,
    fine_sps: 32,
    sync: &[(0, &COSTAS), (36, &COSTAS), (72, &COSTAS)],
    gray: &GRAY,
    data_symbol,
    scramble: None,
    slot_s: 15.0,
    start_s: 0.5,
    early_s: 2.0,
    latest_start_s: 2.6,
    decode_at_s: 14.0,
};

/// FT4: 4-GFSK, 48 ms symbols (20.833 Hz tones), 7.5 s slots; a ramp symbol before the
/// first sync block, so sync symbol 0 starts 48 ms after the nominal 0.5 s.
pub static FT4: Protocol = Protocol {
    name: "FT4",
    tones: 4,
    bits_per_symbol: 2,
    symbols: 103,
    rate: 6000.0,
    sps: 288,
    fine_sps: 36,
    sync: &[(0, &[0, 1, 3, 2]), (33, &[1, 0, 2, 3]), (66, &[2, 3, 1, 0]), (99, &[3, 2, 0, 1])],
    gray: &[0, 1, 3, 2],
    data_symbol: ft4_data_symbol,
    scramble: Some(&FT4_SCRAMBLE),
    slot_s: 7.5,
    start_s: 0.548,
    early_s: 1.0,
    latest_start_s: 1.6,
    decode_at_s: 6.2,
};

/// FT4's scrambling vector (rvec in WSJT-X's genft4.f90).
const FT4_SCRAMBLE: [u8; 77] = [
    0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1,
];

fn ft4_data_symbol(j: usize) -> usize {
    match j {
        0..29 => 4 + j,
        29..58 => 8 + j,
        _ => 12 + j,
    }
}

/// Tone sequence of each FT8 sync block.
pub const COSTAS: [usize; 7] = [3, 1, 4, 0, 6, 5, 2];
/// 3 code bits (as a number) → tone.
pub const GRAY: [usize; 8] = [0, 1, 3, 2, 5, 6, 4, 7];
/// Symbols per FT8 transmission: three sync blocks of 7 around two data blocks of 29.
pub const SYMBOLS: usize = 79;
pub const SYMBOL_SECONDS: f64 = 0.16;
pub const TONE_SPACING_HZ: f64 = 6.25;
/// FT8 slot length and the nominal start of a transmission within it.
pub const SLOT_SECONDS: f64 = 15.0;
pub const START_SECONDS: f64 = 0.5;

/// Tones of a 174-bit FT8 codeword.
pub fn tones(codeword: &[u8; ldpc::N]) -> [usize; SYMBOLS] {
    let v = FT8.tones_of(codeword);
    let mut t = [0usize; SYMBOLS];
    t.copy_from_slice(&v);
    t
}

/// Index of each FT8 data symbol (58 of them) in the 79-symbol transmission.
pub fn data_symbol(j: usize) -> usize {
    if j < 29 { 7 + j } else { 14 + j }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ft4_layout() {
        // Every symbol is either sync or data, exactly once.
        let mut used = vec![0; FT4.symbols];
        for &(s, p) in FT4.sync {
            for k in 0..p.len() {
                used[s + k] += 1;
            }
        }
        for j in 0..FT4.data_symbols() {
            used[(FT4.data_symbol)(j)] += 1;
        }
        assert!(used.iter().all(|&u| u == 1), "{used:?}");
        assert_eq!(FT4.data_symbols(), 87);
        assert!((FT4.tone_spacing() - 20.833).abs() < 0.01);
        assert_eq!(FT8.tone_spacing(), 6.25);
    }
}
