//! FT8 reception: 15 s slots of 8-FSK (6.25 Hz tone spacing, 0.16 s symbols) with
//! Costas sync arrays, LDPC (174,91) coding and 77-bit messages, as defined by WSJT-X
//! (K1JT et al.). The decoder here is written for SeDiR; only the code tables and the
//! message encoding constants come from WSJT-X.

pub mod decode;
pub mod ldpc;
pub mod message;
pub mod tables;
pub mod worker;

/// Tone sequence of each sync block.
pub const COSTAS: [usize; 7] = [3, 1, 4, 0, 6, 5, 2];
/// 3 code bits (as a number) → tone.
pub const GRAY: [usize; 8] = [0, 1, 3, 2, 5, 6, 4, 7];
/// Symbols per transmission: three sync blocks of 7 around two data blocks of 29.
pub const SYMBOLS: usize = 79;
pub const SYMBOL_SECONDS: f64 = 0.16;
pub const TONE_SPACING_HZ: f64 = 6.25;
/// Slot length and the nominal start of a transmission within it.
pub const SLOT_SECONDS: f64 = 15.0;
pub const START_SECONDS: f64 = 0.5;

/// Tones of a 174-bit codeword.
pub fn tones(codeword: &[u8; ldpc::N]) -> [usize; SYMBOLS] {
    let mut t = [0usize; SYMBOLS];
    for start in [0usize, 36, 72] {
        t[start..start + 7].copy_from_slice(&COSTAS);
    }
    for j in 0..58 {
        let b = &codeword[3 * j..3 * j + 3];
        t[data_symbol(j)] = GRAY[(b[0] as usize) << 2 | (b[1] as usize) << 1 | b[2] as usize];
    }
    t
}

/// Index of each data symbol (58 of them) in the 79-symbol transmission.
pub fn data_symbol(j: usize) -> usize {
    if j < 29 { 7 + j } else { 14 + j }
}
