//! FT8 error control: 14-bit CRC and the (174,91) LDPC code, decoded by belief
//! propagation (sum-product).

use super::tables::{GENERATOR, MN};

pub const N: usize = 174;
pub const K: usize = 91;
pub const M: usize = N - K;

/// CRC-14 of a 77-bit message, as WSJT-X computes it: an augmented CRC with polynomial
/// 0x2757 over 96 bits (the message, then 19 zero bits).
pub fn crc14(message: &[u8]) -> u16 {
    let mut reg: u16 = 0;
    for i in 0..96 {
        let bit = if i < 77 { message[i] & 1 } else { 0 } as u16;
        let top = (reg >> 13) & 1;
        reg = ((reg << 1) | bit) & 0x3FFF;
        if top == 1 {
            reg ^= 0x2757;
        }
    }
    reg
}

/// The 91 bits of a generator row (message bit 1 first).
fn generator_row(row: usize) -> [u8; K] {
    let mut bits = [0u8; K];
    for (j, c) in GENERATOR[row].chars().enumerate() {
        let v = c.to_digit(16).expect("hex") as u8;
        for b in 0..4 {
            let col = j * 4 + b;
            if col < K {
                bits[col] = (v >> (3 - b)) & 1;
            }
        }
    }
    bits
}

/// Codeword of a 77-bit message: message, CRC-14, then 83 parity bits.
pub fn encode(message: &[u8]) -> [u8; N] {
    let mut word = [0u8; N];
    word[..77].copy_from_slice(&message[..77]);
    let crc = crc14(message);
    for i in 0..14 {
        word[77 + i] = ((crc >> (13 - i)) & 1) as u8;
    }
    for p in 0..M {
        let row = generator_row(p);
        word[K + p] = row.iter().zip(&word[..K]).map(|(a, b)| a & b).fold(0, |s, x| s ^ x);
    }
    word
}

/// For each parity check, the code bits in it (0-based).
fn checks() -> Vec<Vec<usize>> {
    let mut nm = vec![Vec::new(); M];
    for (bit, rows) in MN.iter().enumerate() {
        for &r in rows {
            nm[r as usize - 1].push(bit);
        }
    }
    nm
}

/// Whether a hard-decided codeword satisfies every parity check.
pub fn parity_ok(word: &[u8]) -> bool {
    checks().iter().all(|c| c.iter().fold(0, |s, &b| s ^ word[b]) == 0)
}

/// Belief-propagation decoder.
pub struct Decoder {
    nm: Vec<Vec<usize>>,
}

impl Default for Decoder {
    fn default() -> Self {
        Self { nm: checks() }
    }
}

impl Decoder {
    /// Decodes from log-likelihood ratios `llr[i] = ln(P(bit=1)/P(bit=0))`. Returns the
    /// codeword once every parity check holds, within `max_iterations`.
    pub fn decode(&self, llr: &[f32; N], max_iterations: usize) -> Option<[u8; N]> {
        // Internally L = ln(P0/P1), the usual sign for the tanh rule.
        let prior: Vec<f32> = llr.iter().map(|v| -v).collect();
        // Check-to-bit messages, indexed like `nm`.
        let mut to_bit: Vec<Vec<f32>> = self.nm.iter().map(|c| vec![0.0; c.len()]).collect();
        let mut total = prior.clone();
        let mut word = [0u8; N];
        for _ in 0..max_iterations {
            // Bit totals and the hard decision.
            total.copy_from_slice(&prior);
            for (c, bits) in self.nm.iter().enumerate() {
                for (k, &b) in bits.iter().enumerate() {
                    total[b] += to_bit[c][k];
                }
            }
            for (w, t) in word.iter_mut().zip(&total) {
                *w = (*t < 0.0) as u8;
            }
            if self.nm.iter().all(|c| c.iter().fold(0, |s, &b| s ^ word[b]) == 0) {
                return Some(word);
            }
            // Check updates: tanh rule, excluding each bit's own message.
            for (c, bits) in self.nm.iter().enumerate() {
                let t: Vec<f32> = bits.iter().enumerate().map(|(k, &b)| ((total[b] - to_bit[c][k]) / 2.0).tanh()).collect();
                for k in 0..bits.len() {
                    let prod: f32 = t.iter().enumerate().filter(|(j, _)| *j != k).map(|(_, v)| *v).product();
                    to_bit[c][k] = 2.0 * prod.clamp(-0.999_999, 0.999_999).atanh();
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_message(seed: u32) -> Vec<u8> {
        let mut s = seed;
        (0..77)
            .map(|_| {
                s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (s >> 31) as u8
            })
            .collect()
    }

    #[test]
    fn generator_and_parity_tables_agree() {
        // Every codeword the generator makes must pass every parity check: this shows
        // both tables were taken over correctly.
        for seed in 0..20 {
            let word = encode(&test_message(seed));
            assert!(parity_ok(&word), "seed {seed}");
        }
        // And a single flipped bit must fail some check.
        let mut word = encode(&test_message(1));
        word[100] ^= 1;
        assert!(!parity_ok(&word));
    }

    #[test]
    fn every_bit_is_in_three_checks() {
        let nm = checks();
        assert_eq!(nm.iter().map(|c| c.len()).sum::<usize>(), 174 * 3);
        assert!(nm.iter().all(|c| c.len() == 6 || c.len() == 7));
    }

    #[test]
    fn crc_detects_changes() {
        let m = test_message(5);
        let c = crc14(&m);
        let mut m2 = m.clone();
        m2[40] ^= 1;
        assert_ne!(c, crc14(&m2));
        assert!(c < 1 << 14);
    }

    #[test]
    fn decodes_noisy_codeword() {
        let word = encode(&test_message(9));
        let mut seed = 77u32;
        let mut noise = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        // BPSK-like LLRs at a modest SNR: bit 1 → +, bit 0 → −, with noise.
        let mut llr = [0f32; N];
        for (l, &b) in llr.iter_mut().zip(&word) {
            *l = (if b == 1 { 2.0 } else { -2.0 }) + noise() * 5.0;
        }
        let errors = llr.iter().zip(&word).filter(|(l, b)| (**l > 0.0) != (**b == 1)).count();
        assert!(errors > 5, "test should start with errors ({errors})");
        let decoded = Decoder::default().decode(&llr, 40).expect("decoded");
        assert_eq!(decoded, word);
    }
}
