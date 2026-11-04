//! RDS (Radio Data System, IEC 62106) decoder: from the FM multiplex signal to the
//! station name, radio text, programme type and clock.
//!
//! 57 kHz subcarrier → baseband → Costas loop (BPSK) → chip timing (Gardner, 2375 Hz) →
//! biphase pairing → differential decoding → 26-bit blocks with offset words → groups.

use std::f64::consts::PI;

use num_complex::Complex;

use crate::demod::{FirDecimator, Mixer, halfband, lowpass, taps_for};

const SUBCARRIER: f64 = 57_000.0;
/// Biphase chips per second (two per data bit at 1187.5 bit/s).
const CHIP_RATE: f64 = 2_375.0;
/// Check word generator g(x) = x¹⁰ + x⁸ + x⁷ + x⁵ + x⁴ + x³ + 1.
const POLY: u32 = 0x5B9;
/// Offset words added to the check word of blocks A, B, C, D and C'.
const OFFSETS: [u16; 5] = [0x0FC, 0x198, 0x168, 0x1B4, 0x350];
const C_PRIME: usize = 4;

/// Check word (without offset) of a block's 16 data bits.
pub fn checkword(data: u16) -> u16 {
    let mut r = (data as u32) << 10;
    for bit in (10..26).rev() {
        if r & (1 << bit) != 0 {
            r ^= POLY << (bit - 10);
        }
    }
    (r & 0x3FF) as u16
}

/// Which offset word, if any, makes a 26-bit block valid.
fn block_kind(block: u32) -> Option<usize> {
    let syndrome = ((block & 0x3FF) as u16) ^ checkword((block >> 10) as u16);
    OFFSETS.iter().position(|&o| o == syndrome)
}

/// Clock time and date sent by the station (group 4A), in UTC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockTime {
    pub year: u32,
    pub month: u32,
    pub day: u32,
    pub hour: u32,
    pub minute: u32,
    /// Local time offset in half hours.
    pub offset_half_hours: i32,
}

impl ClockTime {
    /// Local time as "HH:MM".
    pub fn local_hm(&self) -> String {
        let minutes = (self.hour * 60 + self.minute) as i32 + self.offset_half_hours * 30;
        let m = minutes.rem_euclid(24 * 60);
        format!("{:02}:{:02}", m / 60, m % 60)
    }
}

/// Everything decoded from one station.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RdsInfo {
    pub pi: Option<u16>,
    /// Programme service name as received so far (8 characters, missing ones blank).
    pub ps: String,
    /// The name once all four parts arrived the same way twice in a row.
    pub stable_ps: Option<String>,
    pub radio_text: String,
    pub pty: Option<u8>,
    pub tp: bool,
    pub ta: bool,
    pub music: bool,
    pub clock: Option<ClockTime>,
    /// Alternative frequencies (Hz).
    pub af: Vec<f64>,
    pub groups: u32,
    pub synced: bool,
}

impl Default for Groups {
    fn default() -> Self {
        Self { info: RdsInfo::default(), ps: [b' '; 8], ps_seen: 0, last_complete_ps: None, rt: [0; 64], rt_seen: 0, rt_ab: None }
    }
}

/// Group assembly and the text buffers behind `RdsInfo`.
struct Groups {
    info: RdsInfo,
    ps: [u8; 8],
    ps_seen: u8,
    last_complete_ps: Option<[u8; 8]>,
    rt: [u8; 64],
    rt_seen: u64,
    rt_ab: Option<bool>,
}

impl Groups {
    fn handle(&mut self, a: u16, b: u16, c: Option<u16>, d: Option<u16>) {
        let info = &mut self.info;
        if info.pi != Some(a) {
            // Another station: forget what belonged to the previous one.
            *self = Groups::default();
            self.info.synced = true;
        }
        let info = &mut self.info;
        info.pi = Some(a);
        info.groups += 1;
        let group = b >> 12;
        let version_b = b & 0x800 != 0;
        info.tp = b & 0x400 != 0;
        info.pty = Some(((b >> 5) & 0x1F) as u8);
        match (group, version_b) {
            (0, _) => {
                info.ta = b & 0x10 != 0;
                info.music = b & 0x8 != 0;
                if let (false, Some(c)) = (version_b, c) {
                    for code in [(c >> 8) as u8, c as u8] {
                        if (1..=204).contains(&code) {
                            let hz = 87.5e6 + code as f64 * 100e3;
                            if !info.af.contains(&hz) && info.af.len() < 25 {
                                info.af.push(hz);
                            }
                        }
                    }
                }
                if let Some(d) = d {
                    let seg = (b & 3) as usize;
                    self.ps[seg * 2] = (d >> 8) as u8;
                    self.ps[seg * 2 + 1] = d as u8;
                    self.ps_seen |= 1 << seg;
                    if self.ps_seen == 0xF {
                        self.ps_seen = 0;
                        if self.last_complete_ps == Some(self.ps) {
                            self.info.stable_ps = Some(decode_text(&self.ps).trim().to_string());
                        }
                        self.last_complete_ps = Some(self.ps);
                    }
                    self.info.ps = decode_text(&self.ps);
                }
            }
            (2, _) => {
                let ab = b & 0x10 != 0;
                if self.rt_ab.is_some_and(|x| x != ab) {
                    self.rt = [0; 64];
                    self.rt_seen = 0;
                }
                self.rt_ab = Some(ab);
                let seg = (b & 0xF) as usize;
                let chars: Vec<u8> = match (version_b, c, d) {
                    (false, Some(c), Some(d)) => vec![(c >> 8) as u8, c as u8, (d >> 8) as u8, d as u8],
                    (true, _, Some(d)) => vec![(d >> 8) as u8, d as u8],
                    _ => Vec::new(),
                };
                let at = seg * chars.len();
                for (i, ch) in chars.iter().enumerate() {
                    if at + i < 64 {
                        self.rt[at + i] = *ch;
                    }
                }
                if !chars.is_empty() {
                    self.rt_seen |= 1 << seg;
                }
                let end = self.rt.iter().position(|&c| c == 0x0D).unwrap_or(64);
                let text: Vec<u8> = self.rt[..end].iter().map(|&c| if c == 0 { b' ' } else { c }).collect();
                self.info.radio_text = decode_text(&text).trim_end().to_string();
            }
            (4, false) => {
                if let (Some(c), Some(d)) = (c, d) {
                    let mjd = (((b & 3) as u32) << 15) | ((c >> 1) as u32);
                    let hour = (((c & 1) as u32) << 4) | ((d >> 12) as u32);
                    let minute = ((d >> 6) & 0x3F) as u32;
                    let offset = (d & 0x1F) as i32 * if d & 0x20 != 0 { -1 } else { 1 };
                    if let Some((year, month, day)) = mjd_to_date(mjd) {
                        if hour < 24 && minute < 60 {
                            self.info.clock = Some(ClockTime { year, month, day, hour, minute, offset_half_hours: offset });
                        }
                    }
                }
            }
            _ => {}
        }
    }
}

/// Modified Julian Day to (year, month, day), per IEC 62106 Annex G.
fn mjd_to_date(mjd: u32) -> Option<(u32, u32, u32)> {
    if mjd < 15079 {
        return None;
    }
    let mjd = mjd as f64;
    let y1 = ((mjd - 15078.2) / 365.25).floor();
    let m1 = ((mjd - 14956.1 - (y1 * 365.25).floor()) / 30.6001).floor();
    let day = mjd - 14956.0 - (y1 * 365.25).floor() - (m1 * 30.6001).floor();
    let k = if m1 == 14.0 || m1 == 15.0 { 1.0 } else { 0.0 };
    Some(((y1 + k + 1900.0) as u32, (m1 - 1.0 - k * 12.0) as u32, day as u32))
}

/// RDS characters (EBU Latin table, IEC 62106 Annex E) to text. 0x20–0x7D are as in
/// ASCII; the upper half is mapped for the letters used in Turkish and other European
/// languages. Turkish stations send ğ/Ğ with the code of ǧ/Ǧ, so those show as ğ/Ğ.
pub fn decode_text(bytes: &[u8]) -> String {
    const HIGH: [&str; 6] = [
        "áàéèíìóòúùÑÇŞß¡Ĳ",
        "âäêëîïôöûüñçşğıĳ",
        "ª©‰Ğěňőπ€£$←↑→↓",
        "º¹²³±İńűµ¿÷°¼½¾§",
        "ÁÀÉÈÍÌÓÒÚÙŘČŠŽÐĿ",
        "ÂÄÊËÎÏÔÖÛÜřčšžđŀ",
    ];
    bytes
        .iter()
        .map(|&b| match b {
            0x20..=0x7D => b as char,
            0x80..=0xDF => {
                let row: Vec<char> = HIGH[(b as usize - 0x80) >> 4].chars().collect();
                row.get(b as usize & 0xF).copied().unwrap_or(' ')
            }
            _ => ' ',
        })
        .collect()
}

/// Decodes RDS from a multiplex signal.
pub struct RdsDecoder {
    mixer: Mixer,
    halfbands: Vec<FirDecimator<Complex<f32>>>,
    filter: FirDecimator<Complex<f32>>,
    /// Samples per chip after decimation.
    sps: f64,
    // Costas loop.
    phase: f64,
    freq: f64,
    alpha: f64,
    beta: f64,
    power: f64,
    // Chip timing: time of the next mid or strobe sample, in input samples after the
    // previous one (0 = previous sample, 1 = current).
    next: f64,
    is_strobe: bool,
    prev_x: f32,
    mid: f32,
    last_chip: f32,
    prev_chip: f32,
    amplitude: f32,
    // Biphase pairing: how well chips pair up when the pair ends on an even or odd chip.
    chip_count: u64,
    pairing: [f32; 2],
    last_bit: u8,
    // Blocks.
    reg: u32,
    bits: u64,
    candidate: Option<(u64, usize)>,
    synced: bool,
    expected: usize,
    in_block: u32,
    bad_blocks: u32,
    group: [Option<u16>; 4],
    groups: Groups,
    scratch: Vec<Complex<f32>>,
    scratch2: Vec<Complex<f32>>,
}

impl RdsDecoder {
    /// `rate` is the multiplex sample rate.
    pub fn new(rate: f64) -> Self {
        let mut r = rate;
        let mut halfbands = Vec::new();
        while r / 2.0 >= 16e3 {
            halfbands.push(halfband());
            r /= 2.0;
        }
        let filter = FirDecimator::new(&lowpass(2.4e3 / r, taps_for(r, 1.2e3, 255)), 1);
        // Second-order loop, 20 Hz noise bandwidth.
        let (bn, zeta) = (20.0 / r, 0.707);
        let theta = bn / (zeta + 0.25 / zeta);
        let d = 1.0 + 2.0 * zeta * theta + theta * theta;
        Self {
            mixer: Mixer::new(-SUBCARRIER, rate),
            halfbands,
            filter,
            sps: r / CHIP_RATE,
            phase: 0.0,
            freq: 0.0,
            alpha: 4.0 * zeta * theta / d,
            beta: 4.0 * theta * theta / d,
            power: 1e-6,
            next: 0.0,
            is_strobe: true,
            prev_x: 0.0,
            mid: 0.0,
            last_chip: 0.0,
            prev_chip: 0.0,
            amplitude: 1e-3,
            chip_count: 0,
            pairing: [0.0; 2],
            last_bit: 0,
            reg: 0,
            bits: 0,
            candidate: None,
            synced: false,
            expected: 0,
            in_block: 0,
            bad_blocks: 0,
            group: [None; 4],
            groups: Groups::default(),
            scratch: Vec::new(),
            scratch2: Vec::new(),
        }
    }

    pub fn info(&self) -> &RdsInfo {
        &self.groups.info
    }

    pub fn process(&mut self, mpx: &[f32]) {
        let (mut a, mut b) = (std::mem::take(&mut self.scratch), std::mem::take(&mut self.scratch2));
        a.clear();
        a.extend(mpx.iter().map(|&x| Complex::new(x, 0.0)));
        self.mixer.mix(&mut a);
        for hb in &mut self.halfbands {
            b.clear();
            hb.process(&a, &mut b);
            std::mem::swap(&mut a, &mut b);
        }
        b.clear();
        self.filter.process(&a, &mut b);
        for &z in &b {
            let x = self.costas(z);
            self.timing(x);
        }
        (self.scratch, self.scratch2) = (a, b);
    }

    /// Removes the carrier phase; returns the in-phase part.
    fn costas(&mut self, z: Complex<f32>) -> f32 {
        let y = Complex::new(z.re as f64, z.im as f64) * Complex::from_polar(1.0, -self.phase);
        self.power += (y.norm_sqr() - self.power) * 1e-3;
        let error = (y.re * y.im / self.power.max(1e-12)).clamp(-1.0, 1.0);
        self.freq += self.beta * error;
        self.phase = (self.phase + self.freq + self.alpha * error) % (2.0 * PI);
        y.re as f32
    }

    /// Gardner timing recovery at the chip rate (linear interpolation).
    fn timing(&mut self, x: f32) {
        while self.next <= 1.0 {
            let v = self.prev_x + (x - self.prev_x) * self.next as f32;
            if self.is_strobe {
                // Negative when sampling late (the midpoint is past the zero crossing):
                // the next interval is shortened by it.
                let error = (self.last_chip - v) * self.mid / (self.amplitude * self.amplitude).max(1e-12);
                self.amplitude += (v.abs() - self.amplitude) * 0.01;
                self.last_chip = v;
                self.next += self.sps / 2.0 * (1.0 + 0.05 * error.clamp(-1.0, 1.0) as f64);
                self.chip(v);
            } else {
                self.mid = v;
                self.next += self.sps / 2.0;
            }
            self.is_strobe = !self.is_strobe;
        }
        self.next -= 1.0;
        self.prev_x = x;
    }

    /// Pairs chips into biphase bits, following whichever pairing gives clear bits.
    fn chip(&mut self, c: f32) {
        let parity = (self.chip_count & 1) as usize;
        self.chip_count += 1;
        let soft = self.prev_chip - c;
        self.pairing[parity] += (soft.abs() - self.pairing[parity]) * 0.02;
        let chosen = if self.pairing[0] >= self.pairing[1] { 0 } else { 1 };
        if parity == chosen {
            let raw = (soft > 0.0) as u8;
            let bit = raw ^ self.last_bit;
            self.last_bit = raw;
            self.bit(bit);
        }
        self.prev_chip = c;
    }

    fn bit(&mut self, bit: u8) {
        self.reg = ((self.reg << 1) | bit as u32) & 0x3FF_FFFF;
        self.bits += 1;
        if !self.synced {
            if self.bits < 26 {
                return;
            }
            let Some(kind) = block_kind(self.reg) else { return };
            let kind = if kind == C_PRIME { 2 } else { kind };
            if let Some((pos, prev)) = self.candidate {
                let distance = self.bits - pos;
                let blocks = (kind + 4 - prev) % 4;
                if blocks != 0 && distance == 26 * blocks as u64 {
                    self.synced = true;
                    self.groups.info.synced = true;
                    self.expected = (kind + 1) % 4;
                    self.in_block = 0;
                    self.bad_blocks = 0;
                    self.group = [None; 4];
                    if kind == 0 {
                        self.group[0] = Some((self.reg >> 10) as u16);
                    }
                    return;
                }
            }
            self.candidate = Some((self.bits, kind));
            return;
        }
        self.in_block += 1;
        if self.in_block < 26 {
            return;
        }
        self.in_block = 0;
        let kind = block_kind(self.reg);
        let ok = match self.expected {
            2 => matches!(kind, Some(2) | Some(C_PRIME)),
            e => kind == Some(e),
        };
        let data = (self.reg >> 10) as u16;
        self.group[self.expected] = ok.then_some(data);
        self.bad_blocks = if ok { self.bad_blocks.saturating_sub(1) } else { self.bad_blocks + 2 };
        if self.expected == 3 {
            if let [Some(a), Some(b), c, d] = self.group {
                self.groups.handle(a, b, c, d);
            }
            self.group = [None; 4];
        }
        self.expected = (self.expected + 1) % 4;
        // Mostly bad blocks: the signal is gone or sync slipped; search again.
        if self.bad_blocks > 40 {
            self.synced = false;
            self.groups.info.synced = false;
            self.candidate = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::demod::MpxChannel;

    #[test]
    fn check_words_and_offsets() {
        // A block is valid for exactly the offset it was built with.
        for (k, &offset) in OFFSETS.iter().enumerate() {
            let data = 0x7A12u16;
            let block = ((data as u32) << 10) | (checkword(data) ^ offset) as u32;
            assert_eq!(block_kind(block), Some(k));
            assert_eq!(block_kind(block ^ 0x100), None, "one wrong bit must be caught");
        }
    }

    #[test]
    fn dates_and_text() {
        // MJD 45218 = 1982-09-06 (the standard's example).
        assert_eq!(mjd_to_date(45218), Some((1982, 9, 6)));
        assert_eq!(decode_text(b"KRAL FM "), "KRAL FM ");
        assert_eq!(decode_text(&[0x9C, 0x9E, 0x8B, 0xB5, 0x9D]), "şıÇİğ");
    }

    /// RDS groups for a station: PS in four 0A groups, radio text in 2A groups.
    fn groups(pi: u16, ps: &str, rt: &str) -> Vec<[u16; 4]> {
        let ps: Vec<u8> = ps.bytes().collect();
        let mut out = Vec::new();
        for seg in 0..4u16 {
            let b = (0 << 12) | (10 << 5) | seg;
            out.push([pi, b, 0xE0CD, ((ps[seg as usize * 2] as u16) << 8) | ps[seg as usize * 2 + 1] as u16]);
        }
        let mut rt: Vec<u8> = rt.bytes().collect();
        rt.push(0x0D);
        while rt.len() % 4 != 0 {
            rt.push(b' ');
        }
        for (seg, chunk) in rt.chunks(4).enumerate() {
            let b = (2 << 12) | (10 << 5) | seg as u16;
            out.push([pi, b, ((chunk[0] as u16) << 8) | chunk[1] as u16, ((chunk[2] as u16) << 8) | chunk[3] as u16]);
        }
        out
    }

    /// Bits of the groups with check words, differentially encoded.
    fn encode(groups: &[[u16; 4]], repeat: usize) -> Vec<u8> {
        let mut bits = Vec::new();
        for _ in 0..repeat {
            for g in groups {
                for (k, &data) in g.iter().enumerate() {
                    let block = ((data as u32) << 10) | (checkword(data) ^ OFFSETS[k]) as u32;
                    bits.extend((0..26).rev().map(|i| ((block >> i) & 1) as u8));
                }
            }
        }
        let mut prev = 0;
        bits.iter()
            .map(|&d| {
                prev ^= d;
                prev
            })
            .collect()
    }

    /// FM broadcast at `rate`: a 1 kHz tone, the 19 kHz pilot and RDS, at `offset_hz`.
    fn fm_station(bits: &[u8], rate: f64, offset_hz: f64, noise: f32) -> Vec<Complex<f32>> {
        let chip_samples = rate / CHIP_RATE;
        let n = (bits.len() as f64 * 2.0 * chip_samples) as usize;
        let mut phase = 0.0f64;
        let mut seed = 12345u32;
        (0..n)
            .map(|i| {
                // Biphase chips as half-sine pulses (band-limited enough for the test).
                let at = i as f64 / chip_samples;
                let chip = at as usize;
                let b = bits[(chip / 2).min(bits.len() - 1)];
                let sign = if (b == 1) == (chip % 2 == 0) { 1.0 } else { -1.0 };
                let r = sign * (PI * at.fract()).sin();
                let t = i as f64 / rate;
                let mpx = 0.5 * (2.0 * PI * 1e3 * t).sin()
                    + 0.09 * (2.0 * PI * 19e3 * t).sin()
                    + 0.04 * r * (2.0 * PI * SUBCARRIER * t).sin();
                phase += 2.0 * PI * (offset_hz + 75e3 * mpx) / rate;
                let mut noise_sample = || {
                    seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
                };
                Complex::from_polar(0.5, phase as f32) + Complex::new(noise_sample(), noise_sample()) * noise
            })
            .collect()
    }

    #[test]
    fn decodes_station_name_and_text() {
        let (rate, offset) = (2.4e6, 200e3);
        let g = groups(0x7A12, "SEDIR FM", "Mugladan merhaba");
        let bits = encode(&g, 3);
        let iq = fm_station(&bits, rate, offset, 0.3);
        let mut channel = MpxChannel::new(rate);
        let mut rds = RdsDecoder::new(channel.rate);
        let mut mpx = Vec::new();
        for block in iq.chunks(48_000) {
            mpx.clear();
            channel.process(block, offset, &mut mpx);
            rds.process(&mpx);
        }
        let info = rds.info();
        assert_eq!(info.pi, Some(0x7A12), "{info:?}");
        assert_eq!(info.stable_ps.as_deref(), Some("SEDIR FM"));
        assert_eq!(info.pty, Some(10));
        assert!(info.radio_text.starts_with("Mu"), "{:?}", info.radio_text);
        assert!(info.groups > 20, "{} groups", info.groups);
    }
}
