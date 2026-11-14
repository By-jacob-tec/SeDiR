//! Icom CI-V: frames, frequencies and the few commands SeDiR uses.
//!
//! A frame is `FE FE <to> <from> <command…> FD`. Frequencies are ten BCD digits, least
//! significant byte first. The transmitter is keyed only by `set_ptt`, which SeDiR uses
//! when the user transmits.

/// Address SeDiR uses as the controller (the usual PC address).
pub const CONTROLLER: u8 = 0xE0;
/// Address every transceiver answers to.
pub const BROADCAST: u8 = 0x00;

const PREAMBLE: u8 = 0xFE;
const END: u8 = 0xFD;
/// Sent on the CI-V bus when two frames collided.
const COLLISION: u8 = 0xFC;
/// Reply: command accepted.
pub const OK: u8 = 0xFB;
/// Reply: command refused.
pub const NG: u8 = 0xFA;
/// Longest frame accepted; longer garbage is dropped.
const MAX_FRAME: usize = 1200;

/// Icom models SeDiR knows, identified by their default CI-V address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    Ic705,
    Ic7300,
    Ic7300Mk2,
    /// Another Icom transceiver (its CI-V address).
    Other(u8),
}

impl Model {
    pub fn from_address(address: u8) -> Self {
        match address {
            0xA4 => Model::Ic705,
            0x94 => Model::Ic7300,
            0xB6 => Model::Ic7300Mk2,
            a => Model::Other(a),
        }
    }

    /// From the name the radio gives over the network ("IC-705", "IC-7300MK2"…).
    pub fn from_name(name: &str, address: u8) -> Self {
        let n: String = name.to_ascii_uppercase().chars().filter(|c| c.is_ascii_alphanumeric()).collect();
        if n.contains("7300MK2") || n.contains("7300M2") {
            Model::Ic7300Mk2
        } else if n.contains("7300") {
            Model::Ic7300
        } else if n.contains("705") {
            Model::Ic705
        } else {
            Model::from_address(address)
        }
    }

    pub fn name(self) -> String {
        match self {
            Model::Ic705 => "IC-705".into(),
            Model::Ic7300 => "IC-7300".into(),
            Model::Ic7300Mk2 => "IC-7300MK2".into(),
            Model::Other(a) => format!("Icom (CI-V {a:02X}h)"),
        }
    }

    /// Receive range (Hz).
    pub fn range(self) -> (f64, f64) {
        match self {
            // 30 kHz – 199.999 MHz and 400 – 470 MHz; the gap is refused by the radio.
            Model::Ic705 => (30e3, 470e6),
            Model::Ic7300 | Model::Ic7300Mk2 => (30e3, 74.8e6),
            Model::Other(_) => (30e3, 470e6),
        }
    }
}

/// `FE FE to E0 command… FD`.
pub fn frame(to: u8, command: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(command.len() + 5);
    f.extend([PREAMBLE, PREAMBLE, to, CONTROLLER]);
    f.extend_from_slice(command);
    f.push(END);
    f
}

/// Ten BCD digits, least significant pair first (1 Hz resolution, up to 9.99 GHz).
pub fn frequency_bcd(hz: u64) -> [u8; 5] {
    let mut out = [0u8; 5];
    let mut v = hz.min(9_999_999_999);
    for b in &mut out {
        let lo = (v % 10) as u8;
        v /= 10;
        let hi = (v % 10) as u8;
        v /= 10;
        *b = hi << 4 | lo;
    }
    out
}

/// Reads a BCD frequency of 4 or 5 bytes (least significant first).
pub fn parse_frequency(bytes: &[u8]) -> Option<u64> {
    if !(4..=5).contains(&bytes.len()) {
        return None;
    }
    let mut hz = 0u64;
    for &b in bytes.iter().rev() {
        let (hi, lo) = (b >> 4, b & 0x0F);
        if hi > 9 || lo > 9 {
            return None;
        }
        hz = hz * 100 + u64::from(hi) * 10 + u64::from(lo);
    }
    Some(hz)
}

/// Asks who is there: the transceiver answers `19 00 <its address>`.
pub fn read_id() -> Vec<u8> {
    frame(BROADCAST, &[0x19, 0x00])
}

pub fn read_frequency(to: u8) -> Vec<u8> {
    frame(to, &[0x03])
}

pub fn set_frequency(to: u8, hz: f64) -> Vec<u8> {
    let mut c = vec![0x05];
    c.extend(frequency_bcd(hz.round().max(0.0) as u64));
    frame(to, &c)
}

/// Operating modes as CI-V numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioMode {
    Lsb = 0x00,
    Usb = 0x01,
    Am = 0x02,
    Cw = 0x03,
    Rtty = 0x04,
    Fm = 0x05,
}

/// Mode and filter (1 = FIL1, the widest by default).
pub fn set_mode(to: u8, mode: RadioMode, filter: u8) -> Vec<u8> {
    frame(to, &[0x06, mode as u8, filter.clamp(1, 3)])
}

/// Data mode (USB-D …: transmit audio from USB / LAN) on or off, with filter FIL1.
pub fn set_data_mode(to: u8, on: bool) -> Vec<u8> {
    frame(to, &[0x1A, 0x06, u8::from(on), u8::from(on)])
}

/// Transmit (true) or receive (false).
pub fn set_ptt(to: u8, transmit: bool) -> Vec<u8> {
    frame(to, &[0x1C, 0x00, u8::from(transmit)])
}

/// RF gain 0–255 (sent as four BCD digits, most significant first).
pub fn set_rf_gain(to: u8, level: u8) -> Vec<u8> {
    let v = u16::from(level);
    let (thousands_hundreds, tens_ones) = ((v / 100) as u8, (v % 100) as u8);
    let bcd = |n: u8| (n / 10) << 4 | (n % 10);
    frame(to, &[0x14, 0x02, bcd(thousands_hundreds), bcd(tens_ones)])
}

/// A received frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub to: u8,
    pub from: u8,
    /// Command, sub-command and data.
    pub data: Vec<u8>,
}

impl Frame {
    /// The frequency in a frequency report (`00` broadcast on change, or the answer to `03`).
    pub fn frequency(&self) -> Option<u64> {
        match self.data.first() {
            Some(0x00) | Some(0x03) => parse_frequency(&self.data[1..]),
            _ => None,
        }
    }

    /// The address in an answer to `19 00`.
    pub fn id(&self) -> Option<u8> {
        match self.data.as_slice() {
            [0x19, 0x00, a, ..] => Some(*a),
            _ => None,
        }
    }

    pub fn is_ok(&self) -> bool {
        self.data == [OK]
    }

    pub fn is_ng(&self) -> bool {
        self.data == [NG]
    }
}

/// Splits a byte stream into frames. Echoes of our own frames (sent from the controller
/// address) and collided frames are dropped.
#[derive(Default)]
pub struct Parser {
    buf: Vec<u8>,
}

impl Parser {
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        self.buf.extend_from_slice(bytes);
        let mut frames = Vec::new();
        loop {
            // Drop everything before the first preamble pair.
            let Some(start) = self.buf.windows(2).position(|w| w == [PREAMBLE, PREAMBLE]) else {
                let keep = usize::from(self.buf.last() == Some(&PREAMBLE));
                let cut = self.buf.len() - keep;
                self.buf.drain(..cut);
                break;
            };
            self.buf.drain(..start);
            // Extra preamble bytes are allowed.
            let body = self.buf.iter().skip(2).position(|&b| b != PREAMBLE).map(|p| p + 2);
            let Some(body) = body else { break };
            let Some(end) = self.buf[body..].iter().position(|&b| b == END || b == COLLISION).map(|p| p + body) else {
                if self.buf.len() > MAX_FRAME {
                    self.buf.clear();
                }
                break;
            };
            let collided = self.buf[end] == COLLISION;
            let content: Vec<u8> = self.buf[body..end].to_vec();
            self.buf.drain(..=end);
            if collided || content.len() < 3 {
                continue;
            }
            let frame = Frame { to: content[0], from: content[1], data: content[2..].to_vec() };
            if frame.from != CONTROLLER {
                frames.push(frame);
            }
        }
        frames
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frequencies_are_bcd_least_significant_first() {
        // 14.074 MHz = 0014074000 → 00 40 07 14 00
        assert_eq!(frequency_bcd(14_074_000), [0x00, 0x40, 0x07, 0x14, 0x00]);
        assert_eq!(parse_frequency(&[0x00, 0x40, 0x07, 0x14, 0x00]), Some(14_074_000));
        assert_eq!(parse_frequency(&[0x56, 0x34, 0x12, 0x45, 0x01]), Some(145_123_456));
        assert_eq!(parse_frequency(&[0x5A, 0, 0, 0, 0]), None);
        assert_eq!(set_frequency(0x94, 7_074_000.0), [0xFE, 0xFE, 0x94, 0xE0, 0x05, 0x00, 0x40, 0x07, 0x07, 0x00, 0xFD]);
    }

    #[test]
    fn transmit_commands() {
        assert_eq!(set_ptt(0xB6, true), [0xFE, 0xFE, 0xB6, 0xE0, 0x1C, 0x00, 0x01, 0xFD]);
        assert_eq!(set_ptt(0xB6, false), [0xFE, 0xFE, 0xB6, 0xE0, 0x1C, 0x00, 0x00, 0xFD]);
        assert_eq!(set_data_mode(0x94, true), [0xFE, 0xFE, 0x94, 0xE0, 0x1A, 0x06, 0x01, 0x01, 0xFD]);
        assert_eq!(set_mode(0x94, RadioMode::Cw, 1), [0xFE, 0xFE, 0x94, 0xE0, 0x06, 0x03, 0x01, 0xFD]);
    }

    #[test]
    fn rf_gain_is_four_bcd_digits() {
        assert_eq!(set_rf_gain(0xA4, 128), [0xFE, 0xFE, 0xA4, 0xE0, 0x14, 0x02, 0x01, 0x28, 0xFD]);
        assert_eq!(set_rf_gain(0xA4, 255), [0xFE, 0xFE, 0xA4, 0xE0, 0x14, 0x02, 0x02, 0x55, 0xFD]);
    }

    #[test]
    fn parser_splits_frames_and_drops_echoes_and_noise() {
        let mut p = Parser::default();
        let mut stream = vec![0x12, 0xFE];
        stream.extend(set_frequency(0x94, 7e6)); // our echo
        stream.extend([0xFE, 0xFE, 0xE0, 0x94, 0x19, 0x00, 0x94, 0xFD]);
        stream.extend([0xFE, 0xFE, 0x00, 0x94, 0x00, 0x00, 0x40, 0x07, 0x14]); // split here
        let frames = p.push(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].id(), Some(0x94));
        let frames = p.push(&[0x00, 0xFD, 0xFE, 0xFE, 0xE0, 0x94, 0xFB, 0xFD]);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].frequency(), Some(14_074_000));
        assert!(frames[1].is_ok());
        // A collision drops the frame.
        assert!(p.push(&[0xFE, 0xFE, 0xE0, 0x94, 0xFB, 0xFC]).is_empty());
        assert!(p.push(&[0xFE, 0xFE, 0xE0, 0x94, 0xFA, 0xFD])[0].is_ng());
    }

    #[test]
    fn models_from_address_and_name() {
        assert_eq!(Model::from_address(0xA4), Model::Ic705);
        assert_eq!(Model::from_name("IC-7300MK2", 0x00), Model::Ic7300Mk2);
        assert_eq!(Model::from_name("IC-705", 0x00), Model::Ic705);
        assert_eq!(Model::from_name("", 0x94), Model::Ic7300);
        assert_eq!(Model::Ic7300.range().1, 74.8e6);
    }
}
