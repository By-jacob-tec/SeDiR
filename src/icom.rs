//! Icom transceivers (IC-705, IC-7300, IC-7300MK2…) as receivers: tuned over CI-V, with
//! samples made from the 12 kHz IF the radio sends instead of audio when its
//! "AF/IF Output Select" is set to IF.
//!
//! Over USB the radio is two devices: a serial port for CI-V and a sound card for the IF.
//! Over the network (IC-705 WLAN, IC-7300MK2 LAN) both come through `icomnet`.
//!
//! The IF is filtered by the radio's own IF filter, so the radio is put in AM with its
//! widest filter (FIL1): that shows about ±5 kHz around the radio's frequency, and SeDiR
//! demodulates inside it. SeDiR never transmits; it only sends frequency, mode and RF gain.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

use num_complex::Complex;
use serialport::{SerialPort, SerialPortType};

use crate::audio::{self, AudioInput};
use crate::civ::{self, Model, RadioMode};
use crate::icomnet::{self, Credentials, IcomNet};
use crate::ifiq::IfToIq;

/// Device arguments of a USB radio: `icom-usb:<serial port>|<sound input id>`.
pub const USB_PREFIX: &str = "icom-usb:";
/// Device arguments of a network radio: `icom:<user>:<password>@<address>[:port]`.
pub const NET_PREFIX: &str = "icom:";
/// Complex samples per second produced (half the 48 kHz IF rate).
pub const SAMPLE_RATE: f64 = 24_000.0;
/// The IF is recorded at this rate (both over USB and the network).
const IF_RATE: u32 = icomnet::SAMPLE_RATE;
/// CI-V over USB; the radios' "CI-V USB Baud Rate" defaults to Auto.
const BAUD: u32 = 115_200;
const ANSWER_TIMEOUT: Duration = Duration::from_millis(400);
/// No IF for this long is an error.
const SILENCE_TIMEOUT: Duration = Duration::from_secs(5);
/// Frequency reports this soon after SeDiR tuned are taken as the radio repeating it.
const OWN_CHANGE: Duration = Duration::from_millis(500);
/// The radio's frequency is asked this often, so its dial is followed even when its
/// "CI-V Transceive" is off.
const POLL_INTERVAL: Duration = Duration::from_millis(300);
/// Recording looked at to tell the IF from audio (2 s at 48 kHz).
const CHECK_SAMPLES: usize = 96_000;

/// Tells the 12 kHz IF (energy above ~7 kHz) from demodulated audio (energy below ~4 kHz)
/// or silence, in the first seconds of recording.
#[derive(Default)]
struct IfCheck {
    samples: usize,
    lp1: f32,
    lp2: f32,
    low: f64,
    total: f64,
    done: bool,
}

/// What the start of the recording looked like.
#[derive(Debug, PartialEq)]
enum IfVerdict {
    If,
    Audio,
    Silence,
}

impl IfCheck {
    /// None until enough has been recorded, then the verdict once.
    fn push(&mut self, x: &[f32]) -> Option<IfVerdict> {
        if self.done {
            return None;
        }
        // Two one-pole low-passes at 4 kHz (48 kHz rate).
        const A: f32 = 0.408;
        for &v in x {
            self.lp1 += A * (v - self.lp1);
            self.lp2 += A * (self.lp1 - self.lp2);
            self.low += f64::from(self.lp2 * self.lp2);
            self.total += f64::from(v * v);
        }
        self.samples += x.len();
        if self.samples < CHECK_SAMPLES {
            return None;
        }
        self.done = true;
        let rms = (self.total / self.samples as f64).sqrt();
        Some(if rms < 3e-5 {
            IfVerdict::Silence
        } else if self.low > 0.4 * self.total {
            IfVerdict::Audio
        } else {
            IfVerdict::If
        })
    }
}
/// USB vendor ids: Icom's own USB (IC-705, IC-7300MK2) and the Silicon Labs bridge of
/// the IC-7300 (whose serial number starts with the model name).
const ICOM_VID: u16 = 0x0C26;
const SILABS_VID: u16 = 0x10C4;

pub fn is_icom(args: &str) -> bool {
    args.starts_with(USB_PREFIX) || args.starts_with(NET_PREFIX)
}

pub fn usb_args(port: &str, audio_id: &str) -> String {
    format!("{USB_PREFIX}{port}|{audio_id}")
}

/// Serial port and sound input id of USB arguments (the id may be empty).
fn parse_usb(args: &str) -> Option<(&str, &str)> {
    let rest = args.strip_prefix(USB_PREFIX)?;
    Some(rest.split_once('|').unwrap_or((rest, "")))
}

/// Network arguments without the password, for showing.
pub fn net_label(args: &str) -> String {
    let rest = args.strip_prefix(NET_PREFIX).unwrap_or(args);
    match Credentials::parse(rest) {
        Some(c) => format!("Icom · {}@{}", c.username, c.host),
        None => "Icom".into(),
    }
}

/// An Icom radio found on USB.
#[derive(Clone, Debug)]
pub struct Found {
    pub label: String,
    pub args: String,
    pub model: Model,
}

/// Whether a recording device looks like an Icom's USB sound card.
fn is_icom_sound(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    ["usb audio codec", "icom", "ic-7", "ic-9", "ic-705", "ic-r", "7300"].iter().any(|k| n.contains(k))
}

/// Whether the arguments are a USB radio (whose sound input can be chosen).
pub fn is_usb(args: &str) -> bool {
    args.starts_with(USB_PREFIX)
}

/// USB arguments with the sound input replaced (empty: found by its name).
pub fn with_sound(args: &str, sound_id: &str) -> String {
    match parse_usb(args) {
        Some((port, _)) => usb_args(port, sound_id),
        None => args.to_string(),
    }
}

/// Serial ports that may be an Icom radio, with the USB serial number (the IC-7300MK2
/// has two ports with the same one).
fn candidate_ports() -> Vec<(String, String)> {
    let Ok(ports) = serialport::available_ports() else { return Vec::new() };
    ports
        .into_iter()
        .filter_map(|p| match &p.port_type {
            SerialPortType::UsbPort(u) => {
                let text = [&u.serial_number, &u.product, &u.manufacturer]
                    .iter()
                    .filter_map(|s| s.as_deref())
                    .collect::<Vec<_>>()
                    .join(" ")
                    .to_ascii_uppercase();
                let icom = u.vid == ICOM_VID || text.contains("ICOM") || (u.vid == SILABS_VID && text.contains("IC-"));
                icom.then(|| (p.port_name.clone(), u.serial_number.clone().unwrap_or_default()))
            }
            _ => None,
        })
        .collect()
}

/// Finds Icom radios on USB serial ports, each paired with an Icom-like sound input
/// (in order, when there are several). Ports in use by another program are skipped.
pub fn discover_usb() -> Vec<Found> {
    let ports = candidate_ports();
    if ports.is_empty() {
        return Vec::new();
    }
    let mut sounds = audio::input_devices().into_iter().filter(|d| is_icom_sound(&d.name));
    let mut found = Vec::new();
    let mut seen: Vec<(String, u8)> = Vec::new();
    for (port, serial_number) in ports {
        let Ok(mut serial) = open_serial(&port) else { continue };
        let Some(address) = identify(serial.as_mut()) else { continue };
        drop(serial);
        // A second CI-V port of the same radio.
        if !serial_number.is_empty() && seen.contains(&(serial_number.clone(), address)) {
            continue;
        }
        seen.push((serial_number, address));
        let model = Model::from_address(address);
        let sound = sounds.next().map(|d| d.id).unwrap_or_default();
        found.push(Found { label: format!("{} · {port}", model.name()), args: usb_args(&port, &sound), model });
    }
    found
}

fn open_serial(port: &str) -> Result<Box<dyn SerialPort>, String> {
    let mut serial = serialport::new(port, BAUD).timeout(Duration::from_millis(20)).open().map_err(|e| format!("{port}: {e}"))?;
    // DTR and RTS can be set up on the radio to key the transmitter or send CW: keep them off.
    let _ = serial.write_data_terminal_ready(false);
    let _ = serial.write_request_to_send(false);
    Ok(serial)
}

/// Asks who is on the port; the radio's CI-V address.
fn identify(serial: &mut dyn SerialPort) -> Option<u8> {
    let _ = serial.clear(serialport::ClearBuffer::Input);
    serial.write_all(&civ::read_id()).ok()?;
    let mut parser = civ::Parser::default();
    let until = Instant::now() + ANSWER_TIMEOUT;
    let mut buf = [0u8; 256];
    while Instant::now() < until {
        match serial.read(&mut buf) {
            Ok(n) if n > 0 => {
                if let Some(a) = parser.push(&buf[..n]).iter().filter(|f| f.to == civ::CONTROLLER).find_map(|f| f.id()) {
                    return Some(a);
                }
            }
            _ => {}
        }
    }
    None
}

/// Asks a network radio for its CI-V address, when its login details did not say.
fn ask_address(net: &IcomNet) -> Option<u8> {
    net.send_civ(&civ::read_id());
    let mut parser = civ::Parser::default();
    let until = Instant::now() + Duration::from_secs(1);
    while let Some(left) = until.checked_duration_since(Instant::now()) {
        let bytes = net.civ_rx.recv_timeout(left).ok()?;
        if let Some(a) = parser.push(&bytes).iter().filter(|f| f.to == civ::CONTROLLER).find_map(|f| f.id()) {
            return Some(a);
        }
    }
    None
}

enum Link {
    Usb { serial: Box<dyn SerialPort>, sound: AudioInput },
    Net(IcomNet),
}

pub struct IcomRadio {
    link: Link,
    address: u8,
    model: Model,
    converter: IfToIq,
    ready: VecDeque<Complex<f32>>,
    raw: Vec<f32>,
    converted: Vec<Complex<f32>>,
    parser: civ::Parser,
    /// Last frequency set or reported (Hz).
    frequency: f64,
    /// A frequency change made on the radio itself, not yet passed on.
    tuned: Option<f64>,
    /// When SeDiR last tuned the radio.
    tuned_by_us: Instant,
    last_if: Instant,
    poll_at: Instant,
    check: IfCheck,
}

impl IcomRadio {
    pub fn open(args: &str) -> Result<Self, String> {
        let (link, address, model) = if let Some((port, sound_id)) = parse_usb(args) {
            let mut serial = open_serial(port)?;
            let address = identify(serial.as_mut())
                .ok_or_else(|| format!("{port}: no CI-V answer (radio on? SET > Connectors > CI-V: CI-V USB Port, Baud Rate Auto)"))?;
            let sound_id = if sound_id.is_empty() {
                let inputs = audio::input_devices();
                match inputs.iter().find(|d| is_icom_sound(&d.name)) {
                    Some(d) => d.id.clone(),
                    None => {
                        let names: Vec<&str> = inputs.iter().map(|d| d.name.as_str()).collect();
                        return Err(format!(
                            "the radio's USB sound card was not found; choose it under \"Radio sound input\" (inputs: {})",
                            names.join(", ")
                        ));
                    }
                }
            } else {
                sound_id.to_string()
            };
            let sound = audio::open_input(&sound_id, IF_RATE)?;
            (Link::Usb { serial, sound }, address, Model::from_address(address))
        } else {
            let rest = args.strip_prefix(NET_PREFIX).ok_or("not an Icom address")?;
            let creds = Credentials::parse(rest).ok_or("Icom network address: icom:user:password@address")?;
            let net = IcomNet::connect(&creds)?;
            let address = match net.info.civ_address {
                0 => ask_address(&net).ok_or("the radio did not tell its CI-V address")?,
                a => a,
            };
            let model = Model::from_name(&net.info.name, address);
            (Link::Net(net), address, model)
        };
        let mut radio = Self {
            link,
            address,
            model,
            converter: IfToIq::new(f64::from(IF_RATE), false),
            ready: VecDeque::new(),
            raw: Vec::new(),
            converted: Vec::new(),
            parser: civ::Parser::default(),
            frequency: f64::NAN,
            tuned: None,
            tuned_by_us: Instant::now(),
            last_if: Instant::now(),
            poll_at: Instant::now(),
            check: IfCheck::default(),
        };
        // The widest IF filter there is: AM, FIL1.
        radio.send(&civ::set_mode(address, RadioMode::Am, 1))?;
        Ok(radio)
    }

    pub fn model_name(&self) -> String {
        match &self.link {
            Link::Net(net) if !net.info.name.is_empty() => format!("{} · network", net.info.name),
            Link::Net(_) => format!("{} · network", self.model.name()),
            Link::Usb { sound, .. } => format!("{} · {}", self.model.name(), sound.name),
        }
    }

    pub fn frequency_range(&self) -> (f64, f64) {
        self.model.range()
    }

    fn send(&mut self, frame: &[u8]) -> Result<(), String> {
        match &mut self.link {
            Link::Usb { serial, .. } => serial.write_all(frame).map_err(|e| format!("CI-V: {e}")),
            Link::Net(net) => {
                net.send_civ(frame);
                Ok(())
            }
        }
    }

    pub fn set_frequency(&mut self, hz: f64) -> Result<(), String> {
        self.frequency = hz.round();
        self.tuned_by_us = Instant::now();
        self.send(&civ::set_frequency(self.address, hz))
    }

    /// The rate is fixed by the IF; nothing to set.
    pub fn set_sample_rate(&mut self, _rate: f64) -> Result<(), String> {
        Ok(())
    }

    /// RF gain in percent; None leaves the radio's setting.
    pub fn set_gain(&mut self, gain: Option<f64>) -> Result<(), String> {
        match gain {
            Some(g) => self.send(&civ::set_rf_gain(self.address, (g.clamp(0.0, 100.0) / 100.0 * 255.0).round() as u8)),
            None => Ok(()),
        }
    }

    /// A frequency change made on the radio (dial, band key) since the last call.
    pub fn take_tuned(&mut self) -> Option<f64> {
        self.tuned.take()
    }

    /// Frames from the radio: follows frequency changes made on it.
    fn poll_civ(&mut self) {
        let mut bytes = Vec::new();
        match &mut self.link {
            Link::Usb { serial, .. } => {
                if serial.bytes_to_read().unwrap_or(0) > 0 {
                    let mut buf = [0u8; 512];
                    if let Ok(n) = serial.read(&mut buf) {
                        bytes.extend_from_slice(&buf[..n]);
                    }
                }
            }
            Link::Net(net) => bytes.extend(net.civ_rx.try_iter().flatten()),
        }
        if bytes.is_empty() {
            return;
        }
        for frame in self.parser.push(&bytes) {
            if frame.from != self.address || !(frame.to == civ::BROADCAST || frame.to == civ::CONTROLLER) {
                continue;
            }
            if let Some(hz) = frame.frequency().map(|f| f as f64) {
                if self.tuned_by_us.elapsed() < OWN_CHANGE {
                    continue;
                }
                if (hz - self.frequency).abs() >= 1.0 || self.frequency.is_nan() {
                    self.frequency = hz;
                    self.tuned = Some(hz);
                }
            }
        }
    }

    /// Fills `out` with complex samples; waits for the radio when none are ready.
    pub fn read(&mut self, out: &mut [Complex<f32>]) -> Result<usize, String> {
        if self.poll_at.elapsed() >= POLL_INTERVAL {
            self.poll_at = Instant::now();
            self.send(&civ::read_frequency(self.address))?;
        }
        self.poll_civ();
        if self.ready.is_empty() {
            self.raw.clear();
            match &mut self.link {
                Link::Usb { sound, .. } => sound.read(&mut self.raw, Duration::from_millis(200))?,
                Link::Net(net) => {
                    if let Some(e) = net.error() {
                        return Err(e);
                    }
                    if let Ok(first) = net.audio_rx.recv_timeout(Duration::from_millis(200)) {
                        let more = net.audio_rx.try_iter();
                        self.raw.extend(std::iter::once(first).chain(more).flatten().map(|v| f32::from(v) / 32768.0));
                    }
                }
            }
            if self.raw.is_empty() {
                if self.last_if.elapsed() > SILENCE_TIMEOUT {
                    return Err(format!("{}: no IF from the radio", self.model.name()));
                }
                return Ok(0);
            }
            self.last_if = Instant::now();
            match self.check.push(&self.raw) {
                Some(IfVerdict::Audio) => {
                    return Err(format!(
                        "{} sends audio, not its IF: on the radio set SET > Connectors > ACC/USB Output Select (IC-7300) or USB / LAN / WLAN AF/IF Output > Output Select = IF",
                        self.model.name()
                    ));
                }
                Some(IfVerdict::Silence) => {
                    let input = match &self.link {
                        Link::Usb { sound, .. } => sound.name.clone(),
                        Link::Net(_) => "network audio".into(),
                    };
                    return Err(format!("{input}: silence; is this the radio's sound input, and its IF output level above 0?"));
                }
                _ => {}
            }
            self.converted.clear();
            self.converter.process(&self.raw, &mut self.converted);
            self.ready.extend(self.converted.drain(..));
        }
        let n = out.len().min(self.ready.len());
        for (o, s) in out.iter_mut().zip(self.ready.drain(..n)) {
            *o = s;
        }
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arguments_round_trip() {
        let a = usb_args("COM5", "wasapi:{0.0.1.00000000}.{abc|def}");
        assert!(is_icom(&a));
        assert_eq!(parse_usb(&a), Some(("COM5", "wasapi:{0.0.1.00000000}.{abc|def}")));
        assert_eq!(parse_usb("icom-usb:/dev/ttyUSB0"), Some(("/dev/ttyUSB0", "")));
        assert_eq!(net_label("icom:op:secret@192.168.1.50"), "Icom · op@192.168.1.50");
        assert!(is_icom("icom:op:secret@192.168.1.50"));
        assert!(!is_icom("ip:192.168.1.47"));
    }

    #[test]
    fn icom_sound_cards_are_recognised() {
        assert!(is_icom_sound("Microphone (USB Audio CODEC )"));
        assert!(is_icom_sound("Mikrofon (2- USB Audio CODEC)"));
        assert!(!is_icom_sound("Microphone (Realtek High Definition Audio)"));
        assert!(!is_icom_sound("Mikrofon (5- USB PnP Sound Device)"));
        assert!(is_icom_sound("Microphone (IC-7300MK2 Audio)"));
        assert_eq!(with_sound("icom-usb:COM2|old", "new"), "icom-usb:COM2|new");
        assert_eq!(with_sound("icom:u:p@h", "new"), "icom:u:p@h");
    }

    #[test]
    fn network_radio_gives_samples_around_its_frequency() {
        let radio = crate::icomnet::tests::FakeRadio::start("secret");
        let args = format!("{NET_PREFIX}user:{}@127.0.0.1:{}", radio.password, radio.port);
        let mut icom = IcomRadio::open(&args).expect("open");
        assert_eq!(icom.model, Model::Ic705);
        assert_eq!(icom.model_name(), "IC-705 · network");
        icom.set_frequency(14_074_000.0).unwrap();
        icom.set_gain(Some(50.0)).unwrap();
        // The fake radio sends a tone at 13.5 kHz in the IF: 1.5 kHz above the dial.
        let mut samples = vec![Complex::new(0.0, 0.0); 12_000];
        let mut filled = 0;
        let started = Instant::now();
        while filled < samples.len() && started.elapsed() < Duration::from_secs(3) {
            filled += icom.read(&mut samples[filled..]).unwrap();
        }
        assert_eq!(filled, samples.len());
        let tail = &samples[2_000..];
        let power = |hz: f64| {
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (i, s) in tail.iter().enumerate() {
                let (sn, cs) = (-std::f64::consts::TAU * hz * i as f64 / SAMPLE_RATE).sin_cos();
                re += s.re as f64 * cs - s.im as f64 * sn;
                im += s.re as f64 * sn + s.im as f64 * cs;
            }
            (re * re + im * im).sqrt() / tail.len() as f64
        };
        assert!((power(1_500.0) - 0.25).abs() < 0.03, "tone amplitude {}", power(1_500.0));
        assert!(power(-1_500.0) < 0.01);
        let seen = radio.civ_seen.lock().unwrap().clone();
        assert!(seen.contains(&civ::set_mode(0xA4, RadioMode::Am, 1)), "{seen:02X?}");
        assert!(seen.contains(&civ::set_frequency(0xA4, 14_074_000.0)));
        assert!(seen.contains(&civ::set_rf_gain(0xA4, 128)));
    }

    fn verdict(tone_hz: f64, amplitude: f64) -> Option<IfVerdict> {
        let x: Vec<f32> =
            (0..CHECK_SAMPLES).map(|i| (amplitude * (std::f64::consts::TAU * tone_hz * i as f64 / 48_000.0).sin()) as f32).collect();
        let mut c = IfCheck::default();
        let v = c.push(&x[..1000]).or_else(|| c.push(&x[1000..]));
        assert!(c.push(&x).is_none(), "verdict once");
        v
    }

    #[test]
    fn if_is_told_from_audio_and_silence() {
        assert_eq!(verdict(12_500.0, 0.1), Some(IfVerdict::If));
        assert_eq!(verdict(8_000.0, 0.1), Some(IfVerdict::If));
        assert_eq!(verdict(1_000.0, 0.1), Some(IfVerdict::Audio));
        assert_eq!(verdict(2_500.0, 0.1), Some(IfVerdict::Audio));
        assert_eq!(verdict(1_000.0, 0.0), Some(IfVerdict::Silence));
    }

    /// Lists Icom radios on USB: `cargo test --lib icom_usb -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn icom_usb_devices() {
        for f in discover_usb() {
            println!("{:?} | {} | {}", f.model, f.label, f.args);
        }
    }
}
