//! One interface over the back ends (SoapySDR, libiio and Icom transceivers).

use num_complex::Complex;
use soapysdr::Direction::Rx;

use crate::device::{Backend, DeviceInfo, DeviceKind};
use crate::icom::IcomRadio;
use crate::iio::IioRadio;

/// Receive settings.
#[derive(Clone, Debug)]
pub struct Tuning {
    pub center_hz: f64,
    pub sample_rate: f64,
    /// Fixed gain in dB; None for automatic gain.
    pub gain: Option<f64>,
}

/// What the opened device reports about itself.
#[derive(Clone, Debug, Default)]
pub struct DriverInfo {
    /// Tunable range (Hz).
    pub frequency_range: Option<(f64, f64)>,
    /// Sample rate range (S/s).
    pub rate_range: Option<(f64, f64)>,
    /// Hardware description, e.g. the RTL-SDR tuner or the Pluto model.
    pub model: String,
}

pub enum Radio {
    Soapy { device: soapysdr::Device, stream: soapysdr::RxStream<Complex<f32>>, kind: DeviceKind },
    Iio(IioRadio),
    Icom(Box<IcomRadio>),
}

fn span(ranges: &[soapysdr::Range]) -> Option<(f64, f64)> {
    let lo = ranges.iter().map(|r| r.minimum).fold(f64::INFINITY, f64::min);
    let hi = ranges.iter().map(|r| r.maximum).fold(f64::NEG_INFINITY, f64::max);
    (lo.is_finite() && hi > lo).then_some((lo, hi))
}

impl Radio {
    pub fn open(info: &DeviceInfo, t: &Tuning) -> Result<(Self, DriverInfo), String> {
        let (mut radio, driver) = match info.kind.backend() {
            Backend::Soapy => {
                let device = soapysdr::Device::new(info.args.as_str()).map_err(|e| e.to_string())?;
                let driver = DriverInfo {
                    frequency_range: device.frequency_range(Rx, 0).ok().as_deref().and_then(span),
                    rate_range: device.get_sample_rate_range(Rx, 0).ok().as_deref().and_then(span),
                    model: device.hardware_key().unwrap_or_default(),
                };
                let stream = device.rx_stream::<Complex<f32>>(&[0]).map_err(|e| e.to_string())?;
                (Radio::Soapy { device, stream, kind: info.kind }, driver)
            }
            Backend::Iio => {
                let radio = IioRadio::open(&info.args)?;
                let id = radio.identity();
                let driver = DriverInfo { frequency_range: id.frequency_range, rate_range: id.rate_range, model: id.model };
                (Radio::Iio(radio), driver)
            }
            Backend::Icom => {
                let radio = IcomRadio::open(&info.args)?;
                let driver = DriverInfo { frequency_range: Some(radio.frequency_range()), rate_range: None, model: radio.model_name() };
                (Radio::Icom(Box::new(radio)), driver)
            }
        };
        radio.set_sample_rate(t.sample_rate)?;
        radio.set_frequency(t.center_hz)?;
        radio.set_gain(t.gain)?;
        if let Radio::Soapy { stream, .. } = &mut radio {
            stream.activate(None).map_err(|e| e.to_string())?;
        }
        Ok((radio, driver))
    }

    pub fn set_frequency(&mut self, hz: f64) -> Result<(), String> {
        match self {
            Radio::Soapy { device, .. } => device.set_frequency(Rx, 0, hz, ()).map_err(|e| e.to_string()),
            Radio::Iio(r) => r.set_frequency(hz),
            Radio::Icom(r) => r.set_frequency(hz),
        }
    }

    /// Changed without stopping the stream: some rtlsdr.dll versions crash on deactivate.
    pub fn set_sample_rate(&mut self, rate: f64) -> Result<(), String> {
        match self {
            Radio::Soapy { device, kind, .. } => {
                device.set_sample_rate(Rx, 0, rate).map_err(|e| e.to_string())?;
                // The HackRF's analog filter follows the rate only when asked.
                if *kind == DeviceKind::HackRf {
                    let _ = device.set_bandwidth(Rx, 0, rate * 0.75);
                }
                Ok(())
            }
            Radio::Iio(r) => r.set_sample_rate(rate),
            Radio::Icom(r) => r.set_sample_rate(rate),
        }
    }

    pub fn set_gain(&mut self, gain: Option<f64>) -> Result<(), String> {
        match self {
            Radio::Soapy { device, kind: DeviceKind::HackRf, .. } => {
                // One gain value is split between the LNA (8 dB steps) and the VGA (2 dB steps),
                // the LNA first (better for noise). The 14 dB RF amplifier stays off.
                let db = gain.unwrap_or(40.0).clamp(0.0, 102.0);
                let lna = ((db * 0.6 / 8.0).floor() * 8.0).min(40.0);
                let vga = (((db - lna) / 2.0).round() * 2.0).clamp(0.0, 62.0);
                [("AMP", 0.0), ("LNA", lna), ("VGA", vga)]
                    .into_iter()
                    .try_for_each(|(name, value)| device.set_gain_element(Rx, 0, name, value))
                    .map_err(|e| e.to_string())
            }
            Radio::Soapy { device, .. } => {
                let result = match gain {
                    Some(db) => device.set_gain_mode(Rx, 0, false).and_then(|_| device.set_gain(Rx, 0, db)),
                    None => device.set_gain_mode(Rx, 0, true),
                };
                result.map_err(|e| e.to_string())
            }
            Radio::Iio(r) => r.set_gain(gain),
            Radio::Icom(r) => r.set_gain(gain),
        }
    }

    /// Number of samples read; `Ok(None)` when samples were lost (overflow).
    pub fn read(&mut self, out: &mut [Complex<f32>]) -> Result<Option<usize>, String> {
        match self {
            Radio::Soapy { stream, .. } => match stream.read(&mut [out], 1_000_000) {
                Ok(n) => Ok(Some(n)),
                Err(e) if e.code == soapysdr::ErrorCode::Overflow => Ok(None),
                Err(e) if e.code == soapysdr::ErrorCode::Timeout => Ok(Some(0)),
                Err(e) => Err(e.to_string()),
            },
            Radio::Iio(r) => r.read(out).map(Some),
            Radio::Icom(r) => r.read(out).map(Some),
        }
    }

    /// Transmits through a transceiver (Icom); other receivers cannot.
    pub fn transmit(&mut self, audio: std::sync::Arc<Vec<f32>>, start_at: f64, output: Option<String>) -> Result<(), String> {
        match self {
            Radio::Icom(r) => r.transmit(audio, start_at, output),
            _ => Err("this receiver cannot transmit".into()),
        }
    }

    pub fn stop_transmit(&mut self) {
        if let Radio::Icom(r) = self {
            r.stop_transmit();
        }
    }

    pub fn set_radio_mode(&mut self, mode: crate::civ::RadioMode, data: bool) -> Result<(), String> {
        match self {
            Radio::Icom(r) => r.set_radio_mode(mode, data),
            _ => Ok(()),
        }
    }

    pub fn take_tx_error(&mut self) -> Option<String> {
        match self {
            Radio::Icom(r) => r.take_tx_error(),
            _ => None,
        }
    }

    /// A frequency the receiver was tuned to by hand (the dial of an Icom transceiver).
    pub fn take_tuned(&mut self) -> Option<f64> {
        match self {
            Radio::Icom(r) => r.take_tuned(),
            _ => None,
        }
    }
}
