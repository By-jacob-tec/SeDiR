//! Supported receivers, their limits and finding them.

use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::iio;

/// libiio's network port.
const IIOD_PORT: u16 = 30431;
/// Network receivers that do not answer this fast are skipped while searching.
const PROBE_TIMEOUT: Duration = Duration::from_millis(400);
/// The ADALM-Pluto's address over its USB network interface.
pub const PLUTO_DEFAULT_IP: &str = "192.168.2.1";

/// How samples are read from the device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    Soapy,
    Iio,
}

/// Receiver kind; sets its nominal limits and how its gain works.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeviceKind {
    RtlSdr,
    HackRf,
    /// ADALM-Pluto (AD9363; many units report the AD9364 range).
    Pluto,
    /// LibreSDR and other AD9361 boards (Zynq + AD9361 over Ethernet).
    LibreSdr,
    /// Any other SoapySDR device; limits come from its driver.
    Other,
}

impl DeviceKind {
    pub fn name(self) -> &'static str {
        match self {
            DeviceKind::RtlSdr => "RTL-SDR",
            DeviceKind::HackRf => "HackRF",
            DeviceKind::Pluto => "ADALM-Pluto",
            DeviceKind::LibreSdr => "LibreSDR / AD9361",
            DeviceKind::Other => "SoapySDR",
        }
    }

    pub fn backend(self) -> Backend {
        match self {
            DeviceKind::Pluto | DeviceKind::LibreSdr => Backend::Iio,
            _ => Backend::Soapy,
        }
    }

    /// Nominal limits, used before the device is opened. The driver's own ranges
    /// replace the frequency range once it is open (see `Limits::refine`).
    pub fn limits(self) -> Limits {
        let ms = |v: &[f64]| v.iter().map(|r| r * 1e6).collect::<Vec<_>>();
        match self {
            // R820T/R828D tuner. Above 2.4 MS/s the RTL2832U drops samples on most PCs.
            DeviceKind::RtlSdr => Limits {
                min_hz: 24e6,
                max_hz: 1766e6,
                rates: ms(&[0.25, 1.024, 1.4, 1.8, 2.048, 2.4, 2.56, 2.88, 3.2]),
                max_rate: 2.4e6,
                gain: (0.0, 49.6),
                auto_gain: true,
                weak_below_hz: None,
            },
            DeviceKind::HackRf => Limits {
                min_hz: 1e6,
                max_hz: 6e9,
                rates: ms(&[2.0, 4.0, 8.0, 10.0, 12.5, 16.0, 20.0]),
                max_rate: 20e6,
                gain: (0.0, 102.0),
                auto_gain: false,
                // Works down to 1 MHz, but with a much poorer front end below ~10 MHz.
                weak_below_hz: Some(10e6),
            },
            // AD9363 data sheet range; units with the AD9364 range report it when opened.
            // USB 2.0 carries about 4 MS/s of 16-bit IQ without loss.
            DeviceKind::Pluto => Limits {
                min_hz: 325e6,
                max_hz: 3.8e9,
                rates: ms(&[2.4, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]),
                max_rate: 4e6,
                gain: (0.0, 71.0),
                auto_gain: true,
                weak_below_hz: None,
            },
            // Gigabit Ethernet: 12 MS/s measured without loss.
            DeviceKind::LibreSdr => Limits {
                min_hz: 70e6,
                max_hz: 6e9,
                rates: ms(&[2.4, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0]),
                max_rate: 12e6,
                gain: (0.0, 71.0),
                auto_gain: true,
                weak_below_hz: None,
            },
            DeviceKind::Other => Limits {
                min_hz: 0.0,
                max_hz: f64::INFINITY,
                rates: ms(&[0.25, 1.0, 2.0, 2.4]),
                max_rate: 2.4e6,
                gain: (0.0, 50.0),
                auto_gain: true,
                weak_below_hz: None,
            },
        }
    }

    /// Gain used when the device is first selected (dB).
    pub fn default_gain(self) -> f64 {
        match self {
            DeviceKind::RtlSdr => 30.0,
            DeviceKind::HackRf => 40.0,
            DeviceKind::Pluto | DeviceKind::LibreSdr => 40.0,
            DeviceKind::Other => 30.0,
        }
    }
}

/// What a receiver can do.
#[derive(Clone, Debug, PartialEq)]
pub struct Limits {
    pub min_hz: f64,
    pub max_hz: f64,
    /// Sample rates offered (S/s), ascending.
    pub rates: Vec<f64>,
    /// Highest rate the device carries without dropping samples; the default.
    pub max_rate: f64,
    /// Adjustable gain range (dB).
    pub gain: (f64, f64),
    pub auto_gain: bool,
    /// Below this frequency the device works but poorly.
    pub weak_below_hz: Option<f64>,
}

/// Whether a frequency can be received.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FreqCheck {
    Ok,
    /// Inside the range, but where the device performs poorly.
    Weak,
    TooLow,
    TooHigh,
}

impl Limits {
    pub fn check(&self, hz: f64) -> FreqCheck {
        if hz < self.min_hz {
            FreqCheck::TooLow
        } else if hz > self.max_hz {
            FreqCheck::TooHigh
        } else if self.weak_below_hz.is_some_and(|w| hz < w) {
            FreqCheck::Weak
        } else {
            FreqCheck::Ok
        }
    }

    /// Takes the frequency range the driver reports and drops rates it cannot set.
    pub fn refine(&mut self, frequency: Option<(f64, f64)>, rate: Option<(f64, f64)>) {
        if let Some((lo, hi)) = frequency.filter(|(lo, hi)| hi > lo && *hi > 0.0) {
            self.min_hz = lo;
            self.max_hz = hi;
        }
        if let Some((lo, hi)) = rate.filter(|(lo, hi)| hi > lo) {
            let keep: Vec<f64> = self.rates.iter().copied().filter(|r| (lo..=hi).contains(r)).collect();
            if !keep.is_empty() {
                self.rates = keep;
            }
            self.max_rate = self.max_rate.min(hi);
        }
    }

    /// The offered rate closest to `rate`.
    pub fn nearest_rate(&self, rate: f64) -> f64 {
        self.rates.iter().copied().min_by(|a, b| (a - rate).abs().total_cmp(&(b - rate).abs())).unwrap_or(rate)
    }
}

/// A selectable receiver.
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    pub label: String,
    /// "key=value,..." arguments for SoapySDR, a URI for libiio (e.g. "ip:192.168.2.1").
    pub args: String,
    pub kind: DeviceKind,
}

/// SoapySDR devices other than sound cards. Pluto devices found by SoapyPlutoSDR are
/// skipped: they are read over libiio directly.
pub fn soapy_devices() -> Vec<DeviceInfo> {
    let Ok(found) = soapysdr::enumerate("") else { return Vec::new() };
    found
        .iter()
        .filter(|a| !matches!(a.get("driver"), Some("audio") | Some("plutosdr")))
        .map(|a| {
            let args = a.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(",");
            let kind = match a.get("driver") {
                Some("rtlsdr") => DeviceKind::RtlSdr,
                Some("hackrf") => DeviceKind::HackRf,
                _ => DeviceKind::Other,
            };
            let label = match (kind, a.get("label")) {
                (DeviceKind::Other, Some(l)) => l.to_string(),
                (_, Some(l)) => format!("{} · {l}", kind.name()),
                (_, None) => kind.name().to_string(),
            };
            DeviceInfo { label, args, kind }
        })
        .collect()
}

/// Opens an AD936x context briefly to tell a Pluto from other boards.
fn identify_iio(uri: &str) -> Option<DeviceInfo> {
    let radio = iio::IioRadio::open(uri).ok()?;
    let model = radio.identity().model;
    let kind = if model.contains("Pluto") { DeviceKind::Pluto } else { DeviceKind::LibreSdr };
    let label = if model.is_empty() { format!("{} · {uri}", kind.name()) } else { format!("{model} · {uri}") };
    Some(DeviceInfo { label, args: uri.to_string(), kind })
}

/// `address` is an IP address, a host name or a full libiio URI ("ip:…").
pub fn network_uri(address: &str) -> String {
    let address = address.trim();
    if address.contains(':') && !address.starts_with('[') { address.to_string() } else { format!("ip:{address}") }
}

/// Whether something answers on the libiio port, so that unreachable addresses do not
/// hold up the search for the operating system's long connection timeout.
fn reachable(uri: &str) -> bool {
    let Some(host) = uri.strip_prefix("ip:") else { return true };
    let Ok(addrs) = (host, IIOD_PORT).to_socket_addrs() else { return false };
    addrs.take(2).any(|a: SocketAddr| TcpStream::connect_timeout(&a, PROBE_TIMEOUT).is_ok())
}

/// Finds every receiver: SoapySDR devices, AD936x devices on USB, and at the given
/// network addresses plus the Pluto's default address. Takes up to a few seconds.
pub fn discover(network: &[String]) -> Vec<DeviceInfo> {
    let mut found = soapy_devices();
    if iio::library_status().is_ok() {
        for (uri, _) in iio::scan_usb() {
            found.extend(identify_iio(&uri));
        }
        let mut uris: Vec<String> = network.iter().map(|a| network_uri(a)).collect();
        let pluto = network_uri(PLUTO_DEFAULT_IP);
        if !uris.contains(&pluto) {
            uris.push(pluto);
        }
        for uri in uris {
            if reachable(&uri) {
                found.extend(identify_iio(&uri));
            }
        }
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checks_frequency_against_limits() {
        let rtl = DeviceKind::RtlSdr.limits();
        assert_eq!(rtl.check(145e6), FreqCheck::Ok);
        assert_eq!(rtl.check(7e6), FreqCheck::TooLow);
        assert_eq!(rtl.check(2.4e9), FreqCheck::TooHigh);
        let hackrf = DeviceKind::HackRf.limits();
        assert_eq!(hackrf.check(7e6), FreqCheck::Weak);
        assert_eq!(hackrf.check(5.8e9), FreqCheck::Ok);
    }

    #[test]
    fn driver_ranges_replace_nominal_ones() {
        let mut pluto = DeviceKind::Pluto.limits();
        assert_eq!(pluto.check(100e6), FreqCheck::TooLow);
        pluto.refine(Some((70e6, 6e9)), Some((2_083_333.0, 61.44e6)));
        assert_eq!(pluto.check(100e6), FreqCheck::Ok);
        assert_eq!(pluto.rates.first(), Some(&2.4e6));
        assert_eq!(pluto.nearest_rate(3.9e6), 4e6);
    }

    #[test]
    fn network_addresses_become_uris() {
        assert_eq!(network_uri("192.168.1.47"), "ip:192.168.1.47");
        assert_eq!(network_uri("ip:pluto.local"), "ip:pluto.local");
        assert_eq!(network_uri(" pluto.local "), "ip:pluto.local");
    }

    /// Lists connected receivers: `cargo test --lib discover -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn discover_prints_devices() {
        let extra: Vec<String> = std::env::var("SEDIR_NET").ok().into_iter().collect();
        for d in discover(&extra) {
            println!("{:?} | {} | {}", d.kind, d.label, d.args);
        }
    }
}
