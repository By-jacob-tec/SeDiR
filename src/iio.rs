//! AD9361/AD9363-based receivers over libiio (ADALM-Pluto, LibreSDR and similar).
//!
//! libiio is loaded at run time: if it is not installed the program still starts and
//! only these devices are unavailable. The libiio 0.x C API is used (on Windows the
//! `libiio.dll` that comes with PothosSDR, on Linux `libiio.so.0`).

use std::ffi::{CStr, CString, c_char, c_uint, c_void};

use libloading::Library;
use num_complex::Complex;

/// Samples per transfer: about 40 ms of signal. Over the network every transfer is a
/// round trip, so high rates need large buffers to keep up.
fn buffer_samples(rate: f64) -> usize {
    ((rate * 0.04) as usize).next_power_of_two().clamp(65_536, 1 << 20)
}
/// The AD9361 ADC is 12-bit; samples arrive in the ±2048 range.
const FULL_SCALE: f32 = 2048.0;
/// Network and USB operations give up after this long (ms).
const TIMEOUT_MS: c_uint = 3000;

type Ptr = *mut c_void;

/// libiio functions in use.
struct Api {
    create_context: unsafe extern "C" fn(*const c_char) -> Ptr,
    context_destroy: unsafe extern "C" fn(Ptr),
    context_set_timeout: unsafe extern "C" fn(Ptr, c_uint) -> i32,
    context_attr: unsafe extern "C" fn(Ptr, *const c_char) -> *const c_char,
    find_device: unsafe extern "C" fn(Ptr, *const c_char) -> Ptr,
    find_channel: unsafe extern "C" fn(Ptr, *const c_char, bool) -> Ptr,
    attr_write: unsafe extern "C" fn(Ptr, *const c_char, *const c_char) -> isize,
    attr_read: unsafe extern "C" fn(Ptr, *const c_char, *mut c_char, usize) -> isize,
    channel_enable: unsafe extern "C" fn(Ptr),
    create_buffer: unsafe extern "C" fn(Ptr, usize, bool) -> Ptr,
    buffer_refill: unsafe extern "C" fn(Ptr) -> isize,
    buffer_start: unsafe extern "C" fn(Ptr) -> *const c_void,
    buffer_end: unsafe extern "C" fn(Ptr) -> *const c_void,
    buffer_destroy: unsafe extern "C" fn(Ptr),
    scan_create: unsafe extern "C" fn(*const c_char, c_uint) -> Ptr,
    scan_destroy: unsafe extern "C" fn(Ptr),
    scan_list: unsafe extern "C" fn(Ptr, *mut *mut Ptr) -> isize,
    info_list_free: unsafe extern "C" fn(*mut Ptr),
    info_uri: unsafe extern "C" fn(Ptr) -> *const c_char,
    info_description: unsafe extern "C" fn(Ptr) -> *const c_char,
    // The library must stay open as long as the function pointers are valid.
    _lib: Library,
}

impl Api {
    fn load() -> Result<Self, String> {
        let names: &[&str] = if cfg!(windows) {
            &["libiio.dll", "iio.dll"]
        } else if cfg!(target_os = "macos") {
            // Homebrew's lib folders are not on the default dlopen search path.
            &["libiio.dylib", "/opt/homebrew/lib/libiio.dylib", "/usr/local/lib/libiio.dylib", "/Library/Frameworks/iio.framework/iio"]
        } else {
            &["libiio.so.0", "libiio.so"]
        };
        let lib = names.iter().find_map(|&n| unsafe { Library::new(n) }.ok()).ok_or("libiio not found")?;
        unsafe {
            macro_rules! sym {
                ($name:literal) => {
                    *lib.get(concat!($name, "\0").as_bytes()).map_err(|e| format!("libiio: {e}"))?
                };
            }
            Ok(Self {
                create_context: sym!("iio_create_context_from_uri"),
                context_destroy: sym!("iio_context_destroy"),
                context_set_timeout: sym!("iio_context_set_timeout"),
                context_attr: sym!("iio_context_get_attr_value"),
                find_device: sym!("iio_context_find_device"),
                find_channel: sym!("iio_device_find_channel"),
                attr_write: sym!("iio_channel_attr_write"),
                attr_read: sym!("iio_channel_attr_read"),
                channel_enable: sym!("iio_channel_enable"),
                create_buffer: sym!("iio_device_create_buffer"),
                buffer_refill: sym!("iio_buffer_refill"),
                buffer_start: sym!("iio_buffer_start"),
                buffer_end: sym!("iio_buffer_end"),
                buffer_destroy: sym!("iio_buffer_destroy"),
                scan_create: sym!("iio_create_scan_context"),
                scan_destroy: sym!("iio_scan_context_destroy"),
                scan_list: sym!("iio_scan_context_get_info_list"),
                info_list_free: sym!("iio_context_info_list_free"),
                info_uri: sym!("iio_context_info_get_uri"),
                info_description: sym!("iio_context_info_get_description"),
                _lib: lib,
            })
        }
    }

    /// Reads a channel attribute as text.
    fn read_attr(&self, channel: Ptr, attr: &str) -> Option<String> {
        let mut buf = vec![0u8; 256];
        let n = unsafe { (self.attr_read)(channel, cstr(attr).as_ptr(), buf.as_mut_ptr() as *mut c_char, buf.len()) };
        if n <= 0 {
            return None;
        }
        let end = buf.iter().position(|&b| b == 0).unwrap_or(n as usize);
        Some(String::from_utf8_lossy(&buf[..end]).trim().to_string())
    }
}

/// Whether libiio can be loaded (for diagnostics).
pub fn library_status() -> Result<(), String> {
    Api::load().map(|_| ())
}

fn cstr(s: &str) -> CString {
    CString::new(s).expect("text without NUL")
}

fn text(p: *const c_char) -> Option<String> {
    (!p.is_null()).then(|| unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned())
}

/// libiio contexts found on USB: (URI, description).
pub fn scan_usb() -> Vec<(String, String)> {
    let Ok(api) = Api::load() else { return Vec::new() };
    unsafe {
        let scan = (api.scan_create)(cstr("usb").as_ptr(), 0);
        if scan.is_null() {
            return Vec::new();
        }
        let mut list: *mut Ptr = std::ptr::null_mut();
        let n = (api.scan_list)(scan, &mut list);
        let mut found = Vec::new();
        for i in 0..n.max(0) as usize {
            let info = *list.add(i);
            if let Some(uri) = text((api.info_uri)(info)) {
                found.push((uri, text((api.info_description)(info)).unwrap_or_default()));
            }
        }
        if !list.is_null() {
            (api.info_list_free)(list);
        }
        (api.scan_destroy)(scan);
        found
    }
}

/// What an AD936x context reports about itself.
#[derive(Clone, Debug, Default)]
pub struct Identity {
    /// The `hw_model` context attribute, e.g. "Analog Devices PlutoSDR Rev.B (Z7010-AD9363A)".
    pub model: String,
    /// Tunable receive range (Hz), from the LO's `frequency_available`.
    pub frequency_range: Option<(f64, f64)>,
    /// Sample rate range (S/s), from `sampling_frequency_available`.
    pub rate_range: Option<(f64, f64)>,
}

/// "[70000000 1 6000000000]" → (70e6, 6e9).
fn parse_range(text: &str) -> Option<(f64, f64)> {
    let inner = text.trim().trim_start_matches('[').trim_end_matches(']');
    let parts: Vec<f64> = inner.split_whitespace().filter_map(|p| p.parse().ok()).collect();
    match parts.as_slice() {
        [lo, _, hi] => Some((*lo, *hi)),
        [lo, hi] => Some((*lo, *hi)),
        _ => None,
    }
}

/// An open AD936x receiver.
pub struct IioRadio {
    api: Api,
    context: Ptr,
    /// The receive settings channel of ad9361-phy (sample rate, bandwidth, gain).
    rx_config: Ptr,
    /// The RX_LO channel of ad9361-phy.
    rx_lo: Ptr,
    rx: Ptr,
    i: Ptr,
    q: Ptr,
    /// Created on the first read, so that settings made before it apply to the first block;
    /// made again when the sample rate changes (its size follows the rate).
    buffer: Ptr,
    rate: f64,
    pending: Vec<Complex<f32>>,
    position: usize,
}

// The libiio objects belong only to this struct and are used on a single thread.
unsafe impl Send for IioRadio {}

impl IioRadio {
    /// `uri` e.g. "ip:192.168.2.1" or "usb:1.5.5".
    pub fn open(uri: &str) -> Result<Self, String> {
        let api = Api::load()?;
        unsafe {
            let context = (api.create_context)(cstr(uri).as_ptr());
            if context.is_null() {
                return Err(format!("could not connect to {uri}"));
            }
            (api.context_set_timeout)(context, TIMEOUT_MS);
            let find = |dev: &str| (api.find_device)(context, cstr(dev).as_ptr());
            let (phy, rx) = (find("ad9361-phy"), find("cf-ad9361-lpc"));
            if phy.is_null() || rx.is_null() {
                (api.context_destroy)(context);
                return Err(format!("{uri} is not an AD936x receiver"));
            }
            let channel = |dev, name: &str, output| (api.find_channel)(dev, cstr(name).as_ptr(), output);
            let rx_config = channel(phy, "voltage0", false);
            let rx_lo = channel(phy, "altvoltage0", true);
            let (i, q) = (channel(rx, "voltage0", false), channel(rx, "voltage1", false));
            if [rx_config, rx_lo, i, q].iter().any(|c| c.is_null()) {
                (api.context_destroy)(context);
                return Err(format!("{uri}: receiver channels not found"));
            }
            Ok(Self { api, context, rx_config, rx_lo, rx, i, q, buffer: std::ptr::null_mut(), rate: 2.4e6, pending: Vec::new(), position: 0 })
        }
    }

    pub fn identity(&self) -> Identity {
        let model = text(unsafe { (self.api.context_attr)(self.context, cstr("hw_model").as_ptr()) }).unwrap_or_default();
        Identity {
            model,
            frequency_range: self.api.read_attr(self.rx_lo, "frequency_available").as_deref().and_then(parse_range),
            rate_range: self.api.read_attr(self.rx_config, "sampling_frequency_available").as_deref().and_then(parse_range),
        }
    }

    fn write(&self, channel: Ptr, attr: &str, value: &str) -> Result<(), String> {
        let n = unsafe { (self.api.attr_write)(channel, cstr(attr).as_ptr(), cstr(value).as_ptr()) };
        if n < 0 { Err(format!("{attr}={value}: error {n}")) } else { Ok(()) }
    }

    pub fn set_frequency(&mut self, hz: f64) -> Result<(), String> {
        self.write(self.rx_lo, "frequency", &format!("{}", hz.round() as i64))
    }

    /// Sets the analog filter together with the sample rate. The filter is opened to the
    /// sample rate: when narrower, signals near the band edge are attenuated.
    pub fn set_sample_rate(&mut self, rate: f64) -> Result<(), String> {
        self.write(self.rx_config, "sampling_frequency", &format!("{}", rate.round() as i64))?;
        let bandwidth = rate.clamp(200_000.0, 56_000_000.0);
        self.write(self.rx_config, "rf_bandwidth", &format!("{}", bandwidth.round() as i64))?;
        if buffer_samples(rate) != buffer_samples(self.rate) && !self.buffer.is_null() {
            unsafe { (self.api.buffer_destroy)(self.buffer) };
            self.buffer = std::ptr::null_mut();
        }
        self.rate = rate;
        Ok(())
    }

    /// `None` for automatic gain (slow AGC), otherwise a fixed gain in dB.
    pub fn set_gain(&mut self, gain: Option<f64>) -> Result<(), String> {
        match gain {
            Some(db) => {
                self.write(self.rx_config, "gain_control_mode", "manual")?;
                self.write(self.rx_config, "hardwaregain", &format!("{db:.0}"))
            }
            None => self.write(self.rx_config, "gain_control_mode", "slow_attack"),
        }
    }

    /// Fills `out`; reads a new block from the device if needed.
    pub fn read(&mut self, out: &mut [Complex<f32>]) -> Result<usize, String> {
        if self.position >= self.pending.len() {
            unsafe {
                if self.buffer.is_null() {
                    (self.api.channel_enable)(self.i);
                    (self.api.channel_enable)(self.q);
                    self.buffer = (self.api.create_buffer)(self.rx, buffer_samples(self.rate), false);
                    if self.buffer.is_null() {
                        return Err("could not create the sample buffer".into());
                    }
                }
                let n = (self.api.buffer_refill)(self.buffer);
                if n < 0 {
                    return Err(format!("sample read failed: error {n}"));
                }
                self.pending.clear();
                self.position = 0;
                let start = (self.api.buffer_start)(self.buffer) as *const i16;
                let end = (self.api.buffer_end)(self.buffer) as *const i16;
                let raw = std::slice::from_raw_parts(start, end.offset_from(start) as usize);
                // I and Q channels interleaved, signed 16-bit.
                self.pending.extend(
                    raw.chunks_exact(2).map(|iq| Complex::new(iq[0] as f32 / FULL_SCALE, iq[1] as f32 / FULL_SCALE)),
                );
            }
        }
        let n = out.len().min(self.pending.len() - self.position);
        out[..n].copy_from_slice(&self.pending[self.position..self.position + n]);
        self.position += n;
        Ok(n)
    }
}

impl Drop for IioRadio {
    fn drop(&mut self) {
        unsafe {
            if !self.buffer.is_null() {
                (self.api.buffer_destroy)(self.buffer);
            }
            (self.api.context_destroy)(self.context);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_iio_ranges() {
        assert_eq!(parse_range("[70000000 1 6000000000]"), Some((70e6, 6e9)));
        assert_eq!(parse_range("[2083333 1 61440000]"), Some((2_083_333.0, 61.44e6)));
        assert_eq!(parse_range("garbage"), None);
    }

    /// With a real device: `SEDIR_IIO_URI=ip:192.168.1.47 cargo test --lib iio -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn identifies_device() {
        let Some(uri) = std::env::var_os("SEDIR_IIO_URI") else { return };
        let radio = IioRadio::open(uri.to_str().unwrap()).unwrap();
        println!("{:?}", radio.identity());
        println!("USB: {:?}", scan_usb());
    }
}
