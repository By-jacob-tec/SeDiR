//! The main window.

use std::sync::mpsc::{self, Receiver};

use eframe::egui::{
    self, Align2, Color32, ColorImage, CornerRadius, FontId, Mesh, Pos2, Rect, RichText, Sense, Shape, Stroke, TextureHandle,
    TextureOptions, pos2, vec2,
};

use sedir::audio::{self, AudioBuffer, AudioOutput, OutputDevice, Volume};
use sedir::beacon::{self, BeaconStatus, QO100_BEACON_HZ, SharedBeacon};
use sedir::classify::{self, Classification, Reason};
use sedir::demod::Mode;
use sedir::detect::{self, Signal};
use sedir::device::{self, DeviceInfo, DeviceKind, FreqCheck, Limits};
use sedir::freq;
use sedir::ft8::worker::{DIALS, FT4_DIALS, Ft8, Ft8Message, dials};
use sedir::ft8::{FT4, FT8};
use sedir::listen::{ListenSettings, Listener};
use sedir::radio::Tuning;
use sedir::rds::RdsInfo;
use sedir::source::{self, Command, Event, Frame, Handle, Stats, View};
use sedir::stations::Stations;

use crate::i18n::{Lang, Strings, fill};
use crate::settings::Settings;
use crate::ui::palette::*;
use crate::ui::{apply_theme, brand, card, colormap, dot, freq_tuner, metric, wheel_steps};

const WATERFALL_WIDTH: usize = 2048;
const WATERFALL_ROWS: usize = 400;
/// Weight of a new spectrum frame in the displayed (smoothed) one.
const SMOOTHING: f32 = 0.4;
/// Signals are searched every this many frames.
const SCAN_EVERY_FRAMES: u32 = 6;
/// Used until the user picks a frequency (2 m amateur band).
const DEFAULT_FREQUENCY: f64 = 145_500_000.0;
const DEFAULT_THRESHOLD_DB: f32 = 8.0;
/// Mouse wheel steps offered for the listening frequency (Hz).
const STEPS: [f64; 12] = [1.0, 10.0, 100.0, 500.0, 1e3, 2.5e3, 5e3, 6.25e3, 10e3, 12.5e3, 25e3, 100e3];
/// Narrowest visible band offered (Hz).
const MIN_SPAN: f64 = 5e3;
/// Window width (points) from which the settings are laid out in two columns.
const WIDE_WINDOW: f32 = 1700.0;
/// Narrowest window (points) that still leaves the spectrum some room beside two columns.
const TWO_COLUMNS_MIN: f32 = 1250.0;
const ZOOM_STEP: f32 = 0.1;
const ZOOM_RANGE: (f32, f32) = (0.7, 2.5);
/// Subscriber id of the listening thread at the source.
const LISTEN_ID: u32 = 1;
/// Blocks the listening thread may have queued (~0.6 s).
const LISTEN_QUEUE: usize = 32;
/// Subscriber id of the station name scanner.
const SCAN_ID: u32 = 2;
/// Subscriber id of a mode recognition job.
const CLASSIFY_ID: u32 = 3;
/// Subscriber id of the beacon tracker.
const BEACON_ID: u32 = 4;
/// Subscriber ids of the FT8 and FT4 decoders.
const FT_IDS: [u32; 2] = [5, 6];
/// Audio range of FT8/FT4 above the dial frequency (Hz), shaded on the spectrum.
const FT8_AUDIO_HZ: (f64, f64) = (100.0, 3100.0);

/// One of the slot decoders (FT8, FT4) with its switch and dial frequency.
struct FtDecoder {
    worker: Ft8,
    on: bool,
    /// Dial frequency (radio frequency, Hz).
    dial_hz: f64,
}

impl FtDecoder {
    fn name(&self) -> &'static str {
        self.worker.protocol.name
    }
}
/// Converter LO of a QO-100 LNB: the beacon can be followed with it.
const QO100_LO_HZ: f64 = 9_750e6;
/// Signals at least this wide and this far above the noise, inside the FM broadcast
/// bands (OIRT 65.8–74 MHz included), are searched for RDS.
const FM_MIN_WIDTH_HZ: f64 = 50e3;
const FM_MIN_SNR_DB: f32 = 12.0;
const FM_BANDS_HZ: (f64, f64) = (64e6, 110e6);
const LOGO_PIXELS: usize = 128;
const DEFAULT_VOLUME: f32 = 60.0;

#[derive(Clone, Copy, PartialEq)]
struct FrameInfo {
    center_hz: f64,
    sample_rate: f64,
    lo_hz: f64,
    hi_hz: f64,
    bin_hz: f64,
    points: usize,
}

pub struct SedirApp {
    lang: Lang,
    /// Text size is chosen (saved, or picked from the screen size on the first frames).
    zoom_ready: bool,
    /// Height (points) the settings cards took last frame, all columns together.
    cards_height: f32,
    settings: Settings,

    devices: Vec<DeviceInfo>,
    discovery: Option<Receiver<Vec<DeviceInfo>>>,
    device_index: usize,
    limits: Limits,
    driver_model: String,
    network_entry: String,

    handle: Option<Handle>,
    opened: bool,
    error: Option<String>,
    stats: Stats,

    /// Receiver centre and listening frequency, as radio frequencies (before the converter).
    center_hz: f64,
    vfo_hz: f64,
    /// Local oscillator of a converter in front of the receiver (LNB, up-converter):
    /// radio frequency = receiver frequency + this. 0 without one; negative for an
    /// up-converter.
    lo_hz: f64,
    /// Error of that LO (Hz), measured on a beacon or set by hand: the effective LO is
    /// `lo_hz + correction_hz`.
    correction_hz: f64,
    beacon_lock: bool,
    beacon: Option<SharedBeacon>,
    /// FT8 and FT4.
    ft: [FtDecoder; 2],
    sample_rate: f64,
    gain_db: f64,
    auto_gain: bool,
    view: View,
    step_hz: f64,
    threshold_db: f32,
    entry: String,
    entry_note: Option<(String, Color32)>,

    frame: Option<FrameInfo>,
    smoothed: Vec<f32>,
    range: (f32, f32),
    waterfall: ColorImage,
    waterfall_tex: Option<TextureHandle>,
    signals: Vec<Signal>,
    frames_since_scan: u32,

    mode: Mode,
    bandwidth_hz: f64,
    squelch_on: bool,
    squelch_db: f32,
    listener: Listener,
    stations: Stations,
    find_names: bool,
    rds: Option<RdsInfo>,
    auto_mode: bool,
    /// Recognition in progress.
    classifying: Option<Receiver<Option<Classification>>>,
    /// Last result: Some(Some) recognised, Some(None) failed.
    classification: Option<Option<Classification>>,

    audio_buffer: AudioBuffer,
    volume: Volume,
    volume_pct: f32,
    muted: bool,
    audio_devices: Vec<OutputDevice>,
    audio_device: Option<String>,
    audio: Option<AudioOutput>,
    audio_error: Option<String>,

    logo_tex: TextureHandle,
}

impl SedirApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        apply_theme(&cc.egui_ctx);
        let settings = Settings::load();
        if let Some(z) = settings.zoom {
            cc.egui_ctx.set_zoom_factor(z.clamp(ZOOM_RANGE.0, ZOOM_RANGE.1));
        }
        let center = settings.frequency_hz.unwrap_or(DEFAULT_FREQUENCY);
        let limits = DeviceKind::Other.limits();
        let mode = settings.mode.as_deref().and_then(Mode::from_name).unwrap_or(Mode::Nfm);
        let volume_pct = settings.volume.unwrap_or(DEFAULT_VOLUME);
        let logo = ColorImage::from_rgba_unmultiplied([LOGO_PIXELS, LOGO_PIXELS], &crate::logo::rgba(LOGO_PIXELS, false));
        let find_names = settings.find_names.unwrap_or(true);
        let lo_hz = settings.lo_hz.unwrap_or(0.0);
        let correction_hz = settings.correction_hz.unwrap_or(0.0);
        let beacon_lock = settings.beacon_lock.unwrap_or(true);
        let auto_mode = settings.auto_mode.unwrap_or(true);
        let mut app = Self {
            ft: [
                FtDecoder {
                    worker: Ft8::new(&FT8),
                    on: settings.ft8_on.unwrap_or(false),
                    dial_hz: settings.ft8_dial_hz.unwrap_or(DIALS[0].0),
                },
                FtDecoder {
                    worker: Ft8::new(&FT4),
                    on: settings.ft4_on.unwrap_or(false),
                    dial_hz: settings.ft4_dial_hz.unwrap_or(FT4_DIALS[0].0),
                },
            ],
            correction_hz,
            beacon_lock,
            beacon: None,
            auto_mode,
            classifying: None,
            classification: None,
            stations: Stations::new(find_names),
            find_names,
            rds: None,
            mode,
            bandwidth_hz: mode.default_bandwidth(),
            squelch_on: false,
            squelch_db: -60.0,
            listener: Listener::new(ListenSettings {
                frequency_hz: center,
                mode,
                bandwidth_hz: mode.default_bandwidth(),
                squelch_db: None,
                audio_rate: 48_000.0,
            }),
            audio_buffer: AudioBuffer::default(),
            volume: Volume::new(volume_gain(volume_pct)),
            volume_pct,
            muted: false,
            audio_devices: audio::output_devices(),
            audio_device: settings.audio_device.clone(),
            audio: None,
            audio_error: None,
            logo_tex: cc.egui_ctx.load_texture("logo", logo, TextureOptions::LINEAR),
            lang: settings.lang.unwrap_or_else(Lang::from_system),
            zoom_ready: settings.zoom.is_some(),
            cards_height: 0.0,
            threshold_db: settings.threshold_db.unwrap_or(DEFAULT_THRESHOLD_DB),
            settings,
            devices: Vec::new(),
            discovery: None,
            device_index: 0,
            sample_rate: limits.max_rate,
            limits,
            driver_model: String::new(),
            network_entry: String::new(),
            handle: None,
            opened: false,
            error: None,
            stats: Stats::default(),
            center_hz: center,
            vfo_hz: center,
            lo_hz,
            gain_db: 30.0,
            auto_gain: false,
            view: View::full(2.4e6),
            step_hz: mode.default_step(),
            entry: String::new(),
            entry_note: None,
            frame: None,
            smoothed: Vec::new(),
            range: (-110.0, -30.0),
            waterfall: ColorImage::new([WATERFALL_WIDTH, WATERFALL_ROWS], vec![DEEP; WATERFALL_WIDTH * WATERFALL_ROWS]),
            waterfall_tex: None,
            signals: Vec::new(),
            frames_since_scan: 0,
        };
        app.discover(&cc.egui_ctx);
        app
    }

    fn t(&self) -> &'static Strings {
        self.lang.strings()
    }

    // ---- Devices ----

    /// Searches for receivers in the background.
    fn discover(&mut self, ctx: &egui::Context) {
        let (tx, rx) = mpsc::channel();
        let network = self.settings.network_devices.clone();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let _ = tx.send(device::discover(&network));
            ctx.request_repaint();
        });
        self.discovery = Some(rx);
    }

    fn poll_discovery(&mut self) {
        let Some(found) = self.discovery.as_ref().and_then(|rx| rx.try_recv().ok()) else { return };
        self.discovery = None;
        self.devices = found;
        let index = self
            .settings
            .device
            .as_ref()
            .and_then(|args| self.devices.iter().position(|d| &d.args == args))
            .unwrap_or(0);
        if !self.devices.is_empty() {
            self.select_device(index);
        }
    }

    fn device(&self) -> Option<&DeviceInfo> {
        self.devices.get(self.device_index)
    }

    /// Takes the nominal limits of the device and opens the widest band it carries.
    fn select_device(&mut self, index: usize) {
        self.device_index = index;
        let Some(kind) = self.device().map(|d| d.kind) else { return };
        self.limits = kind.limits();
        self.driver_model.clear();
        self.sample_rate = self.limits.max_rate;
        self.gain_db = kind.default_gain();
        self.auto_gain = false;
        self.view = View::full(self.sample_rate);
        self.entry_note = self.frequency_note(self.vfo_hz).err().or_else(|| self.frequency_note(self.vfo_hz).ok().flatten());
    }

    /// Whether the radio frequency `hz` can be received (after the converter, if any): Ok
    /// with an optional caution, or Err with the reason. The range is given in radio
    /// frequencies too.
    fn frequency_note(&self, hz: f64) -> Result<Option<(String, Color32)>, (String, Color32)> {
        let t = self.t();
        let name = self.device().map_or(DeviceKind::Other.name(), |d| d.kind.name());
        let (lo, hi) = (freq::short(self.limits.min_hz + self.lo()), freq::short(self.limits.max_hz + self.lo()));
        match self.limits.check(hz - self.lo()) {
            FreqCheck::Ok => Ok(None),
            FreqCheck::Weak => {
                let below = freq::short(self.limits.weak_below_hz.unwrap_or(0.0));
                Ok(Some((fill(t.weak_range, &[&name, &freq::short(hz), &below]), WARN)))
            }
            FreqCheck::TooLow => Err((fill(t.too_low, &[&freq::short(hz), &name, &lo, &hi]), BAD)),
            FreqCheck::TooHigh => Err((fill(t.too_high, &[&freq::short(hz), &name, &lo, &hi]), BAD)),
        }
    }

    fn add_network_device(&mut self, ctx: &egui::Context) {
        let address = self.network_entry.trim().to_string();
        if address.is_empty() {
            return;
        }
        if !self.settings.network_devices.contains(&address) {
            self.settings.network_devices.push(address);
            self.settings.save();
        }
        self.network_entry.clear();
        self.discover(ctx);
    }

    // ---- Running ----

    fn gain(&self) -> Option<f64> {
        (!self.auto_gain).then_some(self.gain_db)
    }

    fn send(&self, cmd: Command) {
        if let Some(h) = &self.handle {
            let _ = h.commands.send(cmd);
        }
    }

    fn start(&mut self, ctx: &egui::Context) {
        let Some(device) = self.device().cloned() else { return };
        if let Err(note) = self.frequency_note(self.center_hz) {
            self.entry_note = Some(note);
            return;
        }
        let tuning = Tuning { center_hz: self.center_hz - self.lo(), sample_rate: self.sample_rate, gain: self.gain() };
        self.view = self.view.clamped(self.sample_rate);
        let ctx = ctx.clone();
        self.handle = Some(source::spawn(device.clone(), tuning, self.view, move || ctx.request_repaint()));
        let (tx, rx) = mpsc::sync_channel(LISTEN_QUEUE);
        self.send(Command::Subscribe(LISTEN_ID, tx));
        self.sync_listener();
        self.listener.spawn(rx, self.audio_buffer.clone(), self.stations.clone());
        let (tx, rx) = mpsc::sync_channel(LISTEN_QUEUE);
        self.send(Command::Subscribe(SCAN_ID, tx));
        self.stations.spawn_scanner(rx);
        for k in 0..self.ft.len() {
            let (tx, rx) = mpsc::sync_channel(LISTEN_QUEUE);
            self.send(Command::Subscribe(FT_IDS[k], tx));
            self.ft[k].worker.spawn(rx);
        }
        if self.audio.is_none() {
            self.open_audio();
        }
        self.opened = false;
        self.error = None;
        self.stats = Stats::default();
        self.frame = None;
        self.settings.device = Some(device.args);
        self.settings.frequency_hz = Some(self.vfo_hz);
        self.settings.save();
    }

    fn stop(&mut self) {
        self.handle = None;
        self.beacon = None;
        self.opened = false;
        self.stats = Stats::default();
        self.audio_buffer.clear();
    }

    // ---- Listening and sound ----

    /// Opens the chosen playback device; the demodulator follows its sample rate.
    fn open_audio(&mut self) {
        self.audio = None;
        self.audio_buffer.clear();
        match audio::open(self.audio_device.as_deref(), self.audio_buffer.clone(), self.volume.clone()) {
            Ok(out) => {
                self.audio_error = None;
                let rate = out.sample_rate as f64;
                self.listener.update(|s| s.audio_rate = rate);
                self.audio = Some(out);
            }
            Err(e) => self.audio_error = Some(e),
        }
    }

    fn apply_volume(&self) {
        self.volume.set(if self.muted { 0.0 } else { volume_gain(self.volume_pct) });
    }

    fn set_mode(&mut self, mode: Mode) {
        if mode == self.mode {
            return;
        }
        self.mode = mode;
        self.bandwidth_hz = mode.default_bandwidth();
        self.step_hz = mode.default_step();
        self.settings.mode = Some(mode.name().to_string());
        self.settings.save();
    }

    /// Effective converter LO: nominal plus the measured or set error.
    fn lo(&self) -> f64 {
        self.lo_hz + self.correction_hz
    }

    /// Takes a new LO correction. The receiver stays where it is, so the signal being
    /// listened to stays; only the radio frequencies shown move to their true values.
    fn apply_correction(&mut self, correction: f64) {
        let shift = correction - self.correction_hz;
        self.correction_hz = correction;
        self.center_hz += shift;
        self.vfo_hz += shift;
        // Kept so the next start begins close; saved only on real changes.
        if (self.settings.correction_hz.unwrap_or(0.0) - correction).abs() > 20.0 {
            self.settings.correction_hz = Some(correction);
            self.settings.save();
        }
    }

    /// Follows the QO-100 beacon while running with the QO-100 LNB and the lock on.
    fn update_beacon_tracker(&mut self) {
        let wanted = self.beacon_lock && self.lo_hz == QO100_LO_HZ && self.handle.is_some();
        if wanted && self.beacon.is_none() {
            let (tx, rx) = mpsc::sync_channel(LISTEN_QUEUE);
            self.send(Command::Subscribe(BEACON_ID, tx));
            self.beacon = Some(beacon::spawn(rx, QO100_BEACON_HZ, self.lo_hz, self.correction_hz));
        } else if !wanted && self.beacon.is_some() {
            self.send(Command::Unsubscribe(BEACON_ID));
            self.beacon = None;
        }
        // Carried along the measured drift every frame: the correction changes smoothly,
        // so FT8 tones and the listened signal stay put between measurements.
        let measured = self.beacon.as_ref().and_then(|b| b.lock().unwrap().correction_at(std::time::Instant::now()));
        if let Some(c) = measured {
            if (c - self.correction_hz).abs() > 0.05 {
                self.apply_correction(c);
            }
        }
    }

    /// Changes the converter LO. The receiver stays tuned where it is; the radio
    /// frequencies shown move with the LO.
    fn set_converter(&mut self, lo: f64) {
        let shift = lo - self.lo_hz;
        self.lo_hz = lo;
        // A tracker follows one LO; it starts again for the new one.
        if self.beacon.take().is_some() {
            self.send(Command::Unsubscribe(BEACON_ID));
        }
        self.center_hz += shift;
        self.vfo_hz += shift;
        self.frame = None;
        self.signals.clear();
        self.classification = None;
        self.settings.lo_hz = Some(lo);
        self.settings.frequency_hz = Some(self.vfo_hz);
        self.settings.save();
    }

    /// A click on the spectrum at `hz`, on `signal` if one was found there. With automatic
    /// mode the signal is measured and the result applied when it arrives; otherwise the
    /// current mode is kept.
    fn select_at(&mut self, hz: f64, signal: Option<Signal>) {
        if self.auto_mode && self.handle.is_some() {
            let target = signal.map_or(hz, |s| s.center_hz);
            self.set_vfo(target);
            let (tx, rx) = mpsc::sync_channel(LISTEN_QUEUE);
            // Replaces a job still running: its channel closes and it stops.
            self.send(Command::Subscribe(CLASSIFY_ID, tx));
            let width = signal.map_or(3e3, |s| s.bandwidth_hz);
            self.classifying = Some(classify::spawn_job(rx, target - self.lo(), width, self.lo()));
            self.classification = None;
        } else {
            let hz = match signal {
                Some(s) if self.mode.snaps_to_center() => self.mode.round_to_channel(s.center_hz),
                _ => hz,
            };
            self.set_vfo(hz);
        }
    }

    fn poll_classification(&mut self) {
        let Some(result) = self.classifying.as_ref().and_then(|rx| rx.try_recv().ok()) else { return };
        self.classifying = None;
        self.send(Command::Unsubscribe(CLASSIFY_ID));
        if let Some(c) = &result {
            self.set_mode(c.mode);
            let (lo, hi) = c.mode.bandwidth_range();
            self.bandwidth_hz = c.bandwidth_hz.clamp(lo, hi);
            self.set_vfo(c.frequency_hz);
        }
        self.classification = Some(result);
    }

    /// Hands the current listening settings to the demodulator thread.
    fn sync_listener(&self) {
        let squelch = self.squelch_on.then_some(self.squelch_db);
        let (frequency, mode, bandwidth) = (self.vfo_hz - self.lo(), self.mode, self.bandwidth_hz);
        self.listener.update(|s| {
            s.frequency_hz = frequency;
            s.mode = mode;
            s.bandwidth_hz = bandwidth;
            s.squelch_db = squelch;
        });
    }

    fn poll_events(&mut self) {
        let Some(handle) = &self.handle else { return };
        let events: Vec<Event> = handle.events.try_iter().collect();
        for event in events {
            match event {
                Event::Opened(driver) => {
                    self.opened = true;
                    self.driver_model = driver.model;
                    let kind = self.device().map_or(DeviceKind::Other, |d| d.kind);
                    // The HackRF driver reports the synthesiser's range (to 7.25 GHz), wider than the
                    // receiver is specified for; its nominal range is kept.
                    let frequency = if kind == DeviceKind::HackRf { None } else { driver.frequency_range };
                    self.limits.refine(frequency, driver.rate_range);
                    let rate = self.limits.nearest_rate(self.sample_rate);
                    if rate != self.sample_rate {
                        self.set_sample_rate(rate);
                    }
                }
                Event::Frame(frame) => self.push_frame(frame),
                Event::Stats(s) => self.stats = s,
                Event::Error(e) => {
                    self.error = Some(e);
                    self.handle = None;
                    self.opened = false;
                }
            }
        }
    }

    fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        // Zoom is kept as a fraction, so the same part of the band stays in view.
        let full = self.view.span_hz >= self.frame.map_or(self.view.span_hz, |f| f.sample_rate) * 0.999;
        self.view = if full { View::full(rate) } else { self.view.clamped(rate) };
        self.send(Command::SampleRate(rate));
        self.send(Command::View(self.view));
    }

    /// Visible band (Hz, absolute).
    fn view_range(&self) -> (f64, f64) {
        let c = self.center_hz + self.view.offset_hz;
        (c - self.view.span_hz / 2.0, c + self.view.span_hz / 2.0)
    }

    fn set_view(&mut self, view: View) {
        self.view = view.clamped(self.sample_rate);
        self.send(Command::View(self.view));
    }

    /// Tunes the receiver's centre. Refused (with a note) outside the device's range.
    fn retune(&mut self, hz: f64) -> bool {
        match self.frequency_note(hz) {
            Err(note) => {
                self.entry_note = Some(note);
                false
            }
            Ok(note) => {
                self.entry_note = note;
                self.center_hz = hz.round();
                self.send(Command::Tune(self.center_hz - self.lo()));
                true
            }
        }
    }

    /// Moves the listening frequency; the view follows, inside the received band when
    /// possible, otherwise by retuning.
    fn set_vfo(&mut self, hz: f64) {
        let hz = hz.max(0.0).round();
        if let Err(note) = self.frequency_note(hz) {
            self.entry_note = Some(note);
            return;
        }
        self.vfo_hz = hz;
        let (lo, hi) = self.view_range();
        if (lo..=hi).contains(&hz) {
            return;
        }
        let room = self.sample_rate / 2.0 - self.view.span_hz / 2.0;
        let offset = hz - self.center_hz;
        if offset.abs() <= room {
            self.set_view(View { offset_hz: offset, span_hz: self.view.span_hz });
        } else if self.retune(hz) {
            self.set_view(View { offset_hz: 0.0, span_hz: self.view.span_hz });
        }
    }

    /// Frequency typed in the entry field: centre the band on it and listen there.
    fn submit_entry(&mut self) {
        let t = self.t();
        let Some(hz) = freq::parse(&self.entry) else {
            self.entry_note = Some((fill(t.bad_number, &[&self.entry.trim()]), BAD));
            return;
        };
        if self.retune(hz) {
            self.vfo_hz = self.center_hz;
            self.set_view(View { offset_hz: 0.0, span_hz: self.view.span_hz });
            self.entry.clear();
            self.settings.frequency_hz = Some(self.vfo_hz);
            self.settings.save();
        }
    }

    // ---- Spectrum data ----

    fn push_frame(&mut self, frame: Frame) {
        // The source works in the receiver's own frequencies; the window in radio frequencies.
        let info = FrameInfo {
            center_hz: frame.center_hz + self.lo(),
            sample_rate: frame.sample_rate,
            lo_hz: frame.lo_hz + self.lo(),
            hi_hz: frame.hi_hz + self.lo(),
            bin_hz: frame.bin_hz,
            points: frame.db.len(),
        };
        let previous = self.frame.replace(info);
        let same_scale = previous.is_some_and(|p| {
            p.points == info.points && ((p.hi_hz - p.lo_hz) - (info.hi_hz - info.lo_hz)).abs() < info.bin_hz
        });
        let moved = previous.map_or(0.0, |p| info.lo_hz - p.lo_hz);
        if !same_scale || self.smoothed.len() != frame.db.len() {
            self.smoothed = frame.db.clone();
            self.waterfall.pixels.fill(DEEP);
        } else if moved.abs() > info.bin_hz / 2.0 {
            // Panned: shift the history instead of blurring it into the new position.
            let step = (info.hi_hz - info.lo_hz) / info.points as f64;
            let shift = (moved / step).round() as isize;
            shift_values(&mut self.smoothed, shift, frame.db.as_slice());
            let wf_shift = (moved / (info.hi_hz - info.lo_hz) * WATERFALL_WIDTH as f64).round() as isize;
            for row in self.waterfall.pixels.chunks_exact_mut(WATERFALL_WIDTH) {
                shift_row(row, wf_shift, DEEP);
            }
        } else {
            for (s, v) in self.smoothed.iter_mut().zip(&frame.db) {
                *s += SMOOTHING * (v - *s);
            }
        }

        // Track the display range slowly from the noise floor and the peak.
        let mut sorted = self.smoothed.clone();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted[sorted.len() / 10] - 8.0;
        let top = (sorted[sorted.len() - 1] + 8.0).max(floor + 30.0);
        if previous.is_none() {
            self.range = (floor, top);
        } else {
            self.range.0 += 0.08 * (floor - self.range.0);
            self.range.1 += 0.08 * (top - self.range.1);
        }

        // Scroll the waterfall down one row and write the new row at the top.
        self.waterfall.pixels.copy_within(0..WATERFALL_WIDTH * (WATERFALL_ROWS - 1), WATERFALL_WIDTH);
        let (lo, hi) = self.range;
        let n = frame.db.len();
        for (x, px) in self.waterfall.pixels[..WATERFALL_WIDTH].iter_mut().enumerate() {
            let a = x * n / WATERFALL_WIDTH;
            let b = ((x + 1) * n / WATERFALL_WIDTH).max(a + 1).min(n);
            let v = frame.db[a..b].iter().copied().fold(f32::NEG_INFINITY, f32::max);
            *px = colormap((v - lo) / (hi - lo));
        }
        if let Some(tex) = &mut self.waterfall_tex {
            tex.set(self.waterfall.clone(), TextureOptions::LINEAR);
        }

        self.frames_since_scan += 1;
        if self.frames_since_scan >= SCAN_EVERY_FRAMES {
            self.frames_since_scan = 0;
            self.signals = detect::find(&self.smoothed, info.lo_hz, info.hi_hz, self.threshold_db);
            // FM stations, strongest first, for the RDS name scanner.
            let mut fm: Vec<&Signal> = self
                .signals
                .iter()
                .filter(|s| s.bandwidth_hz >= FM_MIN_WIDTH_HZ && s.snr_db >= FM_MIN_SNR_DB)
                .filter(|s| (FM_BANDS_HZ.0..=FM_BANDS_HZ.1).contains(&s.center_hz))
                .collect();
            fm.sort_by(|a, b| b.snr_db.total_cmp(&a.snr_db));
            // The scanner and the name store work in the receiver's frequencies.
            self.stations.set_targets(fm.iter().map(|s| s.center_hz - self.lo()).collect());
        }
    }

    /// The signal under `hz`, allowing `slack` Hz around its edges.
    fn signal_at(&self, hz: f64, slack: f64) -> Option<Signal> {
        self.signals.iter().copied().find(|s| (hz - s.center_hz).abs() <= s.bandwidth_hz / 2.0 + slack)
    }

    // ---- Header ----

    fn header(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        ui.horizontal(|ui| {
            ui.add(egui::Image::new(&self.logo_tex).fit_to_exact_size(vec2(52.0, 52.0)));
            ui.vertical(|ui| {
                brand(ui, 30.0);
                ui.label(RichText::new(t.app_subtitle).size(12.0).color(MUTED));
            });
            ui.add_space(28.0);
            ui.vertical(|ui| {
                ui.label(RichText::new(t.listening_caps).size(10.0).color(MUTED));
                let mut hz = self.vfo_hz;
                if freq_tuner(ui, &mut hz, 26.0) {
                    self.set_vfo(hz);
                }
            });
            ui.add_space(12.0);
            ui.label(RichText::new(self.mode.name()).size(18.0).strong().monospace().color(AMBER));
            if self.opened {
                ui.add_space(8.0);
                let level = self.listener.level_db();
                metric(ui, t.level_caps, &format!("{level:.0} dBFS"), TEXT);
            }
            if let Some(s) = self.signal_at(self.vfo_hz, 0.0) {
                metric(ui, t.width_caps, &freq::width(s.bandwidth_hz), TEXT);
                metric(ui, t.snr_caps, &format!("{:.0} dB", s.snr_db), snr_color(s.snr_db));
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let running = self.handle.is_some();
                let button = if running {
                    egui::Button::new(RichText::new(t.stop).strong().color(Color32::WHITE)).fill(BAD.gamma_multiply(0.75))
                } else {
                    egui::Button::new(RichText::new(t.start).strong().color(DEEP)).fill(ACCENT)
                };
                if ui.add_enabled(running || self.device().is_some(), button.min_size(vec2(140.0, 36.0)).corner_radius(8)).clicked() {
                    if running {
                        self.stop();
                    } else {
                        self.start(ui.ctx());
                    }
                }
                ui.add_space(8.0);
                let before = self.lang;
                egui::ComboBox::from_id_salt("language")
                    .width(96.0)
                    .selected_text(self.lang.native_name())
                    .show_ui(ui, |ui| {
                        for l in Lang::ALL {
                            ui.selectable_value(&mut self.lang, l, l.native_name());
                        }
                    })
                    .response
                    .on_hover_text(t.language);
                if self.lang != before {
                    self.settings.lang = Some(self.lang);
                    self.settings.save();
                }
                // Text size (also Ctrl + / Ctrl − / Ctrl 0); right to left, so A+ first.
                ui.add_space(8.0);
                let zoom = ui.ctx().zoom_factor();
                for (label, step) in [("A+", ZOOM_STEP), ("A−", -ZOOM_STEP)] {
                    let button = egui::Button::new(RichText::new(label).strong()).min_size(vec2(32.0, 26.0));
                    if ui.add(button).on_hover_text(fill(t.zoom_tooltip, &[&format!("{:.0}", zoom * 100.0)])).clicked() {
                        ui.ctx().set_zoom_factor((zoom + step).clamp(ZOOM_RANGE.0, ZOOM_RANGE.1));
                    }
                }
            });
        });
    }

    // ---- Left panel ----

    /// Cards `cards` (indices into receiver, frequency, listen, audio, digital, view).
    /// Returns the height the cards took.
    fn sidebar(&mut self, ui: &mut egui::Ui, cards: &[usize]) -> f32 {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let t = self.t();
            let top = ui.cursor().top();
            for &i in cards {
                match i {
                    0 => card(ui, t.card_receiver, |ui| self.receiver_card(ui)),
                    1 => card(ui, t.card_frequency, |ui| self.frequency_card(ui)),
                    2 => card(ui, t.card_listen, |ui| self.listen_card(ui)),
                    3 => card(ui, t.card_audio, |ui| self.audio_card(ui)),
                    4 => card(ui, t.card_digital, |ui| self.digital_card(ui)),
                    _ => card(ui, t.card_view, |ui| self.view_card(ui)),
                }
            }
            ui.cursor().top() - top
        })
        .inner
    }

    fn listen_card(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        let mut mode = self.mode;
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            for m in Mode::ALL {
                let selected = mode == m;
                let text = RichText::new(m.name()).monospace().strong().color(if selected { DEEP } else { TEXT });
                let button = egui::Button::new(text).min_size(vec2(44.0, 26.0)).corner_radius(6);
                let button = if selected { button.fill(ACCENT) } else { button };
                if ui.add(button).clicked() {
                    mode = m;
                }
            }
        });
        self.set_mode(mode);
        ui.horizontal(|ui| {
            if ui.checkbox(&mut self.auto_mode, t.auto_mode).on_hover_text(t.auto_mode_tooltip).changed() {
                self.settings.auto_mode = Some(self.auto_mode);
                self.settings.save();
            }
        });
        if self.classifying.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label(RichText::new(t.recognising).size(12.0).color(MUTED));
            });
        } else if let Some(result) = &self.classification {
            let text = match result {
                Some(c) => {
                    let why = c.reasons.iter().map(|r| reason_text(*r, t)).collect::<Vec<_>>().join(", ");
                    fill(t.recognised, &[&c.mode.name(), &format!("{:.0}", c.confidence * 100.0), &why])
                }
                None => t.not_recognised.to_string(),
            };
            ui.label(RichText::new(text).size(12.0).color(if result.is_some() { ACCENT } else { WARN }));
        }
        ui.add_space(4.0);
        let (lo, hi) = self.mode.bandwidth_range();
        let mut khz = self.bandwidth_hz / 1e3;
        ui.horizontal(|ui| {
            ui.label(t.filter);
            let step = if hi <= 4e3 { 0.05 } else { 0.5 };
            if ui.add(egui::Slider::new(&mut khz, lo / 1e3..=hi / 1e3).step_by(step).suffix(" kHz")).changed() {
                self.bandwidth_hz = khz * 1e3;
            }
        });
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.squelch_on, t.squelch).on_hover_text(t.squelch_tooltip);
            ui.add_enabled(self.squelch_on, egui::Slider::new(&mut self.squelch_db, -120.0..=0.0).step_by(1.0).suffix(" dB"));
        });
        if self.handle.is_some() {
            level_bar(ui, self.listener.level_db(), self.squelch_on.then_some(self.squelch_db));
        }
    }

    fn save_ft(&mut self, k: usize) {
        let (on, dial) = (Some(self.ft[k].on), Some(self.ft[k].dial_hz));
        if k == 0 {
            (self.settings.ft8_on, self.settings.ft8_dial_hz) = (on, dial);
        } else {
            (self.settings.ft4_on, self.settings.ft4_dial_hz) = (on, dial);
        }
        self.settings.save();
    }

    fn digital_card(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        for k in 0..self.ft.len() {
            if k > 0 {
                ui.add_space(6.0);
            }
            let name = self.ft[k].name();
            let slot = format!("{}", self.ft[k].worker.protocol.slot_s);
            if ui.checkbox(&mut self.ft[k].on, fill(t.ft8_decode, &[&name])).on_hover_text(fill(t.ft8_tooltip, &[&slot])).changed() {
                self.save_ft(k);
            }
            ui.horizontal(|ui| {
                ui.label(t.ft8_dial);
                let list = dials(self.ft[k].worker.protocol);
                let old = self.ft[k].dial_hz;
                let current = list.iter().find(|d| d.0 == old).map_or_else(|| mhz_text(old), |d| d.1.to_string());
                let mut dial = old;
                egui::ComboBox::from_id_salt(("ft_dial", k)).selected_text(current).show_ui(ui, |ui| {
                    for &(hz, name) in list {
                        ui.selectable_value(&mut dial, hz, format!("{name} · {}", freq::short(hz)));
                    }
                });
                let mut mhz = dial / 1e6;
                if ui.add(egui::DragValue::new(&mut mhz).speed(0.001).max_decimals(4).suffix(" MHz")).changed() {
                    dial = (mhz * 1e6).round();
                }
                if dial != old {
                    self.ft[k].dial_hz = dial;
                    self.ft[k].worker.clear();
                    self.save_ft(k);
                }
                if ui.button(t.ft8_listen).clicked() {
                    self.set_mode(Mode::Usb);
                    self.bandwidth_hz = 2.8e3;
                    self.set_vfo(dial);
                }
            });
            if self.ft[k].on && self.opened {
                let half = self.sample_rate / 2.0;
                let inside = (self.center_hz - half..self.center_hz + half).contains(&self.ft[k].dial_hz);
                if !inside {
                    ui.label(RichText::new(t.ft8_outside).size(12.0).color(WARN));
                }
            }
        }
    }

    /// Status line of one slot decoder above the message list.
    fn ft_status(&mut self, ui: &mut egui::Ui, k: usize) {
        let t = self.t();
        let d = &self.ft[k];
        let p = d.worker.protocol;
        let status = d.worker.status();
        ui.horizontal(|ui| {
            let badge = egui::Button::new(RichText::new(p.name).size(11.0).strong().color(DEEP)).fill(ft_color(k)).corner_radius(4).sense(Sense::hover());
            ui.add(badge).on_hover_text(fill(t.ft8_tooltip, &[&format!("{}", p.slot_s)]));
            ui.label(RichText::new(mhz_text(d.dial_hz)).size(12.0).monospace().color(MUTED));
            let text = match status.last_slot {
                Some(slot) => {
                    let mut s = fill(t.ft8_last, &[&utc_hms(slot), &status.last_count]);
                    if status.coverage_s < p.decode_at_s - 0.5 {
                        s += &format!(" · {}", fill(t.ft8_partial, &[&format!("{:.1}", status.coverage_s)]));
                    }
                    s
                }
                None => t.ft8_waiting.to_string(),
            };
            ui.label(RichText::new(text).size(12.0).color(MUTED));
            if status.delay_s != 0.0 {
                ui.label(RichText::new(fill(t.ft8_delay, &[&format!("{:+.2}", status.delay_s)])).size(12.0).monospace().color(MUTED))
                    .on_hover_text(t.ft8_delay_tooltip);
            }
            // Seconds left in the current slot.
            let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64());
            let left = p.slot_s - now % p.slot_s;
            ui.label(RichText::new(format!("{left:>4.1} s")).size(12.0).monospace().color(MUTED));
            if k == 0 || !self.ft[0].on {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button(t.ft8_clear).clicked() {
                        for d in &self.ft {
                            d.worker.clear();
                        }
                    }
                });
            }
        });
    }

    /// Decoded FT8 and FT4 messages together, newest at the bottom.
    fn ft8_panel(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        for k in 0..self.ft.len() {
            if self.ft[k].on {
                self.ft_status(ui, k);
            }
        }
        let mut messages: Vec<Ft8Message> = self.ft.iter().filter(|d| d.on).flat_map(|d| d.worker.messages()).collect();
        // By slot start; FT8 and FT4 slots interleave. Stable, so each slot keeps its order.
        messages.sort_by_key(|m| m.slot);
        let both = self.ft.iter().all(|d| d.on);
        egui::ScrollArea::vertical().max_height(ui.available_height()).auto_shrink([false, false]).stick_to_bottom(true).show(ui, |ui| {
            egui::Grid::new("ft8_messages").num_columns(if both { 6 } else { 5 }).spacing(vec2(14.0, 2.0)).striped(true).show(ui, |ui| {
                let mut heads = vec!["UTC", "dB", "DT", "Hz", t.ft8_message];
                if both {
                    heads.insert(0, t.ft_mode);
                }
                for h in heads {
                    ui.label(RichText::new(h).size(11.0).strong().color(MUTED));
                }
                ui.end_row();
                let latest = |mode: &str| messages.iter().rev().find(|m| m.mode == mode).map(|m| m.slot);
                let latest = [latest(FT8.name), latest(FT4.name)];
                for m in &messages {
                    let k = usize::from(m.mode == FT4.name);
                    let fresh = Some(m.slot) == latest[k];
                    let color = if m.text.starts_with("CQ ") { GOOD } else if fresh { TEXT } else { MUTED };
                    let mono = |s: String| RichText::new(s).size(13.0).monospace().color(color);
                    if both {
                        ui.label(RichText::new(m.mode).size(11.0).strong().color(ft_color(k)));
                    }
                    ui.label(mono(utc_hms(m.slot)));
                    ui.label(mono(format!("{:+3.0}", m.snr_db)));
                    ui.label(mono(format!("{:+.1}", m.dt_s)));
                    ui.label(mono(format!("{:4.0}", m.audio_hz)));
                    ui.label(mono(m.text.clone()));
                    ui.end_row();
                }
            });
        });
    }

    fn audio_card(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        let current = self
            .audio_device
            .as_ref()
            .and_then(|id| self.audio_devices.iter().find(|d| &d.id == id))
            .map_or(t.system_default.to_string(), |d| d.name.clone());
        let mut choice = self.audio_device.clone();
        ui.label(t.output);
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("audio_device").width(ui.available_width() - 36.0).selected_text(current).show_ui(ui, |ui| {
                ui.selectable_value(&mut choice, None, t.system_default);
                for d in &self.audio_devices {
                    ui.selectable_value(&mut choice, Some(d.id.clone()), &d.name);
                }
            });
            if ui.button("⟳").on_hover_text(t.refresh_audio).clicked() {
                self.audio_devices = audio::output_devices();
            }
        });
        if choice != self.audio_device {
            self.audio_device = choice;
            self.settings.audio_device = self.audio_device.clone();
            self.settings.save();
            if self.audio.is_some() || self.handle.is_some() {
                self.open_audio();
            }
        }
        ui.horizontal(|ui| {
            ui.label(t.volume);
            let slider = ui.add(egui::Slider::new(&mut self.volume_pct, 0.0..=100.0).step_by(1.0).suffix(" %"));
            if slider.changed() {
                self.apply_volume();
            }
            if slider.drag_stopped() || slider.lost_focus() {
                self.settings.volume = Some(self.volume_pct);
                self.settings.save();
            }
        });
        if ui.checkbox(&mut self.muted, t.mute).changed() {
            self.apply_volume();
        }
        if let Some(out) = &self.audio {
            ui.label(RichText::new(format!("{} · {} kHz", out.name, out.sample_rate as f64 / 1e3)).size(11.0).color(MUTED));
        }
        if let Some(e) = &self.audio_error {
            ui.label(RichText::new(fill(t.audio_error, &[e])).size(12.0).color(BAD));
        }
    }

    fn receiver_card(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        let running = self.handle.is_some();
        if self.discovery.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label(RichText::new(t.searching).color(MUTED));
            });
        } else if self.devices.is_empty() {
            ui.label(RichText::new(t.no_device).color(WARN));
            ui.label(RichText::new(t.no_device_help).size(12.0).color(MUTED));
        } else {
            let mut index = self.device_index;
            ui.add_enabled_ui(!running, |ui| {
                egui::ComboBox::from_id_salt("device")
                    .width(ui.available_width())
                    .selected_text(self.devices[index].label.clone())
                    .show_ui(ui, |ui| {
                        for (i, d) in self.devices.iter().enumerate() {
                            ui.selectable_value(&mut index, i, &d.label);
                        }
                    });
            });
            if index != self.device_index {
                self.select_device(index);
            }
            let range = format!("{}: {} – {}", t.range, freq::short(self.limits.min_hz), freq::short(self.limits.max_hz));
            ui.label(RichText::new(range).size(12.0).color(MUTED));
            if !self.driver_model.is_empty() {
                ui.label(RichText::new(&self.driver_model).size(11.0).color(MUTED));
            }
        }
        ui.add_enabled_ui(!running && self.discovery.is_none(), |ui| {
            if ui.button(t.search_again).clicked() {
                self.discover(ui.ctx());
            }
        });

        if self.device().is_some() {
            ui.add_space(6.0);
            ui.label(t.bandwidth).on_hover_text(t.bandwidth_tooltip);
            let mut rate = self.sample_rate;
            egui::ComboBox::from_id_salt("rate").width(ui.available_width()).selected_text(freq::rate(rate)).show_ui(ui, |ui| {
                for &r in &self.limits.rates {
                    let label = if r == self.limits.max_rate { format!("{}  ★", freq::rate(r)) } else { freq::rate(r) };
                    ui.selectable_value(&mut rate, r, label);
                }
            });
            if rate != self.sample_rate {
                self.set_sample_rate(rate);
            }

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label(t.gain);
                if self.limits.auto_gain && ui.checkbox(&mut self.auto_gain, t.auto_gain).changed() {
                    self.send(Command::Gain(self.gain()));
                }
            });
            let (lo, hi) = self.limits.gain;
            let slider = egui::Slider::new(&mut self.gain_db, lo..=hi).step_by(1.0).suffix(" dB");
            if ui.add_enabled(!self.auto_gain, slider).changed() {
                self.send(Command::Gain(self.gain()));
            }
        }

        ui.add_space(8.0);
        ui.label(RichText::new(t.network_device).size(12.0).color(MUTED));
        ui.horizontal(|ui| {
            let edit = egui::TextEdit::singleline(&mut self.network_entry).hint_text(t.network_hint).desired_width(ui.available_width() - 60.0);
            let response = ui.add_enabled(!running, edit);
            let enter = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if ui.add_enabled(!running, egui::Button::new(t.add)).clicked() || enter {
                self.add_network_device(ui.ctx());
            }
        });
    }

    fn frequency_card(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        ui.horizontal(|ui| {
            let edit = egui::TextEdit::singleline(&mut self.entry)
                .hint_text(t.freq_hint)
                .font(FontId::monospace(15.0))
                .desired_width(ui.available_width() - 56.0);
            let response = ui.add(edit).on_hover_text(t.freq_entry_tooltip);
            let enter = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if ui.button(RichText::new(t.tune).strong()).clicked() || enter {
                self.submit_entry();
            }
        });
        if let Some((note, color)) = &self.entry_note {
            ui.label(RichText::new(note).size(12.0).color(*color));
        }
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label(t.converter).on_hover_text(t.converter_tooltip);
            let presets = [(0.0, t.converter_none), (9_750e6, t.converter_qo100), (-125e6, t.converter_upconverter)];
            let current = presets.iter().find(|p| p.0 == self.lo_hz).map_or_else(|| freq::short(self.lo_hz.abs()), |p| p.1.to_string());
            let mut lo = self.lo_hz;
            egui::ComboBox::from_id_salt("converter").selected_text(current).show_ui(ui, |ui| {
                for (value, name) in presets {
                    ui.selectable_value(&mut lo, value, name);
                }
            });
            let mut mhz = lo / 1e6;
            if ui.add(egui::DragValue::new(&mut mhz).speed(1.0).suffix(" MHz").max_decimals(3)).changed() {
                lo = mhz * 1e6;
            }
            if lo != self.lo_hz {
                self.set_converter(lo);
            }
        });
        if self.lo_hz != 0.0 {
            ui.horizontal(|ui| {
                ui.label(t.correction).on_hover_text(t.correction_tooltip);
                let mut khz = self.correction_hz / 1e3;
                let locked = self.beacon_lock && self.lo_hz == QO100_LO_HZ;
                let edit = egui::DragValue::new(&mut khz).speed(0.01).suffix(" kHz").max_decimals(3);
                if ui.add_enabled(!locked, edit).changed() {
                    self.apply_correction(khz * 1e3);
                    self.settings.correction_hz = Some(self.correction_hz);
                    self.settings.save();
                }
            });
        }
        if self.lo_hz == QO100_LO_HZ {
            if ui.checkbox(&mut self.beacon_lock, t.beacon_lock).on_hover_text(t.beacon_tooltip).changed() {
                self.settings.beacon_lock = Some(self.beacon_lock);
                self.settings.save();
            }
            if let Some(b) = &self.beacon {
                let (text, color) = match b.lock().unwrap().status {
                    BeaconStatus::Searching => (t.beacon_searching.to_string(), MUTED),
                    BeaconStatus::Locked { snr_db } => (fill(t.beacon_locked, &[&format!("{snr_db:.0}")]), GOOD),
                    BeaconStatus::NotFound => (t.beacon_not_found.to_string(), WARN),
                    BeaconStatus::OutsideBand => (t.beacon_outside.to_string(), WARN),
                };
                ui.label(RichText::new(text).size(12.0).color(color));
            }
        }
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label(t.step);
            egui::ComboBox::from_id_salt("step").selected_text(freq::short(self.step_hz)).show_ui(ui, |ui| {
                for s in STEPS {
                    ui.selectable_value(&mut self.step_hz, s, freq::short(s));
                }
            });
            if ui.button(t.center).clicked() && self.retune(self.vfo_hz) {
                self.set_view(View { offset_hz: 0.0, span_hz: self.view.span_hz });
            }
        });
    }

    fn view_card(&mut self, ui: &mut egui::Ui) {
        let t = self.t();
        ui.horizontal(|ui| {
            ui.label(t.span);
            let full = self.view.span_hz >= self.sample_rate * 0.999;
            let label = if full { format!("{} ({})", t.full_span, freq::short(self.sample_rate)) } else { freq::short(self.view.span_hz) };
            let mut span = self.view.span_hz;
            egui::ComboBox::from_id_salt("span").selected_text(label).show_ui(ui, |ui| {
                let mut s = self.sample_rate;
                let mut first = true;
                while s >= MIN_SPAN {
                    let text = if first { format!("{} ({})", t.full_span, freq::short(s)) } else { freq::short(s) };
                    ui.selectable_value(&mut span, s, text);
                    s /= 2.0;
                    first = false;
                }
            });
            if span != self.view.span_hz {
                self.zoom_to(span, self.vfo_hz);
            }
        });
        ui.horizontal(|ui| {
            ui.label(t.threshold).on_hover_text(t.threshold_tooltip);
            if ui.add(egui::Slider::new(&mut self.threshold_db, 3.0..=25.0).step_by(1.0).suffix(" dB")).changed() {
                self.settings.threshold_db = Some(self.threshold_db);
                self.settings.save();
            }
        });
        if ui.checkbox(&mut self.find_names, t.find_names).on_hover_text(t.find_names_tooltip).changed() {
            self.stations.set_enabled(self.find_names);
            self.settings.find_names = Some(self.find_names);
            self.settings.save();
        }
        if let Some(f) = self.frame {
            ui.label(RichText::new(format!("{}: {}", t.resolution, freq::short(f.bin_hz.max(1.0)))).size(12.0).color(MUTED));
        }
    }

    /// Shows `span` Hz around `anchor` (kept at the same place when possible).
    fn zoom_to(&mut self, span: f64, anchor: f64) {
        let (lo, hi) = self.view_range();
        let k = span / self.view.span_hz;
        let anchor = anchor.clamp(lo, hi);
        let center = anchor - (anchor - (lo + hi) / 2.0) * k;
        self.set_view(View { offset_hz: center - self.center_hz, span_hz: span });
    }

    // ---- Central area ----

    fn central(&mut self, ui: &mut egui::Ui) {
        let area = ui.available_rect_before_wrap();
        ui.allocate_rect(area, Sense::hover());
        let strip = Rect::from_min_size(area.min, vec2(area.width(), 32.0));
        let spec = Rect::from_min_max(pos2(area.left(), strip.bottom() + 6.0), area.max);
        self.signal_strip(ui, strip);
        self.spectrum(ui, spec);
    }

    fn signal_strip(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let t = self.t();
        let mut picked = None;
        ui.scope_builder(egui::UiBuilder::new().max_rect(rect), |ui| {
            egui::ScrollArea::horizontal().auto_shrink([false, true]).show(ui, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(RichText::new(t.signals_caps).size(10.0).strong().color(MUTED));
                    if self.signals.is_empty() {
                        ui.label(RichText::new(t.signals_none).size(11.0).color(MUTED));
                    }
                    for s in &self.signals {
                        let selected = (self.vfo_hz - s.center_hz).abs() <= s.bandwidth_hz / 2.0;
                        let mut text = format!("{:.3}  ·  {}  ·  {:.0} dB", s.center_hz / 1e6, freq::width(s.bandwidth_hz), s.snr_db);
                        if let Some(name) = self.stations.name_near(s.center_hz - self.lo()) {
                            text = format!("{name}  ·  {text}");
                        }
                        let text = RichText::new(text).monospace().size(12.0);
                        let button = if selected {
                            egui::Button::new(text.color(DEEP).strong()).fill(AMBER)
                        } else {
                            egui::Button::new(text.color(TEXT))
                        };
                        if ui.add(button.corner_radius(12)).on_hover_text(t.signal_tooltip).clicked() {
                            picked = Some(s.center_hz);
                        }
                    }
                });
            });
        });
        if let Some(hz) = picked {
            let signal = self.signal_at(hz, 0.0);
            self.select_at(hz, signal);
        }
    }

    fn spectrum(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let tex = self
            .waterfall_tex
            .get_or_insert_with(|| ui.ctx().load_texture("waterfall", self.waterfall.clone(), TextureOptions::LINEAR))
            .clone();
        let response = ui.interact(rect, ui.id().with("spectrum"), Sense::click_and_drag()).on_hover_text_at_pointer(self.t().spectrum_help);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, CornerRadius::same(10), DEEP);

        let full = rect.shrink(1.0);
        let spec_rect = Rect::from_min_size(full.min, vec2(full.width(), full.height() * 0.45));
        let axis_h = 20.0;
        let wf_rect = Rect::from_min_max(pos2(full.left(), spec_rect.bottom() + axis_h), full.max);
        painter.image(tex.id(), wf_rect, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)), Color32::WHITE);

        // Everything is drawn against the band of the last frame.
        let (lo_hz, hi_hz) = self.frame.map_or_else(|| self.view_range(), |f| (f.lo_hz, f.hi_hz));
        let width_hz = (hi_hz - lo_hz).max(1.0);
        let x_of = |hz: f64| full.left() + ((hz - lo_hz) / width_hz) as f32 * full.width();
        let hz_of = |x: f32| lo_hz + ((x - full.left()) / full.width()) as f64 * width_hz;
        let hz_per_px = width_hz / full.width().max(1.0) as f64;

        // dB grid.
        let (lo, hi) = self.range;
        let y_of = |db: f32| spec_rect.bottom() - (db - lo) / (hi - lo) * spec_rect.height();
        let grid = Color32::from_rgb(26, 34, 46);
        let mut db = (lo / 10.0).ceil() * 10.0;
        while db < hi {
            let y = y_of(db);
            painter.line_segment([pos2(spec_rect.left(), y), pos2(spec_rect.right(), y)], Stroke::new(1.0, grid));
            painter.text(pos2(spec_rect.left() + 6.0, y - 1.0), Align2::LEFT_BOTTOM, format!("{db:.0} dB"), FontId::monospace(10.0), MUTED);
            db += 10.0;
        }

        // Frequency axis: a 1-2-5 step leaving at least ~100 pixels between ticks.
        let min_step = hz_per_px * 100.0;
        let step = nice_step(min_step);
        let axis = Rect::from_min_max(pos2(full.left(), spec_rect.bottom()), pos2(full.right(), wf_rect.top()));
        painter.rect_filled(axis, CornerRadius::ZERO, BG);
        let mut f = (lo_hz / step).ceil() * step;
        while f <= hi_hz {
            let x = x_of(f);
            painter.line_segment([pos2(x, spec_rect.top()), pos2(x, spec_rect.bottom())], Stroke::new(1.0, grid));
            painter.text(pos2(x, axis.center().y), Align2::CENTER_CENTER, axis_label(f, step), FontId::monospace(11.0), MUTED);
            f += step;
        }

        // Signals found.
        for s in &self.signals {
            let half = (s.bandwidth_hz / 2.0).max(hz_per_px);
            let band = Rect::from_x_y_ranges(x_of(s.center_hz - half)..=x_of(s.center_hz + half), spec_rect.y_range());
            painter.rect_filled(band, CornerRadius::ZERO, AMBER.gamma_multiply(0.07));
            let tick = Rect::from_x_y_ranges(band.x_range(), spec_rect.top()..=spec_rect.top() + 3.0);
            painter.rect_filled(tick, CornerRadius::ZERO, AMBER.gamma_multiply(0.6));
        }

        // Spectrum: gradient fill below, line on top.
        if self.smoothed.len() > 1 {
            let n = self.smoothed.len();
            let points: Vec<Pos2> = self
                .smoothed
                .iter()
                .enumerate()
                .map(|(i, &v)| {
                    let x = spec_rect.left() + (i as f32 + 0.5) / n as f32 * spec_rect.width();
                    pos2(x, y_of(v).clamp(spec_rect.top(), spec_rect.bottom()))
                })
                .collect();
            let mut mesh = Mesh::default();
            for p in &points {
                let k = 1.0 - (p.y - spec_rect.top()) / spec_rect.height();
                mesh.colored_vertex(*p, SPECTRUM.gamma_multiply(0.12 + 0.35 * k));
                mesh.colored_vertex(pos2(p.x, spec_rect.bottom()), SPECTRUM.gamma_multiply(0.02));
            }
            for i in 0..n as u32 - 1 {
                let a = 2 * i;
                mesh.add_triangle(a, a + 1, a + 2);
                mesh.add_triangle(a + 1, a + 3, a + 2);
            }
            painter.add(Shape::mesh(mesh));
            painter.add(Shape::line(points, Stroke::new(1.3, SPECTRUM)));
        }

        // Station names above their peaks, strongest first; a label that would overlap one
        // already drawn moves up a row, or is left out.
        let mut placed: Vec<Rect> = Vec::new();
        let mut named: Vec<(&Signal, f64, String)> = Vec::new();
        for s in &self.signals {
            let Some((station, name)) = self.stations.station_near(s.center_hz - self.lo()) else { continue };
            // One label per station, on its strongest signal.
            match named.iter_mut().find(|(_, f, _)| *f == station) {
                Some(entry) if entry.0.snr_db < s.snr_db => entry.0 = s,
                Some(_) => {}
                None => named.push((s, station, name)),
            }
        }
        named.sort_by(|a, b| b.0.snr_db.total_cmp(&a.0.snr_db));
        for (s, _, name) in named {
            let x = x_of(s.center_hz);
            if !(full.left()..=full.right()).contains(&x) {
                continue;
            }
            let galley = painter.layout_no_wrap(name, FontId::proportional(12.0), DEEP);
            let size = galley.size() + vec2(10.0, 4.0);
            let peak_y = y_of(s.peak_db).clamp(spec_rect.top() + 40.0, spec_rect.bottom());
            let spot = (0..3).map(|row| {
                let bottom = peak_y - 6.0 - row as f32 * (size.y + 3.0);
                Rect::from_min_size(pos2((x - size.x / 2.0).clamp(full.left() + 2.0, full.right() - size.x - 2.0), bottom - size.y), size)
            });
            if let Some(r) = spot.into_iter().find(|r| r.top() >= spec_rect.top() + 30.0 && !placed.iter().any(|p| p.expand(2.0).intersects(*r))) {
                painter.rect_filled(r, CornerRadius::same(4), AMBER.gamma_multiply(0.9));
                painter.galley(r.min + vec2(5.0, 2.0), galley, DEEP);
                placed.push(r);
            }
        }

        // The FT8/FT4 bands (audio 100–3100 Hz above the dial).
        for (k, d) in self.ft.iter().enumerate().filter(|(_, d)| d.on) {
            let (a, b) = (x_of(d.dial_hz + FT8_AUDIO_HZ.0), x_of(d.dial_hz + FT8_AUDIO_HZ.1));
            if b > full.left() && a < full.right() {
                let color = ft_color(k);
                let band = Rect::from_x_y_ranges(a.max(full.left())..=b.min(full.right()), spec_rect.bottom() - 6.0..=spec_rect.bottom());
                painter.rect_filled(band, CornerRadius::ZERO, color.gamma_multiply(0.7));
                painter.text(pos2(a.max(full.left()) + 3.0, spec_rect.bottom() - 8.0), Align2::LEFT_BOTTOM, d.name(), FontId::proportional(10.0), color);
            }
        }

        // Band the demodulator hears.
        let (plo, phi) = self.mode.passband(self.bandwidth_hz);
        let (ax, bx) = (x_of(self.vfo_hz + plo), x_of(self.vfo_hz + phi).max(x_of(self.vfo_hz + plo) + 2.0));
        let pass = Rect::from_x_y_ranges(ax..=bx, spec_rect.top()..=wf_rect.bottom());
        painter.rect_filled(pass, CornerRadius::ZERO, AMBER.gamma_multiply(0.13));
        for x in [ax, bx] {
            painter.line_segment([pos2(x, spec_rect.top()), pos2(x, spec_rect.bottom())], Stroke::new(1.0, AMBER.gamma_multiply(0.5)));
        }

        // Listening frequency.
        let vx = x_of(self.vfo_hz);
        if (full.left()..=full.right()).contains(&vx) {
            painter.line_segment([pos2(vx, spec_rect.top()), pos2(vx, wf_rect.bottom())], Stroke::new(1.5, AMBER));
            let galley = painter.layout_no_wrap(mhz_text(self.vfo_hz), FontId::monospace(11.0), DEEP);
            let x = (vx - galley.size().x / 2.0 - 6.0).clamp(full.left() + 2.0, full.right() - galley.size().x - 14.0);
            let tag = Rect::from_min_size(pos2(x, spec_rect.top() + 6.0), galley.size() + vec2(12.0, 4.0));
            painter.rect_filled(tag, CornerRadius::same(4), AMBER);
            painter.galley(tag.min + vec2(6.0, 2.0), galley, DEEP);
        }

        // Frequency under the mouse cursor.
        if let Some(pos) = response.hover_pos() {
            painter.line_segment([pos2(pos.x, spec_rect.top()), pos2(pos.x, wf_rect.bottom())], Stroke::new(1.0, Color32::from_white_alpha(60)));
            let label = format!("{} MHz", mhz_text(hz_of(pos.x)));
            let galley = painter.layout_no_wrap(label, FontId::monospace(12.0), TEXT);
            let at = pos2((pos.x + 10.0).min(spec_rect.right() - galley.size().x - 12.0), spec_rect.top() + 34.0);
            let bg = Rect::from_min_size(at - vec2(6.0, 3.0), galley.size() + vec2(12.0, 6.0));
            painter.rect_filled(bg, CornerRadius::same(4), Color32::from_black_alpha(190));
            painter.galley(at, galley, TEXT);
        }

        if self.handle.is_none() {
            return;
        }
        // Dragging moves the visible band: inside the received band while zoomed, by retuning otherwise.
        if response.dragged() && response.drag_delta().x != 0.0 {
            let shift = -(response.drag_delta().x as f64) * hz_per_px;
            let wanted = self.view.offset_hz + shift;
            let room = (self.sample_rate - self.view.span_hz) / 2.0;
            if wanted.abs() <= room {
                self.set_view(View { offset_hz: wanted, span_hz: self.view.span_hz });
            } else {
                let excess = wanted - wanted.clamp(-room, room);
                self.retune(self.center_hz + excess);
                self.set_view(View { offset_hz: wanted.clamp(-room, room), span_hz: self.view.span_hz });
            }
        }
        if response.hovered() {
            let zoom = ui.input(|i| i.zoom_delta());
            let ctrl = ui.input(|i| i.modifiers.command);
            if zoom != 1.0 {
                let anchor = response.hover_pos().map_or(self.vfo_hz, |p| hz_of(p.x));
                let span = (self.view.span_hz / zoom as f64).clamp(MIN_SPAN, self.sample_rate);
                self.zoom_to(span, anchor);
            } else if !ctrl {
                let (steps, shift) = wheel_steps(ui);
                if steps != 0 {
                    let step = if shift { self.step_hz * 10.0 } else { self.step_hz };
                    self.set_vfo(((self.vfo_hz / step).round() + steps as f64) * step);
                }
            }
        }
        if let Some(pos) = response.interact_pointer_pos() {
            if response.clicked() || response.double_clicked() {
                let hz = hz_of(pos.x);
                let signal = self.signal_at(hz, 3.0 * hz_per_px);
                self.select_at(hz, signal);
                if response.double_clicked() && self.retune(self.vfo_hz) {
                    self.set_view(View { offset_hz: 0.0, span_hz: self.view.span_hz });
                }
            }
        }
    }

    /// RDS of the station listened to: name, programme type, traffic flags, clock, radio text.
    fn rds_bar(&self, ui: &mut egui::Ui) {
        let t = self.t();
        ui.horizontal(|ui| {
            let synced = self.rds.as_ref().is_some_and(|r| r.synced);
            let badge = egui::Button::new(RichText::new("RDS").size(11.0).strong().color(DEEP))
                .fill(if synced { ACCENT } else { MUTED })
                .corner_radius(4)
                .sense(Sense::hover());
            ui.add(badge);
            let Some(r) = self.rds.as_ref().filter(|r| r.pi.is_some()) else {
                ui.label(RichText::new(t.rds_searching).size(12.0).color(MUTED));
                return;
            };
            ui.label(RichText::new(r.ps.trim()).size(20.0).monospace().strong().color(AMBER));
            if let Some(pi) = r.pi {
                ui.label(RichText::new(format!("PI {pi:04X}")).size(12.0).monospace().color(MUTED));
            }
            if let Some(pty) = r.pty.filter(|&p| p > 0) {
                ui.label(RichText::new(t.pty[pty as usize & 31]).size(13.0).color(TEXT));
            }
            let flag = |ui: &mut egui::Ui, on: bool, text: &str| {
                ui.label(RichText::new(text).size(11.0).strong().monospace().color(if on { WARN } else { MUTED.gamma_multiply(0.6) }))
                    .on_hover_text(t.traffic_tooltip);
            };
            flag(ui, r.tp, "TP");
            flag(ui, r.ta, "TA");
            if let Some(c) = r.clock {
                ui.label(RichText::new(format!("{} {}", t.clock, c.local_hm())).size(12.0).monospace().color(TEXT));
            }
            if !r.af.is_empty() {
                let list = r.af.iter().map(|f| format!("{:.1}", f / 1e6)).collect::<Vec<_>>().join("  ");
                ui.label(RichText::new(format!("AF {}", r.af.len())).size(12.0).monospace().color(MUTED)).on_hover_text(format!("{}\n{list}", t.af_tooltip));
            }
            ui.add_space(8.0);
            ui.label(RichText::new(&r.radio_text).size(14.0).color(TEXT));
        });
    }

    /// Text decoded from the CW signal listened to, with its speed and a key light.
    fn cw_bar(&self, ui: &mut egui::Ui) {
        let t = self.t();
        let info = self.listener.cw().unwrap_or_default();
        ui.horizontal(|ui| {
            let badge = egui::Button::new(RichText::new("CW").size(11.0).strong().color(DEEP))
                .fill(if info.signal { ACCENT } else { MUTED })
                .corner_radius(4)
                .sense(Sense::hover());
            ui.add(badge).on_hover_text(t.cw_tooltip);
            dot(ui, if info.key_down { AMBER } else { MUTED.gamma_multiply(0.4) });
            if info.signal {
                ui.label(RichText::new(format!("{:.0} WPM", info.wpm)).size(12.0).monospace().color(MUTED));
            }
            if info.text.is_empty() {
                ui.label(RichText::new(t.cw_waiting).size(12.0).color(MUTED));
            } else {
                // The latest text, right-aligned so new letters stay in view.
                let width = ui.available_width();
                let chars = (width / 9.5) as usize;
                let shown: String = info.text.chars().rev().take(chars).collect::<Vec<_>>().into_iter().rev().collect();
                ui.label(RichText::new(shown).size(16.0).monospace().color(TEXT));
            }
        });
    }

    /// Text decoded from the RTTY or PSK31 signal listened to.
    fn data_bar(&self, ui: &mut egui::Ui) {
        let t = self.t();
        let info = self.listener.data().unwrap_or_default();
        ui.horizontal(|ui| {
            let badge = egui::Button::new(RichText::new(self.mode.name()).size(11.0).strong().color(DEEP))
                .fill(if info.signal { ACCENT } else { MUTED })
                .corner_radius(4)
                .sense(Sense::hover());
            ui.add(badge).on_hover_text(if self.mode == Mode::Rtty { t.rtty_tooltip } else { t.psk_tooltip });
            if !info.detail.is_empty() {
                ui.label(RichText::new(&info.detail).size(12.0).monospace().color(MUTED));
            }
            if info.text.is_empty() {
                ui.label(RichText::new(t.data_waiting).size(12.0).color(MUTED));
            } else {
                let width = ui.available_width();
                let chars = (width / 9.5) as usize;
                let shown: String = info.text.chars().rev().take(chars).collect::<Vec<_>>().into_iter().rev().collect();
                ui.label(RichText::new(shown).size(16.0).monospace().color(TEXT));
            }
        });
    }

    fn status_bar(&self, ui: &mut egui::Ui) {
        let t = self.t();
        ui.horizontal(|ui| {
            let label = self.device().map_or("", |d| d.label.as_str());
            let (color, text) = if let Some(e) = &self.error {
                (BAD, fill(t.error, &[e]))
            } else if self.handle.is_none() {
                (MUTED, t.stopped.to_string())
            } else if !self.opened {
                (WARN, fill(t.opening, &[&label]))
            } else {
                let kind = self.device().map_or("", |d| d.kind.name());
                (GOOD, fill(t.running, &[&kind, &freq::rate(self.sample_rate), &freq::short(self.center_hz)]))
            };
            dot(ui, color);
            ui.label(RichText::new(text).size(12.0).color(if self.error.is_some() { BAD } else { MUTED }));
            if self.opened && self.stats.measured_rate > 0.0 {
                let percent = (self.stats.measured_rate / self.sample_rate * 100.0).min(100.0);
                let c = if percent >= 98.0 { GOOD } else if percent >= 90.0 { WARN } else { BAD };
                ui.add_space(12.0);
                ui.label(RichText::new(fill(t.stream, &[&freq::rate(self.stats.measured_rate), &format!("{percent:.0}")])).size(12.0).color(c))
                    .on_hover_text(t.stream_tooltip);
                let lost = self.stats.overflows + self.stats.dropped_blocks;
                if lost > 0 {
                    ui.label(RichText::new(fill(t.lost, &[&lost])).size(12.0).color(WARN));
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new("GPL-3.0").size(11.0).color(MUTED));
                ui.label(RichText::new("yAtölye").size(13.0).strong().color(AMBER));
                ui.label(RichText::new(format!("SeDiR {} ·", env!("CARGO_PKG_VERSION"))).size(12.0).color(MUTED));
            });
        });
    }
}

impl eframe::App for SedirApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        if !self.zoom_ready {
            // First start: a text size to suit the screen, once its size is known.
            let screen = ui.input(|i| {
                let v = i.viewport();
                v.monitor_size.zip(v.native_pixels_per_point).map(|(size, native)| size * i.pixels_per_point() / native)
            });
            if let Some(screen) = screen {
                ui.ctx().set_zoom_factor(default_zoom(screen.y));
                self.zoom_ready = true;
            } else if ui.ctx().cumulative_frame_nr() > 10 {
                self.zoom_ready = true;
            } else {
                ui.ctx().request_repaint();
            }
        } else {
            // Keeps the text size, whether set with the buttons or Ctrl +/−.
            let zoom = (ui.ctx().zoom_factor() * 100.0).round() / 100.0;
            if self.settings.zoom != Some(zoom) {
                self.settings.zoom = Some(zoom);
                self.settings.save();
            }
        }
        self.poll_discovery();
        self.poll_events();
        self.poll_classification();
        self.update_beacon_tracker();
        self.sync_listener();
        // The FT8/FT4 decoders work in the receiver's own frequencies.
        let lo = self.lo();
        for d in &self.ft {
            d.worker.set_dial((d.on && self.opened).then(|| d.dial_hz - lo));
        }
        if self.handle.is_some() {
            // Keeps the signal meter moving between spectrum frames.
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));
        }

        let bar = egui::Frame::new().fill(CARD).inner_margin(egui::Margin::symmetric(14, 8));
        egui::Panel::top("header").frame(bar).show(ui, |ui| self.header(ui));
        egui::Panel::bottom("status")
            .frame(egui::Frame::new().fill(DEEP).inner_margin(egui::Margin::symmetric(12, 4)))
            .show(ui, |ui| self.status_bar(ui));
        // The settings run the full height beside the spectrum. In two columns when the
        // window is wide, or when one column would not fit in its height and there is
        // room beside the spectrum; otherwise one column that scrolls.
        let side = egui::Frame::new().fill(BG).inner_margin(egui::Margin::same(10));
        let width = ui.available_width();
        let overflows = self.cards_height > ui.available_height() - 20.0;
        if width >= WIDE_WINDOW || (overflows && width >= TWO_COLUMNS_MIN) {
            let a = egui::Panel::left("sidebar_a").resizable(true).default_size(310.0).size_range(270.0..=440.0).frame(side)
                .show(ui, |ui| self.sidebar(ui, &[0, 1, 5]));
            let b = egui::Panel::left("sidebar_b").resizable(true).default_size(310.0).size_range(270.0..=440.0)
                .frame(side.inner_margin(egui::Margin { left: 0, ..egui::Margin::same(10) }))
                .show(ui, |ui| self.sidebar(ui, &[2, 3, 4]));
            self.cards_height = a.inner + b.inner;
        } else {
            self.cards_height = egui::Panel::left("sidebar").resizable(true).default_size(310.0).size_range(270.0..=440.0).frame(side)
                .show(ui, |ui| self.sidebar(ui, &[0, 1, 2, 3, 4, 5]))
                .inner;
        }
        self.rds = if self.mode == Mode::Wfm && self.opened { self.listener.rds() } else { None };
        if self.mode == Mode::Wfm && self.opened {
            egui::Panel::bottom("rds")
                .frame(egui::Frame::new().fill(CARD).inner_margin(egui::Margin::symmetric(14, 6)))
                .show(ui, |ui| self.rds_bar(ui));
        }
        if self.ft.iter().any(|d| d.on) && self.opened {
            // Drag its top edge to make it taller.
            egui::Panel::bottom("ft8")
                .resizable(true)
                .default_size(230.0)
                .size_range(110.0..=700.0)
                .frame(egui::Frame::new().fill(CARD).inner_margin(egui::Margin::symmetric(14, 6)))
                .show(ui, |ui| self.ft8_panel(ui));
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
        }
        if self.mode == Mode::Cw && self.opened {
            egui::Panel::bottom("cw")
                .frame(egui::Frame::new().fill(CARD).inner_margin(egui::Margin::symmetric(14, 6)))
                .show(ui, |ui| self.cw_bar(ui));
        }
        if self.mode.is_data() && self.opened {
            egui::Panel::bottom("data")
                .frame(egui::Frame::new().fill(CARD).inner_margin(egui::Margin::symmetric(14, 6)))
                .show(ui, |ui| self.data_bar(ui));
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(BG).inner_margin(egui::Margin::same(10)))
            .show(ui, |ui| self.central(ui));
    }
}

fn reason_text(r: Reason, t: &Strings) -> &'static str {
    let i = match r {
        Reason::Wide => 0,
        Reason::ConstantEnvelope => 1,
        Reason::CarrierAndSidebands => 2,
        Reason::VoiceAboveCarrier => 3,
        Reason::VoiceBelowCarrier => 4,
        Reason::NarrowCarrier => 5,
        Reason::Keyed => 6,
        Reason::BandPlan => 7,
        Reason::VoicePitch => 8,
    };
    t.reasons[i]
}

/// Horizontal meter of the listened signal's power, with the squelch level marked.
fn level_bar(ui: &mut egui::Ui, db: f32, squelch: Option<f32>) {
    let (lo, hi) = (-120.0f32, 0.0f32);
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), 14.0), Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, CornerRadius::same(4), DEEP);
    let k = ((db - lo) / (hi - lo)).clamp(0.0, 1.0);
    let open = squelch.is_none_or(|s| db >= s);
    let fill_rect = Rect::from_min_size(rect.min, vec2(rect.width() * k, rect.height()));
    painter.rect_filled(fill_rect, CornerRadius::same(4), if open { ACCENT } else { MUTED });
    if let Some(s) = squelch {
        let x = rect.left() + rect.width() * ((s - lo) / (hi - lo)).clamp(0.0, 1.0);
        painter.line_segment([pos2(x, rect.top()), pos2(x, rect.bottom())], Stroke::new(2.0, AMBER));
    }
    painter.text(rect.right_center() - vec2(4.0, 0.0), Align2::RIGHT_CENTER, format!("{db:.0} dBFS"), FontId::monospace(10.0), TEXT);
}

/// Text size for a screen `height` points tall (after the system's own display
/// scaling): egui's size up to full HD, larger on taller screens (1440 → 125 %),
/// never smaller, since laptops are already scaled up by the system.
fn default_zoom(height: f32) -> f32 {
    ((height / 1150.0).clamp(1.0, 1.5) * 20.0).round() / 20.0
}

/// Volume percent to a gain factor (squared, which sounds closer to even steps).
fn volume_gain(percent: f32) -> f32 {
    let v = percent / 100.0;
    v * v * 1.5
}

/// Badge and band colour of slot decoder `k` (FT8, FT4).
fn ft_color(k: usize) -> Color32 {
    if k == 0 { ACCENT } else { AMBER }
}

/// Unix seconds → "HHMMSS" (UTC), as FT8 programs show slot times.
fn utc_hms(secs: u64) -> String {
    let s = secs % 86_400;
    format!("{:02}{:02}{:02}", s / 3600, s / 60 % 60, s % 60)
}

/// 145_500_000 → "145.500 000".
fn mhz_text(hz: f64) -> String {
    let hz = hz.max(0.0).round() as u64;
    format!("{}.{:03} {:03}", hz / 1_000_000, hz / 1000 % 1000, hz % 1000)
}

/// Smallest 1-2-5 step (Hz) not below `min`.
fn nice_step(min: f64) -> f64 {
    let mut scale = 1.0;
    loop {
        for m in [1.0, 2.0, 5.0] {
            if m * scale >= min {
                return m * scale;
            }
        }
        scale *= 10.0;
    }
}

/// Axis label with as many decimals (in MHz) as the tick step needs.
fn axis_label(hz: f64, step: f64) -> String {
    let decimals = if step >= 1e6 { 0 } else if step >= 1e5 { 1 } else if step >= 1e4 { 2 } else if step >= 1e3 { 3 } else if step >= 100.0 { 4 } else { 6 };
    format!("{:.*}", decimals, hz / 1e6)
}

fn snr_color(snr: f32) -> Color32 {
    if snr >= 20.0 {
        GOOD
    } else if snr >= 10.0 {
        WARN
    } else {
        BAD
    }
}

/// Moves values by `shift` places towards lower indices (the band moved up by that many
/// points); places uncovered are taken from `fresh`.
fn shift_values(values: &mut [f32], shift: isize, fresh: &[f32]) {
    let n = values.len() as isize;
    let old = values.to_vec();
    for (i, v) in values.iter_mut().enumerate() {
        let src = i as isize + shift;
        *v = if (0..n).contains(&src) { old[src as usize] } else { fresh[i] };
    }
}

fn shift_row(row: &mut [Color32], shift: isize, empty: Color32) {
    let n = row.len() as isize;
    if shift.abs() >= n {
        row.fill(empty);
    } else if shift > 0 {
        row.copy_within(shift as usize.., 0);
        row[(n - shift) as usize..].fill(empty);
    } else if shift < 0 {
        let s = (-shift) as usize;
        row.copy_within(..(n as usize - s), s);
        row[..s].fill(empty);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_zoom_follows_screen_height() {
        // Laptop at 150 % (1080 → 720 points), full HD, 1440p / ultrawide, 4K at 100 %.
        assert_eq!(default_zoom(720.0), 1.0);
        assert_eq!(default_zoom(1080.0), 1.0);
        assert_eq!(default_zoom(1440.0), 1.25);
        assert_eq!(default_zoom(2160.0), 1.5);
    }

    #[test]
    fn formats_and_steps() {
        assert_eq!(mhz_text(145_500_000.0), "145.500 000");
        assert_eq!(mhz_text(7_074_250.0), "7.074 250");
        assert_eq!(nice_step(130_000.0), 200_000.0);
        assert_eq!(nice_step(0.7), 1.0);
        assert_eq!(axis_label(145.25e6, 2.5e5), "145.2");
    }

    #[test]
    fn shifting_keeps_history_aligned() {
        let mut row = [1, 2, 3, 4].map(|v| Color32::from_gray(v));
        shift_row(&mut row, 1, Color32::BLACK);
        assert_eq!(row, [2, 3, 4, 0].map(|v| Color32::from_gray(v)));
        let mut v = [1.0, 2.0, 3.0];
        shift_values(&mut v, -1, &[9.0, 9.0, 9.0]);
        assert_eq!(v, [9.0, 1.0, 2.0]);
    }
}
