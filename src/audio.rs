//! Sound output: the computer's playback devices through cpal (WASAPI on Windows,
//! ALSA/PulseAudio/PipeWire on Linux, Core Audio on macOS).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, SampleFormat, SizedSample};

/// Mono audio waiting for the sound card. The demodulator fills it, the sound card's
/// callback empties it.
#[derive(Clone, Default)]
pub struct AudioBuffer(Arc<Mutex<Queue>>);

#[derive(Default)]
struct Queue {
    samples: VecDeque<f32>,
    /// Playback waits until this many samples are queued (after start or an underrun),
    /// so that small timing jitter does not cause gaps.
    prime: usize,
    primed: bool,
}

impl AudioBuffer {
    pub fn push(&self, samples: &[f32]) {
        self.0.lock().unwrap().samples.extend(samples);
    }

    pub fn len(&self) -> usize {
        self.0.lock().unwrap().samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Drops the oldest samples down to `keep` (when too much has piled up).
    pub fn trim_to(&self, keep: usize) {
        let mut q = self.0.lock().unwrap();
        let excess = q.samples.len().saturating_sub(keep);
        q.samples.drain(..excess);
    }

    pub fn clear(&self) {
        let mut q = self.0.lock().unwrap();
        q.samples.clear();
        q.primed = false;
    }

    pub fn set_prime(&self, samples: usize) {
        self.0.lock().unwrap().prime = samples;
    }

    /// Fills `out`, with silence where there is nothing to play.
    pub(crate) fn pop_into(&self, out: &mut [f32]) {
        let mut q = self.0.lock().unwrap();
        if !q.primed {
            if q.samples.len() < q.prime.max(out.len()) {
                out.fill(0.0);
                return;
            }
            q.primed = true;
        }
        for v in out.iter_mut() {
            match q.samples.pop_front() {
                Some(s) => *v = s,
                None => {
                    *v = 0.0;
                    q.primed = false;
                }
            }
        }
    }
}

/// A playback device.
#[derive(Clone, Debug, PartialEq)]
pub struct OutputDevice {
    /// Stable identifier, kept in the settings.
    pub id: String,
    pub name: String,
}

/// Playback devices of the default sound system.
pub fn output_devices() -> Vec<OutputDevice> {
    let host = cpal::default_host();
    let Ok(devices) = host.output_devices() else { return Vec::new() };
    devices
        .filter_map(|d| {
            let id = d.id().ok()?.to_string();
            let name = d.description().map(|desc| desc.name().to_string()).unwrap_or_else(|_| d.to_string());
            Some(OutputDevice { id, name })
        })
        .collect()
}

/// Volume as a linear factor shared with the sound card's callback.
#[derive(Clone)]
pub struct Volume(Arc<AtomicU32>);

impl Volume {
    pub fn new(v: f32) -> Self {
        Self(Arc::new(AtomicU32::new(v.to_bits())))
    }

    pub fn set(&self, v: f32) {
        self.0.store(v.to_bits(), Ordering::Relaxed);
    }

    fn get(&self) -> f32 {
        f32::from_bits(self.0.load(Ordering::Relaxed))
    }
}

/// An open playback stream; playing stops when it is dropped.
pub struct AudioOutput {
    _stream: cpal::Stream,
    pub sample_rate: u32,
    pub name: String,
}

/// Opens `device_id` (or the system's default device) and plays from `buffer`.
pub fn open(device_id: Option<&str>, buffer: AudioBuffer, volume: Volume) -> Result<AudioOutput, String> {
    let host = cpal::default_host();
    let device = match device_id {
        Some(id) => {
            let id: cpal::DeviceId = id.parse().map_err(|e| format!("{e:?}"))?;
            host.device_by_id(&id).ok_or("the selected sound device is not available")?
        }
        None => host.default_output_device().ok_or("no sound output device")?,
    };
    let name = device.description().map(|d| d.name().to_string()).unwrap_or_else(|_| device.to_string());
    let supported = device.default_output_config().map_err(|e| e.to_string())?;
    let config = supported.config();
    let stream = match supported.sample_format() {
        SampleFormat::F32 => build::<f32>(&device, config.clone(), buffer, volume),
        SampleFormat::I16 => build::<i16>(&device, config.clone(), buffer, volume),
        SampleFormat::U16 => build::<u16>(&device, config.clone(), buffer, volume),
        SampleFormat::I32 => build::<i32>(&device, config.clone(), buffer, volume),
        other => return Err(format!("unsupported sample format {other:?}")),
    }?;
    stream.play().map_err(|e| e.to_string())?;
    Ok(AudioOutput { _stream: stream, sample_rate: config.sample_rate, name })
}

fn build<T: SizedSample + FromSample<f32>>(
    device: &cpal::Device,
    config: cpal::StreamConfig,
    buffer: AudioBuffer,
    volume: Volume,
) -> Result<cpal::Stream, String> {
    let channels = config.channels.max(1) as usize;
    let mut mono = Vec::new();
    device
        .build_output_stream(
            config,
            move |data: &mut [T], _| {
                mono.resize(data.len() / channels, 0.0);
                buffer.pop_into(&mut mono);
                let gain = volume.get();
                for (frame, &v) in data.chunks_exact_mut(channels).zip(&mono) {
                    let s = T::from_sample((v * gain).clamp(-1.0, 1.0));
                    frame.fill(s);
                }
            },
            |e| eprintln!("audio: {e}"),
            None,
        )
        .map_err(|e| e.to_string())
}

// ---- Input (an Icom transceiver's USB sound card carrying its 12 kHz IF) ----

/// A recording device.
#[derive(Clone, Debug, PartialEq)]
pub struct InputDevice {
    pub id: String,
    pub name: String,
}

/// Recording devices of the default sound system.
pub fn input_devices() -> Vec<InputDevice> {
    let host = cpal::default_host();
    let Ok(devices) = host.input_devices() else { return Vec::new() };
    devices
        .filter_map(|d| {
            let id = d.id().ok()?.to_string();
            let name = d.description().map(|desc| desc.name().to_string()).unwrap_or_else(|_| d.to_string());
            Some(InputDevice { id, name })
        })
        .collect()
}

/// Most input kept waiting (seconds); older samples are dropped.
const INPUT_KEEP_SECONDS: usize = 2;

#[derive(Default)]
struct InputQueue {
    samples: VecDeque<f32>,
    error: Option<String>,
    dropped: u64,
}

/// An open recording stream (first channel only); recording stops when it is dropped.
pub struct AudioInput {
    _stream: cpal::Stream,
    queue: Arc<(Mutex<InputQueue>, std::sync::Condvar)>,
    pub sample_rate: u32,
    pub name: String,
}

impl AudioInput {
    /// Moves what has been recorded into `out`, waiting up to `timeout` for something.
    pub fn read(&self, out: &mut Vec<f32>, timeout: std::time::Duration) -> Result<(), String> {
        let (lock, ready) = &*self.queue;
        let mut q = lock.lock().unwrap();
        if q.samples.is_empty() && q.error.is_none() {
            q = ready.wait_timeout(q, timeout).unwrap().0;
        }
        if let Some(e) = q.error.clone() {
            return Err(e);
        }
        out.extend(q.samples.drain(..));
        Ok(())
    }

    /// Samples dropped because nobody read them in time.
    pub fn dropped(&self) -> u64 {
        self.queue.0.lock().unwrap().dropped
    }
}

/// Opens recording device `device_id` at `rate` samples per second.
pub fn open_input(device_id: &str, rate: u32) -> Result<AudioInput, String> {
    let host = cpal::default_host();
    let id: cpal::DeviceId = device_id.parse().map_err(|e| format!("{e:?}"))?;
    let device = host.device_by_id(&id).ok_or("the transceiver's sound device is not available")?;
    let name = device.description().map(|d| d.name().to_string()).unwrap_or_else(|_| device.to_string());
    // Prefer formats with more resolution; any channel count (the first channel is used).
    let rank = |f: SampleFormat| match f {
        SampleFormat::F32 => 0,
        SampleFormat::I32 => 1,
        SampleFormat::I16 => 2,
        SampleFormat::U16 => 3,
        _ => 9,
    };
    let supported = device
        .supported_input_configs()
        .map_err(|e| e.to_string())?
        .filter(|c| rank(c.sample_format()) < 9)
        .filter_map(|c| c.try_with_sample_rate(rate))
        .min_by_key(|c| (rank(c.sample_format()), c.channels()))
        .ok_or_else(|| format!("{name}: {rate} Hz recording is not supported"))?;
    let config = supported.config();
    let queue = Arc::new((Mutex::new(InputQueue::default()), std::sync::Condvar::new()));
    let keep = rate as usize * INPUT_KEEP_SECONDS;
    let stream = match supported.sample_format() {
        SampleFormat::F32 => build_input::<f32>(&device, config, queue.clone(), keep),
        SampleFormat::I32 => build_input::<i32>(&device, config, queue.clone(), keep),
        SampleFormat::I16 => build_input::<i16>(&device, config, queue.clone(), keep),
        SampleFormat::U16 => build_input::<u16>(&device, config, queue.clone(), keep),
        other => return Err(format!("unsupported sample format {other:?}")),
    }?;
    stream.play().map_err(|e| e.to_string())?;
    Ok(AudioInput { _stream: stream, queue, sample_rate: rate, name })
}

fn build_input<T>(
    device: &cpal::Device,
    config: cpal::StreamConfig,
    queue: Arc<(Mutex<InputQueue>, std::sync::Condvar)>,
    keep: usize,
) -> Result<cpal::Stream, String>
where
    T: SizedSample,
    f32: FromSample<T>,
{
    let channels = config.channels.max(1) as usize;
    let errors = queue.clone();
    device
        .build_input_stream(
            config,
            move |data: &[T], _: &cpal::InputCallbackInfo| {
                let (lock, ready) = &*queue;
                let mut q = lock.lock().unwrap();
                q.samples.extend(data.chunks_exact(channels).map(|frame| f32::from_sample_(frame[0])));
                let excess = q.samples.len().saturating_sub(keep);
                if excess > 0 {
                    q.samples.drain(..excess);
                    q.dropped += excess as u64;
                }
                ready.notify_one();
            },
            move |e| {
                let (lock, ready) = &*errors;
                lock.lock().unwrap().error = Some(e.to_string());
                ready.notify_one();
            },
            None,
        )
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn buffer_waits_until_primed_and_reports_underrun() {
        let b = AudioBuffer::default();
        b.set_prime(4);
        b.push(&[0.1, 0.2]);
        let mut out = [9.0; 2];
        b.pop_into(&mut out);
        assert_eq!(out, [0.0, 0.0]);
        b.push(&[0.3, 0.4]);
        b.pop_into(&mut out);
        assert_eq!(out, [0.1, 0.2]);
        let mut out = [9.0; 3];
        b.pop_into(&mut out);
        assert_eq!(out, [0.3, 0.4, 0.0]);
        b.push(&[0.5]);
        let mut out = [9.0; 1];
        b.pop_into(&mut out);
        assert_eq!(out, [0.0], "after an underrun playback waits to prime again");
    }

    /// Lists playback devices: `cargo test --lib audio_devices -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn audio_devices() {
        for d in output_devices() {
            println!("{} | {}", d.name, d.id);
        }
    }
}
