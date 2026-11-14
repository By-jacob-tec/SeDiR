//! FT8 transmission: a message as audio (8-GFSK, as WSJT-X's gen_ft8wave makes it), the
//! usual QSO messages and what to send next when a station answers.

use super::message::{pack_standard, pack_text};
use super::{FT8, SYMBOL_SECONDS, SYMBOLS, TONE_SPACING_HZ, ldpc};

/// Bandwidth-time product of FT8's Gaussian frequency smoothing.
const BT: f64 = 2.0;

/// The 79 tones of a message, or None when it cannot be sent (not a standard message
/// and longer than 13 characters of free text).
pub fn message_tones(text: &str) -> Option<Vec<usize>> {
    let text = text.trim().to_ascii_uppercase();
    let bits = pack_standard(&text).or_else(|| pack_text(&text))?;
    let word = ldpc::encode(&bits);
    Some(FT8.tones_of(&word))
}

/// erf(x), Abramowitz & Stegun 7.1.26 (error below 1.5e-7).
fn erf(x: f64) -> f64 {
    let sign = x.signum();
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.327_591_1 * x);
    let y =
        1.0 - (((((1.061_405_429 * t - 1.453_152_027) * t) + 1.421_413_741) * t - 0.284_496_736) * t + 0.254_829_592) * t * (-x * x).exp();
    sign * y
}

/// Frequency pulse of one symbol, `t` in symbols from its centre.
fn gfsk_pulse(t: f64) -> f64 {
    let c = std::f64::consts::PI * (2.0 / std::f64::consts::LN_2).sqrt();
    0.5 * (erf(c * BT * (t + 0.5)) - erf(c * BT * (t - 0.5)))
}

/// Audio of the tones at `rate`, tone 0 at `base_hz`, peak amplitude `amplitude`:
/// 12.64 s, with smoothed frequency steps and 20 ms ramps at both ends.
pub fn audio(tones: &[usize], base_hz: f64, rate: f64, amplitude: f32) -> Vec<f32> {
    let nsps = (SYMBOL_SECONDS * rate).round() as usize;
    let nsym = tones.len();
    let pulse: Vec<f64> = (0..3 * nsps).map(|i| gfsk_pulse((i as f64 - 1.5 * nsps as f64) / nsps as f64)).collect();
    let peak = std::f64::consts::TAU / nsps as f64;
    let mut dphi = vec![0.0f64; (nsym + 2) * nsps];
    for (j, &tone) in tones.iter().enumerate() {
        for (k, p) in pulse.iter().enumerate() {
            dphi[j * nsps + k] += peak * p * tone as f64;
        }
    }
    // Dummy symbols before and after, with the first and last tones.
    for k in 0..2 * nsps {
        dphi[k] += peak * tones[0] as f64 * pulse[nsps + k];
        dphi[nsym * nsps + k] += peak * tones[nsym - 1] as f64 * pulse[k];
    }
    let carrier = std::f64::consts::TAU * base_hz / rate;
    let mut phase = 0.0f64;
    let mut out = Vec::with_capacity(nsym * nsps);
    for d in &dphi[nsps..(nsym + 1) * nsps] {
        out.push(amplitude * phase.sin() as f32);
        phase = (phase + d + carrier) % std::f64::consts::TAU;
    }
    let ramp = nsps / 8;
    for i in 0..ramp {
        let g = (1.0 - (std::f64::consts::PI * i as f64 / ramp as f64).cos()) as f32 / 2.0;
        out[i] *= g;
        let n = out.len();
        out[n - 1 - i] *= g;
    }
    out
}

/// Width of a transmission above its base frequency (Hz).
pub const WIDTH_HZ: f64 = 8.0 * TONE_SPACING_HZ;
/// Length of a transmission (s).
pub const LENGTH_S: f64 = SYMBOLS as f64 * SYMBOL_SECONDS;

/// A signal report as FT8 sends it: "-05", "+03".
pub fn report(snr_db: f32) -> String {
    let r = (snr_db.round() as i32).clamp(-30, 49);
    format!("{r:+03}")
}

/// The six messages of a QSO, as WSJT-X numbers them (Tx1 … Tx6).
pub fn templates(my_call: &str, my_grid: &str, dx_call: &str, report_sent: &str) -> [String; 6] {
    let (me, dx) = (my_call.trim().to_ascii_uppercase(), dx_call.trim().to_ascii_uppercase());
    let grid: String = my_grid.trim().to_ascii_uppercase().chars().take(4).collect();
    let dx = if dx.is_empty() { "DX".to_string() } else { dx };
    [
        format!("{dx} {me} {grid}").trim().to_string(),
        format!("{dx} {me} {report_sent}"),
        format!("{dx} {me} R{report_sent}"),
        format!("{dx} {me} RR73"),
        format!("{dx} {me} 73"),
        format!("CQ {me} {grid}").trim().to_string(),
    ]
}

/// What to do after a message addressed to us from `dx`.
#[derive(Clone, Debug, PartialEq)]
pub struct Next {
    /// Template to send next (0 = Tx1 … 5 = Tx6), or None to stop transmitting.
    pub tx: Option<usize>,
    /// Their report to us, when the message had one.
    pub report_received: Option<String>,
    /// The QSO is complete (reports exchanged): time to log it.
    pub complete: bool,
}

/// Reads a decoded message. `Some((dx, next))` when it is addressed to `my_call`.
pub fn answer_to(text: &str, my_call: &str) -> Option<(String, Next)> {
    let me = my_call.trim().to_ascii_uppercase();
    let words: Vec<&str> = text.split_whitespace().collect();
    let (to, from, rest) = match words.as_slice() {
        [to, from] => (*to, *from, ""),
        [to, from, rest] => (*to, *from, *rest),
        _ => return None,
    };
    if me.is_empty() || to != me {
        return None;
    }
    let is_report = |s: &str| s.len() == 3 && (s.starts_with('-') || s.starts_with('+')) && s[1..].chars().all(|c| c.is_ascii_digit());
    let next = if is_report(rest) {
        Next { tx: Some(2), report_received: Some(rest.to_string()), complete: false }
    } else if let Some(r) = rest.strip_prefix('R').filter(|r| is_report(r)) {
        Next { tx: Some(3), report_received: Some(r.to_string()), complete: true }
    } else if rest == "RR73" || rest == "RRR" {
        Next { tx: Some(4), report_received: None, complete: true }
    } else if rest == "73" {
        Next { tx: None, report_received: None, complete: true }
    } else {
        // A grid (or nothing): they answered our CQ.
        Next { tx: Some(1), report_received: None, complete: false }
    };
    Some((from.to_string(), next))
}

/// Who sent a decoded message, when it can be told ("CQ X …", "Y X …").
pub fn sender(text: &str) -> Option<String> {
    let words: Vec<&str> = text.split_whitespace().collect();
    match words.as_slice() {
        ["CQ", modifier, call, _] if modifier.len() <= 4 && !modifier.chars().any(|c| c.is_ascii_digit()) => Some(call.to_string()),
        ["CQ", call, ..] => Some(call.to_string()),
        [_, call, ..] => Some(call.to_string()),
        _ => None,
    }
}

/// The 4-character grid at the end of "CQ X FN42" or "Y X FN42", if any.
pub fn grid_of(text: &str) -> Option<String> {
    let last = text.split_whitespace().last()?;
    let b = last.as_bytes();
    (b.len() == 4
        && b[0].is_ascii_uppercase()
        && b[1].is_ascii_uppercase()
        && b[2].is_ascii_digit()
        && b[3].is_ascii_digit()
        && last != "RR73")
        .then(|| last.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ft8::message::{CallBook, unpack};

    #[test]
    fn templates_can_all_be_sent() {
        let t = templates("ta2ai", "km37", "dl1abc", "-07");
        assert_eq!(t[0], "DL1ABC TA2AI KM37");
        assert_eq!(t[2], "DL1ABC TA2AI R-07");
        assert_eq!(t[5], "CQ TA2AI KM37");
        let mut book = CallBook::default();
        for m in &t {
            let bits = pack_standard(m).unwrap_or_else(|| panic!("{m}"));
            assert_eq!(unpack(&bits, &mut book).as_deref(), Some(m.as_str()));
            assert_eq!(message_tones(m).unwrap().len(), 79);
        }
        assert_eq!(report(-7.4), "-07");
        assert_eq!(report(3.0), "+03");
        assert!(message_tones("THIS IS FAR TOO LONG FOR FT8").is_none());
    }

    /// Every symbol of the audio is at the tone it should be.
    #[test]
    fn audio_has_the_right_tones() {
        let text = "CQ TA2AI KM37";
        let tones = message_tones(text).unwrap();
        let rate = 48_000.0;
        let base = 1500.0;
        let a = audio(&tones, base, rate, 0.5);
        let nsps = (SYMBOL_SECONDS * rate) as usize;
        assert_eq!(a.len(), 79 * nsps);
        assert!((LENGTH_S - 12.64).abs() < 1e-9);
        assert!(a.iter().all(|v| v.abs() <= 0.5 + 1e-6));
        for (s, &tone) in tones.iter().enumerate() {
            // Middle half of the symbol, where the frequency has settled.
            let part = &a[s * nsps + nsps / 4..s * nsps + 3 * nsps / 4];
            let power = |f: f64| {
                let (mut re, mut im) = (0.0, 0.0);
                for (i, &v) in part.iter().enumerate() {
                    let p = std::f64::consts::TAU * f * i as f64 / rate;
                    re += v as f64 * p.cos();
                    im += v as f64 * p.sin();
                }
                re * re + im * im
            };
            let best = (0..8).max_by(|&x, &y| power(base + x as f64 * 6.25).total_cmp(&power(base + y as f64 * 6.25))).unwrap();
            assert_eq!(best, tone, "symbol {s}");
        }
    }

    /// The audio, as received, is decoded by SeDiR's own FT8 decoder.
    #[test]
    fn audio_is_decoded() {
        use crate::ft8::decode::decode_slot;
        use num_complex::Complex;
        let p = &FT8;
        let text = "CQ TA2AI KM37";
        let a = audio(&message_tones(text).unwrap(), 1000.0, p.rate, 0.1);
        let lead = ((p.early_s + p.start_s) * p.rate) as usize;
        let total = ((p.early_s + 15.0) * p.rate) as usize;
        let mut x = vec![Complex::new(0.0f32, 0.0); total];
        for (i, v) in a.iter().enumerate() {
            x[lead + i] = Complex::new(*v, 0.0);
        }
        let found = decode_slot(p, &x, 200.0, 1400.0, &mut CallBook::default());
        assert!(found.iter().any(|d| d.text == text), "{:?}", found.iter().map(|d| &d.text).collect::<Vec<_>>());
    }

    #[test]
    fn answers_follow_the_qso() {
        let me = "TA2AI";
        assert_eq!(
            answer_to("TA2AI DL1ABC JO62", me).unwrap(),
            ("DL1ABC".into(), Next { tx: Some(1), report_received: None, complete: false })
        );
        assert_eq!(answer_to("TA2AI DL1ABC -12", me).unwrap().1.tx, Some(2));
        let (_, n) = answer_to("TA2AI DL1ABC R-09", me).unwrap();
        assert_eq!((n.tx, n.report_received.as_deref(), n.complete), (Some(3), Some("-09"), true));
        assert_eq!(answer_to("TA2AI DL1ABC RR73", me).unwrap().1.tx, Some(4));
        assert_eq!(answer_to("TA2AI DL1ABC 73", me).unwrap().1.tx, None);
        assert!(answer_to("TA1XYZ DL1ABC -12", me).is_none());
        assert!(answer_to("CQ DL1ABC JO62", me).is_none());
        assert_eq!(sender("CQ DL1ABC JO62").as_deref(), Some("DL1ABC"));
        assert_eq!(sender("CQ DX DL1ABC JO62").as_deref(), Some("DL1ABC"));
        assert_eq!(sender("TA2AI DL1ABC -12").as_deref(), Some("DL1ABC"));
        assert_eq!(grid_of("CQ DL1ABC JO62").as_deref(), Some("JO62"));
        assert_eq!(grid_of("TA2AI DL1ABC RR73"), None);
    }
}
