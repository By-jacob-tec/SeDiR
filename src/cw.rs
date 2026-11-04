//! Morse (CW) to text. The keyed carrier's envelope is followed with adaptive signal and
//! noise levels; mark and space lengths, measured against a dot length that adapts to the
//! sender's speed, become dots, dashes, letter and word gaps.

use num_complex::Complex;

/// Envelope steps (ms).
const STEP_MS: f64 = 1.0;
/// Text kept for display.
const MAX_TEXT: usize = 400;
/// Speed limits (dot length in ms): 60 to 5 words per minute.
const DOT_RANGE_MS: (f64, f64) = (20.0, 240.0);

const MORSE: &[(&str, char)] = &[
    (".-", 'A'), ("-...", 'B'), ("-.-.", 'C'), ("-..", 'D'), (".", 'E'), ("..-.", 'F'), ("--.", 'G'),
    ("....", 'H'), ("..", 'I'), (".---", 'J'), ("-.-", 'K'), (".-..", 'L'), ("--", 'M'), ("-.", 'N'),
    ("---", 'O'), (".--.", 'P'), ("--.-", 'Q'), (".-.", 'R'), ("...", 'S'), ("-", 'T'), ("..-", 'U'),
    ("...-", 'V'), (".--", 'W'), ("-..-", 'X'), ("-.--", 'Y'), ("--..", 'Z'),
    ("-----", '0'), (".----", '1'), ("..---", '2'), ("...--", '3'), ("....-", '4'),
    (".....", '5'), ("-....", '6'), ("--...", '7'), ("---..", '8'), ("----.", '9'),
    (".-.-.-", '.'), ("--..--", ','), ("..--..", '?'), ("-..-.", '/'), ("-...-", '='),
    (".-.-.", '+'), ("-....-", '-'), (".--.-.", '@'), ("---...", ':'), ("-.--.", '('), ("-.--.-", ')'),
    (".----.", '\''), ("-.-.--", '!'), ("...-.-", '#'),
    // Turkish letters.
    ("-.-..", 'Ç'), ("--.-.", 'Ğ'), ("---.", 'Ö'), ("----", 'Ş'), ("..--", 'Ü'),
];

fn lookup(code: &str) -> char {
    MORSE.iter().find(|(c, _)| *c == code).map_or('*', |&(_, ch)| ch)
}

/// Envelope steps over which the signal and noise levels are measured (2 s).
const LEVEL_WINDOW: usize = 2000;
/// The levels are recomputed every this many steps.
const LEVEL_EVERY: usize = 25;
/// Envelope steps needed before the first levels (0.1 s).
const LEVEL_FIRST: usize = 100;
/// Signal over noise (amplitude ratio of the 95th to the 20th percentile) needed to
/// decode: noise alone gives about 3, a readable CW signal far more.
const MIN_RATIO: f32 = 5.0;
/// Without a signal this long (ms), the speed estimate starts over.
const RESET_AFTER_MS: f64 = 1500.0;

pub struct CwDecoder {
    /// Input samples per envelope step.
    per_step: usize,
    count: usize,
    smooth: f32,
    alpha: f32,
    /// Recent envelope values (ring) for the level percentiles.
    history: Vec<f32>,
    position: usize,
    since_levels: usize,
    silent_ms: f64,
    high: f32,
    low: f32,
    key: bool,
    run_ms: f64,
    dot_ms: f64,
    symbol: String,
    /// The letter gap was handled for the current space.
    letter_done: bool,
    word_done: bool,
    text: String,
    /// Whether a usable signal is present (signal level well over the noise).
    pub signal: bool,
}

impl CwDecoder {
    /// `rate`: sample rate of the channel (complex baseband with the CW tone near 0 Hz).
    pub fn new(rate: f64) -> Self {
        Self {
            per_step: ((rate * STEP_MS / 1000.0) as usize).max(1),
            count: 0,
            smooth: 0.0,
            // ~2 ms envelope smoothing.
            alpha: (1.0 - (-1.0 / (rate * 0.002)).exp()) as f32,
            history: Vec::with_capacity(LEVEL_WINDOW),
            position: 0,
            since_levels: 0,
            silent_ms: 0.0,
            high: 0.0,
            low: 0.0,
            key: false,
            run_ms: 0.0,
            dot_ms: 60.0,
            symbol: String::new(),
            letter_done: true,
            word_done: true,
            text: String::new(),
            signal: false,
        }
    }

    pub fn text(&self) -> &str {
        &self.text
    }

    /// Words per minute (PARIS: one dot is 1200/wpm ms).
    pub fn wpm(&self) -> f64 {
        1200.0 / self.dot_ms
    }

    pub fn key_down(&self) -> bool {
        self.key
    }

    pub fn push(&mut self, channel: &[Complex<f32>]) {
        for z in channel {
            self.smooth += (z.norm() - self.smooth) * self.alpha;
            self.count += 1;
            if self.count == self.per_step {
                self.count = 0;
                self.step(self.smooth);
            }
        }
    }

    fn step(&mut self, v: f32) {
        // Levels from the last 2 s: the 20th percentile is the noise (the key is up at
        // least that share of the time), the 95th the keyed carrier.
        if self.history.len() < LEVEL_WINDOW {
            self.history.push(v);
        } else {
            self.history[self.position] = v;
            self.position = (self.position + 1) % LEVEL_WINDOW;
        }
        self.since_levels += 1;
        if self.since_levels >= LEVEL_EVERY && self.history.len() >= LEVEL_FIRST {
            self.since_levels = 0;
            let mut sorted = self.history.clone();
            sorted.sort_by(f32::total_cmp);
            self.low = sorted[sorted.len() / 5];
            self.high = sorted[sorted.len() * 95 / 100];
            // Hysteresis: a signal must clearly appear, and clearly fade, to switch.
            let ratio = self.high / self.low.max(1e-12);
            self.signal = if self.signal { ratio > MIN_RATIO * 0.7 } else { ratio > MIN_RATIO };
        }
        if self.signal {
            self.silent_ms = 0.0;
        } else {
            self.silent_ms += STEP_MS;
            if self.silent_ms > RESET_AFTER_MS {
                self.dot_ms = 60.0;
                self.symbol.clear();
            }
        }
        let span = self.high - self.low;
        let on = if self.key { v > self.low + 0.35 * span } else { v > self.low + 0.55 * span };
        let on = on && self.signal;

        if on != self.key {
            let length = self.run_ms;
            self.run_ms = 0.0;
            if self.key {
                self.mark(length);
            }
            self.key = on;
            if on {
                self.letter_done = false;
                self.word_done = false;
            }
        }
        self.run_ms += STEP_MS;
        if !self.key {
            // Gaps end letters (3 dots) and words (7 dots); halfway points are used.
            if !self.letter_done && self.run_ms > 2.0 * self.dot_ms {
                self.letter_done = true;
                self.end_letter();
            }
            if !self.word_done && self.run_ms > 5.0 * self.dot_ms {
                self.word_done = true;
                if !self.text.ends_with(' ') && !self.text.is_empty() {
                    self.emit(' ');
                }
            }
        }
    }

    fn mark(&mut self, length: f64) {
        // Shorter than a third of a dot, or than any hand or keyer sends: a noise spike.
        if length < self.dot_ms * 0.3 || length < 12.0 {
            return;
        }
        if length < 2.0 * self.dot_ms {
            self.symbol.push('.');
            self.dot_ms += (length - self.dot_ms) * 0.2;
        } else {
            self.symbol.push('-');
            self.dot_ms += (length / 3.0 - self.dot_ms) * 0.2;
        }
        self.dot_ms = self.dot_ms.clamp(DOT_RANGE_MS.0, DOT_RANGE_MS.1);
        if self.symbol.len() > 7 {
            // No character is this long: noise or a missed gap.
            self.symbol.clear();
            self.emit('*');
        }
    }

    fn end_letter(&mut self) {
        if !self.symbol.is_empty() {
            let ch = lookup(&self.symbol);
            self.symbol.clear();
            self.emit(ch);
        }
    }

    fn emit(&mut self, ch: char) {
        self.text.push(ch);
        if self.text.chars().count() > MAX_TEXT {
            let cut = self.text.char_indices().nth(self.text.chars().count() - MAX_TEXT).map_or(0, |(i, _)| i);
            self.text.drain(..cut);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Keying of `text` at `wpm` as on/off steps of one dot.
    pub fn keying(text: &str) -> Vec<bool> {
        let mut out = Vec::new();
        for (wi, word) in text.split(' ').enumerate() {
            if wi > 0 {
                out.extend([false; 6]); // 1 after the last element + 6 = 7
            }
            for (li, ch) in word.chars().enumerate() {
                if li > 0 {
                    out.extend([false; 2]); // 1 after the element + 2 = 3
                }
                let code = MORSE.iter().find(|(_, c)| *c == ch).unwrap().0;
                for el in code.chars() {
                    let n = if el == '.' { 1 } else { 3 };
                    out.extend(std::iter::repeat_n(true, n));
                    out.push(false);
                }
            }
        }
        out.extend([false; 10]);
        out
    }

    fn decode(text: &str, wpm: f64, noise: f32) -> String {
        let rate = 16_000.0;
        let dot = 1.2 / wpm;
        let keys = keying(text);
        let mut seed = 3u32;
        let mut rnd = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        let n = (keys.len() as f64 * dot * rate) as usize;
        // Noise through a ~500 Hz channel (moving average), as the CW filter passes it.
        let white: Vec<Complex<f32>> = (0..n + 32).map(|_| Complex::new(rnd(), rnd()) * noise).collect();
        let samples: Vec<Complex<f32>> = (0..n)
            .map(|i| {
                let t = i as f64 / rate;
                let on = keys[((t / dot) as usize).min(keys.len() - 1)];
                let tone = if on { Complex::from_polar(0.2, (2.0 * PI * 30.0 * t) as f32) } else { Complex::default() };
                tone + white[i..i + 32].iter().sum::<Complex<f32>>() / 32.0 * 5.6
            })
            .collect();
        let mut d = CwDecoder::new(rate);
        for chunk in samples.chunks(4000) {
            d.push(chunk);
        }
        println!("{wpm} wpm → {:?} (estimated {:.1} wpm)", d.text(), d.wpm());
        d.text().to_string()
    }

    #[test]
    fn decodes_at_several_speeds() {
        for wpm in [12.0, 20.0, 30.0] {
            let text = decode("CQ CQ DE TA4SO TA4SO K", wpm, 0.03);
            assert!(text.contains("DE TA4SO TA4SO K"), "{wpm} wpm: {text:?}");
        }
    }

    #[test]
    fn noise_alone_gives_no_text() {
        let rate = 16_000.0;
        let mut seed = 11u32;
        let mut rnd = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        // Noise through a ~500 Hz channel (moving average), as the CW filter passes it.
        let white: Vec<Complex<f32>> = (0..(rate * 8.0) as usize).map(|_| Complex::new(rnd(), rnd()) * 0.1).collect();
        let samples: Vec<Complex<f32>> = white.windows(32).map(|w| w.iter().sum::<Complex<f32>>() / 32.0).collect();
        let mut d = CwDecoder::new(rate);
        for chunk in samples.chunks(4000) {
            d.push(chunk);
        }
        println!("noise → {:?}", d.text());
        assert!(d.text().trim().len() <= 2, "{:?}", d.text());
    }

    #[test]
    fn weak_signal_still_decodes() {
        // Peak signal about 4.5 times the noise's rms in the channel (~13 dB).
        let text = decode("TEST DE TA4SO K", 18.0, 0.15);
        // A letter may be lost at this level; the call sign should come through.
        assert!(text.contains("TA4SO K"), "{text:?}");
    }

    #[test]
    fn decodes_turkish_letters_and_digits() {
        // The first letter may be lost while the levels settle, as in any decoder.
        let text = decode("VVV ŞÜÇ 73 QRZ?", 18.0, 0.02);
        assert!(text.contains("ŞÜÇ"), "{text:?}");
        assert!(text.contains("73 QRZ?"), "{text:?}");
    }
}
