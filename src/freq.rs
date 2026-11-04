//! Reading and writing frequencies as text.

/// Reads a frequency typed by the user and returns it in Hz.
///
/// A unit suffix (Hz, k/kHz, M/MHz, G/GHz, any case) is optional; without one the
/// number is taken as MHz, the unit used on the display. A comma is accepted as the
/// decimal separator, spaces and underscores are ignored as digit grouping.
/// Examples: "145.5" → 145.5 MHz, "7074k" → 7.074 MHz, "1.2 GHz", "446 006 250 Hz".
pub fn parse(text: &str) -> Option<f64> {
    let cleaned: String = text.chars().filter(|c| !c.is_whitespace() && *c != '_').collect::<String>().replace(',', ".");
    let lower = cleaned.to_ascii_lowercase();
    let (number, scale) = [("ghz", 1e9), ("mhz", 1e6), ("khz", 1e3), ("hz", 1.0), ("g", 1e9), ("m", 1e6), ("k", 1e3)]
        .iter()
        .find_map(|&(unit, scale)| lower.strip_suffix(unit).map(|n| (n, scale)))
        .unwrap_or((lower.as_str(), 1e6));
    let value: f64 = number.parse().ok()?;
    (value.is_finite() && value >= 0.0).then_some(value * scale)
}

/// Frequency with a sensible unit and no trailing zeros, e.g. 1_766_000_000 → "1.766 GHz",
/// 24_000_000 → "24 MHz", 500_000 → "500 kHz".
pub fn short(hz: f64) -> String {
    let (value, unit) = if hz >= 1e9 {
        (hz / 1e9, "GHz")
    } else if hz >= 1e6 {
        (hz / 1e6, "MHz")
    } else if hz >= 1e3 {
        (hz / 1e3, "kHz")
    } else {
        (hz, "Hz")
    };
    let text = format!("{value:.6}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    format!("{text} {unit}")
}

/// Sample rate or bandwidth, e.g. 2_400_000 → "2.4 MS/s".
pub fn rate(sps: f64) -> String {
    if sps >= 1e6 {
        format!("{} MS/s", trim(sps / 1e6, 3))
    } else {
        format!("{} kS/s", trim(sps / 1e3, 1))
    }
}

fn trim(value: f64, decimals: usize) -> String {
    let text = format!("{value:.decimals$}");
    if text.contains('.') { text.trim_end_matches('0').trim_end_matches('.').to_string() } else { text }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_units_and_separators() {
        assert_eq!(parse("145.5"), Some(145.5e6));
        assert_eq!(parse("145,5"), Some(145.5e6));
        assert_eq!(parse("7074k"), Some(7.074e6));
        assert_eq!(parse("7074 kHz"), Some(7.074e6));
        assert_eq!(parse("1.2GHz"), Some(1.2e9));
        assert_eq!(parse("446 006 250 Hz"), Some(446_006_250.0));
        assert_eq!(parse("10 m"), Some(10e6));
        assert_eq!(parse(""), None);
        assert_eq!(parse("abc"), None);
        assert_eq!(parse("-5"), None);
    }

    #[test]
    fn short_form() {
        assert_eq!(short(1_766e6), "1.766 GHz");
        assert_eq!(short(24e6), "24 MHz");
        assert_eq!(short(500e3), "500 kHz");
        assert_eq!(rate(2.4e6), "2.4 MS/s");
        assert_eq!(rate(250e3), "250 kS/s");
    }
}
