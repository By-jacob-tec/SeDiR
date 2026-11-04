//! Settings kept between runs, as "key=value" lines in the user's configuration folder
//! (Windows: %APPDATA%\SeDiR\settings.txt, elsewhere ~/.config/sedir/settings.txt).

use std::path::PathBuf;

use crate::i18n::Lang;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Settings {
    pub lang: Option<Lang>,
    /// Arguments of the last device used, to pick it again.
    pub device: Option<String>,
    pub frequency_hz: Option<f64>,
    pub network_devices: Vec<String>,
    pub threshold_db: Option<f32>,
    /// Demodulation mode name ("NFM", …).
    pub mode: Option<String>,
    /// Playback device id; None for the system default.
    pub audio_device: Option<String>,
    /// Volume in percent.
    pub volume: Option<f32>,
    /// Whether station names are read from RDS in the background.
    pub find_names: Option<bool>,
    /// Whether clicking a signal recognises its mode.
    pub auto_mode: Option<bool>,
    /// Converter local oscillator (Hz), e.g. 9750 MHz for a QO-100 LNB.
    pub lo_hz: Option<f64>,
}

fn path() -> Option<PathBuf> {
    let dir = if cfg!(windows) {
        PathBuf::from(std::env::var_os("APPDATA")?).join("SeDiR")
    } else if let Some(xdg) = std::env::var_os("XDG_CONFIG_HOME") {
        PathBuf::from(xdg).join("sedir")
    } else {
        PathBuf::from(std::env::var_os("HOME")?).join(".config").join("sedir")
    };
    Some(dir.join("settings.txt"))
}

impl Settings {
    pub fn parse(text: &str) -> Self {
        let mut s = Settings::default();
        for line in text.lines() {
            let Some((key, value)) = line.split_once('=') else { continue };
            let value = value.trim();
            match key.trim() {
                "lang" => s.lang = Lang::from_code(value),
                "device" if !value.is_empty() => s.device = Some(value.to_string()),
                "frequency_hz" => s.frequency_hz = value.parse().ok(),
                "network_device" if !value.is_empty() => s.network_devices.push(value.to_string()),
                "threshold_db" => s.threshold_db = value.parse().ok(),
                "mode" if !value.is_empty() => s.mode = Some(value.to_string()),
                "audio_device" if !value.is_empty() => s.audio_device = Some(value.to_string()),
                "volume" => s.volume = value.parse().ok(),
                "find_names" => s.find_names = value.parse().ok(),
                "auto_mode" => s.auto_mode = value.parse().ok(),
                "lo_hz" => s.lo_hz = value.parse().ok(),
                _ => {}
            }
        }
        s
    }

    pub fn to_text(&self) -> String {
        let mut out = String::new();
        if let Some(l) = self.lang {
            out += &format!("lang={}\n", l.code());
        }
        if let Some(d) = &self.device {
            out += &format!("device={d}\n");
        }
        if let Some(f) = self.frequency_hz {
            out += &format!("frequency_hz={f}\n");
        }
        for n in &self.network_devices {
            out += &format!("network_device={n}\n");
        }
        if let Some(t) = self.threshold_db {
            out += &format!("threshold_db={t}\n");
        }
        if let Some(m) = &self.mode {
            out += &format!("mode={m}\n");
        }
        if let Some(a) = &self.audio_device {
            out += &format!("audio_device={a}\n");
        }
        if let Some(v) = self.volume {
            out += &format!("volume={v}\n");
        }
        if let Some(f) = self.find_names {
            out += &format!("find_names={f}\n");
        }
        if let Some(a) = self.auto_mode {
            out += &format!("auto_mode={a}\n");
        }
        if let Some(lo) = self.lo_hz {
            out += &format!("lo_hz={lo}\n");
        }
        out
    }

    pub fn load() -> Self {
        path().and_then(|p| std::fs::read_to_string(p).ok()).map(|t| Self::parse(&t)).unwrap_or_default()
    }

    /// Failures are ignored: losing the settings is not worth interrupting the user.
    pub fn save(&self) {
        if let Some(p) = path() {
            if let Some(dir) = p.parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            let _ = std::fs::write(p, self.to_text());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let s = Settings {
            lang: Some(Lang::Tr),
            device: Some("driver=rtlsdr,serial=00000001".into()),
            frequency_hz: Some(145_500_000.0),
            network_devices: vec!["192.168.1.47".into(), "ip:pluto.local".into()],
            threshold_db: Some(8.0),
            mode: Some("NFM".into()),
            audio_device: Some("wasapi:{0.0.0.00000000}.{abc}".into()),
            volume: Some(70.0),
            find_names: Some(false),
            auto_mode: Some(true),
            lo_hz: Some(9_750e6),
        };
        assert_eq!(Settings::parse(&s.to_text()), s);
    }
}
