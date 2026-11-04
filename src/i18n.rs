//! Interface texts in English and Turkish.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lang {
    En,
    Tr,
}

impl Lang {
    pub const ALL: [Lang; 2] = [Lang::En, Lang::Tr];

    pub fn code(self) -> &'static str {
        match self {
            Lang::En => "en",
            Lang::Tr => "tr",
        }
    }

    pub fn from_code(code: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|l| code.trim().eq_ignore_ascii_case(l.code()))
    }

    pub fn native_name(self) -> &'static str {
        match self {
            Lang::En => "English",
            Lang::Tr => "Türkçe",
        }
    }

    /// Turkish if the operating system's language is Turkish, otherwise English.
    pub fn from_system() -> Self {
        match sys_locale::get_locale() {
            Some(l) if l.to_ascii_lowercase().starts_with("tr") => Lang::Tr,
            _ => Lang::En,
        }
    }

    pub fn strings(self) -> &'static Strings {
        match self {
            Lang::En => &EN,
            Lang::Tr => &TR,
        }
    }
}

/// Replaces each `{}` in `template` with the next value.
pub fn fill(template: &str, values: &[&dyn std::fmt::Display]) -> String {
    let mut out = String::with_capacity(template.len() + 16);
    let mut parts = template.split("{}");
    if let Some(first) = parts.next() {
        out.push_str(first);
    }
    for (i, part) in parts.enumerate() {
        if let Some(v) = values.get(i) {
            out.push_str(&v.to_string());
        }
        out.push_str(part);
    }
    out
}

pub struct Strings {
    pub app_subtitle: &'static str,
    pub language: &'static str,
    pub listening_caps: &'static str,
    pub start: &'static str,
    pub stop: &'static str,
    pub card_receiver: &'static str,
    pub card_frequency: &'static str,
    pub card_view: &'static str,
    pub searching: &'static str,
    pub no_device: &'static str,
    pub no_device_help: &'static str,
    pub search_again: &'static str,
    pub network_device: &'static str,
    pub network_hint: &'static str,
    pub add: &'static str,
    pub range: &'static str,
    pub bandwidth: &'static str,
    pub bandwidth_tooltip: &'static str,
    pub gain: &'static str,
    pub auto_gain: &'static str,
    pub freq_hint: &'static str,
    pub tune: &'static str,
    pub freq_entry_tooltip: &'static str,
    pub bad_number: &'static str,
    pub too_low: &'static str,
    pub too_high: &'static str,
    pub weak_range: &'static str,
    pub step: &'static str,
    pub span: &'static str,
    pub full_span: &'static str,
    pub threshold: &'static str,
    pub threshold_tooltip: &'static str,
    pub resolution: &'static str,
    pub signals_caps: &'static str,
    pub signals_none: &'static str,
    pub signal_tooltip: &'static str,
    pub spectrum_help: &'static str,
    pub stopped: &'static str,
    pub opening: &'static str,
    pub running: &'static str,
    pub stream: &'static str,
    pub stream_tooltip: &'static str,
    pub lost: &'static str,
    pub error: &'static str,
    pub center: &'static str,
    pub snr_caps: &'static str,
    pub width_caps: &'static str,
    pub level_caps: &'static str,
    pub card_listen: &'static str,
    pub card_audio: &'static str,
    pub filter: &'static str,
    pub squelch: &'static str,
    pub squelch_tooltip: &'static str,
    pub output: &'static str,
    pub system_default: &'static str,
    pub volume: &'static str,
    pub mute: &'static str,
    pub refresh_audio: &'static str,
    pub audio_error: &'static str,
    pub find_names: &'static str,
    pub find_names_tooltip: &'static str,
    pub rds_searching: &'static str,
    pub clock: &'static str,
    pub traffic_tooltip: &'static str,
    pub af_tooltip: &'static str,
    /// Programme type names (RDS, European table).
    pub pty: [&'static str; 32],
    pub auto_mode: &'static str,
    pub auto_mode_tooltip: &'static str,
    pub recognising: &'static str,
    pub recognised: &'static str,
    pub not_recognised: &'static str,
    /// Texts for classify::Reason, in its order.
    pub reasons: [&'static str; 9],
    pub cw_waiting: &'static str,
    pub cw_tooltip: &'static str,
    pub converter: &'static str,
    pub converter_tooltip: &'static str,
    pub converter_none: &'static str,
    pub converter_qo100: &'static str,
    pub converter_upconverter: &'static str,
    pub correction: &'static str,
    pub correction_tooltip: &'static str,
    pub beacon_lock: &'static str,
    pub beacon_tooltip: &'static str,
    pub beacon_searching: &'static str,
    pub beacon_locked: &'static str,
    pub beacon_not_found: &'static str,
    pub beacon_outside: &'static str,
}

pub static EN: Strings = Strings {
    app_subtitle: "Wideband SDR receiver",
    language: "Language",
    listening_caps: "LISTENING FREQUENCY",
    start: "▶  Start",
    stop: "■  Stop",
    card_receiver: "RECEIVER",
    card_frequency: "FREQUENCY",
    card_view: "VIEW",
    searching: "Searching for receivers…",
    no_device: "No receiver found.",
    no_device_help: "Connect an RTL-SDR, HackRF, ADALM-Pluto or LibreSDR, or add a network receiver below.",
    search_again: "⟳  Search again",
    network_device: "Network receiver (Pluto / LibreSDR)",
    network_hint: "IP address, e.g. 192.168.1.47",
    add: "Add",
    range: "Range",
    bandwidth: "Bandwidth",
    bandwidth_tooltip: "Sample rate: the width of the band received at once. The highest rate the device carries without loss is chosen by default.",
    gain: "Gain",
    auto_gain: "Automatic",
    freq_hint: "e.g. 145.5  ·  7074k  ·  1.09G",
    tune: "Tune",
    freq_entry_tooltip: "A number without a unit is read as MHz. k, M, G and Hz units are accepted.",
    bad_number: "Could not read “{}” as a frequency.",
    too_low: "{} is below the range of this {} ({} – {}).",
    too_high: "{} is above the range of this {} ({} – {}).",
    weak_range: "The {} receives {}, but poorly below {}.",
    step: "Wheel step",
    span: "Visible band",
    full_span: "Full",
    threshold: "Signal threshold",
    threshold_tooltip: "How far above the noise floor a signal must rise to be listed.",
    resolution: "Resolution",
    signals_caps: "SIGNALS",
    signals_none: "none found in the visible band",
    signal_tooltip: "Select this signal",
    spectrum_help: "Click: select  ·  Double-click: centre  ·  Drag: move  ·  Wheel: step  ·  Ctrl+wheel: zoom",
    stopped: "Stopped",
    opening: "Opening {}…",
    running: "{} · {} at {}",
    stream: "Stream {} ({}%)",
    stream_tooltip: "Samples actually arriving from the device. Below 100% the connection or the computer cannot keep up: choose a narrower bandwidth.",
    lost: "lost {}×",
    error: "Error: {}",
    center: "Centre",
    snr_caps: "SNR",
    width_caps: "WIDTH",
    level_caps: "SIGNAL",
    card_listen: "LISTEN",
    card_audio: "SOUND",
    filter: "Filter",
    squelch: "Squelch",
    squelch_tooltip: "Silences the sound while the signal in the filter is below this level.",
    output: "Output",
    system_default: "System default",
    volume: "Volume",
    mute: "Mute",
    refresh_audio: "Refresh the list of sound devices",
    audio_error: "Sound: {}",
    find_names: "Find station names (RDS)",
    find_names_tooltip: "Reads the RDS name of the FM stations in view in the background and shows it on the spectrum.",
    rds_searching: "looking for RDS…",
    clock: "Time",
    traffic_tooltip: "TP: the station carries traffic news · TA: traffic announcement on air",
    af_tooltip: "Alternative frequencies of this station",
    pty: [
        "None", "News", "Current affairs", "Information", "Sport", "Education", "Drama", "Culture",
        "Science", "Varied", "Pop music", "Rock music", "Easy listening", "Light classical", "Serious classical", "Other music",
        "Weather", "Finance", "Children's programmes", "Social affairs", "Religion", "Phone-in", "Travel", "Leisure",
        "Jazz music", "Country music", "National music", "Oldies music", "Folk music", "Documentary", "Alarm test", "Alarm",
    ],
    auto_mode: "Automatic mode",
    auto_mode_tooltip: "When a signal is clicked, its kind is measured and the mode, filter and tuning are set to match.",
    recognising: "Recognising the signal…",
    recognised: "{} · {}% sure · {}",
    not_recognised: "No clear signal where clicked (empty, too weak, or out of the band); the mode is kept.",
    reasons: [
        "wide",
        "constant amplitude",
        "carrier with sidebands",
        "one-sided, carrier below",
        "one-sided, carrier above",
        "single carrier",
        "keyed on and off",
        "band plan",
        "tuned to the voice",
    ],
    cw_waiting: "waiting for a CW signal…",
    cw_tooltip: "Text decoded from the CW signal; the speed is learned from the sender. Narrow the filter (100–300 Hz) when signals are close together.",
    converter: "Converter LO",
    converter_tooltip: "Local oscillator of an LNB or converter in front of the receiver. Frequencies are shown and typed as radio frequencies; the receiver is tuned to radio frequency minus this.",
    converter_none: "None",
    converter_qo100: "QO-100 LNB (9750 MHz)",
    converter_upconverter: "HF up-converter (125 MHz)",
    correction: "Correction",
    correction_tooltip: "Error of the converter's oscillator. Set by hand, or measured on the beacon when the lock is on.",
    beacon_lock: "Lock to the beacon (10489.500)",
    beacon_tooltip: "Measures the QO-100 lower CW beacon every second and corrects the LNB's frequency error and drift. The signal being listened to stays; the frequencies shown move to their true values.",
    beacon_searching: "Looking for the beacon…",
    beacon_locked: "Locked · {} dB",
    beacon_not_found: "Beacon not found (too weak, or more than 15 kHz off?)",
    beacon_outside: "The beacon is outside the received band",
};

pub static TR: Strings = Strings {
    app_subtitle: "Geniş bant SDR alıcı",
    language: "Dil",
    listening_caps: "DİNLEME FREKANSI",
    start: "▶  Başlat",
    stop: "■  Durdur",
    card_receiver: "ALICI",
    card_frequency: "FREKANS",
    card_view: "GÖRÜNÜM",
    searching: "Alıcılar aranıyor…",
    no_device: "Alıcı bulunamadı.",
    no_device_help: "Bir RTL-SDR, HackRF, ADALM-Pluto veya LibreSDR bağlayın ya da aşağıdan ağ alıcısı ekleyin.",
    search_again: "⟳  Yeniden ara",
    network_device: "Ağ alıcısı (Pluto / LibreSDR)",
    network_hint: "IP adresi, ör. 192.168.1.47",
    add: "Ekle",
    range: "Aralık",
    bandwidth: "Bant genişliği",
    bandwidth_tooltip: "Örnekleme hızı: aynı anda alınan bandın genişliği. Varsayılan olarak cihazın kayıpsız taşıyabildiği en yüksek hız seçilir.",
    gain: "Kazanç",
    auto_gain: "Otomatik",
    freq_hint: "ör. 145.5  ·  7074k  ·  1.09G",
    tune: "Git",
    freq_entry_tooltip: "Birimsiz sayı MHz okunur. k, M, G ve Hz birimleri kabul edilir.",
    bad_number: "“{}” frekans olarak okunamadı.",
    too_low: "{}, bu {} alıcısının aralığının altında ({} – {}).",
    too_high: "{}, bu {} alıcısının aralığının üstünde ({} – {}).",
    weak_range: "{} {} frekansını alır, ama {} altında zayıftır.",
    step: "Tekerlek adımı",
    span: "Görünen bant",
    full_span: "Tamamı",
    threshold: "Sinyal eşiği",
    threshold_tooltip: "Bir sinyalin listelenmesi için gürültü tabanının ne kadar üstüne çıkması gerektiği.",
    resolution: "Çözünürlük",
    signals_caps: "SİNYALLER",
    signals_none: "görünen bantta bulunamadı",
    signal_tooltip: "Bu sinyali seç",
    spectrum_help: "Tıkla: seç  ·  Çift tıkla: ortala  ·  Sürükle: kaydır  ·  Tekerlek: adım  ·  Ctrl+tekerlek: yakınlaştır",
    stopped: "Durduruldu",
    opening: "{} açılıyor…",
    running: "{} · {}, {}",
    stream: "Akış {} (%{})",
    stream_tooltip: "Cihazdan gerçekten gelen örnekler. %100'ün altındaysa bağlantı ya da bilgisayar yetişemiyor: daha dar bir bant genişliği seçin.",
    lost: "{} kez kayıp",
    error: "Hata: {}",
    center: "Ortala",
    snr_caps: "SNR",
    width_caps: "GENİŞLİK",
    level_caps: "SİNYAL",
    card_listen: "DİNLEME",
    card_audio: "SES",
    filter: "Filtre",
    squelch: "Susturucu",
    squelch_tooltip: "Filtredeki sinyal bu düzeyin altındayken sesi keser.",
    output: "Çıkış",
    system_default: "Sistem varsayılanı",
    volume: "Ses düzeyi",
    mute: "Sessiz",
    refresh_audio: "Ses cihazı listesini yenile",
    audio_error: "Ses: {}",
    find_names: "İstasyon adlarını bul (RDS)",
    find_names_tooltip: "Görünen FM istasyonlarının RDS adını arka planda okur ve spektrumda gösterir.",
    rds_searching: "RDS aranıyor…",
    clock: "Saat",
    traffic_tooltip: "TP: istasyon trafik bilgisi verir · TA: şu an trafik duyurusu var",
    af_tooltip: "Bu istasyonun diğer frekansları",
    pty: [
        "Yok", "Haberler", "Güncel olaylar", "Bilgi", "Spor", "Eğitim", "Drama", "Kültür",
        "Bilim", "Çeşitli", "Pop müzik", "Rock müzik", "Hafif müzik", "Hafif klasik", "Klasik müzik", "Diğer müzik",
        "Hava durumu", "Ekonomi", "Çocuk programları", "Toplumsal konular", "Din", "Telefonla katılım", "Seyahat", "Boş zaman",
        "Caz", "Country", "Ulusal müzik", "Eski şarkılar", "Halk müziği", "Belgesel", "Alarm testi", "Alarm",
    ],
    auto_mode: "Otomatik tür",
    auto_mode_tooltip: "Bir sinyale tıklayınca türü ölçülür; tür, filtre ve frekans ona göre ayarlanır.",
    recognising: "Sinyal tanınıyor…",
    recognised: "{} · %{} emin · {}",
    not_recognised: "Tıklanan yerde belirgin sinyal yok (boş, çok zayıf ya da bant dışı); tür değiştirilmedi.",
    reasons: [
        "geniş",
        "sabit genlik",
        "taşıyıcı ve iki yan bant",
        "tek yanlı, taşıyıcı altta",
        "tek yanlı, taşıyıcı üstte",
        "tek taşıyıcı",
        "açılıp kapanıyor",
        "bant planı",
        "sese göre ayarlandı",
    ],
    cw_waiting: "CW sinyali bekleniyor…",
    cw_tooltip: "CW sinyalinden çözülen metin; hız gönderenden öğrenilir. Sinyaller birbirine yakınsa filtreyi daraltın (100–300 Hz).",
    converter: "Dönüştürücü LO",
    converter_tooltip: "Alıcının önündeki LNB ya da dönüştürücünün yerel osilatörü. Frekanslar gerçek radyo frekansı olarak gösterilir ve yazılır; alıcı, radyo frekansından bu değer çıkarılarak ayarlanır.",
    converter_none: "Yok",
    converter_qo100: "QO-100 LNB (9750 MHz)",
    converter_upconverter: "HF yukarı dönüştürücü (125 MHz)",
    correction: "Düzeltme",
    correction_tooltip: "Dönüştürücü osilatörünün hatası. Elle girilir ya da kilit açıkken beacon'da ölçülür.",
    beacon_lock: "Beacon'a kilitle (10489.500)",
    beacon_tooltip: "QO-100 alt CW beacon'ını her saniye ölçer, LNB'nin frekans hatasını ve kaymasını düzeltir. Dinlenen sinyal yerinde kalır; gösterilen frekanslar gerçek değerlerine kayar.",
    beacon_searching: "Beacon aranıyor…",
    beacon_locked: "Kilitli · {} dB",
    beacon_not_found: "Beacon bulunamadı (çok zayıf ya da 15 kHz'ten fazla kayık?)",
    beacon_outside: "Beacon alınan bandın dışında",
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fills_templates() {
        assert_eq!(fill("{} is below {} ({} – {}).", &[&"7 MHz", &"RTL-SDR", &1, &2]), "7 MHz is below RTL-SDR (1 – 2).");
        assert_eq!(fill("no values", &[]), "no values");
    }
}
