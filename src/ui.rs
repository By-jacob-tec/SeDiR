//! Look of the SeDiR window: colours, theme and small widgets.

use eframe::egui::{self, Color32, CornerRadius, FontId, RichText, Sense, Stroke, Vec2, text::LayoutJob, vec2};

/// Colour palette: dark navy surfaces, cedar green accent, amber highlights.
pub mod palette {
    use eframe::egui::Color32;
    pub const BG: Color32 = Color32::from_rgb(11, 15, 22);
    pub const DEEP: Color32 = Color32::from_rgb(7, 10, 15);
    pub const CARD: Color32 = Color32::from_rgb(19, 25, 35);
    pub const BORDER: Color32 = Color32::from_rgb(34, 43, 57);
    pub const TEXT: Color32 = Color32::from_rgb(214, 222, 235);
    pub const MUTED: Color32 = Color32::from_rgb(128, 141, 160);
    pub const ACCENT: Color32 = Color32::from_rgb(95, 196, 128);
    pub const GOOD: Color32 = Color32::from_rgb(74, 222, 128);
    pub const WARN: Color32 = Color32::from_rgb(251, 191, 36);
    pub const BAD: Color32 = Color32::from_rgb(248, 113, 113);
    pub const AMBER: Color32 = Color32::from_rgb(240, 166, 58);
    pub const SPECTRUM: Color32 = Color32::from_rgb(56, 200, 240);
}
use palette::*;

pub fn apply_theme(ctx: &egui::Context) {
    ctx.all_styles_mut(|style| {
        let mut v = egui::Visuals::dark();
        v.panel_fill = BG;
        v.window_fill = CARD;
        v.extreme_bg_color = DEEP;
        v.faint_bg_color = CARD;
        v.window_corner_radius = CornerRadius::same(10);
        v.window_stroke = Stroke::new(1.0, BORDER);
        v.selection.bg_fill = ACCENT.gamma_multiply(0.35);
        v.selection.stroke = Stroke::new(1.0, ACCENT);
        v.hyperlink_color = ACCENT;
        let radius = CornerRadius::same(6);
        v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, BORDER);
        v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
        v.widgets.noninteractive.corner_radius = radius;
        for (w, fill) in [
            (&mut v.widgets.inactive, Color32::from_rgb(28, 36, 49)),
            (&mut v.widgets.hovered, Color32::from_rgb(38, 50, 67)),
            (&mut v.widgets.active, Color32::from_rgb(48, 64, 86)),
            (&mut v.widgets.open, Color32::from_rgb(38, 50, 67)),
        ] {
            w.bg_fill = fill;
            w.weak_bg_fill = fill;
            w.corner_radius = radius;
        }
        v.widgets.inactive.fg_stroke = Stroke::new(1.0, TEXT);
        v.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT.gamma_multiply(0.6));
        v.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
        style.visuals = v;
        style.spacing.item_spacing = vec2(8.0, 6.0);
        style.spacing.button_padding = vec2(10.0, 5.0);
        style.spacing.interact_size.y = 24.0;
    });
}

/// The program name with S, D and R emphasised.
pub fn brand(ui: &mut egui::Ui, size: f32) {
    let mut job = LayoutJob::default();
    for c in "SeDiR".chars() {
        let big = c.is_ascii_uppercase();
        let format = egui::TextFormat {
            font_id: FontId::proportional(if big { size } else { size * 0.72 }),
            color: if big { TEXT } else { ACCENT },
            valign: egui::Align::BOTTOM,
            ..Default::default()
        };
        job.append(&c.to_string(), 0.0, format);
    }
    ui.label(job);
}

/// A titled, bordered card.
pub fn card(ui: &mut egui::Ui, title: &str, add: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(CARD)
        .stroke(Stroke::new(1.0, BORDER))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(egui::Margin::same(12))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(title).size(11.0).strong().color(ACCENT));
            ui.add_space(6.0);
            add(ui);
        });
    ui.add_space(10.0);
}

/// Display text of a frequency: "145.500 000" (MHz, kHz and Hz groups) with the place
/// value (Hz) of each digit, None for separators.
fn tuner_digits(hz: f64) -> Vec<(char, Option<f64>)> {
    let digits = format!("{:07}", hz.max(0.0).round() as u64);
    let count = digits.len();
    let mut out = Vec::new();
    for (k, c) in digits.chars().enumerate() {
        let place = count - 1 - k;
        out.push((c, Some(10f64.powi(place as i32))));
        if place == 6 {
            out.push(('.', None));
        } else if place > 0 && place % 3 == 0 {
            out.push((' ', None));
        }
    }
    out
}

/// Frequency display tunable digit by digit, down to 1 Hz. The mouse wheel over a digit
/// changes it by that digit's place; clicking its upper half increases it, the lower half
/// decreases it. Returns true when the value changed.
pub fn freq_tuner(ui: &mut egui::Ui, hz: &mut f64, size: f32) -> bool {
    let font = FontId::monospace(size);
    let mut delta = 0.0;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        for (c, step) in tuner_digits(*hz) {
            let plain = ui.painter().layout_no_wrap(c.to_string(), font.clone(), TEXT);
            let sense = if step.is_some() { Sense::click() } else { Sense::hover() };
            let (rect, response) = ui.allocate_exact_size(plain.size(), sense);
            let mut galley = plain;
            if let (Some(step), true) = (step, response.hovered()) {
                ui.painter().rect_filled(rect.expand(1.0), CornerRadius::same(3), ACCENT.gamma_multiply(0.22));
                galley = ui.painter().layout_no_wrap(c.to_string(), font.clone(), ACCENT);
                let (steps, _) = wheel_steps(ui);
                delta += steps as f64 * step;
                if let Some(pos) = response.clicked().then(|| response.interact_pointer_pos()).flatten() {
                    delta += if pos.y < rect.center().y { step } else { -step };
                }
            }
            ui.painter().galley(rect.min, galley, TEXT);
        }
        ui.label(RichText::new(" MHz").size(size * 0.55).color(MUTED));
    });
    if delta == 0.0 {
        return false;
    }
    *hz = (*hz + delta).max(0.0).round();
    true
}

/// Mouse wheel steps in this frame (up is positive) and whether Shift is held. The
/// scroll used here is consumed so it does not reach surrounding scroll areas.
pub fn wheel_steps(ui: &egui::Ui) -> (i32, bool) {
    let (steps, shift) = ui.input(|i| {
        let steps: i32 = i
            .events
            .iter()
            .filter_map(|e| match e {
                egui::Event::MouseWheel { delta, .. } => {
                    let d = delta.x + delta.y;
                    (d != 0.0).then_some(d.signum() as i32)
                }
                _ => None,
            })
            .sum();
        (steps, i.modifiers.shift)
    });
    if steps != 0 {
        ui.ctx().input_mut(|i| i.smooth_scroll_delta = Vec2::ZERO);
    }
    (steps, shift)
}

/// Small coloured status dot.
pub fn dot(ui: &mut egui::Ui, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(vec2(10.0, 10.0), Sense::hover());
    ui.painter().circle_filled(rect.center(), 4.0, color);
}

/// Small labelled measurement tile.
pub fn metric(ui: &mut egui::Ui, label: &str, value: &str, color: Color32) {
    egui::Frame::new().fill(DEEP).corner_radius(CornerRadius::same(6)).inner_margin(egui::Margin::symmetric(8, 5)).show(ui, |ui| {
        ui.set_min_width(96.0);
        ui.vertical(|ui| {
            ui.label(RichText::new(label).size(10.0).color(MUTED));
            ui.label(RichText::new(value).size(14.0).monospace().color(color));
        });
    });
}

/// Maps a value in 0..1 to a colour from dark blue through cyan and green to yellow and red.
pub fn colormap(t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    const STOPS: [(f32, [f32; 3]); 7] = [
        (0.0, [0.02, 0.03, 0.06]),
        (0.2, [0.04, 0.08, 0.30]),
        (0.4, [0.05, 0.40, 0.70]),
        (0.6, [0.20, 0.80, 0.60]),
        (0.75, [0.80, 0.90, 0.25]),
        (0.9, [1.0, 0.60, 0.10]),
        (1.0, [1.0, 0.20, 0.15]),
    ];
    let i = STOPS.iter().rposition(|(p, _)| *p <= t).unwrap_or(0).min(STOPS.len() - 2);
    let (p0, c0) = STOPS[i];
    let (p1, c1) = STOPS[i + 1];
    let k = (t - p0) / (p1 - p0);
    let ch = |j: usize| ((c0[j] + k * (c1[j] - c0[j])) * 255.0) as u8;
    Color32::from_rgb(ch(0), ch(1), ch(2))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tuner_groups_digits() {
        let text: String = tuner_digits(145_500_000.0).iter().map(|d| d.0).collect();
        assert_eq!(text, "145.500 000");
        let text: String = tuner_digits(5_800_123_456.0).iter().map(|d| d.0).collect();
        assert_eq!(text, "5 800.123 456");
        let text: String = tuner_digits(7_074_000.0).iter().map(|d| d.0).collect();
        assert_eq!(text, "7.074 000");
        assert_eq!(tuner_digits(7_074_000.0)[0].1, Some(1e6));
    }
}
