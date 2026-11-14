//! FT8 transmitting through an Icom transceiver: the QSO messages (Tx1 … Tx6), when to
//! send them, following the other station's answers, and the controls above the FT8
//! messages.

use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use eframe::egui::{self, Color32, RichText};
use sedir::ft8::tx::{self as ft8tx, Next};
use sedir::ft8::worker::Ft8Message;
use sedir::source::Command;

use super::SedirApp;
use crate::i18n::fill;
use crate::ui::palette::*;

/// Transmit audio rate (the radios' USB sound card and network audio).
const TX_RATE: f64 = 48_000.0;
/// A transmission is handed over this long before its slot starts.
const HAND_OVER_S: f64 = 1.2;
/// Latest start after the slot start that still decodes (s).
const LATE_START_S: f64 = 1.5;
/// TX turns off after this long without the user touching it (WSJT-X's watchdog).
const WATCHDOG_S: f64 = 360.0;
const SLOT_S: f64 = 15.0;

pub(super) fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

/// A finished QSO, for the log.
#[derive(Clone, Debug)]
#[allow(dead_code)] // Read by the log book, which comes next.
pub(super) struct DoneQso {
    pub call: String,
    pub grid: Option<String>,
    pub report_sent: String,
    pub report_received: Option<String>,
    /// Unix seconds (UTC).
    pub time: f64,
    pub dial_hz: f64,
}

pub(super) struct TxState {
    pub enabled: bool,
    pub my_call: String,
    pub my_grid: String,
    pub dx_call: String,
    pub dx_grid: Option<String>,
    pub report_sent: String,
    pub report_received: Option<String>,
    /// Template sent next: 0 = Tx1 … 5 = Tx6 (CQ).
    pub selected: usize,
    pub tx_hz: f64,
    pub level_pct: f32,
    /// Transmit in the second (odd) slots.
    pub second: bool,
    /// Slot of the transmission handed over last, and when it ends.
    scheduled_slot: Option<u64>,
    pub on_air: (f64, f64),
    /// Turn TX off after the transmission now on the air (RR73 or 73 sent).
    stop_after: bool,
    /// Decoded messages up to this slot have been looked at.
    handled_slot: u64,
    /// Last time the user did something with TX.
    touched: f64,
    pub note: Option<(String, Color32)>,
    pub done: Vec<DoneQso>,
}

impl TxState {
    pub fn new(my_call: String, my_grid: String, tx_hz: f64, level_pct: f32) -> Self {
        Self {
            enabled: false,
            my_call,
            my_grid,
            dx_call: String::new(),
            dx_grid: None,
            report_sent: "-10".into(),
            report_received: None,
            selected: 5,
            tx_hz,
            level_pct,
            second: false,
            scheduled_slot: None,
            on_air: (0.0, 0.0),
            stop_after: false,
            handled_slot: 0,
            touched: 0.0,
            note: None,
            done: Vec::new(),
        }
    }

    pub fn messages(&self) -> [String; 6] {
        ft8tx::templates(&self.my_call, &self.my_grid, &self.dx_call, &self.report_sent)
    }

    pub fn on_air_now(&self) -> bool {
        let now = unix_now();
        (self.on_air.0..self.on_air.1).contains(&now)
    }

    fn touch(&mut self) {
        self.touched = unix_now();
    }

    /// The QSO with `dx_call` has had its reports exchanged.
    fn complete(&mut self, dial_hz: f64) {
        if self.dx_call.is_empty() || self.done.last().is_some_and(|d| d.call == self.dx_call && unix_now() - d.time < 600.0) {
            return;
        }
        self.done.push(DoneQso {
            call: self.dx_call.clone(),
            grid: self.dx_grid.clone(),
            report_sent: self.report_sent.clone(),
            report_received: self.report_received.clone(),
            time: unix_now(),
            dial_hz,
        });
    }

    /// Follows an answer from `from`.
    fn follow(&mut self, from: String, next: Next, snr_db: f32, dial_hz: f64) {
        if from != self.dx_call {
            self.dx_call = from;
            self.dx_grid = None;
            self.report_received = None;
            self.report_sent = ft8tx::report(snr_db);
        }
        if let Some(r) = next.report_received {
            self.report_received = Some(r);
        }
        if next.complete {
            self.complete(dial_hz);
        }
        match next.tx {
            Some(t) => self.selected = t,
            None => self.enabled = false,
        }
    }
}

impl SedirApp {
    fn is_icom(&self) -> bool {
        self.device().is_some_and(|d| d.kind.is_icom())
    }

    /// Hands transmissions over in time, follows answers and keeps the watchdog.
    pub(super) fn tx_poll(&mut self) {
        let t = self.t();
        let now = unix_now();
        if self.tx.stop_after && now >= self.tx.on_air.1 {
            self.tx.stop_after = false;
            self.tx.enabled = false;
            if let Some(d) = self.tx.done.last() {
                self.tx.note = Some((fill(t.tx_qso_done, &[&d.call]), GOOD));
            }
        }
        if !self.tx.enabled || !self.opened || !self.is_icom() {
            return;
        }
        self.tx_follow_answers();
        if !self.tx.enabled {
            return;
        }
        if now - self.tx.touched > WATCHDOG_S {
            self.tx.enabled = false;
            self.tx.note = Some((t.tx_watchdog.to_string(), WARN));
            return;
        }
        let slot_now = (now / SLOT_S).floor() as u64;
        for slot in [slot_now, slot_now + 1] {
            if (slot % 2 == 1) != self.tx.second || self.tx.scheduled_slot == Some(slot) {
                continue;
            }
            let nominal = slot as f64 * SLOT_S + 0.5;
            let lead = nominal - now;
            if lead > HAND_OVER_S || lead < -LATE_START_S {
                continue;
            }
            let text = self.tx.messages()[self.tx.selected].clone();
            let Some(tones) = ft8tx::message_tones(&text) else {
                self.tx.enabled = false;
                self.tx.note = Some((fill(t.tx_bad_message, &[&text]), BAD));
                return;
            };
            let audio = ft8tx::audio(&tones, self.tx.tx_hz, TX_RATE, (self.tx.level_pct / 100.0).clamp(0.0, 1.0));
            let start_at = nominal.max(now + 0.25);
            self.send(Command::Transmit { audio: Arc::new(audio), start_at, output: self.settings.icom_output.clone() });
            self.tx.scheduled_slot = Some(slot);
            self.tx.on_air = (start_at, start_at + ft8tx::LENGTH_S);
            // After RR73 or 73 the QSO is over.
            self.tx.stop_after = matches!(self.tx.selected, 3 | 4);
            self.tx.note = None;
            break;
        }
    }

    /// Messages to us decoded since the last look move the QSO on.
    fn tx_follow_answers(&mut self) {
        let Some(d) = self.ft.first() else { return };
        let dial = d.dial_hz;
        let mut messages: Vec<Ft8Message> = d.worker.messages().into_iter().filter(|m| m.slot > self.tx.handled_slot).collect();
        if messages.is_empty() {
            return;
        }
        messages.sort_by_key(|m| m.slot);
        let my = self.tx.my_call.clone();
        for m in &messages {
            let Some((from, next)) = ft8tx::answer_to(&m.text, &my) else { continue };
            // Answers to our CQ from anyone; otherwise only from the station we work.
            if self.tx.selected == 5 || self.tx.dx_call.is_empty() || from == self.tx.dx_call {
                if let Some(g) = ft8tx::grid_of(&m.text) {
                    self.tx.dx_grid = Some(g);
                }
                self.tx.follow(from, next, m.snr_db, dial);
            }
        }
        self.tx.handled_slot = messages.iter().map(|m| m.slot).max().unwrap_or(self.tx.handled_slot);
    }

    /// A decoded message was clicked: answer its sender.
    pub(super) fn tx_pick(&mut self, m: &Ft8Message) {
        let my = self.tx.my_call.trim().to_ascii_uppercase();
        let dial = self.ft.first().map_or(0.0, |d| d.dial_hz);
        if let Some((from, next)) = ft8tx::answer_to(&m.text, &my) {
            self.tx.follow(from, next, m.snr_db, dial);
        } else if let Some(call) = ft8tx::sender(&m.text).filter(|c| *c != my) {
            self.tx.dx_call = call;
            self.tx.dx_grid = ft8tx::grid_of(&m.text);
            self.tx.report_sent = ft8tx::report(m.snr_db);
            self.tx.report_received = None;
            self.tx.selected = 0;
        } else {
            return;
        }
        // The other slot from theirs.
        self.tx.second = (m.slot / 15) % 2 == 0;
        self.tx.handled_slot = self.tx.handled_slot.max(m.slot);
        self.tx.touch();
    }

    pub(super) fn tx_halt(&mut self) {
        self.tx.enabled = false;
        self.tx.stop_after = false;
        self.tx.scheduled_slot = None;
        self.tx.on_air = (0.0, 0.0);
        self.send(Command::StopTransmit);
    }

    fn save_tx(&mut self) {
        let s = &mut self.settings;
        s.my_call = Some(self.tx.my_call.trim().to_ascii_uppercase()).filter(|c| !c.is_empty());
        s.my_grid = Some(self.tx.my_grid.trim().to_ascii_uppercase()).filter(|g| !g.is_empty());
        s.tx_hz = Some(self.tx.tx_hz);
        s.tx_level = Some(self.tx.level_pct);
        s.save();
    }

    /// TX controls above the FT8 messages (Icom transceivers only).
    pub(super) fn tx_panel(&mut self, ui: &mut egui::Ui) {
        if !self.is_icom() {
            return;
        }
        let t = self.t();
        let mut save = false;
        ui.horizontal_wrapped(|ui| {
            let on_air = self.tx.on_air_now();
            let (label, fill_color, text_color) =
                if self.tx.enabled { (t.tx_on, BAD.gamma_multiply(0.8), Color32::WHITE) } else { (t.tx_off, CARD, TEXT) };
            let toggle = egui::Button::new(RichText::new(label).strong().color(text_color)).fill(fill_color);
            if ui.add(toggle).on_hover_text(t.tx_tooltip).clicked() {
                if self.tx.enabled {
                    self.tx_halt();
                } else if self.tx.my_call.trim().is_empty() {
                    self.tx.note = Some((t.tx_need_call.to_string(), BAD));
                } else {
                    self.tx.enabled = true;
                    self.tx.scheduled_slot = None;
                    self.tx.handled_slot = (unix_now() / SLOT_S).floor() as u64 * 15 - 15;
                    self.tx.note = None;
                    self.tx.touch();
                }
            }
            if ui.button(t.tx_halt).clicked() {
                self.tx_halt();
            }
            if on_air {
                ui.label(RichText::new(format!("● {}", t.tx_on_air)).strong().color(BAD));
            }
            ui.separator();
            ui.label(t.my_call);
            save |= ui.add(egui::TextEdit::singleline(&mut self.tx.my_call).desired_width(70.0).char_limit(11)).lost_focus();
            ui.label(t.my_grid);
            save |= ui.add(egui::TextEdit::singleline(&mut self.tx.my_grid).desired_width(44.0).char_limit(6)).lost_focus();
            ui.label(t.tx_freq);
            save |= ui.add(egui::DragValue::new(&mut self.tx.tx_hz).range(200.0..=2900.0).speed(5.0)).changed();
            ui.label(t.tx_level);
            save |= ui.add(egui::DragValue::new(&mut self.tx.level_pct).range(1.0..=100.0).speed(1.0).suffix(" %")).changed();
            ui.label(t.tx_slot);
            let slot_text = if self.tx.second { t.tx_slot_second } else { t.tx_slot_first };
            egui::ComboBox::from_id_salt("tx_slot").selected_text(slot_text).show_ui(ui, |ui| {
                ui.selectable_value(&mut self.tx.second, false, t.tx_slot_first);
                ui.selectable_value(&mut self.tx.second, true, t.tx_slot_second);
            });
        });
        ui.horizontal_wrapped(|ui| {
            ui.label(t.dx_call);
            if ui.add(egui::TextEdit::singleline(&mut self.tx.dx_call).desired_width(80.0).char_limit(11)).changed() {
                self.tx.dx_call = self.tx.dx_call.to_ascii_uppercase();
                self.tx.touch();
            }
            ui.label(t.tx_report);
            ui.add(egui::TextEdit::singleline(&mut self.tx.report_sent).desired_width(36.0).char_limit(3));
            ui.separator();
            let messages = self.tx.messages();
            for (i, m) in messages.iter().enumerate() {
                let text = RichText::new(format!("Tx{} {m}", i + 1)).size(12.0).monospace();
                if ui.selectable_label(self.tx.selected == i, text).clicked() {
                    self.tx.selected = i;
                    self.tx.touch();
                }
            }
        });
        let note = self.tx.note.clone();
        ui.horizontal(|ui| match note {
            Some((text, color)) => {
                ui.label(RichText::new(text).size(12.0).color(color));
            }
            None => {
                ui.label(RichText::new(t.tx_hint).size(11.0).color(MUTED));
            }
        });
        if save {
            self.tx.my_call = self.tx.my_call.trim().to_ascii_uppercase();
            self.tx.my_grid = self.tx.my_grid.trim().to_ascii_uppercase();
            self.save_tx();
        }
        ui.separator();
    }
}
