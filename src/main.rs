//! SeDiR - wideband SDR receiver.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod i18n;
mod logo;
mod settings;
mod ui;

use eframe::egui;

/// In the packaged build the SoapySDR modules are in `lib/SoapySDR/modules0.8` next to
/// the program. SoapySDR searches for modules relative to its root, so the root is set
/// to the program's folder; this also avoids clashing with an installed PothosSDR.
/// Left alone if the folder is missing (development).
fn use_bundled_soapy_modules() {
    let Some(dir) = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf())) else { return };
    if dir.join("lib").join("SoapySDR").join("modules0.8").is_dir() {
        // Set while there are no other threads yet.
        unsafe { std::env::set_var("SOAPY_SDR_ROOT", &dir) };
    }
}

/// Without a bundled copy (development), the SoapySDR modules and libiio of an installed
/// PothosSDR are used. Their dependencies (libusb-1.0.dll and others) are looked up along
/// PATH, where other radio programs often put older copies first and the modules then
/// fail to load ("procedure not found"); PothosSDR's folder is moved to the front.
fn prefer_pothos_dlls() {
    if !cfg!(windows) {
        return;
    }
    let bundled = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("SoapySDR.dll").is_file())).unwrap_or(false);
    let base = std::env::var_os("ProgramFiles").map(std::path::PathBuf::from).unwrap_or_else(|| "C:\\Program Files".into());
    let pothos = base.join("PothosSDR").join("bin");
    if bundled || !pothos.is_dir() {
        return;
    }
    let mut paths = vec![pothos];
    if let Some(path) = std::env::var_os("PATH") {
        paths.extend(std::env::split_paths(&path));
    }
    if let Ok(joined) = std::env::join_paths(paths) {
        // Set while there are no other threads yet.
        unsafe { std::env::set_var("PATH", joined) };
    }
}

/// `sedir --diagnose`: lists what the program finds, on standard output.
fn diagnose() {
    println!("SeDiR {}", env!("CARGO_PKG_VERSION"));
    println!("SOAPY_SDR_ROOT = {:?}", std::env::var_os("SOAPY_SDR_ROOT"));
    println!("libiio: {:?}", sedir::iio::library_status());
    for d in sedir::device::discover(&[]) {
        let l = d.kind.limits();
        println!("  {:?} | {} | {} | {}–{} | max {}", d.kind, d.label, d.args, sedir::freq::short(l.min_hz), sedir::freq::short(l.max_hz), sedir::freq::rate(l.max_rate));
    }
}

fn main() -> eframe::Result {
    use_bundled_soapy_modules();
    prefer_pothos_dlls();
    if std::env::args().any(|a| a == "--diagnose") {
        diagnose();
        return Ok(());
    }
    const ICON: usize = 64;
    let icon = egui::IconData { rgba: logo::rgba(ICON, true), width: ICON as u32, height: ICON as u32 };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("SeDiR")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([900.0, 560.0])
            .with_icon(icon),
        ..Default::default()
    };
    eframe::run_native("SeDiR", options, Box::new(|cc| Ok(Box::new(app::SedirApp::new(cc)))))
}
