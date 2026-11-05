#!/bin/bash
# Builds SeDiR and packs it into dist/SeDiR-<version>-linux-x64.tar.gz.
# The receiver libraries come from the distribution's packages (see README); built on
# Ubuntu 24.04, so the program needs glibc 2.39 or newer (Ubuntu 24.04+, Debian 13+).
#
#   bash tools/package-linux.sh
set -euo pipefail
cd "$(dirname "$0")/.."

version=$(sed -n 's/^version *= *"\(.*\)"/\1/p' Cargo.toml | head -1)
name="SeDiR-$version-linux-x64"
out="dist/$name"

cargo build --release
rm -rf "$out" "dist/$name.tar.gz"
mkdir -p "$out"
cp target/release/sedir "$out/"
strip "$out/sedir"
cp README.md "$out/"
cp LICENSE "$out/LICENSE.txt"
cp assets/sedir-logo.svg "$out/sedir.svg"
cat > "$out/sedir.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=SeDiR
Comment=Wideband SDR receiver
Exec=sedir
Icon=sedir
Terminal=false
Categories=HamRadio;AudioVideo;
EOF
cat > "$out/INSTALL.txt" <<'EOF'
SeDiR for Linux (x86-64, glibc 2.39+: Ubuntu 24.04 or newer, Debian 13 or newer)

1. Receiver libraries and USB access rules (Ubuntu / Debian):

   sudo apt install libsoapysdr0.8 soapysdr0.8-module-rtlsdr soapysdr0.8-module-hackrf \
                    libiio0 libasound2t64 rtl-sdr hackrf

   The rtl-sdr and hackrf packages bring the udev rules that let a normal user open the
   receivers, and keep the DVB-T driver off the RTL-SDR. Unplug and replug them once.

2. Run ./sedir from this folder. To have it in the menu:

   install -Dm755 sedir ~/.local/bin/sedir
   install -Dm644 sedir.svg ~/.local/share/icons/hicolor/scalable/apps/sedir.svg
   install -Dm644 sedir.desktop ~/.local/share/applications/sedir.desktop

Pluto and LibreSDR over the network: type the address (e.g. 192.168.1.47) in the receiver
card and press Add. Settings are kept in ~/.config/sedir/settings.txt.
EOF
chmod 644 "$out"/*
chmod 755 "$out/sedir"
tar -C dist -czf "dist/$name.tar.gz" "$name"
echo "Packaged dist/$name.tar.gz"
