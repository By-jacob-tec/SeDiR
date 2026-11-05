# SeDiR

Wideband SDR receiver written in Rust: a colour spectrum and waterfall over the widest band
your receiver can carry, with every signal in view found and listed. Click a signal and it
is recognised, tuned and demodulated.

*Sedir* is the Taurus cedar that grows in Muğla, Turkey.

**Türkçe:** Geniş bant SDR alıcı. Alıcının taşıyabildiği en geniş bandı renkli spektrum ve
şelale olarak gösterir, banttaki sinyalleri bulup listeler; tıklanan sinyalin türünü tanır,
ayarlar ve demodüle eder. AM, NFM, WFM (RDS), USB, LSB, CW (metne), RTTY, PSK31, FT8, FT4.
QO-100 için LNB ayarı ve beacon kilidi vardır. Arayüz Türkçe ve İngilizcedir.

## Features

- **Receivers found automatically:** RTL-SDR, HackRF One, ADALM-Pluto, LibreSDR and other
  AD9361 boards (USB or network). Frequencies outside the device's range are refused with
  a warning.
- **Spectrum and waterfall** over the full band, narrowed at will; signals found and listed
  with width and strength; station names from RDS shown on FM.
- **Listening:** AM, NFM, WFM with RDS, USB, LSB, CW, RTTY and PSK31, on the sound device of
  your choice. Automatic mode recognition from the signal and the band plan.
- **Text decoding:** CW, RTTY (45.45/50/75 Bd, 170–850 Hz shift found by itself) and PSK31
  in a strip under the spectrum.
- **FT8 and FT4**, together or apart, with SNR, time offset and audio frequency for each
  message. Time comes from the PC clock: keep it on UTC (GPS or NTP).
- **QO-100:** LNB converter setting and lock to the lower CW beacon, which corrects the
  LNB's error and drift continuously (also the receiver's own crystal error). FT8 and FT4
  are on 10489.540 MHz.
- Text size follows the screen (A− / A+ or Ctrl +/− to change); English and Turkish.

## Receivers

| Receiver | Interface | Range (nominal) | Widest band without loss |
|---|---|---|---|
| RTL-SDR (R820T/R828D, V3, V4) | SoapySDR | 24 MHz – 1.766 GHz | 2.4 MS/s |
| HackRF One | SoapySDR | 1 MHz – 6 GHz | 20 MS/s |
| ADALM-Pluto | libiio (USB or network) | 325 MHz – 3.8 GHz (wider on units with the AD9364 range) | ~4 MS/s over USB |
| LibreSDR / AD9361 boards | libiio (network) | 70 MHz – 6 GHz | 10 MS/s on gigabit Ethernet |

Once the device is open, the range it reports replaces the nominal one. The status bar shows
how many samples actually arrive: below 100 %, pick a narrower band.

SeDiR never switches on a receiver's own bias-tee (antenna power). Feed an LNB or a preamp
from an external bias-tee that blocks the DC towards the receiver.

## Windows

Download `SeDiR-<version>-windows-x64.zip` from the
[releases](https://github.com/sozden/SeDiR/releases), unpack it anywhere and run
`sedir.exe`. Nothing needs installing: the receiver libraries are in the folder.

RTL-SDR and HackRF need the WinUSB driver; if they are not found, install it once with
[Zadig](https://zadig.akeo.ie/) (Options → List All Devices, pick the receiver, WinUSB).
Pluto and LibreSDR over the network: type the address (e.g. `192.168.1.47`) in the receiver
card and press Add.

## Use

- Type a frequency in the frequency card: `145.5`, `7074k`, `1.09G` (no unit means MHz).
- Mouse wheel over a digit of the large display changes that digit.
- On the spectrum: click to select, double-click to centre, drag to move, wheel to step,
  Ctrl+wheel to zoom.
- QO-100: choose the converter "QO-100 LNB (9750 MHz)", tick "Lock to the beacon" and
  type `10489.75`.

Settings are kept in `%APPDATA%\SeDiR\settings.txt`.

## Building

Requires Rust (edition 2024) and, on Windows, [PothosSDR](https://downloads.myriadrf.org/builds/PothosSDR/)
(SoapySDR, RTL-SDR and HackRF modules, libiio). On Linux: `libsoapysdr-dev`,
`soapysdr-module-rtlsdr`, `soapysdr-module-hackrf`, `libiio0`.

```
cargo build --release
sedir --diagnose   # lists the receivers found
```

`tools\package-windows.ps1` builds the Windows zip with the libraries it needs.

## Credits

FT8 and FT4 are the work of Joe Taylor K1JT, Steve Franke K9AN, Bill Somerville G4WJS and
the WSJT-X team. SeDiR's decoder is its own code, but its LDPC code tables, CRC and message
encoding are taken from WSJT-X (GPL-3.0); `tools/gen_ft8_tables.sh` regenerates the tables.
RDS follows IEC 62106. The libraries in the Windows package and their licenses are listed in
`tools/THIRD-PARTY.txt`.

## License

GPL-3.0-or-later. A yAtölye project.
