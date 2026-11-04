# SeDiR

Wideband SDR receiver written in Rust: a colour spectrum and waterfall over the widest band
your receiver can carry, with every signal in view found and listed. Clicking a signal
selects it for listening. Analog demodulation (AM, NFM, WFM, SSB, CW) and digital modes
(RTTY, PSK31, FT8, FT4) are on the way.

*Sedir* is the Taurus cedar that grows in Muğla, Turkey.

## Receivers

Found automatically when the program starts:

| Receiver | Interface | Range (nominal) | Widest band without loss |
|---|---|---|---|
| RTL-SDR (R820T/R828D) | SoapySDR | 24 MHz – 1.766 GHz | 2.4 MS/s |
| HackRF One | SoapySDR | 1 MHz – 6 GHz | 20 MS/s |
| ADALM-Pluto | libiio (USB or network) | 325 MHz – 3.8 GHz (wider on units with the AD9364 range) | ~4 MS/s over USB |
| LibreSDR / AD9361 boards | libiio (network) | 70 MHz – 6 GHz | 12 MS/s on gigabit Ethernet |

Once the device is open, the range it reports replaces the nominal one. A frequency outside
the range is refused with a warning. The status bar shows how many samples actually arrive:
below 100 %, pick a narrower band.

## Use

- Type a frequency in the frequency card: `145.5`, `7074k`, `1.09G` (no unit means MHz).
- Mouse wheel over a digit of the large display changes that digit.
- On the spectrum: click to select, double-click to centre, drag to move, wheel to step,
  Ctrl+wheel to zoom.

## Building

Requires Rust (edition 2024) and, on Windows, [PothosSDR](https://downloads.myriadrf.org/builds/PothosSDR/)
(SoapySDR, RTL-SDR and HackRF modules, libiio). On Linux: `libsoapysdr-dev`,
`soapysdr-module-rtlsdr`, `soapysdr-module-hackrf`, `libiio0`.

```
cargo build --release
sedir --diagnose   # lists the receivers found
```

## License

GPL-3.0-or-later. A yAtölye project.
