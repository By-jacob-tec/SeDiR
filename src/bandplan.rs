//! Which mode is customary on a frequency (ITU Region 1, as used in Turkey). Only a
//! hint: the signal's own measurements decide when they are clear.

use crate::demod::Mode;

/// (from MHz, to MHz, mode), first match wins: narrow special segments before wide bands.
const BANDS: &[(f64, f64, Mode)] = &[
    // Amateur CW segments.
    (1.810, 1.838, Mode::Cw),
    (3.500, 3.570, Mode::Cw),
    (7.000, 7.040, Mode::Cw),
    (10.100, 10.130, Mode::Cw),
    (14.000, 14.070, Mode::Cw),
    (18.068, 18.095, Mode::Cw),
    (21.000, 21.070, Mode::Cw),
    (24.890, 24.915, Mode::Cw),
    (28.000, 28.070, Mode::Cw),
    (50.000, 50.100, Mode::Cw),
    (144.000, 144.110, Mode::Cw),
    (432.000, 432.100, Mode::Cw),
    // Amateur phone: below 10 MHz LSB, above USB; VHF/UHF SSB segments.
    (1.838, 2.000, Mode::Lsb),
    (3.570, 3.800, Mode::Lsb),
    (5.351, 5.367, Mode::Usb),
    (7.040, 7.200, Mode::Lsb),
    (10.130, 10.150, Mode::Usb),
    (14.070, 14.350, Mode::Usb),
    (18.095, 18.168, Mode::Usb),
    (21.070, 21.450, Mode::Usb),
    (24.915, 24.990, Mode::Usb),
    (28.070, 29.200, Mode::Usb),
    (29.200, 29.700, Mode::Nfm),
    (50.100, 50.500, Mode::Usb),
    (144.110, 144.500, Mode::Usb),
    (432.100, 432.500, Mode::Usb),
    // Broadcasting: long, medium and short wave AM, FM.
    (0.148, 0.284, Mode::Am),
    (0.526, 1.606, Mode::Am),
    (2.300, 2.495, Mode::Am),
    (3.200, 3.400, Mode::Am),
    (3.900, 4.000, Mode::Am),
    (4.750, 5.060, Mode::Am),
    (5.900, 6.200, Mode::Am),
    (7.200, 7.450, Mode::Am),
    (9.400, 9.900, Mode::Am),
    (11.600, 12.100, Mode::Am),
    (13.570, 13.870, Mode::Am),
    (15.100, 15.800, Mode::Am),
    (17.480, 17.900, Mode::Am),
    (18.900, 19.020, Mode::Am),
    (21.450, 21.850, Mode::Am),
    (25.670, 26.100, Mode::Am),
    (65.800, 74.000, Mode::Wfm),
    (87.500, 108.000, Mode::Wfm),
    // Aviation voice.
    (118.000, 137.000, Mode::Am),
    // Citizens band (FM is most common in Turkey and Europe today).
    (26.965, 27.405, Mode::Nfm),
    // Amateur FM, marine, PMR446, LPD and land mobile.
    (144.500, 146.000, Mode::Nfm),
    (156.000, 163.000, Mode::Nfm),
    (430.000, 432.000, Mode::Nfm),
    (432.500, 440.000, Mode::Nfm),
    (433.050, 434.790, Mode::Nfm),
    (446.000, 446.200, Mode::Nfm),
    (146.000, 174.000, Mode::Nfm),
    (400.000, 470.000, Mode::Nfm),
];

/// The customary mode at `hz`, if the band plan has one.
pub fn hint(hz: f64) -> Option<Mode> {
    let mhz = hz / 1e6;
    BANDS.iter().find(|(lo, hi, _)| (*lo..*hi).contains(&mhz)).map(|b| b.2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn customary_modes() {
        assert_eq!(hint(100.3e6), Some(Mode::Wfm));
        assert_eq!(hint(127.5e6), Some(Mode::Am));
        assert_eq!(hint(7.074e6), Some(Mode::Lsb));
        assert_eq!(hint(7.020e6), Some(Mode::Cw));
        assert_eq!(hint(14.200e6), Some(Mode::Usb));
        assert_eq!(hint(145.500e6), Some(Mode::Nfm));
        assert_eq!(hint(144.300e6), Some(Mode::Usb));
        assert_eq!(hint(446.006e6), Some(Mode::Nfm));
        assert_eq!(hint(2.4e9), None);
    }
}
