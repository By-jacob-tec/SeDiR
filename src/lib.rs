//! SeDiR: wideband SDR receiver. The library holds everything that does not need a
//! window, so that it can be tested on its own.

pub mod detect;
pub mod device;
pub mod dsp;
pub mod freq;
pub mod iio;
pub mod radio;
pub mod source;
