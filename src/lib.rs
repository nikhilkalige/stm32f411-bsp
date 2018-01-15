#![feature(core_intrinsics)]
#![feature(lang_items)]
#![feature(never_type)]
#![feature(const_fn)]
#![feature(unsize)]
#![no_std]

extern crate cast;
#[macro_use]
extern crate cortex_m;
extern crate nb;
pub extern crate embedded_hal as hal;
pub extern crate stm32f411;

pub mod bb;
pub mod time;
pub mod rcc;
pub mod gpio;
pub mod usart;
pub mod spi;
pub mod dma;
pub mod pwm;

use cortex_m::itm;

// TODO remove
// #[lang = "panic_fmt"]
// unsafe extern "C" fn panic_fmt(
//     args: ::core::fmt::Arguments,
//     file: &'static str,
//     line: u32,
//     col: u32,
// ) -> ! {
//     let itm = &*cortex_m::peripheral::ITM::ptr();

//     itm::write_str(&itm.stim[0], "panicked at '");
//     itm::write_fmt(&itm.stim[0], args);
//     iprintln!(&itm.stim[0], "', {}:{}:{}", file, line, col);

//     ::core::intrinsics::abort()
// }
