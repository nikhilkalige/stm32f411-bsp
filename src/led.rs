//! User LEDs

use stm32f411::{GPIOA, RCC};

/// Green LED (PA5)
pub struct Green;

/// Initializes the user LED
pub fn init(gpioa: &GPIOA, rcc: &RCC) {
    // power on GPIOA
    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());

    // configure PA5 as output
    unsafe {
        gpioa.moder.write(|w| w.moder5().bits(1));
    }
    gpioa.bsrr.write(|w| w.bs5().set_bit());
}

impl Green {
    /// Turns the LED on
    pub fn on(&self) {
        unsafe { (*GPIOA.get()).bsrr.write(|w| w.br5().set_bit()) }
    }

    /// Turns the LED off
    pub fn off(&self) {
        unsafe { (*GPIOA.get()).bsrr.write(|w| w.bs5().set_bit()) }
    }
}
