//! User LEDs

use stm32f411::{GPIOA, RCC};

/// Green LED (PA5)
pub struct Green;

/// Initializes the user LED
pub fn init(gpioa: &GPIOA, rcc: &RCC) {
    // power on GPIOA
    rcc.ahb1enr.modify(|_, w| w.gpioaen().enabled());

    // configure PA5 as output
    gpioa.moder.write(|w| w.mode5().output());
    gpioa.bsrr.write(|w| w.bs5.set());
}

impl Green {
    /// Turns the LED on
    pub fn on(&self) {
        unsafe {
            (*GPIOA.get()).bsrr.write(|w| w.br5().reset())
        }
    }

    /// Turns the LED off
    pub fn off(&self) {
        unsafe {
            (*GPIOA.get()).bsrr.write(|w| w.bs5().set())
        }
    }
}
