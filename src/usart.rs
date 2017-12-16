//! APIs for the USART peripherals

use core::ops::Deref;
use core::marker::Unsize;
use core::ptr;

use cast::u16;
use hal;
use nb;
use stm32f411::{DMA1, USART1};
use gpio::{AltFunction, PA2, PA3};
use rcc::{Clocks, ENR};
use time::Bps;

#[derive(Debug)]
pub enum Error {
    /// De-synchronization, excessive noise or a break character detected
    Framing,
    /// Noise detected in the received frame
    Noise,
    /// RX buffer overrun
    Overrun,
    #[doc(hidden)] _Extensible,
}

/// Interrupt event
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    /// RX buffer Not Empty (new data available)
    Rxne,
    /// Transmission Complete
    Tc,
    /// TX buffer Empty (more data can be send)
    Txe,
}

pub struct Usart {
    usart: USART1,
}

pub enum Pins {
    NoRemap(PA2<AltFunction>, PA3<AltFunction>),
}

impl From<(PA2<AltFunction>, PA3<AltFunction>)> for Pins {
    fn from((tx, rx): (PA2<AltFunction>, PA3<AltFunction>)) -> Pins {
        Pins::NoRemap(tx, rx)
    }
}

impl Usart {
    pub fn new<P>(usart: USART1, pins: P, bps: Bps, clocks: Clocks, enr: &mut ENR) -> Usart
    where
        P: Into<Pins>,
    {
        enr.apb1().modify(|_, w| w.usart2en().set_bit());

        match pins.into() {
            Pins::NoRemap(tx, rx) => {
                tx.alternate_function(7);
                rx.alternate_function(7);
            }
        }

        let brr = clocks.pclk1().0 / bps.0;
        assert!(brr > 16, "impossible baud rate");

        usart.brr.write(|w| unsafe { w.bits(brr) });

        Usart { usart }
    }

    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
            Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().set_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
        }
    }

    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
            Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().clear_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
        }
    }

    pub fn split(self) -> (Tx, Rx) {
        (Tx { _0: () }, Rx { _0: () })
    }

    pub fn unwrap(self) -> USART1 {
        self.usart
    }
}

pub struct Rx {
    _0: (),
}

impl hal::serial::Read<u8> for Rx {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let usart = unsafe { &*USART1::ptr() };
        let sr = usart.sr.read();

        if sr.ore().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bit_is_set() {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bit_is_set() {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.rxne().bit_is_set() {
            // NOTE(read_volatile) the register is 9 bits big but we'll only
            // work with the first 8 bits
            Ok(unsafe {
                ptr::read_volatile(&usart.dr as *const _ as *const u8)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

pub struct Tx {
    _0: (),
}

impl hal::serial::Write<u8> for Tx {
    type Error = Error;

    fn write(&mut self, byte: u8) -> nb::Result<(), Error> {
        let usart = unsafe { &*USART1::ptr() };
        let sr = usart.sr.read();

        if sr.ore().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bit_is_set() {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bit_is_set() {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see NOTE in the `read` method
            unsafe { ptr::write_volatile(&usart.dr as *const _ as *mut u8, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}