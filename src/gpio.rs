use stm32f411::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH};
use hal::digital;

use rcc::ENR;

pub trait GpioExt {
    type Parts;

    fn split(self, enr: &mut ENR) -> Self::Parts;
}

// States
pub struct AltFunction;
pub struct Input;
pub struct Output;
pub struct Analog;

use stm32f411::gpioa as GpioaModule;

pub struct Gpio;

#[derive(Copy, Clone)]
pub enum Io {
    Low,
    High
}

#[derive(Copy, Clone)]
pub enum Mode {
    Input,
    Output,
    AlternateFunction,
    Analog
}

#[derive(Copy, Clone)]
pub enum Speed {
    Low,
    Medium,
    Fast,
    High
}

#[derive(Copy, Clone)]
pub enum Pupd {
    No,
    PullUp,
    PullDown,
}

impl Gpio
{
    pub const fn new() -> Self {
        Gpio { }
    }

    pub fn set(port: &GpioaModule::RegisterBlock, pin_no: u32, data: Io) {
        let value: u32 = match data {
            Io::High => 1 << pin_no,
            Io::Low => 1 << (16 + pin_no),
        };
        port.bsrr.write(|w| unsafe { w.bits(value) });
    }

    pub fn get(port: &GpioaModule::RegisterBlock, pin_no: u32) -> Io {
        let value: bool = ((port.idr.read().bits()) & (1 << pin_no)) != 0;
        if value {
            Io::High
        } else {
            Io::Low
        }
    }

    pub fn alternate_function(port:&GpioaModule::RegisterBlock, pin_no: u32, mode: u8) {
        if pin_no < 8 {
            let value = (mode as u32) << (pin_no * 4);
            let mask = !((0b1111 as u32) << (pin_no * 4));
            port.afrl.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) });
        } else {
            let value = (mode as u32) << ((pin_no - 8) * 4);
            let mask = !((0b1111 as u32) << ((pin_no - 8) * 4));
            port.afrh.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) });
        }
    }

    pub fn set_mode(port:&GpioaModule::RegisterBlock, pin_no: u32, mode: Mode) {
        let value: u32 = (mode as u32) << (pin_no * 2);
        let mask = !((0b11 as u32) << (pin_no * 2));
        port.moder.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) })
    }

    pub fn set_speed(port: &GpioaModule::RegisterBlock, pin_no: u32, speed: Speed) {
        let value: u32 = (speed as u32) << (pin_no * 2);
        let mask = !((0b11 as u32) << (pin_no * 2));
        port.ospeedr.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) })
    }

    pub fn set_pupd(port: &GpioaModule::RegisterBlock, pin_no: u32, pupd: Pupd) {
        let value: u32 = (pupd as u32) << (pin_no * 2);
        let mask = !((0b11 as u32) << (pin_no * 2));
        port.pupdr.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) })
    }
}

macro_rules! gpio {
    ($GPIO:ident, $gpio:ident, $iopen:ident, [
        $($PIN:ident: ($pin:ident, $n:expr),)+
    ]) => {
        pub mod $gpio {
            use super::Input;

            pub struct Parts {
                $(pub $pin: super::$PIN<Input>,)+
            }
        }

        impl GpioExt for $GPIO {
            type Parts = $gpio::Parts;

            fn split(self, enr: &mut ENR) -> $gpio::Parts {
                enr.ahb1().modify(|_, w| w.$iopen().set_bit());

                $gpio::Parts {
                    $($pin: $PIN { _state: Input },)+
                }
            }
        }

        $(
            pub struct $PIN<STATE> {
                _state: STATE,
            }

            impl<STATE> $PIN<STATE> {
                pub fn set(&self, data: Io) {
                    unsafe { Gpio::set(&(*$GPIO::ptr()), $n, data); }
                }

                pub fn get(&self) -> Io {
                    unsafe { Gpio::get(&(*$GPIO::ptr()), $n) }
                }

                pub fn alternate_function(&self, mode: u8) {
                    unsafe { Gpio::alternate_function(&(*$GPIO::ptr()), $n, mode); }
                }

                pub fn set_mode(&self, mode: Mode) {
                    unsafe { Gpio::set_mode(&(*$GPIO::ptr()), $n, mode); }
                }

                pub fn set_speed(&self, speed: Speed) {
                    unsafe { Gpio::set_speed(&(*$GPIO::ptr()), $n, speed); }
                }

                pub fn set_pupd(&self, pupd: Pupd) {
                    unsafe { Gpio::set_pupd(&(*$GPIO::ptr()), $n, pupd); }
                }
            }

            impl $PIN<Input> {
                pub fn is_high(&self) -> bool {
                    !self.is_low()
                }

                pub fn is_low(&self) -> bool {
                    // NOTE atomic read with not side effects
                    unsafe { (*$GPIO::ptr()).idr.read().bits() & (1 << $n) == 0 }
                }
            }

            impl digital::OutputPin for $PIN<Output> {
                fn is_high(&self) -> bool {
                    !self.is_low()
                }

                fn is_low(&self) -> bool {
                    // NOTE atomic read with not side effects
                    unsafe { (*$GPIO::ptr()).odr.read().bits() & (1 << $n) == 0 }
                }

                fn set_high(&mut self) {
                    // NOTE atomic write to a stateless register
                    unsafe { (*$GPIO::ptr()).bsrr.write(|w| w.bits(1 << $n)) }
                }

                fn set_low(&mut self) {
                    // NOTE atomic write to a stateless register
                    unsafe { (*$GPIO::ptr()).bsrr.write(|w| w.bits(1 << (16 + $n))) }
                }
            }
        )+
    }
}


gpio!(GPIOA, gpioa, gpioaen, [
    PA0  :  (pa0,  0),
    PA1  :  (pa1,  1),
    PA2  :  (pa2,  2),
    PA3  :  (pa3,  3),
    PA4  :  (pa4,  4),
    PA5  :  (pa5,  5),
    PA6  :  (pa6,  6),
    PA7  :  (pa7,  7),
    PA8  :  (pa8,  8),
    PA9  :  (pa9,  9),
    PA10 : (pa10, 10),
    PA11 : (pa11, 11),
    PA12 : (pa12, 12),
    PA13 : (pa13, 13),
    PA14 : (pa14, 14),
    PA15 : (pa15, 15),
]);

gpio!(GPIOB, gpiob, gpioben, [
    PB0  :  (pb0,  0),
    PB1  :  (pb1,  1),
    PB2  :  (pb2,  2),
    PB3  :  (pb3,  3),
    PB4  :  (pb4,  4),
    PB5  :  (pb5,  5),
    PB6  :  (pb6,  6),
    PB7  :  (pb7,  7),
    PB8  :  (pb8,  8),
    PB9  :  (pb9,  9),
    PB10 : (pb10, 10),
    PB11 : (pb11, 11),
    PB12 : (pb12, 12),
    PB13 : (pb13, 13),
    PB14 : (pb14, 14),
    PB15 : (pb15, 15),
]);

gpio!(GPIOC, gpioc, gpiocen, [
    PC0  :  (pc0,  0),
    PC1  :  (pc1,  1),
    PC2  :  (pc2,  2),
    PC3  :  (pc3,  3),
    PC4  :  (pc4,  4),
    PC5  :  (pc5,  5),
    PC6  :  (pc6,  6),
    PC7  :  (pc7,  7),
    PC8  :  (pc8,  8),
    PC9  :  (pc9,  9),
    PC10 : (pc10, 10),
    PC11 : (pc11, 11),
    PC12 : (pc12, 12),
    PC13 : (pc13, 13),
    PC14 : (pc14, 14),
    PC15 : (pc15, 15),
]);

gpio!(GPIOD, gpiod, gpioden, [
    PD0  :  (pd0,  0),
    PD1  :  (pd1,  1),
    PD2  :  (pd2,  2),
    PD3  :  (pd3,  3),
    PD4  :  (pd4,  4),
    PD5  :  (pd5,  5),
    PD6  :  (pd6,  6),
    PD7  :  (pd7,  7),
    PD8  :  (pd8,  8),
    PD9  :  (pd9,  9),
    PD10 : (pd10, 10),
    PD11 : (pd11, 11),
    PD12 : (pd12, 12),
    PD13 : (pd13, 13),
    PD14 : (pd14, 14),
    PD15 : (pd15, 15),
]);

gpio!(GPIOE, gpioe, gpioeen, [
    PE0  :  (pe0,  0),
    PE1  :  (pe1,  1),
    PE2  :  (pe2,  2),
    PE3  :  (pe3,  3),
    PE4  :  (pe4,  4),
    PE5  :  (pe5,  5),
    PE6  :  (pe6,  6),
    PE7  :  (pe7,  7),
    PE8  :  (pe8,  8),
    PE9  :  (pe9,  9),
    PE10 : (pe10, 10),
    PE11 : (pe11, 11),
    PE12 : (pe12, 12),
    PE13 : (pe13, 13),
    PE14 : (pe14, 14),
    PE15 : (pe15, 15),
]);

gpio!(GPIOH, gpioh, gpiohen, [
    PH0  :  (ph0,  0),
    PH1  :  (ph1,  1),
    PH2  :  (ph2,  2),
    PH3  :  (ph3,  3),
    PH4  :  (ph4,  4),
    PH5  :  (ph5,  5),
    PH6  :  (ph6,  6),
    PH7  :  (ph7,  7),
    PH8  :  (ph8,  8),
    PH9  :  (ph9,  9),
    PH10 : (ph10, 10),
    PH11 : (ph11, 11),
    PH12 : (ph12, 12),
    PH13 : (ph13, 13),
    PH14 : (ph14, 14),
    PH15 : (ph15, 15),
]);