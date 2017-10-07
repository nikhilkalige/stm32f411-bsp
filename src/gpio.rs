use stm32f411::{GPIOB, RCC};
use stm32f411::gpioa;
use core::ops::Deref;
use core::marker::PhantomData;

pub struct Pin<T>
    where T: Deref<Target=gpioa::RegisterBlock>
{
    phantom: PhantomData<*const T>,
    pin: u8,
}

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

impl<T> Pin<T>
    where T: Deref<Target=gpioa::RegisterBlock>
{
    pub const fn new(pin: u8) -> Self {
        Pin {pin, phantom: PhantomData}
    }

    pub fn set(&self, port: &T, data: Io) {
        let value: u32 = match data {
            Io::High => 1 << self.pin,
            Io::Low => 1 << (16 + self.pin),
        };
        port.bsrr.write(|w| unsafe { w.bits(value) });
    }

    pub fn get(&self, port: &T) -> Io {
        let value: bool = ((port.idr.read().bits()) & (1 << self.pin)) != 0;
        if value {
            Io::High
        } else {
            Io::Low
        }
    }

    pub fn alternate_function(&self, port:&T, mode: u8) {
        if self.pin < 8 {
            let value = (mode as u32) << (self.pin * 4);
            let mask = !((0b1111 as u32) << (self.pin * 4));
            port.afrl.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) });
        } else {
            let value = (mode as u32) << ((self.pin - 8) * 4);
            let mask = !((0b1111 as u32) << ((self.pin - 8) * 4));
            port.afrh.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) });
        }
    }

    pub fn set_mode(&self, port:&T, mode: Mode) {
        let value: u32 = (mode as u32) << (self.pin * 2);
        let mask = !((0b11 as u32) << (self.pin * 2));
        port.moder.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) })
    }

    pub fn set_speed(&self, port: &T, speed: Speed) {
        let value: u32 = (speed as u32) << (self.pin * 2);
        let mask = !((0b11 as u32) << (self.pin * 2));
        port.ospeedr.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) })
    }

    pub fn set_pupd(&self, port: &T, pupd: Pupd) {
        let value: u32 = (pupd as u32) << (self.pin * 2);
        let mask = !((0b11 as u32) << (self.pin * 2));
        port.pupdr.modify(|r, w| unsafe { w.bits((r.bits() & mask) | value) })
    }
}

// macro_rules! pin {
//     ($PBX:ident, $bsX:ident, $brX:ident) => {
//         /// Digital output
//         pub struct $PBX;

//         impl $PBX {
//             /// Sets the pin "high" (3V3)
//             pub fn high(&self) {
//                 // NOTE(safe) atomic write
//                 unsafe {
//                     (*GPIOB.get()).bsrr.write(|w| w.$bsX().bit(true));
//                 }
//             }

//             /// Sets the pin "low" (0V)
//             pub fn low(&self) {
//                 // NOTE(safe) atomic write
//                 unsafe {
//                     (*GPIOB.get()).bsrr.write(|w| w.$brX().bit(true));
//                 }
//             }
//         }
//     }
// }

// pin!(PB12, bs12, br12);
// pin!(PB13, bs13, br13);
// pin!(PB14, bs14, br14);
// pin!(PB15, bs15, br15);
