//! Timer

use core::any::{Any, TypeId};
use core::ops::Deref;
use core::marker::PhantomData;

use cast::{u16, u32};
use hal;
use nb::{self, Error};
use stm32f411::{GPIOA, TIM1, TIM3, TIM4, gpioa, tim3, tim1};

/// Channel associated to a timer
#[derive(Clone, Copy, Debug)]
pub enum Channel {
    /// TxC1
    _1,
    /// TxC2
    _2,
    /// TxC3
    _3,
    /// TxC4
    _4,
}

pub unsafe trait TIMBase {
    fn init(&self, timeout: ::apb1::Ticks);
    fn set_timeout(&self, timeout: ::apb1::Ticks);
}

unsafe impl TIMBase for tim3::RegisterBlock {
    fn init(&self, timeout: ::apb1::Ticks) {
        // Configure periodic update event
        self.set_timeout(timeout);

        // Continuous mode
        self.cr1.write(|w| w.opm().clear_bit());

        // Enable the update event interrupt
        self.dier.modify(|_, w| w.uie().set_bit());
    }

    fn set_timeout(&self, timeout: ::apb1::Ticks) {
        let period = timeout.0;

        let psc = u16((period - 1) / (1 << 16)).unwrap();
        let arr = u16(period / u32(psc + 1)).unwrap();
        unsafe {
            self.psc.write(|w| w.psc().bits(psc));
            self.arr.write(|w| w.arr_l().bits(arr));
        }
    }
}

unsafe impl TIMBase for tim1::RegisterBlock {
    fn init(&self, timeout: ::apb1::Ticks) {
        self.set_timeout(timeout);
        self.cr1.write(|w| w.opm().clear_bit());
        self.dier.modify(|_, w| w.uie().set_bit());
    }

    fn set_timeout(&self, timeout: ::apb1::Ticks) {
        let period = timeout.0;
        let psc = u16((period - 1) / (1 << 16)).unwrap();
        let arr = u16(period / u32(psc + 1)).unwrap();
        unsafe {
            self.psc.write(|w| w.psc().bits(psc));
            self.arr.write(|w| w.arr().bits(arr));
        }
    }
}

pub unsafe trait TIM<T>: Deref<Target = T>
    where T: TIMBase 
{
    /// IMPLEMENTATION DETAIL
    type GPIO: Deref<Target = gpioa::RegisterBlock>;

    fn init_(&self, timeout: ::apb1::Ticks) {
        self.init(timeout);
    }

    fn set_timeout_(&self, timeout: ::apb1::Ticks) {
        self.set_timeout(timeout);
    }
}

unsafe impl TIM<tim3::RegisterBlock> for TIM3 {
    type GPIO = GPIOA;
}

unsafe impl TIM<tim3::RegisterBlock> for TIM4 {
    type GPIO = GPIOA;
}

unsafe impl TIM<tim1::RegisterBlock> for TIM1 {
    type GPIO = GPIOA;
}


pub struct Timer<'a, T, R>(pub &'a T, pub PhantomData<R>) where T: 'a;

// impl<'a, T, R> Clone for Timer<'a, T, R>
//     where R: TIMBase, T: Any + TIM<R> 
// {
//     fn clone(&self) -> Self {
//         *self
//     }
// }

// impl<'a, T, R> Copy for Timer<'a, T, R> {}

impl<'a, T, R> Timer<'a, T, R>
    where R: TIMBase, T: Any + TIM<R>
{
    pub const fn new(tim: &'a T) -> Self {
        Timer(tim, PhantomData)
    }
    /// Initializes the timer with a periodic timeout of `frequency` Hz
    ///
    /// NOTE After initialization, the timer will be in the paused state.
    pub fn init<P>(&self, period: P)
        where P: Into<::apb1::Ticks>
    {
        self.0.init_(period.into());
    }
}

impl<'a, T> hal::Timer for Timer<'a, T, tim3::RegisterBlock>
    where T: Any + TIM<tim3::RegisterBlock>
{
    type Time = ::apb1::Ticks;

    fn get_timeout(&self) -> ::apb1::Ticks {
        ::apb1::Ticks(u32(self.0.psc.read().psc().bits() + 1) * u32(self.0.arr.read().bits()))
    }

    fn pause(&self) {
        self.0.cr1.modify(|_, w| w.cen().clear_bit());
    }

    fn restart(&self) {
        unsafe {
            self.0.cnt.modify(|_, w| w.bits(0));
            self.0.cnt.write(|w| w.bits(0));
        }
    }

    fn resume(&self) {
        self.0.cr1.modify(|_, w| w.cen().set_bit());
    }

    fn set_timeout<TO>(&self, timeout: TO)
        where TO: Into<::apb1::Ticks>
    {
        self.0.set_timeout_(timeout.into())
    }

    fn wait(&self) -> nb::Result<(), !> {
        if self.0.sr.read().uif().bit_is_clear() {
            Err(Error::WouldBlock)
        } else {
            self.0.sr.modify(|_, w| w.uif().clear_bit());
            Ok(())
        }
    }
}

impl<'a, T> hal::Timer for Timer<'a, T, tim1::RegisterBlock>
    where T: Any + TIM<tim1::RegisterBlock>
{
    type Time = ::apb1::Ticks;

    fn get_timeout(&self) -> ::apb1::Ticks {
        ::apb1::Ticks(u32(self.0.psc.read().psc().bits() + 1) * u32(self.0.arr.read().bits()))
    }

    fn pause(&self) {
        self.0.cr1.modify(|_, w| w.cen().clear_bit());
    }

    fn restart(&self) {
        unsafe {
            self.0.cnt.modify(|_, w| w.bits(0));
            self.0.cnt.write(|w| w.bits(0));
        }
    }

    fn resume(&self) {
        self.0.cr1.modify(|_, w| w.cen().set_bit());
    }

    fn set_timeout<TO>(&self, timeout: TO)
        where TO: Into<::apb1::Ticks>
    {
        self.0.set_timeout_(timeout.into())
    }

    fn wait(&self) -> nb::Result<(), !> {
        if self.0.sr.read().uif().bit_is_clear() {
            Err(Error::WouldBlock)
        } else {
            self.0.sr.modify(|_, w| w.uif().clear_bit());
            Ok(())
        }
    }
}