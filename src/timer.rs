//! Timer

use core::any::{Any, TypeId};
use core::ops::Deref;

use cast::{u16, u32};
use hal;
use nb::{self, Error};
use stm32f411::{GPIOA, RCC, TIM3, TIM4, gpioa, tim3};

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

/// IMPLEMENTATION DETAIL
pub unsafe trait TIM: Deref<Target = tim3::RegisterBlock> {
    /// IMPLEMENTATION DETAIL
    type GPIO: Deref<Target = gpioa::RegisterBlock>;
}

unsafe impl TIM for TIM3 {
    type GPIO = GPIOA;
}

unsafe impl TIM for TIM4 {
    type GPIO = GPIOA;
}

/// `hal::Timer` implementation
pub struct Timer<'a, T>(pub &'a T) where T: 'a;

impl<'a, T> Clone for Timer<'a, T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<'a, T> Copy for Timer<'a, T> {}

impl<'a, T> Timer<'a, T>
    where T: Any + TIM
{
    /// Initializes the timer with a periodic timeout of `frequency` Hz
    ///
    /// NOTE After initialization, the timer will be in the paused state.
    pub fn init<P>(&self, period: P, rcc: &RCC)
        where P: Into<::apb1::Ticks>
    {
        self.init_(period.into(), rcc)
    }

    fn init_(&self, timeout: ::apb1::Ticks, rcc: &RCC) {
        let tim3 = self.0;

        // Enable TIMx
        if tim3.get_type_id() == TypeId::of::<TIM3>() {
            rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());
        } else if tim3.get_type_id() == TypeId::of::<TIM4>() {
            rcc.apb1enr.modify(|_, w| w.tim4en().set_bit());
        }

        // Configure periodic update event
        self._set_timeout(timeout);

        // Continuous mode
        tim3.cr1.write(|w| w.opm().clear_bit());

        // Enable the update event interrupt
        tim3.dier.modify(|_, w| w.uie().set_bit());
    }

    fn _set_timeout(&self, timeout: ::apb1::Ticks) {
        let period = timeout.0;

        let psc = u16((period - 1) / (1 << 16)).unwrap();
        let arr = u16(period / u32(psc + 1)).unwrap();
        unsafe {
            self.0.psc.write(|w| w.psc().bits(psc));
            self.0.arr.write(|w| w.arr_l().bits(arr));
        }
    }
}

impl<'a, T> hal::Timer for Timer<'a, T>
    where T: Any + TIM
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
        self._set_timeout(timeout.into())
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
