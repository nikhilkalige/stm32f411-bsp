use core::any::{Any, TypeId};
use core::marker::Unsize;

use cast::{u16, u32};
use hal;
use stm32f411::{GPIOA, RCC, TIM1};

use timer::{Channel, TIM};

/// PWM driver
pub struct Pwm<'a, T>(pub &'a T)
where
    T: 'a;

impl<'a> Pwm<'a, TIM1> {
    /// Initializes the PWM module
    pub fn init<P>(&self, period: P)
    where
        P: Into<::apb2::Ticks>,
    {
        self._init(period.into())
    }

    fn _init(&self, period: ::apb2::Ticks) {
        let tim1 = self.0;

        // PWM mode 1
        tim1.ccmr1_output.modify(|_, w| unsafe {{
            w.oc1pe().set_bit()
                .oc1m().bits(0b110)
                .oc2pe().set_bit()
                .oc2m().bits(0b110)
        }});

        tim1.ccmr2_output.modify(|_, w| unsafe{{
            w.oc3pe().set_bit()
                .oc3m().bits(0b110)
                .oc4pe().set_bit()
                .oc4m().bits(0b110)
        }});

        tim1.ccer.modify(|_, w| {
            w.cc1p().clear_bit()
                .cc2p().clear_bit()
                .cc3p().clear_bit()
                .cc4p().clear_bit()
        });

        tim1.bdtr.modify(|_, w| w.moe().set_bit());

        self._set_period(period);

        tim1.cr1.write(|w| unsafe {
            w.cms().bits(0b00)
                .dir().set_bit()
                .opm().clear_bit()
                .cen().set_bit()
        });
    }

    fn _set_period(&self, period: ::apb2::Ticks) {
        let period = period.0;

        let psc = u16((period - 1) / (1 << 16)).unwrap();
        self.0.psc.write(|w| unsafe{ w.psc().bits(psc) });

        let arr = u16(period / u32(psc + 1)).unwrap();
        self.0.arr.write(|w| unsafe{ w.arr().bits(arr) });
    }
}

impl<'a> hal::Pwm for Pwm<'a, TIM1> {
    type Channel = Channel;
    type Time = ::apb2::Ticks;
    type Duty = u16;

    fn disable(&self, channel: Channel) {
        match channel {
            Channel::_1 => self.0.ccer.modify(|_, w| w.cc1e().clear_bit()),
            Channel::_2 => self.0.ccer.modify(|_, w| w.cc2e().clear_bit()),
            Channel::_3 => self.0.ccer.modify(|_, w| w.cc3e().clear_bit()),
            Channel::_4 => self.0.ccer.modify(|_, w| w.cc4e().clear_bit()),
        }
    }

    fn enable(&self, channel: Channel) {
        match channel {
            Channel::_1 => self.0.ccer.modify(|_, w| w.cc1e().set_bit()),
            Channel::_2 => self.0.ccer.modify(|_, w| w.cc2e().set_bit()),
            Channel::_3 => self.0.ccer.modify(|_, w| w.cc3e().set_bit()),
            Channel::_4 => self.0.ccer.modify(|_, w| w.cc4e().set_bit()),
        }
    }

    fn get_duty(&self, channel: Channel) -> u16 {
        match channel {
            Channel::_1 => self.0.ccr1.read().ccr1().bits(),
            Channel::_2 => self.0.ccr2.read().ccr2().bits(),
            Channel::_3 => self.0.ccr3.read().ccr3().bits(),
            Channel::_4 => self.0.ccr4.read().ccr4().bits(),
        }
    }

    fn get_max_duty(&self) -> u16 {
        self.0.arr.read().arr().bits()
    }

    fn get_period(&self) -> ::apb2::Ticks {
        ::apb2::Ticks(u32(self.0.psc.read().bits() * self.0.arr.read().bits()))
    }

    fn set_duty(&self, channel: Channel, duty: u16) {
        unsafe {
            match channel {
                Channel::_1 => self.0.ccr1.write(|w| w.ccr1().bits(duty)),
                Channel::_2 => self.0.ccr2.write(|w| w.ccr2().bits(duty)),
                Channel::_3 => self.0.ccr3.write(|w| w.ccr3().bits(duty)),
                Channel::_4 => self.0.ccr4.write(|w| w.ccr4().bits(duty)),
            }
        }
    }

    fn set_period<P>(&self, period: P)
    where
        P: Into<::apb2::Ticks>,
    {
        self._set_period(period.into())
    }
}