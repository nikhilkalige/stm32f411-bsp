use core::marker::PhantomData;

use cast::{u16, u32};
use hal::PwmPin;
use stm32f411::{TIM1};

// use afio::MAPR;
use bb;
use gpio::{AltFunction, PA8};
use rcc::{Clocks, ENR};
use time::Hertz;

pub struct T1C1 {
    _0: (),
}

pub struct Pwm<CHANNEL, PIN> {
    _channel: PhantomData<CHANNEL>,
    pin: PIN,
}

impl<PIN> Pwm<T1C1, PIN> {
    pub fn unwrap(mut self) -> PIN {
        self.disable();
        self.pin
    }
}

impl<PIN> PwmPin for Pwm<T1C1, PIN> {
    type Duty = u16;

    fn disable(&mut self) {
        unsafe {
            bb::clear(&(*TIM1::ptr()).ccer, 4);
        }
    }

    fn enable(&mut self) {
        unsafe {
            bb::set(&(*TIM1::ptr()).ccer, 4);
        }
    }

    fn get_duty(&self) -> u16 {
        unsafe { (*TIM1::ptr()).ccr2.read().ccr2().bits() }
    }

    fn get_max_duty(&self) -> u16 {
        unsafe { (*TIM1::ptr()).arr.read().arr().bits() }
    }

    fn set_duty(&mut self, duty: u16) {
        unsafe { (*TIM1::ptr()).ccr2.write(|w| w.ccr2().bits(duty)) }
    }
}

pub trait PwmExt<Pins> {
    type Channels;
    type Time;

    fn pwm<F>(
        self,
        pins: Pins,
        frequency: F,
        clocks: Clocks,
        enr: &mut ENR,
    ) -> Self::Channels
    where
        F: Into<Self::Time>;
}

impl PwmExt<PA8<AltFunction>> for TIM1 {
    type Channels = Pwm<T1C1, PA8<AltFunction>>;
    type Time = Hertz;

    fn pwm<F>(
        self,
        pin: PA8<AltFunction>,
        frequency: F,
        clocks: Clocks,
        enr: &mut ENR,
    ) -> Self::Channels
    where
        F: Into<Self::Time>,
    {
        enr.apb2().modify(|_, w| w.tim1en().set_bit());
        pin.alternate_function(1);

        self.ccmr1_output.modify(|_, w| unsafe {{
            w.oc1pe().set_bit()
                .oc1m().bits(0b110)
                .oc2pe().set_bit()
                .oc2m().bits(0b110)
        }});

        self.ccmr2_output.modify(|_, w| unsafe{{
            w.oc3pe().set_bit()
                .oc3m().bits(0b110)
                .oc4pe().set_bit()
                .oc4m().bits(0b110)
        }});

        self.ccer.modify(|_, w| {
            w.cc1p().clear_bit()
                .cc2p().clear_bit()
                .cc3p().clear_bit()
                .cc4p().clear_bit()
        });

        self.bdtr.modify(|_, w| w.moe().set_bit());

        self.cr1.write(|w| unsafe {
            w.cms().bits(0b00)
                .dir().set_bit()
                .opm().clear_bit()
                .cen().set_bit()
        });

        let clk = clocks.pclk2().0 * if clocks.ppre2() == 1 { 1 } else { 2 };
        let freq = frequency.into().0;
        let ticks = clk / freq;
        let psc = u16(ticks / (1 << 16)).unwrap();
        self.psc.write(|w| unsafe { w.psc().bits(psc) });
        let arr = u16(ticks / u32(psc + 1)).unwrap();
        self.arr.write(|w| unsafe { w.arr().bits(arr) });

        Pwm {
            _channel: PhantomData,
            pin,
        }
    }
}