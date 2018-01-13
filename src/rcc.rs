use core::cmp;

use cast::u32;
use stm32f411::{rcc, RCC};

use time::Hertz;

pub enum ClockSource {
    Hsi,
    Hse,
    Pll,
}

pub trait RccExt {
    fn split(self) -> Rcc;
}

impl RccExt for RCC {
    fn split(self) -> Rcc {
        Rcc {
            cfgr: CFGR {
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
            },
            enr: ENR { _0: () },
        }
    }
}

pub struct Rcc {
    pub cfgr: CFGR,
    pub enr: ENR,
}

const HSI: u32 = 16_000_000; // Hz

pub struct CFGR {
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
}

impl CFGR {
    pub fn hclk<F>(&mut self, freq: F) -> &mut Self
    where
        F: Into<Hertz>
    {
        self.hclk = Some(freq.into().0);
        self
    }

    pub fn pclk1<F>(&mut self, freq: F) -> &mut Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    pub fn pclk2<F>(&mut self, freq: F) -> &mut Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    pub fn sysclk<F>(&mut self, freq: F) -> &mut Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    pub fn freeze(self, source: ClockSource) -> Clocks {
        match source {
            ClockSource::Hsi => self.hsi(),
            ClockSource::Hse => self.hsi(),
            ClockSource::Pll => self.pll()
        }
    }

    fn hsi(self) -> Clocks {
        Clocks {
            hclk: Hertz(HSI),
            pclk1: Hertz(HSI),
            pclk2: Hertz(HSI),
            ppre1: 1,
            ppre2: 1,
            sysclk: Hertz(HSI),
        }
    }

    fn pll(self) -> Clocks {
        let pllmul = (4 * self.sysclk.unwrap_or(HSI) + HSI) / HSI / 2;
        let pllmul = cmp::min(cmp::max(pllmul, 2), 16);
        let pllmul_bits = if pllmul == 2 {
            None
        } else {
            Some(pllmul as u8 - 2)
        };

        let sysclk = pllmul * HSI / 2;

        assert!(sysclk < 72_000_000);

        let hpre_bits = self.hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3...5 => 0b1001,
                6...11 => 0b1010,
                12...39 => 0b1011,
                40...95 => 0b1100,
                96...191 => 0b1101,
                192...383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);

        let hclk = sysclk / (1 << (hpre_bits - 0b0111));

        assert!(hclk < 72_000_000);

        let ppre1_bits = self.pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 < 36_000_000);

        let ppre2_bits = self.pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 < 72_000_000);

        // adjust flash wait states
        // unsafe {
        //     acr.acr().write(|w| {
        //         w.latency().bits(if sysclk <= 24_000_000 {
        //             0b000
        //         } else if sysclk <= 48_000_000 {
        //             0b001
        //         } else {
        //             0b010
        //         })
        //     })
        // }

        let rcc = unsafe { &*RCC::ptr() };
        if let Some(pllmul_bits) = pllmul_bits {
            // use PLL as source

            rcc.pllcfgr.write(|w| unsafe { w.pllm().bits(pllmul_bits) });

            rcc.cr.write(|w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_set() {}

            rcc.cfgr.modify(|_, w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .pll()
            });
        } else {
            // use HSI as source

            rcc.cfgr.write(|w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .hsi()
            });
        }

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
        }
    }
}

#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    // TODO remove `allow`
    #[allow(dead_code)]
    ppre2: u8,
    sysclk: Hertz,
}

impl Clocks {
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }
}

// TODO HSE support
// pub enum Source {
//     Hsi,
//     Hse(Hertz),
// }

pub struct ENR {
    _0: (),
}

impl ENR {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ahb1(&mut self) -> &rcc::AHB1ENR {
        unsafe { &(*RCC::ptr()).ahb1enr }
    }

    pub(crate) fn ahb2(&mut self) -> &rcc::AHB2ENR {
        unsafe { &(*RCC::ptr()).ahb2enr }
    }

    pub(crate) fn apb1(&mut self) -> &rcc::APB1ENR {
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn apb2(&mut self) -> &rcc::APB2ENR {
        unsafe { &(*RCC::ptr()).apb2enr }
    }
}

