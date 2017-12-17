//! Serial Peripheral Interface
//!
//! You can use the `Spi` interface with these SPI instances
//!

use core::ptr;
use hal::spi::{self, Mode, Phase, Polarity};
use nb;
use stm32f411::SPI2;

pub use stm32f411::i2s2ext::cr1::DFFW as DataSize;
pub use stm32f411::i2s2ext::cr1::CPOLW as StmPolarity;
pub use stm32f411::i2s2ext::cr1::CPHAW as StmPhase;

use gpio::{AltFunction, PB13, PA11, PA1};
use rcc::{Clocks, ENR};
use time::Hertz;

/// SPI error
#[derive(Debug, PartialEq)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)] _Extensible,
}


pub enum Direction {
    Bidirectional,
    BidirectionalRxOnly,
    Unidirectional,
}

pub enum NSS {
    Soft,
    HardInput,
    HardOutput,
}

pub struct Spi {
    spi: SPI2
}

impl Spi {
    /// MSB Format
    pub fn new(
        spi: SPI2,
        (_sck, _mosi, _miso): (PB13<AltFunction>, PA1<AltFunction>, PA11<AltFunction>),
        enr: &mut ENR
    ) -> Self
    {
        enr.apb1().modify(|_, w| w.spi2en().set_bit());
        _sck.alternate_function(6);
        _miso.alternate_function(6);
        _miso.alternate_function(5);

        Spi { spi }
    }

    pub fn direction(&self, direction: Direction) {
        match direction {
            Direction::Bidirectional => self.spi.cr1.modify(|_, w| w.bidimode().clear_bit()),
            Direction::BidirectionalRxOnly => self.spi.cr1.modify(|_, w| w.rxonly().set_bit()),
            Direction::Unidirectional => self.spi.cr1.modify(|_, w| w.bidimode().set_bit()),
        }
    }

    pub fn data_size(&self, size: DataSize) {
        self.spi.cr1.modify(|_, w| w.dff().variant(size));
    }

    pub fn clk_polarity(&self, polarity: Polarity) {
        let pol = match polarity {
            Polarity::IdleLow => StmPolarity::LOW,
            Polarity::IdleHigh => StmPolarity::HIGH,
        };
        self.spi.cr1.modify(|_, w| w.cpol().variant(pol));
    }

    pub fn clk_phase(&self, phase: Phase) {
        let pha = match phase {
            Phase::CaptureOnFirstTransition => StmPhase::_1EDGE,
            Phase::CaptureOnSecondTransition => StmPhase::_2EDGE,
        };
        self.spi.cr1.modify(|_, w| w.cpha().variant(pha));
    }

    pub fn nss(&self, nss: NSS) {
        match nss {
            NSS::HardInput => self.spi.cr1.modify(|_, w| w.ssm().clear_bit()),
            NSS::HardOutput => self.spi.cr2.modify(|_, w| w.ssoe().set_bit()),
            NSS::Soft => {
                self.spi.cr1.modify(|_, w| w.ssm().set_bit());
                self.spi.cr2.modify(|_, w| w.ssoe().set_bit());
            }
        }
    }

    pub fn set_frequency<F>(&self, clocks: Clocks, freq: F)
    where
        F: Into<Hertz>,
    {
        let br = match clocks.pclk1().0 / freq.into().0 {
            0 => unreachable!(),
            1...2 => 0b000,
            3...5 => 0b001,
            6...11 => 0b010,
            12...23 => 0b011,
            24...47 => 0b100,
            48...95 => 0b101,
            96...191 => 0b110,
            _ => 0b111,
        };
        self.spi.cr1.modify(|_, w| w.br().bits(br));
    }

    pub fn msb_first(&self, msb: bool) {
        if msb {
            self.spi.cr1.modify(|_, w| w.lsbfirst().clear_bit());
        } else {
            self.spi.cr1.modify(|_, w| w.lsbfirst().set_bit());
        }
    }

    pub fn ti_mode(&self, mode: bool) {
        if mode {
            self.spi.cr2.modify(|_, w| w.frf().set_bit());
        } else {
            self.spi.cr2.modify(|_, w| w.frf().clear_bit());
        }
    }

    pub fn crc_calculation(&self, crc: bool) {
        if crc {
            self.spi.cr1.modify(|_, w| w.crcen().set_bit());
        } else {
            self.spi.cr1.modify(|_, w| w.crcen().clear_bit());
        }
    }

    pub fn enable(&self) {
        self.spi.cr1.modify(|_, w| w.spe().set_bit())
    }

    pub fn disable(&self) {
        self.spi.cr1.modify(|_, w| w.spe().clear_bit())
    }
}