//! Serial Peripheral Interface
//!
//! You can use the `Spi` interface with these SPI instances
//!

use core::any::Any;
use core::ops::Deref;
use core::ptr;
use core::marker::Unsize;

use cast::u16;

use static_ref::Static;
use hal;
use nb;
use stm32f411::{SPI1, SPI4, i2s2ext};

//use dma::{self, Buffer, DmaStream1, DmaStream2};
use dma2::{self, DMA, Dma, Buffer, DMAStream};

/// SPI instance that can be used with the `Spi` abstraction
pub unsafe trait SPI: Deref<Target = i2s2ext::RegisterBlock> {
    // type Ticks: Into<u32>;

    // fn init(&self, role: i2s2ext::cr1::MSTRW);
}

unsafe impl SPI for SPI1 {
}

unsafe impl SPI for SPI4 {
}

/// SPI result
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

/// Interrupt event
pub enum Event {
    /// RX buffer Not Empty (new data available)
    Rxne,
    /// Transmission Complete
    Tc,
    /// TX buffer Empty (more data can be send)
    Txe,
}

pub enum Direction {
    Bidirectional,
    BidirectionalRxOnly,
    Unidirectional,
}

pub use stm32f411::i2s2ext::cr1::DFFW as DataSize;
pub use stm32f411::i2s2ext::cr1::CPOLW as Polarity;
pub use stm32f411::i2s2ext::cr1::CPHAW as Phase;
pub use stm32f411::i2s2ext::cr1::BRW as BaudRatePreScale;
pub use stm32f411::i2s2ext::cr1::MSTRW as Role;

pub enum NSS {
    Soft,
    HardInput,
    HardOutput,
}

/// Serial Peripheral Interface
pub struct Spi<'a, S, D>
    where S: Any + SPI,
          D: Any + DMA
{
    pub reg: &'a S,
    pub role: Role,
    // pub dmarx: Option<&'a D>,
    pub dmarx: Option<&'a Dma<'a, D>>,
    pub dmatx: Option<&'a Dma<'a, D>>,
}

// impl<'a, S, D> Clone for Spi<'a, S, D>
//     where S: Any + SPI,
//           D: Any + DMA
// {
//     fn clone(&self) -> Self {
//         *self
//     }
// }

// impl<'a, S> Copy for Spi<'a, S> where S: Any + SPI {}

impl<'a, S, D> Spi<'a, S, D>
    where S: Any + SPI,
          D: Any + DMA
{
    // pub fn new(reg: &'a S, role: Role, dmarx: Option<&'a D>, dmatx: Option<&'a Dma<'a, D>>) -> Spi<'a, S, D> {
    pub fn new(reg: &'a S, role: Role, dmarx: Option<&'a Dma<'a, D>>, dmatx: Option<&'a Dma<'a, D>>) -> Spi<'a, S, D> {
        Spi {reg: reg, role: role, dmarx:dmarx, dmatx:dmatx}
    }

    pub fn init(&self, role: Role) {
        self.reg.cr1.modify(|_, w| w.mstr().variant(role));
    }

    pub fn direction(&self, direction: Direction) {
        match direction {
            Direction::Bidirectional => self.reg.cr1.modify(|_, w| w.bidimode().clear_bit()),
            Direction::BidirectionalRxOnly => self.reg.cr1.modify(|_, w| w.rxonly().set_bit()),
            Direction::Unidirectional => self.reg.cr1.modify(|_, w| w.bidimode().set_bit()),
        }
    }

    pub fn data_size(&self, size: DataSize) {
        self.reg.cr1.modify(|_, w| w.dff().variant(size));
    }

    pub fn clk_polarity(&self, polarity: Polarity) {
        self.reg.cr1.modify(|_, w| w.cpol().variant(polarity));
    }

    pub fn clk_phase(&self, phase: Phase) {
        self.reg.cr1.modify(|_, w| w.cpha().variant(phase));
    }

    pub fn nss(&self, nss: NSS) {
        match nss {
            NSS::HardInput => self.reg.cr1.modify(|_, w| w.ssm().clear_bit()),
            NSS::HardOutput => self.reg.cr2.modify(|_, w| w.ssoe().set_bit()),
            NSS::Soft => {
                self.reg.cr1.modify(|_, w| w.ssm().set_bit());
                self.reg.cr2.modify(|_, w| w.ssoe().set_bit());
            }
        }
    }

    pub fn baud_rate_prescaler(&self, scale: BaudRatePreScale) {
        self.reg.cr1.modify(|_, w| w.br().variant(scale));
    }

    pub fn msb_first(&self, msb: bool) {
        if msb {
            self.reg.cr1.modify(|_, w| w.lsbfirst().clear_bit());
        } else {
            self.reg.cr1.modify(|_, w| w.lsbfirst().set_bit());
        }
    }

    pub fn ti_mode(&self, mode: bool) {
        if mode {
            self.reg.cr2.modify(|_, w| w.frf().set_bit());
        } else {
            self.reg.cr2.modify(|_, w| w.frf().clear_bit());
        }
    }

    pub fn crc_calculation(&self, crc: bool) {
        if crc {
            self.reg.cr1.modify(|_, w| w.crcen().set_bit());
        } else {
            self.reg.cr1.modify(|_, w| w.crcen().clear_bit());
        }
    }

    pub fn enable(&self) {
        self.reg.cr1.modify(|_, w| w.spe().set_bit())
    }

    pub fn disable(&self) {
        self.reg.cr1.modify(|_, w| w.spe().clear_bit())
    }

    pub fn send_dma<B>(&self, buffer: &Static<Buffer<B>>)
        -> ::core::result::Result<(), dma2::Error>
    where B: Unsize<[u8]>
    {
        let spi = self.reg;
        let dma = self.dmatx.unwrap();

        if dma.is_enabled() {
            return Err(dma2::Error::InUse)
        }

        let buffer: &[u8] = buffer.lock();
        dma.set_config(
            buffer.as_ptr() as u32,
            &spi.dr as *const _ as u32,
            u16(buffer.len()).unwrap()
        );

        dma.enable();
        Ok(())
    }

    pub fn rxtx_dma<B>(&self,
        tx_buffer: &Static<Buffer<B>>,
        rx_buffer: &Static<Buffer<B>>)
        -> ::core::result::Result<(), dma2::Error>
    where B: Unsize<[u8]>
    {
        let spi = self.reg;
        let dma_tx = self.dmatx.unwrap();
        let dma_rx = self.dmarx.unwrap();

        if dma_tx.is_enabled() || dma_rx.is_enabled() {
            return Err(dma2::Error::InUse)
        }

        let _tx_buffer: &[u8] = tx_buffer.lock();
        dma_tx.set_config(
            _tx_buffer.as_ptr() as u32,
            &spi.dr as *const _ as u32,
            u16(_tx_buffer.len()).unwrap()
        );

        let _rx_buffer: &[u8] = rx_buffer.lock();
        dma_rx.set_config(
            &spi.dr as *const _ as u32,
            _rx_buffer.as_ptr() as u32,
            u16(_rx_buffer.len()).unwrap()
        );

        dma_rx.enable();
        dma_tx.enable();
        Ok(())
    }

    pub fn transfer<B>(&self, tx_buffer: &[B], rx_buffer: &[B])
        -> ::core::result::Result<(), dma2::Error>
    where B: Unsize<[u8]>
    {
        let spi = self.reg;
        let dma_tx = self.dmatx.unwrap();
        let dma_rx = self.dmarx.unwrap();

        if dma_tx.is_enabled() {
            return Err(dma2::Error::InUse)
        }

        dma_tx.set_config(
            tx_buffer.as_ptr() as u32,
            &spi.dr as *const _ as u32,
            u16(tx_buffer.len()).unwrap()
        );

        dma_rx.set_config(
            &spi.dr as *const _ as u32,
            rx_buffer.as_ptr() as u32,
            u16(tx_buffer.len()).unwrap()
        );

        dma_tx.enable();
        dma_rx.enable();
        Ok(())
    }
}

impl<'a, S, D> hal::Spi<u8> for Spi<'a, S, D>
    where S: Any + SPI,
          D: Any + DMA
{
    type Error = Error;

    fn read(&self) -> Result<u8> {
        let spi = self.reg;
        let sr = spi.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.rxne().bit_is_set() {
            Ok(unsafe { ptr::read_volatile(&spi.dr as *const _ as *const u8) })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn send(&self, byte: u8) -> Result<()> {
        let spi = self.reg;
        let sr = spi.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see note above
            unsafe { ptr::write_volatile(&spi.dr as *const _ as *mut u8, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}