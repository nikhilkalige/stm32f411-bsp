//! Serial Peripheral Interface
//!
//! You can use the `Spi` interface with these SPI instances
//!

use core::marker::Unsize;
use core::ptr;
use cast::u16;

use hal::spi::{self, DmaWrite, Mode, Phase, Polarity};
use hal::blocking;
use nb;
use stm32f411::SPI4;

pub use stm32f411::i2s2ext::cr1::DFFW as DataSize;
pub use stm32f411::i2s2ext::cr1::CPOLW as StmPolarity;
pub use stm32f411::i2s2ext::cr1::CPHAW as StmPhase;
pub use stm32f411::i2s2ext::cr1::MSTRW as Role;

use gpio::{AltFunction, PA1, PA11, PB13};
use rcc::{Clocks, ENR};
use time::Hertz;
use dma::{D2S1, D2S4, Transfer as DmaTransferObject, Static};

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

pub struct Spi<DmaTxStream, DmaRxStream> {
    spi: SPI4,
    dmatx: Option<DmaTxStream>,
    dmarx: Option<DmaRxStream>,
}

impl<DmaTxStream, DmaRxStream> Spi<DmaTxStream, DmaRxStream> {
    /// MSB Format
    pub fn new(
        spi: SPI4,
        (_sck, _mosi, _miso): (PB13<AltFunction>, PA1<AltFunction>, PA11<AltFunction>),
        enr: &mut ENR,
    ) -> Self {
        enr.apb2().modify(|_, w| w.spi4en().set_bit());
        _sck.alternate_function(6);
        _mosi.alternate_function(5);
        _miso.alternate_function(6);

        Spi {
            spi,
            dmatx: None,
            dmarx: None,
        }
    }

    pub fn direction(&self, direction: Direction) {
        match direction {
            Direction::Bidirectional => self.spi.cr1.modify(|_, w| w.bidimode().clear_bit()),
            Direction::BidirectionalRxOnly => self.spi.cr1.modify(|_, w| w.rxonly().set_bit()),
            Direction::Unidirectional => self.spi.cr1.modify(|_, w| w.bidimode().set_bit()),
        }
    }

    pub fn set_role(&self, role: Role) {
        self.spi.cr1.modify(|_, w| w.mstr().variant(role));
    }

    pub fn data_size(&self, size: DataSize) {
        self.spi.cr1.modify(|_, w| w.dff().variant(size));
    }

    pub fn set_mode(&self, mode: Mode) {
        let pol = match mode.polarity {
            Polarity::IdleLow => StmPolarity::LOW,
            Polarity::IdleHigh => StmPolarity::HIGH,
        };
        self.spi.cr1.modify(|_, w| w.cpol().variant(pol));

        let phase = match mode.phase {
            Phase::CaptureOnFirstTransition => StmPhase::_1EDGE,
            Phase::CaptureOnSecondTransition => StmPhase::_2EDGE,
        };
        self.spi.cr1.modify(|_, w| w.cpha().variant(phase));
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

impl<DmaTxStream, DmaRxStream> spi::FullDuplex<u8> for Spi<DmaTxStream, DmaRxStream> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let sr = self.spi.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.rxne().bit_is_set() {
            Ok(unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const u8) })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see note above
            unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<DmaTxStream, DmaRxStream> blocking::spi::FullDuplex<u8> for Spi<DmaTxStream, DmaRxStream> {
    type Error = Error;

    fn transfer<'b>(&mut self, bytes: &'b mut [u8]) -> Result<&'b [u8], Error> {
        blocking::spi::transfer(self, bytes)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for byte in bytes {
            'l: loop {
                let sr = self.spi.sr.read();

                // ignore overruns because we don't care about the incoming data
                // if sr.ovr().bit_is_set() {
                // Err(nb::Error::Other(Error::Overrun))
                // } else
                if sr.modf().bit_is_set() {
                    return Err(Error::ModeFault);
                } else if sr.crcerr().bit_is_set() {
                    return Err(Error::Crc);
                } else if sr.txe().bit_is_set() {
                    // NOTE(write_volatile) see note above
                    unsafe {
                        ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, *byte)
                    }
                    break 'l;
                } else {
                    // try again
                }
            }
        }

        // wait until the transmission of the last byte is done
        while self.spi.sr.read().bsy().bit_is_set() {}

        // clear OVR flag
        unsafe {
            ptr::read_volatile(&self.spi.dr as *const _ as *const u8);
        }
        self.spi.sr.read();

        Ok(())
    }
}


impl<B> DmaWrite<B, u8> for Spi<D2S1, D2S4>
where B: Unsize<[u8]> + 'static
{
    // fn send_dma<Buffer>(self, words: &'static mut Buffer)
    //fn send_dma<B, T>(self, words: &'static mut B) -> DmaTransferObject<D2S1, Static<[u8]>, Spi<D2S1, D2S4>>
    type Transfer = DmaTransferObject<D2S1, &'static mut B, Self>;

    fn send_dma(self, words: &'static mut B) -> Self::Transfer
        // where B: Unsize<[u8]>, T: Transfer<Item=B, Payload=Self>
    {
        {
            // Assume dma object does not panic
            let txstream = self.dmatx.as_ref().unwrap();
            // This is a sanity check. Due to move semantics the channel is *never* in use at this point
            debug_assert!(!txstream.is_enabled());

            let slice: &mut [u8] = words;
            txstream.set_config(
                slice.as_ptr() as u32,
                &self.spi.dr as *const _ as u32,
                u16(slice.len()).unwrap(),
            );

            txstream.enable();
        }
        DmaTransferObject::new(words, self)
    }
}

/*
impl DmaRead<u8> for Spi<D2S1, D2S4> {
    type Transfer = DmaTransferObject<D2S1, Static<[u8]>, Spi<D2S1, D2S4>>;

    fn recieve_dma<Buffer, Spi>(self, words: &'static mut Buffer) -> Self::Transfer
    where
        Buffer: Unsize<[u8]>,
    {
        {
            // Assume dma object does not panic
            let rxstream = self.dmarx.as_ref().unwrap();
            // This is a sanity check. Due to move semantics the channel is *never* in use at this point
            debug_assert!(!rxstream.is_enabled());

            let slice: &mut [u8] = words;
            rxstream.set_config(
                &self.spi.dr as *const _ as u32,
                slice.as_ptr() as u32,
                u16(slice.len()).unwrap(),
            );

            rxstream.enable();
        }
        DmaTransferObject::new(words, self)
    }
}

impl DmaReadWrite<u8> for Spi<D2S1, D2S4> {
    type Transfer = DmaTransferObject<D2S1, (Static<[u8]>, Static<[u8]>), Spi<D2S1, D2S4>>;

    fn transfer_dma<Buffer, Payload>(
        self,
        tx_words: &'static mut Buffer,
        rx_words: &'static mut Buffer,
    ) -> Self::Transfer
    where
        Buffer: Unsize<[u8]>,
    {
        {
            // Assume dma object does not panic
            let rx_stream = self.dmarx.as_ref().unwrap();
            let tx_stream = self.dmarx.as_ref().unwrap();
            // This is a sanity check. Due to move semantics the channel is *never* in use at this point
            debug_assert!(!rx_stream.is_enabled());

            let rx_slice: &mut [u8] = rx_words;
            rx_stream.set_config(
                &self.spi.dr as *const _ as u32,
                rx_slice.as_ptr() as u32,
                u16(rx_slice.len()).unwrap(),
            );

            let tx_slice: &mut [u8] = tx_words;
            tx_stream.set_config(
                tx_slice.as_ptr() as u32,
                &self.spi.dr as *const _ as u32,
                u16(tx_slice.len()).unwrap(),
            );

            rx_stream.enable();
            tx_stream.enable();
        }
        DmaTransferObject::new((tx_words, rx_words), self)
    }
}

impl DmaWrite<u8> for Spi<D2S1, D2S4>
{
    // fn send_dma<Buffer>(self, words: &'static mut Buffer)
    //fn send_dma<B, T>(self, words: &'static mut B) -> DmaTransferObject<D2S1, Static<[u8]>, Spi<D2S1, D2S4>>
    type Transfer = DmaTransferObject<D2S1, Spi<D2S1, D2S4>>;

    fn send_dma(self, words: &'static mut [u8]) -> Self::Transfer
        // where B: Unsize<[u8]>, T: Transfer<Item=B, Payload=Self>
    {
        {
            // Assume dma object does not panic
            let txstream = self.dmatx.as_ref().unwrap();
            // This is a sanity check. Due to move semantics the channel is *never* in use at this point
            debug_assert!(!txstream.is_enabled());

            let slice: &mut [u8] = words;
            txstream.set_config(
                slice.as_ptr() as u32,
                &self.spi.dr as *const _ as u32,
                u16(slice.len()).unwrap(),
            );

            txstream.enable();
        }
        DmaTransferObject::new(self)
    }
}
*/
