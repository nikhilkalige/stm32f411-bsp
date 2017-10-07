use core::any::{Any, TypeId};
use core::marker::Unsize;
use core::ops::Deref;
use core::ptr;

use cast::u16;
use hal;
use hal::serial::Write;
use nb;
use time::U32Ext;

// use static_ref::Ref;
use stm32f411::{usart1, USART1, USART2, USART6};

/// Specialized `Result` type
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

/// IMPLEMENTATION DETAIL
pub unsafe trait Usart: Deref<Target = usart1::RegisterBlock> {
    /// IMPLEMENTATION DETAIL
    type Ticks: Into<u32>;
}

unsafe impl Usart for USART1 {
    type Ticks = ::apb2::Ticks;
}

unsafe impl Usart for USART2 {
    type Ticks = ::apb1::Ticks;
}

unsafe impl Usart for USART6 {
    type Ticks = ::apb1::Ticks;
}

/// An error
#[derive(Debug)]
pub enum Error {
    /// De-synchronization, excessive noise or a break character detected
    Framing,
    /// Noise detected in the received frame
    Noise,
    /// RX buffer overrun
    Overrun,
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

/// Serial interface
///
/// # Interrupts
///
/// - RXNE
pub struct Serial<'a, U>(pub &'a U) where U: Any + Usart;

impl<'a, U> Clone for Serial<'a, U>
    where U: Any + Usart
{
    fn clone(&self) -> Self {
        *self
    }
}

impl<'a, U> Copy for Serial<'a, U> where U: Any + Usart {}

impl<'a, U> Serial<'a, U>
    where U: Any + Usart
{
    /// Initializes the serial interface with a baud rate of `baut_rate` bits
    /// per second
    ///
    /// The serial interface will be configured to use 8 bits of data, 1 stop
    /// bit, no hardware control and to omit parity checking
    pub fn init<B>(&self, baud_rate: B)
        where B: Into<U::Ticks>
    {
        self.set_baud_rate(baud_rate);
        self.enable();
    }

    pub fn set_baud_rate<B>(&self, baud_rate: B)
        where B: Into<U::Ticks>
    {
        let ticks = baud_rate.into();
        let baud = ticks.into();
        self.0.brr.write(|w| unsafe { w.bits(baud) });
    }

    pub fn enable(&self) {
        self.0.cr1.modify(|_, w|
            w.ue().set_bit()
             .te().set_bit()
             .re().set_bit());
    }

    pub fn disable(&self) {
        self.0.cr1.modify(|_, w| w.ue().clear_bit());
    }
}

impl<'a, U> hal::serial::Read<u8> for Serial<'a, U>
where
    U: Any + Usart,
{
    type Error = Error;

    fn read(&self) -> Result<u8> {
        let usart = self.0;
        let sr = usart.sr.read();

        if sr.ore().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bit_is_set() {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bit_is_set() {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.rxne().bit_is_set() {
            // NOTE(read_volatile) the register is 9 bits big but we'll only
            // work with the first 8 bits
            Ok(unsafe {
                ptr::read_volatile(&usart.dr as *const _ as *const u8)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'a, U> Write<u8> for Serial<'a, U>
where
    U: Any + Usart,
{
    type Error = Error;

    fn write(&self, byte: u8) -> Result<()> {
        let usart = self.0;
        let sr = usart.sr.read();

        if sr.ore().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bit_is_set() {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bit_is_set() {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see NOTE in the `read` method
            unsafe {
                ptr::write_volatile(&usart.dr as *const _ as *mut u8, byte)
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'a, U> hal::serial::Write<&'a [u8]> for Serial<'a, U>
where
    U: Any + Usart,
{
    type Error = Error;

    fn write<'b>(&self, buffer: &'b [u8]) -> Result<()> {
        for byte in buffer {
            let status = block!(self.write(*byte));
            match status {
                Err(e) => return Err(nb::Error::Other(e)),
                _ => {}
            }
        }
        Ok(())
    }
}

impl<'a, U> hal::serial::Write<&'a str> for Serial<'a, U>
where
    U: Any + Usart,
{
    type Error = Error;

    fn write<'b>(&self, string: &'a str) -> Result<()> {
        self.write(string.as_bytes())
    }
}

/*
impl<'a> Serial<'a, USART1> {
    /// Starts a DMA transfer to receive serial data into a `buffer`
    ///
    /// This will mutably lock the `buffer` preventing borrowing its contents
    /// The `buffer` can be `release`d after the DMA transfer finishes
    // TODO support circular mode + half transfer interrupt as a double
    // buffering mode
    pub fn read_exact<B>(&self,
                         dma1: &DMA1,
                         buffer: Ref<Buffer<B, Dma1Channel5>>)
                         -> ::core::result::Result<(), dma::Error>
        where B: Unsize<[u8]>
    {
        let usart1 = self.0;

        if dma1.ccr5.read().en().is_set() {
            return Err(dma::Error::InUse);
        }

        let buffer: &mut [u8] = buffer.lock_mut();

        dma1.cndtr5
            .write(|w| unsafe { w.ndt().bits(u16(buffer.len()).unwrap()) });
        dma1.cpar5
            .write(|w| unsafe { w.bits(&usart1.dr as *const _ as u32) });
        dma1.cmar5
            .write(|w| unsafe { w.bits(buffer.as_ptr() as u32) });
        dma1.ccr5.modify(|_, w| w.en().set());

        Ok(())
    }

    /// Starts a DMA transfer to send `buffer` through this serial port
    ///
    /// This will immutably lock the `buffer` preventing mutably borrowing its
    /// contents. The `buffer` can be `release`d after the DMA transfer finishes
    pub fn write_all<B>(&self,
                        dma1: &DMA1,
                        buffer: Ref<Buffer<B, Dma1Channel4>>)
                        -> ::core::result::Result<(), dma::Error>
        where B: Unsize<[u8]>
    {
        let usart1 = self.0;

        if dma1.ccr4.read().en().is_set() {
            return Err(dma::Error::InUse);
        }

        let buffer: &[u8] = buffer.lock();

        dma1.cndtr4
            .write(|w| unsafe { w.ndt().bits(u16(buffer.len()).unwrap()) });
        dma1.cpar4
            .write(|w| unsafe { w.bits(&usart1.dr as *const _ as u32) });
        dma1.cmar4
            .write(|w| unsafe { w.bits(buffer.as_ptr() as u32) });
        dma1.ccr4.modify(|_, w| w.en().set());

        Ok(())
    }
}
*/