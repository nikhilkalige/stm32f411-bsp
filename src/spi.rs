//! Serial Peripheral Interface
//!
//! You can use the `Spi` interface with these SPI instances
//!
//! # SPI1
//!
//! - NSS = PA4
//! - SCK = PA5
//! - MISO = PA6
//! - MOSI = PA7
//!
//! # SPI2
//!
//! - NSS = PB9
//! - SCK = PB10
//! - MISO = PC2
//! - MOSI = PC3
//!
//! # SPI3
//!
//! - NSS = PA15
//! - SCK = PC10
//! - MISO = PC11
//! - MOSI = PC12
//!
//! # SPI4
//!
//! - NSS = PB12
//! - SCK = PB13
//! - MISO = PA11
//! - MOSI = PA1
//!
//! # SPI5
//!
//! - NSS = PB1
//! - SCK = PB0
//! - MISO = PA12
//! - MOSI = PA10

use core::any::{Any};
use core::ops::Deref;
use core::ptr;

use hal;
use nb;
use stm32f411::{DMA1, GPIOA, GPIOB, GPIOC, RCC, SPI1, SPI2, i2s2ext};

use dma::{self, Buffer, DmaStream1, DmaStream2};

/// SPI instance that can be used with the `Spi` abstraction
pub unsafe trait SPI: Deref<Target = i2s2ext::RegisterBlock> {
    /// GPIO block associated to this SPI instance
    // type GPIO: Deref<Target = gpioa::RegisterBlock>;
    type GPIO1: Deref;
    type GPIO2: Deref;
    type Ticks: Into<u32>;

    fn init(&self, gpio1: &Self::GPIO1, gpio2: &Self::GPIO2, rcc: &RCC);
}

unsafe impl SPI for SPI1 {
    type GPIO1 = GPIOA;
    type GPIO2 = GPIOA;
    type Ticks = ::apb2::Ticks;

    fn init(&self, gpioa: &Self::GPIO1, _gpio2: &Self::GPIO2, rcc: &RCC) {
        // enable GPIO's and SPI1
        rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
        rcc.apb2enr.modify(|_, w| w.spi1en().set_bit());

        unsafe {
            gpioa.moder.modify(|_, w| {
                w.moder4().bits(0b10)
                    .moder5().bits(0b10)
                    .moder6().bits(0b10)
                    .moder7().bits(0b10)
            });

            gpioa.afrl.modify(|_, w| {
                w.afrl4().bits(0b101)
                    .afrl5().bits(0b101)
                    .afrl6().bits(0b101)
                    .afrl7().bits(0b101)
            });
        }
    }
}

unsafe impl SPI for SPI2 {
    type GPIO1 = GPIOB;
    type GPIO2 = GPIOC;
    type Ticks = ::apb2::Ticks;

    fn init(&self, gpiob: &Self::GPIO1, gpioc: &Self::GPIO2, rcc: &RCC) {
        // enable GPIO's and SPI1
        rcc.ahb1enr.modify(|_, w| {
            w.gpioben().set_bit()
                .gpiocen().set_bit()
        });
        rcc.apb1enr.modify(|_, w| w.spi2en().set_bit());

        unsafe {
            gpiob.moder.modify(|_, w| {
                w.moder9().bits(0b10)
                    .moder10().bits(0b10)
            });

            gpioc.moder.modify(|_, w| {
                w.moder2().bits(0b10)
                    .moder3().bits(0b10)
            });


            gpiob.afrh.modify(|_, w| {
                w.afrh9().bits(0b101)
                    .afrh10().bits(0b101)
            });
            
            gpioc.afrl.modify(|_, w| {
                w.afrl2().bits(0b101)
                    .afrl3().bits(0b101)
            });
        }
    }
}

// unsafe impl SPI for SPI3 {
//     type GPIO = GPIOB;
// }

// unsafe impl SPI for SPI4 {
//     type GPIO = GPIOB;
// }

// unsafe impl SPI for SPI5 {
//     type GPIO = GPIOB;
// }

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

/// Serial Peripheral Interface
pub struct Spi<'a, S>(pub &'a S)
where
    S: Any + SPI;

impl<'a, S> Clone for Spi<'a, S>
where
    S: Any + SPI,
{
    fn clone(&self) -> Self {
        *self
    }
}

impl<'a, S> Copy for Spi<'a, S>
where
    S: Any + SPI,
{
}

impl<'a, S> Spi<'a, S>
where
    S: Any + SPI,
{
    /// Initializes the spi interface with speed of `speed` bits per second.
    pub fn init<B>(&self, speed: B, gpio1: &S::GPIO1, gpio2: &S::GPIO2, dma: Option<&DMA1>, rcc: &RCC)
        where B: Into<S::Ticks>
    {
        let spi = self.0;
        spi.init(gpio1, gpio2, rcc);
    }

    /// Disables the SPI bus
    ///
    /// **NOTE** This drives the NSS pin high
    pub fn disable(&self) {
        self.0.cr1.modify(|_, w| w.spe().clear_bit())
    }

    /// Enables the SPI bus
    ///
    /// **NOTE** This drives the NSS pin low
    pub fn enable(&self) {
        self.0.cr1.modify(|_, w| w.spe().set_bit())
    }
}

impl<'a, S> hal::Spi<u8> for Spi<'a, S>
where
    S: Any + SPI,
{
    type Error = Error;

    fn read(&self) -> Result<u8> {
        let spi1 = self.0;
        let sr = spi1.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.rxne().bit_is_set() {
            Ok(unsafe {
                ptr::read_volatile(&spi1.dr as *const _ as *const u8)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn send(&self, byte: u8) -> Result<()> {
        let spi1 = self.0;
        let sr = spi1.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see note above
            unsafe { ptr::write_volatile(&spi1.dr as *const _ as *mut u8, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
