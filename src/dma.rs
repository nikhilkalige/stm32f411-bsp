use core::marker::{Unsize, PhantomData};
use core::ops;
use core::sync::atomic::{self, Ordering};
use hal::dma::Transfer as DmaTransfer;
use hal::dma::Error;

use nb;
use stm32f411::{DMA1, DMA2, dma2};
pub use stm32f411::dma2::scr::CHSELW as Channel;
pub use stm32f411::dma2::scr::DIRW as Direction;
pub use stm32f411::dma2::scr::MBURSTW as MemoryBurst;
pub use stm32f411::dma2::scr::PBURSTW as PeripheralBurst;
pub use stm32f411::dma2::scr::PLW as Priority;
pub use stm32f411::dma2::scr::MSIZEW as DataSize;

use rcc::ENR;

pub type Static<T> = &'static mut T;

#[derive(Copy, Clone)]
pub enum Mode {
    Normal,
    Circular,
    PeripheralFlowControl,
}

/// An on-going DMA transfer
// This is bit like a `Future` minus the panicking `poll` method
pub struct Transfer<Stream, B, Payload>
where
    B: 'static
{
    _stream: PhantomData<Stream>,
    buffer: B,
    payload: Payload,
}

impl<Stream, B, Payload> Transfer<Stream, B, Payload> {
    pub(crate) fn new(buffer: B, payload: Payload) -> Self {
        Transfer {
            _stream: PhantomData,
            buffer,
            payload,
        }
    }
}

// impl DmaTransfer<Buffer, Payload> for Transfer<Read, Stream, Buffer, Payload> {
//     fn deref(&Self) -> &Buffer {
//         self.buffer
//     }
// }
// impl<Buffer, Payload> ops::Deref for Transfer<Read, Stream, Buffer, Payload> {
//     type Target = Buffer;

//     fn deref(&self) -> &Buffer {
//         self.buffer
//     }
// }

macro_rules! streams {
    ($DMA:ident, $dmaen:ident, {
        $($STREAM:ident: ($STATUS:ident, ($TCIF:ident, $HTIF:ident, $TEIF:ident, $DMEIF:ident),
                          $INT:ident, ($CTCIF: ident, $CHTIF: ident, $CTEIF: ident, $CDMEIF: ident),
                          $SCR:ident, $SNDTR:ident, $SPAR:ident, $SM0AR:ident, $SM1AR:ident),)+
    }) => {
        impl DmaExt for $DMA {
            type Streams = ($($STREAM),+);

            fn split(self, enr: &mut ENR) -> ($($STREAM),+) {
                enr.ahb1().modify(|_, w| w.$dmaen().set_bit());

                ($($STREAM { _0: () }),+)
            }
        }

        $(
            pub struct $STREAM { _0: () }

            impl<B, Payload> DmaTransfer for Transfer<$STREAM, B, Payload>
            where
                B: Sized
            {
                type Item = B;
                type Payload = Payload;

                fn deref(&self) -> &Self::Item {
                    return &self.buffer
                }

                fn is_done(&self) -> Result<bool, Error> {
                    let dma = unsafe { &*$DMA::ptr() };
                    let isr = dma.$STATUS.read();

                    if isr.$TEIF().bit_is_set() {
                        return Err(Error::Transfer);
                    } else {
                        return Ok(isr.$TCIF().bit_is_set());
                    }
                }

                fn wait(self) -> Result<(Self::Item, Payload), Error> {
                    while !self.is_done()? {}

                    atomic::compiler_fence(Ordering::SeqCst);

                    let dma = unsafe { &*$DMA::ptr() };

                    // clear the "transfer complete" flag
                    dma.$INT.write(|w| w.$CTCIF().set_bit());

                    // disable this channel
                    // XXX maybe not required?
                    // dma.$ccr.modify(|_, w| w.en().clear_bit());

                    Ok((self.buffer, self.payload))
                }
            }

            impl $STREAM {
                pub fn channel(&self, channel: Channel) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.chsel().variant(channel));
                }

                pub fn direction(&self, direction: Direction) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.dir().variant(direction));
                }

                pub fn peripheral_increment(&self, inc: bool) {
                    let dma = unsafe { &*$DMA::ptr() };
                    if inc {
                        dma.$SCR.modify(|_, w| w.pinc().enable());
                    } else {
                        dma.$SCR.modify(|_, w| w.pinc().disable());
                    }
                }

                pub fn memory_increment(&self, inc: bool) {
                    let dma = unsafe { &*$DMA::ptr() };
                    if inc {
                        dma.$SCR.modify(|_, w| w.minc().enable());
                    } else {
                        dma.$SCR.modify(|_, w| w.minc().disable());
                    }
                }

                pub fn periphdata_alignment(&self, size: DataSize) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.psize().variant(size));
                }

                pub fn memdata_alignment(&self, size: DataSize) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.msize().variant(size));
                }

                pub fn mode(&self, mode: Mode) {
                    let dma = unsafe { &*$DMA::ptr() };
                    match mode {
                        Mode::Normal => dma.$SCR.modify(|_, w| w.circ().clear_bit().pfctrl().clear_bit()),
                        Mode::Circular => dma.$SCR.modify(|_, w| w.circ().enable().pfctrl().clear_bit()),
                        Mode::PeripheralFlowControl => dma.$SCR.modify(|_, w| w.circ().disable().pfctrl().set_bit()),
                    }
                }

                pub fn priority(&self, priority: Priority) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.pl().variant(priority));
                }

                // pub fn fifo_mode(&self) {
                //     dma.$SCR.modify(|_, w| w.().variant(priority));
                // }

                // pub fn fifo_threshold(&self, ) {}

                pub fn memory_burst(&self, burst: MemoryBurst) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.mburst().variant(burst));
                }

                pub fn peripheral_burst(&self, burst: PeripheralBurst) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.pburst().variant(burst));
                }

                pub fn enable(&self) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.en().set_bit());
                }

                pub fn disable(&self) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.modify(|_, w| w.en().clear_bit());
                }

                pub fn is_enabled(&self) -> bool {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SCR.read().en().bit_is_set()
                }

                pub fn set_config(&self, src_address: u32, dst_address: u32, length: u16) {
                    let dma = unsafe { &*$DMA::ptr() };
                    dma.$SNDTR.write(|w| unsafe { w.ndt().bits(length) });
                    if dma.$SCR.read().dir().is_periph_to_memory() {
                        dma.$SPAR.write(|w| unsafe { w.bits(src_address) });
                        dma.$SM0AR.write(|w| unsafe { w.bits(dst_address) });
                    }
                    else {
                        dma.$SPAR.write(|w| unsafe { w.bits(dst_address) });
                        dma.$SM0AR.write(|w| unsafe { w.bits(src_address) });
                    }
                }
            }
        )+
    }
}

streams!(DMA1, dma1en, {
    D1S0: (lisr,  (tcif0, htif0, teif0, dmeif0),
           lifcr, (ctcif0, chtif0, cteif0, cdmeif0),
           s0cr, s0ndtr, s0par, s0m0ar, s0m1ar),
    D1S1: (lisr,  (tcif1, htif1, teif1, dmeif1),
           lifcr, (ctcif1, chtif1, cteif1, cdmeif1),
           s1cr, s1ndtr, s1par, s1m0ar, s1m1ar),
    D1S2: (lisr,  (tcif2, htif2, teif2, dmeif2),
           lifcr, (ctcif2, chtif2, cteif2, cdmeif2),
           s2cr, s2ndtr, s2par, s2m0ar, s2m1ar),
    D1S3: (lisr,  (tcif3, htif3, teif3, dmeif3),
           lifcr, (ctcif3, chtif3, cteif3, cdmeif3),
           s3cr, s3ndtr, s3par, s3m0ar, s3m1ar),
    D1S4: (hisr,  (tcif4, htif4, teif4, dmeif4),
           hifcr, (ctcif4, chtif4, cteif4, cdme4f0),
           s4cr, s4ndtr, s4par, s4m0ar, s4m1ar),
    D1S5: (hisr,  (tcif5, htif5, teif5, dmeif5),
           hifcr, (ctcif5, chtif5, cteif5, cdme5f0),
           s5cr, s5ndtr, s5par, s5m0ar, s5m1ar),
    D1S6: (hisr,  (tcif6, htif6, teif6, dmeif6),
           hifcr, (ctcif6, chtif6, cteif6, cdme6f0),
           s6cr, s6ndtr, s6par, s6m0ar, s6m1ar),
    D1S7: (hisr,  (tcif7, htif7, teif7, dmeif7),
           hifcr, (ctcif7, chtif7, cteif7, cdme7f0),
           s7cr, s7ndtr, s7par, s7m0ar, s7m1ar),
});

streams!(DMA2, dma2en, {
    D2S0: (lisr,  (tcif0, htif0, teif0, dmeif0),
           lifcr, (ctcif0, chtif0, cteif0, cdmeif0),
           s0cr, s0ndtr, s0par, s0m0ar, s0m1ar),
    D2S1: (lisr,  (tcif1, htif1, teif1, dmeif1),
           lifcr, (ctcif1, chtif1, cteif1, cdmeif1),
           s1cr, s1ndtr, s1par, s1m0ar, s1m1ar),
    D2S2: (lisr,  (tcif2, htif2, teif2, dmeif2),
           lifcr, (ctcif2, chtif2, cteif2, cdmeif2),
           s2cr, s2ndtr, s2par, s2m0ar, s2m1ar),
    D2S3: (lisr,  (tcif3, htif3, teif3, dmeif3),
           lifcr, (ctcif3, chtif3, cteif3, cdmeif3),
           s3cr, s3ndtr, s3par, s3m0ar, s3m1ar),
    D2S4: (hisr,  (tcif4, htif4, teif4, dmeif4),
           hifcr, (ctcif4, chtif4, cteif4, cdme4f0),
           s4cr, s4ndtr, s4par, s4m0ar, s4m1ar),
    D2S5: (hisr,  (tcif5, htif5, teif5, dmeif5),
           hifcr, (ctcif5, chtif5, cteif5, cdme5f0),
           s5cr, s5ndtr, s5par, s5m0ar, s5m1ar),
    D2S6: (hisr,  (tcif6, htif6, teif6, dmeif6),
           hifcr, (ctcif6, chtif6, cteif6, cdme6f0),
           s6cr, s6ndtr, s6par, s6m0ar, s6m1ar),
    D2S7: (hisr,  (tcif7, htif7, teif7, dmeif7),
           hifcr, (ctcif7, chtif7, cteif7, cdme7f0),
           s7cr, s7ndtr, s7par, s7m0ar, s7m1ar),
});

pub trait DmaExt {
    type Streams;

    fn split(self, enr: &mut ENR) -> Self::Streams;
}
