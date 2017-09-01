//! Direct Memroy Access (DMA)

use core::cell::{Cell, UnsafeCell};
use core::marker::{PhantomData, Unsize};
use core::ops::Deref;
use core::{ops, slice};
use core::any::Any;

use nb;
use stm32f411::{dma2, DMA1, DMA2};

pub enum DMAStream {
    Stream0,
    Stream1,
    Stream2,
    Stream3,
    Stream4,
    Stream5,
    Stream6,
    Stream7,
}


pub unsafe trait DMA: Deref<Target = dma2::RegisterBlock> {
}

unsafe impl DMA for DMA1 {
}

pub struct Dma<'a, U>
    where U: Any + DMA
{
    pub reg: &'a U,
    stream: DMAStream,
}
/*
impl<'a, U> Clone for DMAInstance<'a, U>
    where U: Any + DMA
{
    fn clone(&self) -> Self {
        *self
    }
}
*/
// impl<'a, U> Copy for DMAInstance<'a, U> where U: Any + DMA {}

impl<'a, U> Dma<'a, U>
    where U: Any + DMA
{
    pub fn new(reg: &'a U, stream: DMAStream) -> Dma<U> {
        Dma {reg: reg, stream: stream}
    }

    pub fn init(& mut self, stream: DMAStream) {
        self.stream = stream;
    }

    pub fn channel(&self, channel: dma2::s0cr::CHSELW) {
        match self.stream {
            DMAStream::Stream0 => {
                self.reg.s0cr.modify(|_, w| w.chsel().variant(channel));
            }
            _ => {}
        }
    }

    pub fn direction(&self, direction: dma2::s0cr::DIRW) {
        match self.stream {
            DMAStream::Stream0 => {
                self.reg.s0cr.modify(|_, w| w.dir().variant(direction));
            }
            _ => {}
        }
    }
}