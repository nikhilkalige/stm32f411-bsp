//! Direct Memroy Access (DMA)

use core::cell::{Cell, UnsafeCell};
use core::marker::PhantomData;
use core::ops::Deref;
use core::ops;
use core::any::Any;

use nb;
use stm32f411::{DMA1, DMA2, dma2};

pub use stm32f411::dma2::scr::CHSELW as Channel;
pub use stm32f411::dma2::scr::DIRW as Direction;
pub use stm32f411::dma2::scr::MBURSTW as MemoryBurst;
pub use stm32f411::dma2::scr::PBURSTW as PeripheralBurst;
pub use stm32f411::dma2::scr::PLW as Priority;
pub use stm32f411::dma2::scr::MSIZEW as DataSize;

pub struct DMA1Stream0();
pub struct DMA2Stream1();
pub struct DMA2Stream4();

#[derive(Copy, Clone)]
pub enum DMAStream {
    Stream0,
    Stream1,
    Stream2,
    Stream3,
    Stream4,
    // Stream5,
    // Stream6,
    // Stream7,
}

/// DMA error
#[derive(Debug)]
pub enum Error {
    /// DMA channel in use
    InUse,
    /// Previous data got overwritten before it could be read because it was
    /// not accessed in a timely fashion
    Overrun,
    /// Transfer error
    Transfer,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum State {
    // A new `Buffer` starts in this state. We set it to zero to place this
    // buffer in the .bss section
    Unlocked = 0,

    Locked,
    MutLocked,
}

#[derive(Copy, Clone)]
pub enum Mode {
    Normal,
    Circular,
    PeripheralFlowControl,
}

pub unsafe trait DMA: Deref<Target = dma2::RegisterBlock> {
    fn scr(&self, stream: DMAStream) -> &dma2::SCR {
        match stream {
            DMAStream::Stream0 => &self.s0cr,
            DMAStream::Stream1 => &self.s1cr,
            DMAStream::Stream2 => &self.s2cr,
            DMAStream::Stream3 => &self.s3cr,
            DMAStream::Stream4 => &self.s4cr,
        }
    }

    fn sndtr(&self, stream: DMAStream) -> &dma2::SNDTR {
        match stream {
            DMAStream::Stream0 => &self.s0ndtr,
            DMAStream::Stream1 => &self.s1ndtr,
            DMAStream::Stream2 => &self.s2ndtr,
            DMAStream::Stream3 => &self.s3ndtr,
            DMAStream::Stream4 => &self.s4ndtr,
        }
    }

    fn spar(&self, stream: DMAStream) -> &dma2::SPAR {
        match stream {
            DMAStream::Stream0 => &self.s0par,
            DMAStream::Stream1 => &self.s1par,
            DMAStream::Stream2 => &self.s2par,
            DMAStream::Stream3 => &self.s3par,
            DMAStream::Stream4 => &self.s4par,
        }
    }

    fn sm0ar(&self, stream: DMAStream) -> &dma2::SM0AR {
        match stream {
            DMAStream::Stream0 => &self.s0m0ar,
            DMAStream::Stream1 => &self.s1m0ar,
            DMAStream::Stream2 => &self.s2m0ar,
            DMAStream::Stream3 => &self.s3m0ar,
            DMAStream::Stream4 => &self.s4m0ar,
        }
    }

    fn sm1ar(&self, stream: DMAStream) -> &dma2::SM1AR {
        match stream {
            DMAStream::Stream0 => &self.s0m1ar,
            DMAStream::Stream1 => &self.s1m1ar,
            DMAStream::Stream2 => &self.s2m1ar,
            DMAStream::Stream3 => &self.s3m1ar,
            DMAStream::Stream4 => &self.s4m1ar,
        }
    }

    fn sfcr(&self, stream: DMAStream) -> &dma2::SFCR {
        match stream {
            DMAStream::Stream0 => &self.s0fcr,
            DMAStream::Stream1 => &self.s1fcr,
            DMAStream::Stream2 => &self.s2fcr,
            DMAStream::Stream3 => &self.s3fcr,
            DMAStream::Stream4 => &self.s4fcr,
        }
    }
}

unsafe impl DMA for DMA1 {}

unsafe impl DMA for DMA2 {}

pub struct Dma<'a, U>
where
    U: Any + DMA,
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
where
    U: Any + DMA,
{
    pub fn new(reg: &'a U, stream: DMAStream) -> Dma<U> {
        Dma {
            reg: reg,
            stream: stream,
        }
    }

    pub fn init(&mut self, stream: DMAStream) {
        self.stream = stream;
    }

    pub fn channel(&self, channel: dma2::scr::CHSELW) {
        self.reg
            .scr(self.stream)
            .modify(|_, w| w.chsel().variant(channel));
    }

    pub fn direction(&self, direction: Direction) {
        self.reg
            .scr(self.stream)
            .modify(|_, w| w.dir().variant(direction));
    }

    pub fn peripheral_increment(&self, inc: bool) {
        if inc {
            self.reg.scr(self.stream).modify(|_, w| w.pinc().enable());
        } else {
            self.reg.scr(self.stream).modify(|_, w| w.pinc().disable());
        }
    }

    pub fn memory_increment(&self, inc: bool) {
        if inc {
            self.reg.scr(self.stream).modify(|_, w| w.minc().enable());
        } else {
            self.reg.scr(self.stream).modify(|_, w| w.minc().disable());
        }
    }

    pub fn periphdata_alignment(&self, size: DataSize) {
        self.reg
            .scr(self.stream)
            .modify(|_, w| w.psize().variant(size));
    }

    pub fn memdata_alignment(&self, size: DataSize) {
        self.reg
            .scr(self.stream)
            .modify(|_, w| w.msize().variant(size));
    }

    pub fn mode(&self, mode: Mode) {
        match mode {
            Mode::Normal => self.reg
                .scr(self.stream)
                .modify(|_, w| w.circ().clear_bit().pfctrl().clear_bit()),
            Mode::Circular => self.reg
                .scr(self.stream)
                .modify(|_, w| w.circ().enable().pfctrl().clear_bit()),
            Mode::PeripheralFlowControl => self.reg
                .scr(self.stream)
                .modify(|_, w| w.circ().disable().pfctrl().set_bit()),
        }
    }

    pub fn priority(&self, priority: Priority) {
        self.reg
            .scr(self.stream)
            .modify(|_, w| w.pl().variant(priority));
    }

    // pub fn fifo_mode(&self) {
    //     self.reg.scr(self.stream).modify(|_, w| w.().variant(priority));
    // }

    // pub fn fifo_threshold(&self, ) {}

    pub fn memory_burst(&self, burst: MemoryBurst) {
        self.reg
            .scr(self.stream)
            .modify(|_, w| w.mburst().variant(burst));
    }

    pub fn peripheral_burst(&self, burst: PeripheralBurst) {
        self.reg
            .scr(self.stream)
            .modify(|_, w| w.pburst().variant(burst));
    }

    pub fn enable(&self) {
        self.reg.scr(self.stream).modify(|_, w| w.en().set_bit());
    }

    pub fn disable(&self) {
        self.reg.scr(self.stream).modify(|_, w| w.en().clear_bit());
    }

    pub fn is_enabled(&self) -> bool {
        if self.reg.scr(self.stream).read().en().bit_is_set() {
            true
        }
        else {
            false
        }
    }

    pub fn set_config(&self, src_address: u32, dst_address: u32, length: u16) {
        self.reg.sndtr(self.stream).write(|w| unsafe { w.ndt().bits(length) });
        if self.reg.scr(self.stream).read().dir().is_periph_to_memory() {
            self.reg.spar(self.stream).write(|w| unsafe { w.bits(src_address) });
            self.reg.sm0ar(self.stream).write(|w| unsafe { w.bits(dst_address) });
        }
        else {
            self.reg.spar(self.stream).write(|w| unsafe { w.bits(dst_address) });
            self.reg.sm0ar(self.stream).write(|w| unsafe { w.bits(src_address) });
        }
    }

}

// DMA buffer definitions
type BorrowFlag = usize;

const UNUSED: BorrowFlag = 0;
const WRITING: BorrowFlag = !0;

/// Wraps a borrowed reference to a value in a `Buffer`
pub struct Ref<'a, T>
where
    T: 'a,
{
    data: &'a T,
    flag: &'a Cell<BorrowFlag>,
}

impl<'a, T> ops::Deref for Ref<'a, T> {
    type Target = T;

    fn deref(&self) -> &T {
        self.data
    }
}

impl<'a, T> Drop for Ref<'a, T> {
    fn drop(&mut self) {
        self.flag.set(self.flag.get() - 1);
    }
}

/// A wrapper type for a mutably borrowed value from a `Buffer`
pub struct RefMut<'a, T>
where
    T: 'a,
{
    data: &'a mut T,
    flag: &'a Cell<BorrowFlag>,
}

impl<'a, T> ops::Deref for RefMut<'a, T> {
    type Target = T;

    fn deref(&self) -> &T {
        self.data
    }
}

impl<'a, T> ops::DerefMut for RefMut<'a, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.data
    }
}

impl<'a, T> Drop for RefMut<'a, T> {
    fn drop(&mut self) {
        self.flag.set(UNUSED);
    }
}

/// Buffer to be used with a certain DMA `CHANNEL`
// NOTE(packed) workaround for rust-lang/rust#41315
#[repr(packed)]
pub struct Buffer<T> {
    data: UnsafeCell<T>,
    flag: Cell<BorrowFlag>,
    state: Cell<State>,
    stream: DMAStream,
}

impl<T> Buffer<T> {
    /// Creates a new buffer
    pub const fn new(data: T, stream: DMAStream) -> Self {
        Buffer {
            stream: stream,
            data: UnsafeCell::new(data),
            state: Cell::new(State::Unlocked),
            flag: Cell::new(0),
        }
    }

    /// Immutably borrows the wrapped value.
    ///
    /// The borrow lasts until the returned `Ref` exits scope. Multiple
    /// immutable borrows can be taken out at the same time.
    ///
    /// # Panics
    ///
    /// Panics if the value is currently mutably borrowed.
    pub fn borrow(&self) -> Ref<T> {
        assert_ne!(self.flag.get(), WRITING);
        self.flag.set(self.flag.get() + 1);

        Ref {
            data: unsafe { &*self.data.get() },
            flag: &self.flag,
        }
    }

    /// Mutably borrows the wrapped value.
    ///
    /// The borrow lasts until the returned `RefMut` exits scope. The value
    /// cannot be borrowed while this borrow is active.
    ///
    /// # Panics
    ///
    /// Panics if the value is currently borrowed.
    pub fn borrow_mut(&self) -> RefMut<T> {
        assert_eq!(self.flag.get(), UNUSED);
        self.flag.set(WRITING);

        RefMut {
            data: unsafe { &mut *self.data.get() },
            flag: &self.flag,
        }
    }

    pub(crate) fn lock(&self) -> &T {
        assert_eq!(self.state.get(), State::Unlocked);
        assert_ne!(self.flag.get(), WRITING);

        self.flag.set(self.flag.get() + 1);
        self.state.set(State::Locked);

        unsafe { &*self.data.get() }
    }

    pub(crate) fn lock_mut(&self) -> &mut T {
        assert_eq!(self.state.get(), State::Unlocked);
        assert_eq!(self.flag.get(), UNUSED);

        self.flag.set(WRITING);
        self.state.set(State::MutLocked);

        unsafe { &mut *self.data.get() }
    }

    unsafe fn unlock(&self, state: State) {
        match state {
            State::Locked => self.flag.set(self.flag.get() - 1),
            State::MutLocked => self.flag.set(UNUSED),
            _ => { /* unreachable!() */ }
        }

        self.state.set(State::Unlocked);
    }

    // FIXME these `release` methods probably want some of sort of barrier
    /// Waits until the DMA releases this buffer
    pub fn release<D:DMA>(&self, dma: &D) -> nb::Result<(), Error> {
        let state = self.state.get();

        if state == State::Unlocked {
            return Ok(());
        }

        let dma_status = match self.stream {
            DMAStream::Stream0 => (
                dma.lisr.read().teif0().bit_is_set(),
                dma.lisr.read().tcif0().bit_is_set(),
            ),
            DMAStream::Stream1 => (
                dma.lisr.read().teif1().bit_is_set(),
                dma.lisr.read().tcif1().bit_is_set(),
            ),
            DMAStream::Stream2 => (
                dma.lisr.read().teif2().bit_is_set(),
                dma.lisr.read().tcif2().bit_is_set(),
            ),
            DMAStream::Stream3 => (
                dma.lisr.read().teif3().bit_is_set(),
                dma.lisr.read().tcif3().bit_is_set(),
            ),
            DMAStream::Stream4 => (
                dma.hisr.read().teif4().bit_is_set(),
                dma.hisr.read().tcif4().bit_is_set(),
            ),
        };

        if dma_status.0 {
            return Err(nb::Error::Other(Error::Transfer));
        } else if dma_status.1 {
            unsafe { self.unlock(state) }
            match self.stream {
                DMAStream::Stream0 => dma.lifcr.write(|w| w.ctcif0().set_bit()),
                DMAStream::Stream1 => dma.lifcr.write(|w| w.ctcif1().set_bit()),
                DMAStream::Stream2 => dma.lifcr.write(|w| w.ctcif2().set_bit()),
                DMAStream::Stream3 => dma.lifcr.write(|w| w.ctcif3().set_bit()),
                DMAStream::Stream4 => dma.lifcr.write(|w| w.ctcif2().set_bit()),
            }

            dma.scr(self.stream).modify(|_, w| w.en().disable());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
