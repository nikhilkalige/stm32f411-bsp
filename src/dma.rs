//! Direct Memory Access (DMA)

use core::cell::{Cell, UnsafeCell};
use core::marker::{PhantomData, Unsize};
use core::{ops, slice};

use nb;
use stm32f411::DMA1;
use volatile_register::RO;

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

/// Channel 1 of DMA1
pub struct DmaStream1 {
}

/// Channel 2 of DMA1
pub struct DmaStream2 {
}

/// Channel 4 of DMA1
pub struct DmaStream4 {
}

/// Channel 5 of DMA1
pub struct DmaStream5 {
}

/// Buffer to be used with a certain DMA `STREAM`
pub struct Buffer<T, STREAM> {
    _marker: PhantomData<STREAM>,
    data: UnsafeCell<T>,
    flag: Cell<BorrowFlag>,
    status: Cell<Status>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Status {
    Locked,
    MutLocked,
    Unlocked,
}

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

/// A wrapper type for a mutably borrowed value from a `Buffer``
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

impl<T, STREAM> Buffer<T, STREAM> {
    /// Creates a new buffer
    pub const fn new(data: T) -> Self {
        Buffer {
            _marker: PhantomData,
            data: UnsafeCell::new(data),
            flag: Cell::new(0),
            status: Cell::new(Status::Unlocked),
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
        assert_eq!(self.status.get(), Status::Unlocked);
        assert_ne!(self.flag.get(), WRITING);

        self.flag.set(self.flag.get() + 1);
        self.status.set(Status::Locked);

        unsafe { &*self.data.get() }
    }

    pub(crate) fn lock_mut(&self) -> &mut T {
        assert_eq!(self.status.get(), Status::Unlocked);
        assert_eq!(self.flag.get(), UNUSED);

        self.flag.set(WRITING);
        self.status.set(Status::MutLocked);

        unsafe { &mut *self.data.get() }
    }

    unsafe fn unlock(&self, status: Status) {
        match status {
            Status::Locked => self.flag.set(self.flag.get() - 1),
            Status::MutLocked => self.flag.set(UNUSED),
            _ => { /* unreachable!() */ }
        }

        self.status.set(Status::Unlocked);
    }
}

// FIXME these `release` methods probably want some of sort of barrier
impl<T> Buffer<T, DmaStream2> {
    /// Waits until the DMA releases this buffer
    pub fn release(&self, dma1: &DMA1) -> nb::Result<(), Error> {
        let status = self.status.get();

        if status == Status::Unlocked {
            return Ok(());
        }

        if dma1.lisr.read().teif2().bit_is_set() {
            Err(nb::Error::Other(Error::Transfer))
        } else if dma1.lisr.read().tcif2().bit_is_set() {
            unsafe { self.unlock(status) }
            dma1.lifcr.write(|w| w.ctcif2().set_bit());
            dma1.s1cr.modify(|_, w| w.en().clear_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<T> Buffer<T, DmaStream4> {
    /// Waits until the DMA releases this buffer
    pub fn release(&self, dma1: &DMA1) -> nb::Result<(), Error> {
        let status = self.status.get();

        if status == Status::Unlocked {
            return Ok(());
        }

        if dma1.hisr.read().teif4().bit_is_set() {
            Err(nb::Error::Other(Error::Transfer))
        } else if dma1.hisr.read().tcif4().bit_is_set() {
            unsafe { self.unlock(status) }
            dma1.hifcr.write(|w| w.ctcif4().set_bit());
            dma1.s4cr.modify(|_, w| w.en().clear_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<T> Buffer<T, DmaStream5> {
    /// Waits until the DMA releases this buffer
    pub fn release(&self, dma1: &DMA1) -> nb::Result<(), Error> {
        let status = self.status.get();

        if status == Status::Unlocked {
            return Ok(());
        }

        if dma1.hisr.read().teif5().bit_is_set() {
            Err(nb::Error::Other(Error::Transfer))
        } else if dma1.hisr.read().tcif5().bit_is_set() {
            unsafe { self.unlock(status) }
            dma1.hifcr.write(|w| w.ctcif5().set_bit());
            dma1.s5cr.modify(|_, w| w.en().clear_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// A circular buffer associated to a DMA `STREAM`
pub struct CircBuffer<T, B, STREAM>
where
    B: Unsize<[T]>,
{
    _marker: PhantomData<STREAM>,
    _t: PhantomData<[T]>,
    buffer: UnsafeCell<[B; 2]>,
    status: Cell<CircStatus>,
}

impl<T, B, STREAM> CircBuffer<T, B, STREAM>
where
    B: Unsize<[T]>,
{
    pub(crate) fn lock(&self) -> &[B; 2] {
        assert_eq!(self.status.get(), CircStatus::Free);

        self.status.set(CircStatus::MutatingFirstHalf);

        unsafe { &*self.buffer.get() }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CircStatus {
    /// Not in use by the DMA
    Free,
    /// The DMA is mutating the first half of the buffer
    MutatingFirstHalf,
    /// The DMA is mutating the second half of the buffer
    MutatingSecondHalf,
}

impl<T, B> CircBuffer<T, B, DmaStream1>
where
    B: Unsize<[T]>,
    T: Atomic,
{
    /// Constructs a circular buffer from two halves
    pub const fn new(buffer: [B; 2]) -> Self {
        CircBuffer {
            _t: PhantomData,
            _marker: PhantomData,
            buffer: UnsafeCell::new(buffer),
            status: Cell::new(CircStatus::Free),
        }
    }

    /// Yields read access to the half of the circular buffer that's not
    /// currently being mutated by the DMA
    pub fn read(&self, dma1: &DMA1) -> nb::Result<&[RO<T>], Error> {
        let status = self.status.get();

        assert_ne!(status, CircStatus::Free);

        let isr = dma1.lisr.read();

        if isr.teif1().bit_is_set() {
            Err(nb::Error::Other(Error::Transfer))
        } else {
            match status {
                CircStatus::MutatingFirstHalf => {
                    if isr.tcif1().bit_is_set() {
                        Err(nb::Error::Other(Error::Overrun))
                    } else if isr.htif1().bit_is_set() {
                        dma1.lifcr.write(|w| w.chtif1().set_bit());

                        self.status.set(CircStatus::MutatingSecondHalf);

                        unsafe {
                            let half: &[T] = &(*self.buffer.get())[0];
                            Ok(slice::from_raw_parts(
                                half.as_ptr() as *const _,
                                half.len(),
                            ))
                        }
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
                CircStatus::MutatingSecondHalf => {
                    if isr.htif1().bit_is_set() {
                        Err(nb::Error::Other(Error::Overrun))
                    } else if isr.tcif1().bit_is_set() {
                        dma1.lifcr.write(|w| w.ctcif1().set_bit());

                        self.status.set(CircStatus::MutatingFirstHalf);

                        unsafe {
                            let half: &[T] = &(*self.buffer.get())[1];
                            Ok(slice::from_raw_parts(
                                half.as_ptr() as *const _,
                                half.len(),
                            ))
                        }
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
                _ => unreachable!(),
            }
        }
    }
}

/// Values that can be atomically read
pub trait Atomic: Copy {}

impl Atomic for u8 {}
impl Atomic for u16 {}
impl Atomic for u32 {}
