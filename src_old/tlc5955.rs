//! TLC5955 PWM Led driver

use generic_array::{GenericArray, ArrayLength};
use static_ref::Static;
use core::marker::Unsize;
use semihosting::hio;
use core::fmt::Write;
use core::ops::DerefMut;
use dma2::{self, DMA, Dma, Buffer, DMAStream};

const CHANNELS_PER_TLC: u8 = 48;
const LEDS_PER_CHIP: u8 = CHANNELS_PER_TLC / 3;
const DC_DATA: u8 = 0x6A;
const DC_BITS: u32 = 7;
const MC_DATA: u8 = 5;
const MC_BITS: u32 = 3;
const BC_DATA: u8 = 110;
const BC_BITS: u32 = 7;
const NO_LED_DRIVERS: usize = 4;

pub trait TLCHardwareLayer {
    fn as_gpio(&self);
    fn as_spi(&self);
    fn latch(&self, delay: u32);
    fn write_bit(&self, bit: u8);
    fn delay(&self, count: u16);
    fn read_write_byte(&self, byte: u8) -> u8;
    fn write<B>(&self, tx_buffer: &Buffer<B>,
        rx_buffer: &Buffer<B>)where B: Unsize<[u8]>;
    fn wait<B>(&self, buffer: &Buffer<B>)where B: Unsize<[u8]>;
    fn dump_buffer(&self, buffer: &[u8]);
    fn debug(&self, data: &str);
}
// struct TLC5955<T: FixedSizeArray<u8>> {
// struct TLC5955<N: ArrayLength<u8>> {

struct RGB<T> {
    red: T,
    green: T,
    blue: T,
}

impl Default for RGB<u8> {
    fn default() -> RGB<u8> {
        RGB { red: 0, blue: 0, green: 0}
    }
}

impl Default for RGB<u16> {
    fn default() -> RGB<u16> {
        RGB { red: 0, blue: 0, green: 0}
    }
}

pub struct TLC5955 {
    no_chips: u8,
    function_data: u8,
    brightness: RGB<u8>,
    max_current: RGB<u8>,
}

impl TLC5955 {
    pub fn new(no_chips: u8) -> TLC5955 {
        TLC5955 {
            no_chips: no_chips,
            function_data: 0,
            brightness: Default::default(),
            max_current: Default::default(),
        }
    }

    pub fn setup<B, I>(&mut self,
        tx_buffer: &Static<[Buffer<B>; NO_LED_DRIVERS]>,
        rx_buffer: &Static<[Buffer<B>; NO_LED_DRIVERS]>,
        interface: &I)
        where I: TLCHardwareLayer, B: Unsize<[u8]>
    {
        interface.debug("Sending control register data to TLC5955\n");
        for buffer in tx_buffer.iter() {
            self.fill_control_data(&mut *buffer.borrow_mut());
            interface.dump_buffer(&*buffer.borrow_mut());
        }
        self.send_data(true, tx_buffer, rx_buffer, interface);

        for buffer in tx_buffer.iter() {
            clear_buffer(&mut *buffer.borrow_mut());
        }
        interface.debug("Read data after zeros.\n");
        self.send_data(true, tx_buffer, rx_buffer, interface);

        for (txb, rxb) in tx_buffer.iter().zip(rx_buffer.iter()) {
            self.fill_control_data(&mut *txb.borrow_mut());
            if !compare_buffers(&*txb.borrow(), &*rxb.borrow()) {
                interface.debug("Ouch, read control data does not match!\n");
                interface.dump_buffer(&*rxb.borrow_mut());
                //loop {
                // }
            } else {
                interface.debug("Read control good.\n");
            }
        }

        // Send the control data the second time.
        self.send_data(true, tx_buffer, rx_buffer, interface);

        for buffer in tx_buffer.iter() {
            clear_buffer(&mut *buffer.borrow_mut());
            {
                let buffer: &mut[u8] = &mut *buffer.borrow_mut();
                let mut i = 0;
                while i < buffer.len() {
                    buffer[i] = 0xFF;
                    i += 6;
                }
            }
        }

        interface.debug("Load GS Data\n");
        for buffer in tx_buffer.iter() {
            interface.dump_buffer(&*buffer.borrow_mut());
        }
        self.send_data(false, tx_buffer, rx_buffer, interface);
        interface.debug("Read GS Data\n");
        for buffer in rx_buffer.iter() {
            interface.dump_buffer(&*buffer.borrow_mut());
        }

        let mut count:u8 = 0;
        let mut inc: usize = 1;
        loop {
            for txb in tx_buffer.iter() {
                clear_buffer(&mut *txb.borrow_mut());
                {
                    let buffer: &mut[u8] = &mut *txb.borrow_mut();
                    let mut i = 0;
                    // let inc: usize = (count as usize) % 6;
                    while i < buffer.len() {
                        buffer[i + inc] = 0xFF;
                        i += 6;
                    }
                }
            }
            inc = match(inc) {
                1 => 3,
                3 => 5,
                5 => 1,
                _ => 1
            };

            interface.delay(100);
            self.send_data(false, tx_buffer, rx_buffer, interface);
            count = count.wrapping_add(1);
        }
    }

    pub fn send_data<B, I>(&self, is_control: bool,
        tx_buffer: &Static<[Buffer<B>; NO_LED_DRIVERS]>,
        rx_buffer: &Static<[Buffer<B>; NO_LED_DRIVERS]>,
        interface: &I)
        where I: TLCHardwareLayer, B: Unsize<[u8]>
    {
        for (txb, rxb) in tx_buffer.iter().zip(rx_buffer.iter()) {
            interface.as_gpio();
            if is_control {
                interface.write_bit(1);
            }
            else {
                interface.write_bit(0);
            }
            interface.as_spi();
            interface.write(txb, rxb);
            interface.wait(txb);
            interface.wait(rxb);
        }
        interface.latch(1);
    }

    pub fn set_function_data(&mut self, dsprpt: bool, tmgrst: bool, rfresh: bool, espwm: bool, lsdvlt: bool) {
        let mut data: u8 = 0;
        data |= if dsprpt { 1 << 0 } else { 0 };
        data |= if tmgrst { 1 << 0 } else { 0 };
        data |= if rfresh { 1 << 0 } else { 0 };
        data |= if espwm { 1 << 0 } else { 0 };
        data |= if lsdvlt { 1 << 0 } else { 0 };
        self.function_data = data;
    }

    pub fn setall_dcdata<B>(self, buffer: &Static<Buffer<B>>, value: u8)
        where B: Unsize<[u8]> {
        let buffer: &mut[u8] = buffer.lock_mut();

        for index in 0..buffer.len() {
            buffer[index] = value;
        }
    }

    pub fn set_dcdata<B>(self, buffer: &Static<Buffer<B>>,
                                 led_num: u16, red: u8, green: u8, blue: u8)
        where B: Unsize<[u8]> {
        let buffer: &mut[u8] = buffer.lock_mut();
        let index = (led_num * 3) as usize;

        buffer[index] = red;
        buffer[index + 1] = green;
        buffer[index + 2] = blue;
    }

    pub fn set_brightness_current(&mut self, red: u8, green: u8, blue: u8) {
        self.brightness = RGB { red: red, blue: blue, green: green };
    }

    pub fn setall_led<B>(self, buffer: &Static<Buffer<B>>,
                                 red: u16, green: u16, blue: u16)
        where B: Unsize<[u16]> {
        let buffer: &mut[u16] = buffer.lock_mut();
        let no_leds = LEDS_PER_CHIP * self.no_chips;

        for index in 0..no_leds {
            let idx = (index * 3) as usize;
            buffer[idx + 0] = red;
            buffer[idx + 1] = green;
            buffer[idx + 2] = blue;
        }
    }

    pub fn set_led<B>(self, buffer: &Static<Buffer<B>>,
                              led_num: u16, red: u16, green: u16, blue: u16)
        where B: Unsize<[u16]> {
        let buffer: &mut[u16] = buffer.lock_mut();
        let index = (led_num * 3) as usize;

        buffer[index] = red;
        buffer[index + 1] = green;
        buffer[index + 2] = blue;
    }

    pub fn set_max_current(&mut self, red: u8, green: u8, blue: u8) {
        self.max_current = RGB { red: red, blue: blue, green: green };
    }

    fn fill_control_data(&mut self, buffer: &mut[u8]) {
        let chunk_size = buffer.len() / (self.no_chips as usize);
        for chunk in buffer.chunks_mut(chunk_size) {
            let mut pos = 0;
            for index in 0..CHANNELS_PER_TLC {
                pos = add_bits(chunk, DC_DATA as u32, DC_BITS, pos);
            }

            for index in 0..3 {
                pos = add_bits(chunk, MC_DATA as u32, MC_BITS, pos);
            }

            for index in 0..3 {
                pos = add_bits(chunk, BC_DATA as u32, BC_BITS, pos);
            }
            pos = add_bits(chunk, 0x0B as u32, 5, pos);
            pos = 760;
            add_bits(chunk, 0x96 as u32, 8, pos);
        }
    }
}

fn add_bits(buffer: &mut[u8], mut val: u32, mut size: u32, mut pos: usize) -> usize {
    let len = buffer.len();
    while (size > 0) {
        if (val & 1) > 0 {
            buffer[len - 1 -  (pos >> 3)] |= 1 << (pos & 7);
        }
        val >>= 1;
        pos += 1;
        size -= 1;
    }
    return pos;
}

fn clear_buffer(buffer: &mut[u8]) {
    for index in 0..buffer.len() {
        buffer[index] = 0;
    }
}

fn compare_buffers(buffer1: &[u8], buffer2: &[u8]) -> bool {
    buffer1.iter().eq(buffer2)
}

fn shift_buffer_by_7(buffer: &mut [u8]) {
    let mut old: u8 = 0;
    for index in 0..buffer.len() {
        let mut val = buffer[index];
        buffer[index] = old.wrapping_shl(1) | val.wrapping_shr(7);
        old = val;
    }
}