use stm32f411::SYST;
use cortex_m::peripheral::SystClkSource;


pub fn delay_us(syst: &SYST, delay: ::time::Microseconds) {
    setup_counter(syst, delay);
    syst.clear_current();
    syst.enable_counter();
    while !syst.has_wrapped() {}
}

pub fn delay_ms(syst: &SYST, delay: ::time::Milliseconds) {
    setup_counter(syst, delay);
    syst.clear_current();
    syst.enable_counter();
    while !syst.has_wrapped() {}
}

fn setup_counter<T: Into<::sysclk::Ticks>>(syst: &SYST, ticks: T)
{
    let ticks_: u32 = ticks.into().into();
    if ticks_ > 0x00ffffff {
        panic!("Delay is too long!");
    }
    syst.set_reload(ticks_);
}

pub fn init_systick(syst: &SYST) {
    syst.set_clock_source(SystClkSource::Core);
}