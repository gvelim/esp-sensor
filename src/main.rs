#![no_std]
#![no_main]

use critical_section::Mutex;
use core::cell::RefCell;

use esp_backtrace as _;
use esp_println::println;
use esp32c3::{
    clock::ClockControl, 
    peripherals::{Peripherals, Interrupt}, 
    prelude::*, 
    timer::TimerGroup, 
    Rtc, IO, Delay, gpio::{Gpio9, Input, PullDown, Event}, interrupt, pulse_control::{ClockSource, PulseCode, RepeatMode}
};

static BUTTON: Mutex<RefCell<Option<Gpio9<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    println!("Hello world!");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    let mut led = io.pins.gpio7.into_push_pull_output();
    led.set_high().expect("Cannot set GPIO7 to high");

    let mut button = io.pins.gpio9.into_pull_down_input();
    button.listen(Event::FallingEdge);
    critical_section::with(|cs|
        BUTTON.borrow_ref_mut(cs).replace(button)
    );

    interrupt::enable(Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();
    unsafe {
        esp32c3::riscv::interrupt::enable();
    }

    let mut rmt = esp32c3::PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control, 
        ClockSource::APB, 
        0, 
        0, 
        0).unwrap();

    rmt.channel0
        .set_idle_output_level(false)
        .set_idle_output(true)
        .set_carrier_modulation(false)
        .set_channel_divider(1);

    let mut ch0 = rmt.channel0.assign_pin(io.pins.gpio2);

    // NanosDuration::<u32>::from_ticks((SK68XX_T0L_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500);
    let t1 = PulseCode { level1: true, length1: 56u32.nanos(), level2: false, length2: 48u32.nanos() };
    // let t1: u32 = (1u32 << 31) + (56u32 << 16) + (0u32 << 15) + 48u32;
    let t0 = PulseCode { level1: true, length1: 28u32.nanos(), level2: false, length2: 64u32.nanos() };
    // let t0: u32 = (1u32 << 31) + (28u32 << 16) + (0u32 << 15) + 64u32;
    let rst = PulseCode { level1: false, length1: 2400u32.nanos(), level2: false, length2: 2400u32.nanos() };
    // let rst = (1u32 << 31) + (2400u32 << 16) + (0u32 << 15) + 2400u32;
    let eot = PulseCode { level1: false, length1: 0u32.nanos(), level2: false, length2: 0u32.nanos() };
    // let eot = 0u32;

    println!("TH {:b}, TL {:b}",u32::from(t1), u32::from(t0));
    println!("TH {:X}, TL {:X}",u32::from(t1), u32::from(t0));
    println!("TH {}, TL {}",u32::from(t1), u32::from(t0));

    let mut rgb_data = [
        t1, t1, t1, t1, t1, t1, t1, t1, // Green
        t0, t0, t0, t0, t0, t0, t0, t0, // Red
        t0, t0, t0, t0, t0, t0, t0, t0, // Blue
        rst, eot // end of transmission marker
    ];

    let mut delay = Delay::new(&clocks);
    loop {
        led.toggle().unwrap();
        ch0.send_pulse_sequence(RepeatMode::SingleShot, &rgb_data).unwrap();
        rgb_data[0..24].rotate_left(1);
        delay.delay_ms(500u32);
    }
}

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        println!("GPIO interrupt");
        BUTTON.borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}