#![no_std]
#![no_main]

use critical_section::Mutex;
use core::cell::RefCell;
use core::ops::BitAnd;

use esp_backtrace as _;
use esp_println::println;
use esp32c3::{
    clock::ClockControl, 
    peripherals::{Peripherals, Interrupt, APB_SARADC},
    prelude::*, 
    timer::TimerGroup, 
    Rtc, IO, Delay, gpio::{Gpio9, Input, PullDown, Event}, interrupt, pulse_control::{ClockSource, PulseCode, RepeatMode}
};
use esp32c3::peripheral::Peripheral;

struct State {
    button: Option<Gpio9<Input<PullDown>>>,
    tmp: Option<&'static TempSensor>,
    fps : [u32;13]
}

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(
    State {
        button: None,
        tmp: None,
        fps: [10u32, 20u32, 40u32, 80u32, 160u32, 320u32, 640u32, 1280u32, 640u32, 320u32, 80u32, 40u32, 20u32]
    }
));


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
    let tmps = TempSensor::init();
    button.listen(Event::FallingEdge);
    critical_section::with(|cs| {
        let mut cs = STATE.borrow_ref_mut(cs);
        cs.button.replace(button);
        cs.tmp.replace(tmps);
    });

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
        0
    ).unwrap();

    rmt.channel0
        .set_idle_output_level(false)
        .set_idle_output(true)
        .set_carrier_modulation(false)
        .set_channel_divider(1);

    let mut ch0 = rmt.channel0.assign_pin(io.pins.gpio2);


    let mut rgb_data = RgbLed::new();
    let mut g = Bounce::new(15, (u8::MIN,32));
    let mut r = Bounce::new(0, (u8::MIN,32));
    let mut b = Bounce::new(31, (u8::MIN,32));

    let mut delay = Delay::new(&clocks);
    loop {
        led.toggle().unwrap();

        rgb_data.set(RGB::GREEN, g.next().unwrap());
        rgb_data.set(RGB::RED, r.next().unwrap());
        rgb_data.set(RGB::BLUE, b.next().unwrap());

        ch0.send_pulse_sequence_raw(RepeatMode::SingleShot, &rgb_data.data).unwrap();

        delay.delay_ms(
            critical_section::with(|cs| STATE.borrow_ref(cs).fps[0])
        );
    }
}

enum TOGGLE { UP, DOWN }
struct Bounce<T> {
    rng: (T,T),
    val : T,
    toggle: TOGGLE
}
impl Bounce<u8> {
    fn new(val: u8, rng: (u8, u8)) -> Bounce<u8> {
        Bounce { rng, val, toggle: TOGGLE::UP }
    }
}
impl Iterator for Bounce<u8> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        let (min,max) = self.rng;
        match self {
            &mut Bounce{ val, toggle:TOGGLE::UP, .. } if val == max => self.toggle = TOGGLE::DOWN,
            &mut Bounce{ val, toggle:TOGGLE::DOWN,..} if val == min => self.toggle = TOGGLE::UP,
            &mut _ => {}
        }
        self.val = match self.toggle {
            TOGGLE::UP => self.val + 1,
            TOGGLE::DOWN => self.val - 1,
        };
        Some(self.val)
    }
}

// NanosDuration::<u32>::from_ticks((SK68XX_T0L_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500);
// let t1 = PulseCode { level1: true, length1: 56u32.nanos(), level2: false, length2: 48u32.nanos() };
// let t1: u32 = (1u32 << 31) + (56u32 << 16) + (0u32 << 15) + 48u32;
// let t0 = PulseCode { level1: true, length1: 28u32.nanos(), level2: false, length2: 64u32.nanos() };
const T1: u32 = (1u32 << 31) + (56u32 << 16) + (0u32 << 15) + 48u32;
const T0: u32 = (1u32 << 31) + (28u32 << 16) + (0u32 << 15) + 64u32;
const RST: u32 = (1u32 << 31) + (2400u32 << 16) + (0u32 << 15) + 2400u32;
const END: u32 = 0u32;

#[derive(Clone, Copy)]
enum RGB {GREEN=0, RED=8, BLUE=16}
struct RgbLed {
    data: [u32;26],
}

impl RgbLed {
    fn new() -> RgbLed {
        let mut data = [0u32;26];
        data[24] = RST;
        data[25] = END;
        RgbLed { data }
    }
    fn set(&mut self, colour: RGB, mut val: u8) {
        for idx in 0..8 {
            self.data[colour as usize + idx] = match val.bitand(128u8) { 0 => T0, _ => T1, };
            val = val << 1;
        }
    }
}


#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        let mut state = STATE.borrow_ref_mut(cs);
        state.fps.rotate_left(1);
        println!("GPIO interrupt - fps({})", state.fps[0]);
        println!("ESP32-C3 Temp - {:.2}°C", state.tmp.as_ref().unwrap().read_temp());
        state.button.as_mut().unwrap().clear_interrupt();
    });
}

/// The actual temperature (°C) can be obtained by converting the output of temperature sensor via the following
/// formula:
/// T(°C) = 0.4386 * VALUE – 27.88 * offset–20.52
/// Offset(-10°C - 80°C) = 0
/// 
const ADC_CTLR: u32 = 0x6004_0000;                               // The addresses in this section are relative to the ADC controller base address
const APB_SARADC_CTRL_REG: u32 = 0x000_0000 + ADC_CTLR;          // SAR ADC control register 1
const APB_SARADC_XPD_SAR_FORCE: u32 = 0x1800_0000;               // Force select XPD SAR. (R/W)

const APB_SARADC_APB_TSENS_CTRL_REG: u32 = 0x0058 + ADC_CTLR;   // Temperature sensor control register 1
const APB_SARADC_TSENS_IN_INV: u32 = 1 << 12;                   // Invert temperature sensor input value. (R/W)
const APB_SARADC_TSENS_PU: u32 = 1 << 21;                       // Temperature sensor power up. (R/W)
                                                                // to start XPD_SAR, and then to enable temperature sensor;
const SYSTEM_REG: u32 = 0x600C_0000;
const SYSTEM_PERIP_CLK_EN1_REG: u32 = 0x0014 + SYSTEM_REG;      // to enable temperature sensor clock;
const SYSTEM_TSENS_CLK_EN: u32 = 1 << 9;                       // Set this bit to enable TSENS clock. (R/W)

const APB_SARADC_APB_TSENS_CTRL2_REG: u32 = 0x005C + ADC_CTLR;  // Temperature sensor control register 2
const APB_SARADC_TSENS_XPD_WAIT: u32 = 0x0000_0700;             // 0-11 The wait time before temperature sensor is powered up. (R/W)
                                                                // Wait for APB_SARADC_TSENS_XPD_WAIT clock cycles till the reset of temperature sensor is released, the
const APB_SARADC_TSENS_CLK_SEL:u32 = 1 << 15;                   // Choose working clock for temperature sensor. 0:RC_FAST_CLK. 1: XTAL_CLK. (R/W)
const APB_SARADC_TSENS_OUT: u32 = 0x0000_00FF;                  // Wait for a while and then read the data from APB_SARADC_TSENS_OUT

#[repr(C)]
struct TempSensor {
    ctrl2_reg: u32,
    ctrl_reg: u32
}
impl TempSensor {
    fn init() -> &'static mut TempSensor {
        unsafe {
            let mut tsen = APB_SARADC_APB_TSENS_CTRL_REG as *mut TempSensor;
            *(APB_SARADC_CTRL_REG as *mut u32) |= APB_SARADC_XPD_SAR_FORCE;
            println!(">APB_SARADC_APB_TSENS_CTRL_REG {:b}",*(APB_SARADC_CTRL_REG as *mut u32));

            println!("<APB_SARADC_APB_TSENS_CTRL_REG {:b}",(*tsen).ctrl_reg);
            (*tsen).ctrl_reg |= APB_SARADC_TSENS_IN_INV;
            (*tsen).ctrl_reg |= APB_SARADC_TSENS_PU;
            println!(">APB_SARADC_APB_TSENS_CTRL_REG {:b}",(*tsen).ctrl_reg);

            println!(">SYSTEM_PERIP_CLK_EN1_REG {:b}",*(SYSTEM_PERIP_CLK_EN1_REG as *mut u32));
            *(SYSTEM_PERIP_CLK_EN1_REG as *mut u32) |= SYSTEM_TSENS_CLK_EN;
            println!(">SYSTEM_PERIP_CLK_EN1_REG {:b}",*(SYSTEM_PERIP_CLK_EN1_REG as *mut u32));

            println!("<APB_SARADC_APB_TSENS_CTRL2_REG {:b}",!(*tsen).ctrl2_reg);// Power up temperature sensor
            (*tsen).ctrl2_reg |= (APB_SARADC_TSENS_XPD_WAIT & 0x0F);
            (*tsen).ctrl2_reg &= !APB_SARADC_TSENS_CLK_SEL;
            println!("<APB_SARADC_APB_TSENS_CTRL2_REG {:b}",!(*tsen).ctrl2_reg);// Power up temperature sensor

            &mut *tsen
        }
    }
    fn read_temp(&self) -> f32 {
        println!("APB_SARADC_APB_TSENS_CTRL2_REG = {:b}", self.ctrl2_reg);
        println!("APB_SARADC_APB_TSENS_CTRL_REG = {:b}", self.ctrl_reg);
        0.4386f32 * (self.ctrl_reg & APB_SARADC_TSENS_OUT) as f32 - 27.88f32 * 0f32 - 20.52f32
    }
}