#![no_std]
#![no_main]

use defmt::*;   // `defmt`クレートが提供するデバッグプリント機能を使う
use defmt_rtt as _; // `defmt`の出力チャンネルをSWD経由で出力する"RTT"にする
use panic_probe as _;   // `panic`時にデバッグヒント情報を出力する

use core::fmt::Write;

use embedded_hal::{adc::OneShot, timer::CountDown};
use fugit::{ExtU32, RateExtU32};

use rp_pico::{entry, hal, pac};

// PWM
use embedded_hal::PwmPin;

// led
use embedded_hal::digital::v2::OutputPin;

// SSD1306 driver
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, Ssd1306};

mod fmtbuf;
use fmtbuf::FmtBuf;

// thread setup
use cortex_m::peripheral::syst::SystClkSource;
use cortexm_threads::{init, create_thread, create_thread_with_config, sleep};

// global variables
static mut ADC_READ: u16 = 0u16;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const PWM_50_HZ: u8 = 20u8;


fn mythread_1 () -> ! {
    info!("Hello from thread 1");

    // Clock setup ====================================
    let mut my_pac = pac::Peripherals::take().unwrap();
    let mut my_watchdog = hal::Watchdog::new(my_pac.WATCHDOG);
    let my_clocks = hal::clocks::init_clocks_and_plls(XTAL_FREQ_HZ, my_pac.XOSC, my_pac.CLOCKS, my_pac.PLL_SYS, my_pac.PLL_USB, &mut my_pac.RESETS, &mut my_watchdog).ok().unwrap();

    // GPIO setup ====================================
    let sio = hal::Sio::new(my_pac.SIO);
    let pins = rp_pico::Pins::new(my_pac.IO_BANK0, my_pac.PADS_BANK0, sio.gpio_bank0, &mut my_pac.RESETS);



    // Delay setup ====================================
    let timer = hal::Timer::new(my_pac.TIMER, &mut my_pac.RESETS);
    let mut delay = timer.count_down();

    // LED setup ====================================
    let mut led_pin = pins.gpio14.into_push_pull_output();
    led_pin.set_high().unwrap();
    delay.start(500.millis());
    nb::block!(delay.wait()).unwrap();
    led_pin.set_low().unwrap();


    // ADC setup ====================================
    let mut adc = hal::Adc::new(my_pac.ADC, &mut my_pac.RESETS);
    let mut adc_pin_0 = pins.gpio26.into_floating_input();

    // SSD1306 setup ====================================
    let sda_pin = pins.gpio16.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<hal::gpio::FunctionI2C>();

    let i2c = hal::I2C::i2c0(
        my_pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut my_pac.RESETS,
        &my_clocks.peripheral_clock,
    );

    let interface = ssd1306::I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT_9X18_BOLD).text_color(BinaryColor::On).build();


    // Servo setup ====================================
    let mut pwm_slices = hal::pwm::Slices::new(my_pac.PWM, &mut my_pac.RESETS);
    let pwm = &mut pwm_slices.pwm7;
    pwm.set_ph_correct();

    pwm.set_div_int(PWM_50_HZ);
    pwm.enable();
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio15);
    info!("PWM setup done");

    let mut buf = FmtBuf::new();
    buf.write_str("Hello Rust!").unwrap();

    loop {
        buf.reset();
        display.clear();

        unsafe {
            ADC_READ = adc.read(&mut adc_pin_0).unwrap();
        }
        let duty: u16 = (4095u16 - unsafe { ADC_READ }) * 4u16 + 2500u16;
        channel.set_duty(duty);

        // led output
        if duty > 4500 {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }

        // display output
        buf.write_fmt(format_args!("Servo: {}", duty)).unwrap();
        Text::with_baseline("Hello Rust!", Point::new(0, 0), text_style, Baseline::Top).draw(&mut display).unwrap();
        Text::with_baseline(buf.as_str(), Point::new(0, 32), text_style, Baseline::Top).draw(&mut display).unwrap();
        display.flush().unwrap();

        // delay
        delay.start(500.millis());
        nb::block!(delay.wait()).unwrap();
    }

}

fn mythread_2 () -> ! {
    info!("Hello from thread 2");
    // TODO: Set GPIO and use them
    loop {
        unsafe {
            info!("ADC: {}", ADC_READ);
        }
        sleep(1000);
    }
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut syst = cp.SYST;

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(80_000);
    syst.enable_counter();
    syst.enable_interrupt();

	let mut stack1 = [0xDEADBEEF; 512];
    let mut stack2 = [0xDEADBEEF; 512];

    let _ = create_thread(
        &mut stack1,
        mythread_1
    );
    let _ = create_thread(&mut stack2, mythread_2);

    init();
}
