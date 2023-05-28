#![no_std]
#![no_main]

use panic_halt as _;
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


#[entry]
fn main() -> ! {
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;
    const PWM_50_HZ: u8 = 20u8;

    // Clock setup ====================================
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // GPIO setup ====================================
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Delay setup ====================================
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // LED setup ====================================
    let mut led_pin = pins.gpio15.into_push_pull_output();
    led_pin.set_high().unwrap();
    delay.start(500.millis());
    nb::block!(delay.wait()).unwrap();
    led_pin.set_low().unwrap();


    // ADC setup ====================================
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = pins.gpio26.into_floating_input();

    // SSD1306 setup ====================================
    let sda_pin = pins.gpio16.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<hal::gpio::FunctionI2C>();

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let interface = ssd1306::I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT_9X18_BOLD).text_color(BinaryColor::On).build();
    let mut buf = FmtBuf::new();

    // Servo setup ====================================
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_ph_correct();

    pwm.set_div_int(PWM_50_HZ);
    pwm.enable();
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio1);

    // Main loop ====================================
    loop {
        buf.reset();
        display.clear();

        let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();

        // servo output
        let duty: u16 = (4095u16 - pin_adc_counts) * 4u16 + 2500u16;
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

