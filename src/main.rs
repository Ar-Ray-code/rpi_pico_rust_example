#![no_std]
#![no_main]

use defmt::*; // `defmt`クレートが提供するデバッグプリント機能を使う
use defmt_rtt as _; // `defmt`の出力チャンネルをSWD経由で出力する"RTT"にする
use panic_probe as _; // `panic`時にデバッグヒント情報を出力する

use core::fmt::Write;

use embedded_hal::{adc::OneShot, timer::CountDown};
use fugit::{ExtU32, RateExtU32};

use rp_pico::{
    entry,
    hal::{self, gpio::Function},
    pac::{self, ADC},
};

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
use cortexm_threads::{create_thread, create_thread_with_config, init, sleep};

// global variables
static mut ADC_READ: u16 = 0u16;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const PWM_50_HZ: u8 = 20u8;

static mut GPIO_14: Option<
    crate::hal::gpio::Pin<
        crate::hal::gpio::bank0::Gpio14,
        crate::hal::gpio::Output<crate::hal::gpio::PushPull>,
    >,
> = None;

static mut ADC: Option<crate::hal::adc::Adc> = None;
static mut ADC_PIN_0: Option<
    crate::hal::gpio::Pin<
        crate::hal::gpio::bank0::Gpio26,
        crate::hal::gpio::Input<crate::hal::gpio::Floating>,
    >,
> = None;

static mut DISPLAY: Option<
    Ssd1306<
        I2CInterface<
            hal::I2C<
                pac::I2C0,
                (
                    hal::gpio::Pin<hal::gpio::bank0::Gpio16, Function<hal::gpio::I2C>>,
                    hal::gpio::Pin<hal::gpio::bank0::Gpio17, Function<hal::gpio::I2C>>,
                ),
            >,
        >,
        DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
    >,
> = None;

static mut PWM_CHANNEL_B: Option<
    hal::pwm::Channel<hal::pwm::Pwm7, hal::pwm::FreeRunning, hal::pwm::B>,
> = None;

fn adc_read_thread() -> ! {
    info!("Hello from thread 1");

    loop {
        unsafe {
            ADC_READ = ADC
                .as_mut()
                .unwrap()
                .read(ADC_PIN_0.as_mut().unwrap())
                .unwrap();
            info!("ADC: {}", ADC_READ);
        }
        sleep(100);
    }
}

fn pwm_led_thread() -> ! {
    info!("Hello from thread 2");
    loop {
        unsafe {
            let duty: u16 = (4095u16 - ADC_READ) * 4u16 + 2500u16;

            let channel = PWM_CHANNEL_B.as_mut().unwrap();
            channel.set_duty(duty);
            info!("ADC (Duty): {} ({})", ADC_READ, duty);

            if duty > 4500 {
                GPIO_14.as_mut().unwrap().set_high().unwrap();
            } else {
                GPIO_14.as_mut().unwrap().set_low().unwrap();
            }
        }
        sleep(100);
    }
}

fn display_thread() -> ! {
    info!("Hello from display thread");
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    loop {
        let mut buf = FmtBuf::new();
        unsafe {
            let duty: u16 = (4095u16 - ADC_READ) * 4u16 + 2500u16;
            buf.write_fmt(format_args!("Servo: {}", duty)).unwrap();

            DISPLAY.as_mut().unwrap().clear();
            Text::with_baseline(buf.as_str(), Point::new(0, 32), text_style, Baseline::Top)
                .draw(DISPLAY.as_mut().unwrap())
                .unwrap();
            Text::with_baseline("Hello Rust!", Point::new(0, 0), text_style, Baseline::Top)
                .draw(DISPLAY.as_mut().unwrap())
                .unwrap();
            DISPLAY.as_mut().unwrap().flush().unwrap();
        }
        sleep(100);
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

    // Clock setup ====================================
    let mut my_pac = pac::Peripherals::take().unwrap();
    let mut my_watchdog = hal::Watchdog::new(my_pac.WATCHDOG);
    let my_clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        my_pac.XOSC,
        my_pac.CLOCKS,
        my_pac.PLL_SYS,
        my_pac.PLL_USB,
        &mut my_pac.RESETS,
        &mut my_watchdog,
    )
    .ok()
    .unwrap();

    // GPIO setup ====================================
    let sio = hal::Sio::new(my_pac.SIO);
    let pins = rp_pico::Pins::new(
        my_pac.IO_BANK0,
        my_pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut my_pac.RESETS,
    );

    // LED setup ====================================
    let led_pin = pins.gpio14.into_push_pull_output();
    unsafe {
        GPIO_14 = Some(led_pin);
    }

    // ADC setup ====================================
    let adc = hal::Adc::new(my_pac.ADC, &mut my_pac.RESETS);
    let adc_pin_0 = pins.gpio26.into_floating_input();
    unsafe {
        ADC = Some(adc);
    }
    unsafe {
        ADC_PIN_0 = Some(adc_pin_0);
    }

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
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    unsafe {
        DISPLAY = Some(display);
    }

    // Servo setup ====================================
    let pwm_slices = hal::pwm::Slices::new(my_pac.PWM, &mut my_pac.RESETS);
    let mut pwm = pwm_slices.pwm7;

    pwm.set_ph_correct();
    pwm.set_div_int(PWM_50_HZ);
    pwm.enable();
    let mut channel = pwm.channel_b;
    channel.output_to(pins.gpio15);
    unsafe {
        PWM_CHANNEL_B = Some(channel);
    }

    // Thread setup ====================================
    let mut stack1 = [0xDEADBEEF; 512];
    let mut stack2 = [0xDEADBEEF; 512];
    let mut stack3 = [0xDEADBEEF; 512];

    let _ = create_thread(&mut stack1, adc_read_thread);
    let _ = create_thread(&mut stack2, pwm_led_thread);
    let _ = create_thread(&mut stack3, display_thread);

    init();
}
