//! Blinky of onboard RGB LED, Use GPIOHS
//!
//! Board: Maixduino
#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use k210_hal::stdout::Stdout;
use k210_hal::{fpioa, gpio::Gpio, pac, prelude::*};
use k210_hal::fpioa::functions;
use k210_hal::gpiohs::Gpiohs;
use panic_halt as _;

use k210_playground::delay::McycleDelay;

#[riscv_rt::entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut sysctl = p.SYSCTL.constrain();
    let fpioa = p.FPIOA.split(&mut sysctl.apb0);

    // uart
    let clocks = k210_hal::clock::Clocks::new();
    let serial = p.UARTHS.configure(115_200.bps(), &clocks);
    let (mut tx, _) = serial.split();
    let mut stdout = Stdout(&mut tx);

    // delay
    let mut delay = McycleDelay::new(&clocks);

    // LEDs
    let gpio = p.GPIO.split(&mut sysctl.apb0);
    let gpiohs = p.GPIOHS.split();

    let io13 = fpioa.io13.into_function(functions::GPIOHS0);
    let mut red = Gpiohs::new(gpiohs.gpiohs0, io13).into_push_pull_output();

    let io12 = fpioa.io12.into_function(fpioa::GPIOHS1);
    let mut green = Gpiohs::new(gpiohs.gpiohs1, io12).into_push_pull_output();

    let io14 = fpioa.io14.into_function(fpioa::GPIOHS2);
    let mut blue = Gpiohs::new(gpiohs.gpiohs2, io14).into_push_pull_output();

    // disable LCD backlight
    let io17 = fpioa.io17.into_function(fpioa::GPIO7);
    let mut back_light = Gpio::new(gpio.gpio7, io17).into_push_pull_output();
    back_light.set_high().unwrap();

    //blue.set_low().ok();

    writeln!(stdout, "Waking on LED...").unwrap();

    loop {
        writeln!(stdout, "toggle...").unwrap();
        delay.delay_ms(500);
        red.toggle().ok();
        delay.delay_ms(500);
        green.toggle().ok();
        delay.delay_ms(500);
        blue.toggle().ok();
    }
}
