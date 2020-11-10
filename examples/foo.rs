#![no_std]
#![no_main]

extern crate panic_halt;

use k210_hal::pac::Peripherals;
use k210_hal::prelude::*;
// use k210_hal::serial::Tx;
use embedded_hal::digital::v2::OutputPin;
use k210_hal::stdout::Stdout;
use k210_hal::{fpioa, gpio::Gpio, pac};
use riscv::register::mhartid;
use riscv_rt::entry;

// Rgb565
// ST7789
#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    //configure_fpioa(p.FPIOA);
    let mut sysctl = dp.SYSCTL.constrain();
    let fpioa = dp.FPIOA.split(&mut sysctl.apb0);
    let gpio = dp.GPIO.split(&mut sysctl.apb0);

    // Configure clocks (TODO)
    let clocks = k210_hal::clock::Clocks::new();

    // Configure UART
    let serial = dp.UARTHS.configure(115_200.bps(), &clocks);
    let (mut tx, _) = serial.split();
    let mut stdout = Stdout(&mut tx);

    // 8-line spi

    // new(spi: SPI, dc: DC, cs: CS)
    let mut cs =
        Gpio::new(gpio.gpio6, fpioa.io36.into_function(fpioa::GPIO6)).into_push_pull_output();
    let mut rst =
        Gpio::new(gpio.gpio7, fpioa.io37.into_function(fpioa::GPIO7)).into_push_pull_output();
    let mut dc =
        Gpio::new(gpio.gpio5, fpioa.io38.into_function(fpioa::GPIO5)).into_push_pull_output();
    let mut wr =
        Gpio::new(gpio.gpio4, fpioa.io39.into_function(fpioa::GPIO4)).into_push_pull_output();

    writeln!(stdout, "Hello, Rust from hart {}", mhartid::read()).unwrap();
    writeln!(stdout, "CPU freq: {}", clocks.cpu().0).unwrap();
    writeln!(stdout, "APB0 freq: {}", clocks.apb0().0).unwrap();
    writeln!(stdout, "Waking other harts...").unwrap();

    loop {}
}
