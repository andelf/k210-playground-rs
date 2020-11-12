#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use k210_hal::stdout::Stdout;
use k210_hal::{fpioa, gpio::Gpio, pac, prelude::*};
use panic_halt as _;

use maixduino_playground::delay::McycleDelay;

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

    // led blue
    let gpio = p.GPIO.split(&mut sysctl.apb0);
    let io14 = fpioa.io14.into_function(fpioa::GPIO6);
    let mut blue = Gpio::new(gpio.gpio6, io14).into_push_pull_output();

    blue.set_low().ok();

    writeln!(stdout, "Waking on LED...").unwrap();

    loop {
        writeln!(
            stdout,
            "toggle..."
        )
        .unwrap();
        delay.delay_ms(500);
        blue.toggle().ok();
    }
}
