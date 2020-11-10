#![no_std]
#![no_main]

extern crate panic_halt;

use k210_hal::pac::Peripherals;
use k210_hal::prelude::*;
// use k210_hal::serial::Tx;
use k210_hal::stdout::Stdout;
use riscv::register::mhartid;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    //configure_fpioa(p.FPIOA);

    // Configure clocks (TODO)
    let clocks = k210_hal::clock::Clocks::new();

    // Configure UART
    let serial = dp.UARTHS.configure(115_200.bps(), &clocks);
    let (mut tx, _) = serial.split();

    let mut stdout = Stdout(&mut tx);

    writeln!(stdout, "Hello, Rust from hart {}", mhartid::read()).unwrap();
    writeln!(stdout, "Waking other harts...").unwrap();

    loop {}
}
