#![no_std]
#![no_main]
#![feature(asm)]

extern crate panic_halt;

use k210_hal::pac::Peripherals;
use k210_hal::prelude::*;
use k210_hal::time::Hertz;
// use k210_hal::serial::Tx;
// use embedded_hal::digital::OutputPin;
// use embedded_hal::digital::OutputPin;
use embedded_hal::prelude::*;
use k210_hal::stdout::Stdout;
use k210_hal::{fpioa, gpio::Gpio, pac};
use riscv::register::mhartid;
use riscv_rt::entry;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_hal::blocking::spi::write::Default as _DefaultWriteForFullDuplex;
use maixduino_playground::delay::McycleDelay;
use maixduino_playground::spi::{self, Spi};
use st7789::{Orientation, ST7789};

use embedded_graphics::fonts::{Font6x8, Text};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{primitive_style, text_style};

use num_traits::{Float, Num};

/*
CONFIG_CPU_DEFAULT_FREQ=400000000

#define FREQ_PLL0_MAX        1200000000UL //1800MHz max
#define FREQ_PLL0_DEFAULT    (CONFIG_CPU_DEFAULT_FREQ*2)
#define FREQ_PLL0_MIN        52000000UL
#define FREQ_PLL1_MAX        1200000000UL //1800MHz max
#define FREQ_PLL1_DEFAULT    400000000UL
#define FREQ_PLL1_MIN        26000000UL
#define FREQ_PLL2_DEFAULT    45158400UL
*/

const FREQ_PLL0_DEFAULT: u32 = 400000000 * 2;
const FREQ_PLL1_DEFAULT: u32 = 400000000;
const FREQ_PLL2_DEFAULT: u32 = 45158400;

// Rgb565
// ST7789
#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let sysctl = dp.SYSCTL;

    let sysctl2 = unsafe { core::mem::MaybeUninit::<k210_pac::SYSCTL>::zeroed().assume_init() };

    // TODO: set for spi0
    /*
    unsafe {
        sysctl.clk_en_cent.write(|w| w.apb2_clk_en().set_bit());
        sysctl.clk_en_peri.write(|w| w.spi0_clk_en().set_bit());
        sysctl.clk_th1.write(|w| w.spi0_clk().bits(0x0));
    }*/

    //configure_fpioa(p.FPIOA);
    let mut sysctl = sysctl.constrain();
    let fpioa = dp.FPIOA.split(&mut sysctl.apb0);
    let gpio = dp.GPIO.split(&mut sysctl.apb0);

    let clocks = k210_hal::clock::Clocks::new();

    // Configure UART
    let serial = dp.UARTHS.configure(115_200.bps(), &clocks);
    let (mut tx, _) = serial.split();
    let mut stdout = Stdout(&mut tx);

    let mut delay = McycleDelay::new(&clocks);

    writeln!(stdout, "== Hello from Rust! ==").unwrap();

    writeln!(
        stdout,
        "[Maix.rs]Pll0:freq:{}",
        sysctl_pll_get_freq_pll0(&sysctl2)
    );
    writeln!(
        stdout,
        "[Maix.rs]Pll1:freq:{}",
        sysctl_pll_get_freq_pll1(&sysctl2)
    );
    writeln!(
        stdout,
        "[Maix.rs]Pll2:freq:{}",
        sysctl_pll_get_freq_pll2(&sysctl2)
    );
    writeln!(
        stdout,
        "[Maix.rs]cpu:freq:{}",
        sysctl_pll_get_freq_cpu(&sysctl2)
    );


    writeln!(
        stdout,
        "[Maix.rs]spi0:freq:{}",
        sysctl_clock_get_freq_spi0(&sysctl2)
    );

    // doing init seq in MaxiPy
    // Configure clocks (TODO)
    // init seq from maixpy
    /*
    [MAIXPY]Pll0:freq:832000000
    [MAIXPY]Pll1:freq:398666666
    [MAIXPY]Pll2:freq:45066666
    [MAIXPY]cpu:freq:416000000
    [MAIXPY]kpu:freq:398666666
    */


/*
    writeln!(
        stdout,
        "[Maix.rs]Pll0:freq:{}",
        sysctl_pll_get_freq_pll0(&sysctl2)
    );
    writeln!(
        stdout,
        "[Maix.rs]Pll1:freq:{}",
        sysctl_pll_get_freq_pll1(&sysctl2)
    );
    writeln!(
        stdout,
        "[Maix.rs]Pll2:freq:{}",
        sysctl_pll_get_freq_pll2(&sysctl2)
    );
    writeln!(
        stdout,
        "[Maix.rs]cpu:freq:{}",
        sysctl_pll_get_freq_cpu(&sysctl2)
    );
*/
    writeln!(
        stdout,
        "clk_en_cent/apb2_clk_en: {:08x}",
        sysctl2.clk_en_cent.read().bits()
    );
    writeln!(
        stdout,
        "clk_en_peri/spi0_clk_en: {:08x}",
        sysctl2.clk_en_peri.read().bits()
    );
    writeln!(stdout, "clk_th1: {:08x}", sysctl2.clk_th1.read().bits());


   // sysctl_pll_set_freq_pll0(&sysctl2, 806.mhz());
    /*
    [Maix.rs]Pll0:freq:780000000
    [Maix.rs]Pll1:freq:299000000
    [Maix.rs]Pll2:freq:299000000
    [Maix.rs]cpu:freq:390000000
    */


    delay.delay_ms(1000);

    writeln!(
        stdout,
        "apb2 enabled: {}",
        sysctl2.clk_en_cent.read().apb2_clk_en().bits()
    );

    unsafe {
        //sysctl2.clk_en_cent.write(|w| w.apb2_clk_en().set_bit());
        writeln!(stdout, "stage 1");
        delay.delay_ms(1000);
        sysctl2.clk_en_peri.write(|w| w.spi0_clk_en().set_bit());
        writeln!(stdout, "stage 2");
        delay.delay_ms(1000);
        sysctl2.clk_th1.write(|w| w.spi0_clk().bits(0x0));
        writeln!(stdout, "stage 3");
        delay.delay_ms(1000);
    }

    writeln!(stdout, "CPU freq: {}", clocks.cpu().0).unwrap();
    writeln!(stdout, "APB0 freq: {}", clocks.apb0().0).unwrap();

    // 8-line spi

    // new(spi: SPI, dc: DC, cs: CS)
    // SIPEED_ST7789_SS_PIN
    let cs = fpioa.io36.into_function(fpioa::SPI0_SS3);

    // SIPEED_ST7789_RST_PIN
    let rst = Gpio::new(gpio.gpio7, fpioa.io37.into_function(fpioa::GPIO7)).into_push_pull_output();
    // SIPEED_ST7789_DCX_PIN
    let dc = Gpio::new(gpio.gpio5, fpioa.io38.into_function(fpioa::GPIO5)).into_push_pull_output();
    // SIPEED_ST7789_SCLK_PIN, WR, clk
    let _sclk = fpioa.io39.into_function(fpioa::SPI0_SCLK);

    // sysctl_set_spi0_dvp_data(1);
    let mut spi = Spi::spi0(
        dp.SPI0,
        cs,
        spi::MODE_0,
        spi::FrameFormat::Octal,
        spi::Endian::Little,
        // &dp.SYSCTL,
    );
    // not cpu clock. via sysctl
    spi.set_clock_rate(16.mhz(), 390000000.hz());

    let di = SPIInterfaceNoCS::new(spi, dc);
    let mut lcd = ST7789::new(di, rst, 320, 240);

    for i in 0..10 {
        delay.delay_ms(500);
        writeln!(stdout, "delay {}", i).unwrap();
    }

    lcd.init(&mut delay).unwrap();
    delay.delay_ms(500);
    lcd.set_orientation(Orientation::Landscape).unwrap();
    // lcd.set_offset(1, 26);

    // Clear screen
    Rectangle::new(Point::new(0, 0), Point::new(240 - 1, 240 - 1))
        .into_styled(primitive_style!(fill_color = Rgb565::BLACK))
        .draw(&mut lcd)
        .unwrap();
    let style = text_style!(
        font = Font6x8,
        text_color = Rgb565::BLACK,
        background_color = Rgb565::GREEN
    );
    Text::new(" Hello Rust! ", Point::new(40, 35))
        .into_styled(style)
        .draw(&mut lcd)
        .unwrap();

    writeln!(stdout, "Waking on...").unwrap();

    drop(_sclk);
    loop {}
}

const SYSCTRL_CLOCK_FREQ_IN0: u32 = 26000000;

// sysctl_clock_get_freq
// (PLL)These clock directly under PLL clock domain
// They are using gated divider.
fn sysctl_pll_get_freq_pll0(sysctl: &pac::SYSCTL) -> u32 {
    let freq_in = SYSCTRL_CLOCK_FREQ_IN0;

    let nr = sysctl.pll0.read().clkr().bits() + 1;
    let nf = sysctl.pll0.read().clkf().bits() + 1;
    let od = sysctl.pll0.read().clkod().bits() + 1;

    ((freq_in as f64) / (nr as f64) * (nf as f64) / (od as f64)) as u32
}

fn sysctl_pll_get_freq_pll1(sysctl: &pac::SYSCTL) -> u32 {
    let freq_in = SYSCTRL_CLOCK_FREQ_IN0;

    let nr = sysctl.pll1.read().clkr().bits() + 1;
    let nf = sysctl.pll1.read().clkf().bits() + 1;
    let od = sysctl.pll1.read().clkod().bits() + 1;

    ((freq_in as f64) / (nr as f64) * (nf as f64) / (od as f64)) as u32
}

fn sysctl_pll_get_freq_pll2(sysctl: &pac::SYSCTL) -> u32 {
    // Get input freq accroding select register

    // [0] = SYSCTL_SOURCE_IN0,
    // [1] = SYSCTL_SOURCE_PLL0,
    // [2] = SYSCTL_SOURCE_PLL1,
    let freq_in = match sysctl.pll2.read().ckin_sel().bits() {
        0 => SYSCTRL_CLOCK_FREQ_IN0,
        1 => sysctl_pll_get_freq_pll0(sysctl),
        2 => sysctl_pll_get_freq_pll1(sysctl),
        _ => 0,
    };

    let nr = sysctl.pll2.read().clkr().bits() + 1;
    let nf = sysctl.pll2.read().clkf().bits() + 1;
    let od = sysctl.pll2.read().clkod().bits() + 1;

    ((freq_in as f64) / (nr as f64) * (nf as f64) / (od as f64)) as u32
}

// SPI clock
// source = sysctl_clock_source_get_freq(SYSCTL_SOURCE_PLL0);
// result = source / ((sysctl_clock_get_threshold(SYSCTL_THRESHOLD_SPI0) + 1) * 2);
fn sysctl_clock_get_freq_spi0(sysctl: &pac::SYSCTL) -> u32 {
    let source = sysctl_pll_get_freq_pll0(sysctl);
    // SYSCTL_THRESHOLD_SPI0
    let th = sysctl.clk_th1.read().spi0_clk().bits();
    source / ((th + 1) * 2) as u32
}



// sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)
fn sysctl_pll_get_freq_cpu(sysctl: &pac::SYSCTL) -> u32 {
    // SYSCTL_CLOCK_SELECT_ACLK
    match sysctl.clk_sel0.read().aclk_sel().bit() {
        false => SYSCTRL_CLOCK_FREQ_IN0,
        // sysctl_clock_source_get_freq(SYSCTL_SOURCE_PLL0) /
        //        (2ULL << sysctl_clock_get_threshold(SYSCTL_THRESHOLD_ACLK));
        true => {
            let th = sysctl.clk_sel0.read().aclk_divider_sel().bits();
            sysctl_pll_get_freq_pll0(sysctl) / (2 << th)
        }
    }
}

// sysctl_source
//    SYSCTL_SOURCE_IN0, // 0
//    SYSCTL_SOURCE_PLL0,
//    SYSCTL_SOURCE_PLL1,
//    SYSCTL_SOURCE_PLL2,
//    SYSCTL_SOURCE_ACLK,

// 1. Change CPU CLK to XTAL
//    sysctl_clock_set_clock_select(SYSCTL_CLOCK_SELECT_ACLK, SYSCTL_SOURCE_IN0)
// 2. Disable PLL output
// 3. Turn off PLL
// 4. Set PLL new value
// 5. Power on PLL
// 6. Reset PLL then Release Reset
// 7. Get lock status, wait PLL stable
// 8. Enable PLL output
// 9. Change CPU CLK to PLL
//    sysctl_clock_set_clock_select(SYSCTL_CLOCK_SELECT_ACLK, SYSCTL_SOURCE_PLL0)
fn sysctl_pll_set_freq_pll0(sysctl: &pac::SYSCTL, freq: impl Into<Hertz>) -> u32 {
    unsafe {
        // 1. Change CPU CLK to XTAL
        // SYSCTL_SOURCE_IN0 = 0, only for ppl0
        sysctl.clk_sel0.write(|w| w.aclk_sel().clear_bit());

        // 2. Disable PLL output
        sysctl.pll0.write(|w| w.out_en().clear_bit());

        // 3. Turn off PLL
        sysctl.pll0.write(|w| w.pwrd().clear_bit());

        // 4. Set PLL new value
        // result = sysctl_pll_source_set_freq(pll, SYSCTL_SOURCE_IN0, pll_freq);
        let freq_in = SYSCTRL_CLOCK_FREQ_IN0;
        let freq_out = freq.into().0;
        let (nrx, nfx, no, nb) = calculate_pll_register_value(freq_in, freq_out).unwrap();
        sysctl.pll0.write(|w| {
            w.clkr()
                .bits(nrx - 1)
                .clkf()
                .bits(nfx - 1)
                .clkod()
                .bits(no - 1)
                .bwadj()
                .bits(nb - 1)
        });

        // 5. Power on PLL
        sysctl.pll0.write(|w| w.pwrd().set_bit());
        // wait >100ns
        for _ in 0..1000 {
            asm!("nop");
        }

        // 6. Reset PLL then Release Reset
        sysctl.pll0.write(|w| w.reset().clear_bit());
        sysctl.pll0.write(|w| w.reset().set_bit());
        // wait >100ns
        for _ in 0..1000 {
            asm!("nop");
        }
        sysctl.pll0.write(|w| w.reset().clear_bit());

        // 7. Get lock status, wait PLL stable
        while sysctl.pll_lock.read().pll_lock0().bits() == 3 {
            sysctl.pll_lock.write(|w| w.pll_slip_clear0().set_bit());
        }

        // 8. Enable PLL output
        sysctl.pll0.write(|w| w.out_en().set_bit());

        // 9. Change CPU CLK to PLL
        sysctl.clk_sel0.write(|w| w.aclk_sel().set_bit());
    }

    sysctl_pll_get_freq_pll0(sysctl)
}

// nrx, nfx, no, nb
// clkr, clkf, clkod, bwad
#[allow(warnings)]
fn calculate_pll_register_value(freq_in: u32, freq_out: u32) -> Option<(u8, u8, u8, u8)> {
    /* constants */
    const vco_min: f64 = 3.5e+08;
    const vco_max: f64 = 1.75e+09;
    const ref_min: f64 = 1.36719e+07;
    const ref_max: f64 = 1.75e+09;
    const nr_min: i32 = 1;
    const nr_max: i32 = 16;
    const nf_min: i32 = 1;
    const nf_max: i32 = 64;
    const no_min: i32 = 1;
    const no_max: i32 = 16;
    const nb_min: i32 = 1;
    const nb_max: i32 = 64;
    const max_vco: i32 = 1;
    const ref_rng: i32 = 1;

    /* variables */
    let mut nr: i32 = 0;
    let mut nrx: i32 = 0;
    let mut nf: i32 = 0;
    let mut _nfi: i32 = 0;
    let mut no: i32 = 0;
    let mut noe: i32 = 0;
    let mut not: i32 = 0;
    let mut nor: i32 = 0;
    let mut nore: i32 = 0;
    let mut nb: i32 = 0;
    let mut first: i32 = 0;
    let mut firstx: i32 = 0;
    let mut found: i32 = 0;

    let mut nfx: i64 = 0;
    let (mut fin, mut fout, mut fvco) = (0_f64, 0_f64, 0_f64);
    let (mut val, mut nval, mut err, mut merr, mut terr) = (0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    let (mut x_nrx, mut x_no, mut x_nb) = (0_i32, 0_i32, 0_i32);
    let mut x_nfx: i64 = 0;
    let (mut x_fvco, mut x_err) = (0_f64, 0_f64);

    fin = freq_in as f64;
    fout = freq_out as f64;
    val = fout / fin;
    terr = 0.5 / ((nf_max / 2) as f64);
    first = 1;
    firstx = 1;

    if terr != -2.0 {
        first = 0;
        if terr == 0.0 {
            terr = 1e-16;
        }
        merr = fabs(terr);
    }
    found = 0;
    for nfi in val as i32..nf_max {
        nr = rint(nfi as f64 / val) as i32;
        if nr == 0 {
            continue;
        }
        if ref_rng != 0 && nr < nr_min {
            continue;
        }
        if fin / nr as f64 > ref_max {
            continue;
        }
        nrx = nr;
        nf = nfi;
        nfx = nfi as i64;
        nval = nfx as f64 / nr as f64;
        if nf == 0 {
            nf = 1;
        }
        err = 1_f64 - nval / val;

        if first != 0 || fabs(err) < merr * (1.0 + 1e-6) || fabs(err) < 1e-16 {
            not = floor(vco_max / fout) as i32;
            // a for loop
            no = i32::min(no_max, not);
            loop {
                if no > no_min {
                    if ref_rng != 0 && (nr / no < nr_min) {
                        no -= 1;
                        continue;
                    }
                    if nr % no == 0 {
                        break;
                    }
                } else {
                    break;
                }
                no -= 1;
            }
            if nr % no != 0 {
                continue;
            }

            nor = if not > no_max { no_max / no } else { not / no };
            nore = nf_max / nf;
            if nor > nore {
                nor = nore;
            }
            noe = ceil(vco_min / fout) as i32;
            if max_vco == 0 {
                nore = (noe - 1) / no + 1;
                nor = nore;
                not = 0; /* force next if to fail */
            }
            if (((no * nor) < (not >> 1)) || ((no * nor) < noe)) && ((no * nor) < (nf_max / nf)) {
                no = nf_max / nf;
                if no > no_max {
                    no = no_max;
                }
                if no > not {
                    no = not;
                }
                nfx *= no as i64;
                nf *= no;
                if (no > 1) && (firstx == 0) {
                    continue;
                }
            /* wait for larger nf in later iterations */
            } else {
                nrx /= no;
                nfx *= nor as i64;
                nf *= nor;
                no *= nor;
                if no > no_max {
                    continue;
                }
                if (nor > 1) && (firstx == 0) {
                    continue;
                }
                /* wait for larger nf in later iterations */
            }

            nb = nfx as i32;
            if nb < nb_min {
                nb = nb_min;
            }
            if nb > nb_max {
                continue;
            }

            fvco = fin / (nrx as f64) * (nfx as f64);
            if fvco < vco_min {
                continue;
            }
            if fvco > vco_max {
                continue;
            }
            if nf < nf_min {
                continue;
            }
            if (ref_rng != 0) && (fin / (nrx as f64) < ref_min) {
                continue;
            }
            if (ref_rng != 0) && (nrx > nr_max) {
                continue;
            }
            if !(((firstx != 0) && (terr < 0.0))
                || (fabs(err) < merr * (1.0 - 1e-6))
                || ((max_vco != 0) && (no > x_no)))
            {
                continue;
            }
            if (firstx == 0) && (terr >= 0.0) && (nrx > x_nrx) {
                continue;
            }

            found = 1;
            x_no = no;
            x_nrx = nrx;
            x_nfx = nfx;
            x_nb = nb;
            x_fvco = fvco;
            x_err = err;
            first = 0;
            firstx = 0;
            merr = fabs(err);
            if terr != -1.0 {
                continue;
            }
        }
    }
    if found == 0 {
        return None;
    }
    nrx = x_nrx;
    nfx = x_nfx;
    no = x_no;
    nb = x_nb;
    fvco = x_fvco;
    err = x_err;
    if (terr != -2.0) && (fabs(err) >= terr * (1.0 - 1e-6)) {
        return None;
    }

    Some((nrx as u8, nfx as u8, no as u8, nb as u8))
}

fn fabs(a: f64) -> f64 {
    if a >= 0.0 {
        a
    } else {
        -a
    }
}

fn rint(a: f64) -> f64 {
    if a >= 0.0 {
        (a + 0.5) as i32 as f64
    } else {
        (a - 0.5) as i32 as f64
    }
}

fn floor(n: f64) -> f64 {
    n.floor()
    /*
    if n % 1.0 >= 0.0 {
        n - n % 1.0
    } else {
        n - (1.0 + n % 1.0)
    }
    */
}

fn ceil(n: f64) -> f64 {
    -floor(-n)
}
