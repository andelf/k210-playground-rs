use core::marker::PhantomData;

use embedded_hal::spi;
pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};
use k210_hal::clock::Clocks;
use nb::block;

// a register block
use k210_hal::fpioa::{self, functions};
use k210_hal::time::Hertz;
use k210_pac::SYSCTL;
use k210_pac::{SPI0, SPI1};

use embedded_hal::prelude::*;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    #[doc(hidden)]
    _Extensible,
}

pub struct Spi<SPI, PIN, FrameSize> {
    spi: SPI,
    cs: PIN,
    _marker: PhantomData<FrameSize>,
}

// cs: SIPEED_ST7789_SS
// #define SIPEED_ST7789_SS           3
impl<PIN> Spi<SPI0, PIN, u8> {
    /// spi_init
    pub fn spi0(
        spi: SPI0,
        cs: PIN,
        mode: Mode,
        frame_format: FrameFormat,
        endian: Endian,
        // clocks: &Clocks,
        // sysctl: &SYSCTL,
    ) -> Self
    where
        PIN: fpioa::Mode<functions::SPI0_SS3>,
    {
        // # spi_init((spi_device_num_t spi_num , spi_work_mode_t work_mode ,
        //     spi_frame_format_t frame_format,
        //     size_t data_bit_length ,
        //     uint32_t endian)
        //
        //        configASSERT(data_bit_length >= 4 && data_bit_length <= 32);

        // 1. spi_clk_init(spi_num)
        // - sysctl_clock_enable(SYSCTL_CLOCK_SPI0 + spi_num);
        //   - sysctl_clock_bus_en(clock, 1);
        //   - sysctl_clock_device_en(clock, 1);
        // - sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI0 + spi_num, 0);
        unsafe {
            // These peripheral devices are under APB2: SPI0, SPI1
            // sysctl.clk_en_cent.modify(|_, w| w.apb2_clk_en().set_bit());
            // sysctl.clk_en_peri.modify(|_, w| w.spi0_clk_en().set_bit());
            // sysctl.clk_th1.modify(|_, w| w.spi0_clk().bits(0x0));
        }

        // TODO: check frame_format against data_bit_length

        // 2. spi_adapter
        let work_mode = hal_mode_to_pac(mode);
        let default_tmod = crate::pac::spi0::ctrlr0::TMOD_A::TRANS_RECV;
        let frame_format = frame_format_to_pac(frame_format);

        let data_bit_length = 8;

        unsafe {
            if spi.baudr.read().bits() == 0 {
                spi.baudr.write(|w| w.bits(0x14));
            }
            spi.imr.write(|w| w.bits(0x00));
            spi.dmacr.write(|w| w.bits(0x00));
            spi.dmatdlr.write(|w| w.bits(0x10));
            spi.dmardlr.write(|w| w.bits(0x00));
            spi.ser.write(|w| w.bits(0x00));
            spi.ssienr.write(|w| w.bits(0x00));

            spi.ctrlr0.write(|w| {
                w.work_mode()
                    .variant(work_mode)
                    .frame_format()
                    .variant(frame_format)
                    .data_length()
                    .bits(data_bit_length - 1)
            });

            // Valid only when SSI_SPI_MODE is either set to Dual or Quad or Octal mode.
            spi.spi_ctrlr0.reset();
            spi.endian.write(|w| w.bits(endian as u32));

            // fake
            /*
            spi.spi_ctrlr0.write(|w| {
                w.aitm()
                    .as_frame_format()
                    .addr_length()
                    .bits(0)
                    .inst_length()
                    .bits(2)
                    .wait_cycles()
                    .bits(0)
            }); */
        }

        Spi {
            spi,
            cs,
            _marker: PhantomData,
        }
    }

    // TODO: use Clocks
    pub fn set_clock_rate(&mut self, freq: impl Into<Hertz>, clock: impl Into<Hertz>) {
        // spi_set_clk_rate(spi_device_num_t(_spiNum), _freq);
        //   These clock under PLL0 clock domain, using even divider.
        //   sysctl_clock_get_freq(SYSCTL_CLOCK_SPI0)
        let mut br = match clock.into().0 / freq.into().0 {
            rate if rate < 2 => 2,
            rate if rate > 65534 => 65534,
            rate => rate,
        };
        unsafe {
            self.spi.baudr.write(|w| w.bits(br));
        }
    }

    // fn set_tmod(&mut self, )

    pub fn send_data_normal(&mut self, tx: &[u8]) {
        self.spi
            .ctrlr0
            .modify(|_, w| w.tmod().variant(crate::pac::spi0::ctrlr0::TMOD_A::TRANS));
        unsafe {
            self.spi.ssienr.write(|w| w.bits(0x01));
            self.spi.ser.write(|w| w.bits(1 << 3)); // chip_select
        }

        let mut tx_len = tx.len() as u32;
        let mut i = 0;
        while tx_len > 0 {
            let mut fifo_len = self.spi.txflr.read().bits();
            fifo_len = u32::min(fifo_len, tx_len);
            // SPI_TRANS_CHAR
            for _ in 0..fifo_len {
                unsafe {
                    self.spi.dr[0].write(|w| w.bits(tx[i] as u32));
                }
                i += 1;
            }
            tx_len -= fifo_len;
        }

        // bit 2: TFE, Transmit FIFO Empty
        //     0x0 NOT_EMPTY
        //     0x1 EMPTY
        // bit 0: BUSY, SSI Busy Flag
        while self.spi.sr.read().bits() & 0b101 != 0b100 {}
        unsafe {
            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));
        }
    }

    fn read_to_end(&mut self) -> nb::Result<(), Error> {
        // spi_receive_data_standard
        unsafe {
            // TODO: spi_set_tmod(spi_num, SPI_TMOD_RECV)
            // or
            // spi_set_tmod(spi_num, SPI_TMOD_EEROM)?
            self.spi
                .ctrlr0
                .modify(|_, w| w.tmod().variant(crate::pac::spi0::ctrlr0::TMOD_A::RECV));

            // NOTE: date_bit_length will be handled by type-safty
            // date_bit_length = 8
            // frame_width = 1

            // spi_handle->ctrlr1 = (uint32_t)(v_rx_len - 1);
            self.spi.ctrlr1.write(|w| w.bits(0)); // recv len - 1
            self.spi.ssienr.write(|w| w.bits(0x01)); // SS

            // NOTE: no cmd
            // self.spi.dr[0].write(|w| w.bits(0xffffffff));

            // for SPI0_SS3 = 3
            self.spi.ser.write(|w| w.bits(1 << 3));

            // rx FIFO Level register
            loop {
                let fifo_len = self.spi.rxflr.read().bits();
                if fifo_len == 0 {
                    break;
                }
                for _ in 0..fifo_len {
                    let _rx = self.spi.dr[0].read().bits();
                }
            }
            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00)); // SS
            Ok(())
        }
    }
}

/*
// share almost the same as SPI0
impl<PIN> Spi<SPI1, PIN, u8> {
    pub fn spi1(
        spi: SPI1,
        cs: PIN,
        mode: Mode,
        frame_format: FrameFormat,
        endian: Endian,
        // clocks: &Clocks,
        sysctl: &SYSCTL,
    ) -> Self
    where
        PIN: fpioa::Mode<functions::SPI1_SS3>,
    {
        // clocks
        unsafe {
            // SPI1 is under APB2.
            sysctl.clk_en_cent.modify(|_, w| w.apb2_clk_en().set_bit());
            sysctl.clk_en_peri.modify(|_, w| w.spi1_clk_en().set_bit());
            sysctl.clk_th1.modify(|_, w| w.spi1_clk().bits(0x0));
        }

        let work_mode = hal_mode_to_pac(mode);
        let default_tmod = crate::pac::spi0::ctrlr0::TMOD_A::TRANS_RECV;
        let frame_format = frame_format_to_pac(frame_format);

        let data_bit_length = 8; // TODO: support more

        unsafe {
            if spi.baudr.read().bits() == 0 {
                spi.baudr.write(|w| w.bits(0x14));
            }
            spi.imr.write(|w| w.bits(0x00));
            spi.dmacr.write(|w| w.bits(0x00));
            spi.dmatdlr.write(|w| w.bits(0x10));
            spi.dmardlr.write(|w| w.bits(0x00));
            spi.ser.write(|w| w.bits(0x00));
            spi.ssienr.write(|w| w.bits(0x00));

            spi.ctrlr0.write(|w| {
                w.work_mode()
                    .variant(work_mode)
                    .tmod()
                    .variant(default_tmod)
                    .frame_format()
                    .variant(frame_format)
                    .data_length()
                    .bits(data_bit_length - 1)
            });

            spi.spi_ctrlr0.reset();
            spi.endian.write(|w| w.bits(endian as u32));

            spi.spi_ctrlr0.write(|w| {
                w.aitm()
                    .as_frame_format()
                    .addr_length()
                    .bits(0)
                    .inst_length()
                    .bits(2)
                    .wait_cycles()
                    .bits(0)
            });
        }
        Spi { spi, cs }
    }

    pub fn init_non_standard(
        &mut self,
        inst_len: u8,
        addr_len: u8,
        wait_cycles: u8,
        trans_mode: (),
    ) {
        let inst_len = match inst_len {
            0 => 0,
            4 => 1,
            8 => 2,
            16 => 3,
            _ => return (),
        };
        let addr_len = addr_len / 4;
        unsafe {
            self.spi.spi_ctrlr0.write(|w| {
                w.aitm()
                    .as_frame_format()
                    .addr_length()
                    .bits(addr_len)
                    .inst_length()
                    .bits(inst_len)
                    .wait_cycles()
                    .bits(wait_cycles)
            });
        }
    }

    pub fn set_clock_rate(&mut self, freq: impl Into<Hertz>, clock: impl Into<Hertz>) {
        //   sysctl_clock_get_freq(SYSCTL_CLOCK_SPI1)
        let mut br = match clock.into().0 / freq.into().0 {
            rate if rate < 2 => 2,
            rate if rate > 65534 => 65534,
            rate => rate,
        };
        unsafe {
            self.spi.baudr.write(|w| w.bits(br));
        }
    }

    fn read_to_end(&mut self) -> nb::Result<(), Error> {
        unsafe {
            self.spi
                .ctrlr0
                .modify(|_, w| w.tmod().variant(crate::pac::spi0::ctrlr0::TMOD_A::RECV));

            // NOTE: date_bit_length will be handled by type-safty
            // date_bit_length = 8
            // frame_width = 1

            // spi_handle->ctrlr1 = (uint32_t)(v_rx_len - 1);
            self.spi.ctrlr1.write(|w| w.bits(0)); // recv len - 1
            self.spi.ssienr.write(|w| w.bits(0x01)); // SS

            // NOTE: no cmd
            // self.spi.dr[0].write(|w| w.bits(0xffffffff));

            // for SPI0_SS3 = 3
            self.spi.ser.write(|w| w.bits(1 << 3));

            loop {
                // rx FIFO Level register
                let fifo_len = self.spi.rxflr.read().bits();
                if fifo_len == 0 {
                    break;
                }
                for _ in 0..fifo_len {
                    let _rx = self.spi.dr[0].read().bits();
                }
            }
            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00)); // SS
            Ok(())
        }
    }
}
*/

impl<PIN: fpioa::Mode<functions::SPI0_SS3>> embedded_hal::spi::FullDuplex<u8>
    for Spi<SPI0, PIN, u8>
{
    /// An enumeration of SPI errors
    type Error = Error;

    /// Reads the word stored in the shift register
    ///
    /// **NOTE** A word must be sent to the slave before attempting to call this
    /// method.
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        // spi_receive_data_standard
        unsafe {
            // TODO: spi_set_tmod(spi_num, SPI_TMOD_RECV)
            // or
            // spi_set_tmod(spi_num, SPI_TMOD_EEROM)?
            self.spi
                .ctrlr0
                .modify(|_, w| w.tmod().variant(crate::pac::spi0::ctrlr0::TMOD_A::RECV));

            // NOTE: date_bit_length will be handled by type-safty
            // date_bit_length = 8
            // frame_width = 1

            // spi_handle->ctrlr1 = (uint32_t)(v_rx_len - 1);
            self.spi.ctrlr1.write(|w| w.bits(0)); // recv len - 1
            self.spi.ssienr.write(|w| w.bits(0x01));

            // NOTE: no cmd
            // self.spi.dr[0].write(|w| w.bits(0xffffffff));

            // for SPI0_SS3 = 3
            self.spi.ser.write(|w| w.bits(1 << 3));

            // rx FIFO Level register
            let fifo_len = self.spi.rxflr.read().bits();
            if fifo_len > 0 {
                let rx = self.spi.dr[0].read().bits();

                self.spi.ser.write(|w| w.bits(0x00));
                self.spi.ssienr.write(|w| w.bits(0x00));

                Ok(rx as u8)
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    /// Sends a word to the slave
    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        // spi_send_data_standard
        //   spi_send_data_normal(spi_device_num_t spi_num, spi_chip_select_t chip_select,
        //     const uint8_t *tx_buff, size_t tx_len)
        //     spi_set_tmod(spi_num, SPI_TMOD_TRANS);

        unsafe {
            self.spi
                .ctrlr0
                .modify(|_, w| w.tmod().variant(crate::pac::spi0::ctrlr0::TMOD_A::TRANS));

            // transfer width: char, short, int
            // let data_bit_length = self.spi.ctrlr0.read().data_length().bits();
            // default char. TODO: others
            // 8bit data length
            // self.spi.ctrlr0.modify(|_, w| w.data_length().bits(8 - 1));

            self.spi.ssienr.write(|w| w.bits(0x01));
            // for SPI0_SS3 = 3
            self.spi.ser.write(|w| w.bits(1 << 3));

            // ??
            if self.spi.sr.read().bits() & 0x05 != 0x04 {
                return Err(nb::Error::WouldBlock);
            }

            // SPI_TRANS_CHAR/default
            let mut fifo_len = 32 - self.spi.txflr.read().bits();
            if fifo_len > 0 {
                self.spi.dr[0].write(|w| w.bits(word as u32));

                self.spi.ser.write(|w| w.bits(0x00));
                self.spi.ssienr.write(|w| w.bits(0x00));
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }

            // while self.spi.sr.read().bits() & 0x05 != 0x04 {}
        }
    }
}

// pub fn debug_send_data

/*
impl<PIN: fpioa::Mode<functions::SPI0_SS3>> embedded_hal::blocking::spi::write::Default<u8>
    for Spi<SPI0, PIN>
{
}*/

impl<PIN: fpioa::Mode<functions::SPI0_SS3>> embedded_hal::blocking::spi::Write<u8>
    for Spi<SPI0, PIN, u8>
{
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        unsafe {
            self.spi
                .ctrlr0
                .modify(|_, w| w.tmod().variant(crate::pac::spi0::ctrlr0::TMOD_A::TRANS));

            // transfer width: char, short, int
            // let data_bit_length = self.spi.ctrlr0.read().data_length().bits();
            // default char. TODO: others
            // 8bit data length
            // self.spi.ctrlr0.modify(|_, w| w.data_length().bits(8 - 1));

            self.spi.ssienr.write(|w| w.bits(0x01));
            // for SPI0_SS3 = 3
            self.spi.ser.write(|w| w.bits(1 << 3));

            // SPI_TRANS_CHAR/default
            // let mut fifo_len = 32 - self.spi.txflr.read().bits();
            for &word in words {
                while 32 - self.spi.txflr.read().bits() == 0 {}

                self.spi.dr[0].write(|w| w.bits(word as u32));
            }
            while self.spi.sr.read().bits() & 0x05 != 0x04 {}

            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));

            let _ = self.read_to_end();
            Ok(())

            // while self.spi.sr.read().bits() & 0x05 != 0x04 {}
        }

        /*
        for word in words {
            block!(self.send(word.clone()))?;
            block!(self.read())?;
        }*/
    }
}

/*
impl<PIN: fpioa::Mode<functions::SPI1_SS3>> embedded_hal::blocking::spi::Write<u8>
    for Spi<SPI1, PIN>
{
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        unsafe {
            self.spi
                .ctrlr0
                .modify(|_, w| w.tmod().variant(crate::pac::spi0::ctrlr0::TMOD_A::TRANS));

            // transfer width: char, short, int
            // let data_bit_length = self.spi.ctrlr0.read().data_length().bits();
            // default char. TODO: others
            // 8bit data length
            // self.spi.ctrlr0.modify(|_, w| w.data_length().bits(8 - 1));

            self.spi.ssienr.write(|w| w.bits(0x01));
            // for SPI1_SS3 = 3
            self.spi.ser.write(|w| w.bits(1 << 3));

            // SPI_TRANS_CHAR/default
            // let mut fifo_len = 32 - self.spi.txflr.read().bits();
            for &word in words {
                while 32 - self.spi.txflr.read().bits() == 0 {}

                self.spi.dr[0].write(|w| w.bits(word as u32));
            }
            while self.spi.sr.read().bits() & 0x05 != 0x04 {}

            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));

            // let _ = self.read_to_end();
            Ok(())

            // while self.spi.sr.read().bits() & 0x05 != 0x04 {}
        }

        /*
        for word in words {
            block!(self.send(word.clone()))?;
            block!(self.read())?;
        }*/
    }
}
*/

// NOTE: do not support data frame size other than u8, u16, u32
// Valid value for device: 4 to 32

/// SPI Frame Format
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FrameFormat {
    /// Standard SPI Frame
    Standard,
    /// Dual SPI Frame
    Dual,
    /// Quad SPI Frame
    Quad,
    /// Octal SPI Frame
    Octal,
}
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Endian {
    Little = 0,
    Big = 1,
}

impl Default for Endian {
    fn default() -> Self {
        Endian::Little
    }
}

#[inline]
fn hal_mode_to_pac(mode: Mode) -> crate::pac::spi0::ctrlr0::WORK_MODE_A {
    use crate::pac::spi0::ctrlr0::WORK_MODE_A;
    use {Phase::*, Polarity::*};
    match (mode.polarity, mode.phase) {
        (IdleLow, CaptureOnFirstTransition) => WORK_MODE_A::MODE0,
        (IdleLow, CaptureOnSecondTransition) => WORK_MODE_A::MODE1,
        (IdleHigh, CaptureOnFirstTransition) => WORK_MODE_A::MODE2,
        (IdleHigh, CaptureOnSecondTransition) => WORK_MODE_A::MODE3,
    }
}

#[inline]
fn frame_format_to_pac(frame_format: FrameFormat) -> crate::pac::spi0::ctrlr0::FRAME_FORMAT_A {
    use crate::pac::spi0::ctrlr0::FRAME_FORMAT_A;
    match frame_format {
        FrameFormat::Standard => FRAME_FORMAT_A::STANDARD,
        FrameFormat::Dual => FRAME_FORMAT_A::DUAL,
        FrameFormat::Quad => FRAME_FORMAT_A::QUAD,
        FrameFormat::Octal => FRAME_FORMAT_A::OCTAL,
    }
}
