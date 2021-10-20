//! Serial Peripheral Interface (SPI)
//!
//! This module implements the [embedded-hal](embedded-hal) traits for
//! master mode SPI.
//!
//! # Usage
//!
//! In the simplest case, SPI can be initialised from the device
//! peripheral and the GPIO pins.
//!
//! ```
//! use stm32h7xx_hal::spi;
//!
//! let dp = ...;                   // Device peripherals
//! let (sck, miso, mosi) = ...;    // GPIO pins
//!
//! let spi = dp.SPI1.spi((sck, miso, mosi), spi::MODE_0, 1.mhz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! The GPIO pins should be supplied as a
//! tuple in the following order:
//!
//! - Serial Clock (SCK)
//! - Master In Slave Out (MISO)
//! - Master Out Slave In (MOSI)
//!
//! If one of the pins is not required, explicitly pass one of the
//! filler types instead:
//!
//! ```
//! let spi = dp.SPI1.spi((sck, spi::NoMiso, mosi), spi::MODE_0, 1.mhz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! ## Word Sizes
//!
//! The word size used by the SPI controller must be indicated to the
//! compiler. This can be done either using an explicit type
//! annotation, or with a type hint. The possible word sizes are 8
//! bits (`u8`) or 16 bits (`u16`).
//!
//! For example, an explict type annotation:
//! ```
//! let _: spi:Spi<_, _, u8> = dp.SPI1.spi((sck, spi::NoMiso, mosi), spi::MODE_0, 1.mhz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! ## Clocks
//!
//! The bitrate calculation is based upon the clock currently assigned
//! in the RCC CCIP register. The default assignments are:
//!
//! - SPI1, SPI2, SPI3: __PLL1 Q CK__
//! - SPI4, SPI5: __APB__
//! - SPI6: __PCLK4__
//!
//! [embedded_hal]: https://docs.rs/embedded-hal/0.2.3/embedded_hal/spi/index.html

use crate::hal;
pub use crate::hal::spi::{
    Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};

#[cfg(feature = "rm0455")]
use crate::stm32::rcc::{cdccip1r as ccip1r, srdccipr};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::rcc::{d2ccip1r as ccip1r, d3ccipr as srdccipr};

use crate::stm32;
use crate::stm32::spi1::{
    cfg1::MBR_A as MBR, cfg2::COMM_A as COMM, cfg2::SSIOP_A as SSIOP,
};
use core::convert::From;
use core::marker::PhantomData;
use core::ptr;

use crate::stm32::{SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};

use crate::gpio::gpioa::{PA11, PA12, PA15, PA4, PA5, PA6, PA7, PA9};
use crate::gpio::gpiob::{
    PB10, PB12, PB13, PB14, PB15, PB2, PB3, PB4, PB5, PB9,
};
use crate::gpio::gpioc::{PC1, PC10, PC11, PC12, PC2, PC3};
use crate::gpio::gpiod::{PD3, PD6, PD7};
use crate::gpio::gpioe::{PE11, PE12, PE13, PE14, PE2, PE4, PE5, PE6};
use crate::gpio::gpiof::{PF11, PF6, PF7, PF8, PF9};
use crate::gpio::gpiog::{PG10, PG11, PG12, PG13, PG14, PG8, PG9};
use crate::gpio::gpioh::{PH5, PH6, PH7};
#[cfg(not(feature = "rm0468"))]
use crate::gpio::gpioi::{PI0, PI1, PI2, PI3};
#[cfg(not(feature = "stm32h7b0"))]
use crate::gpio::gpioj::{PJ10, PJ11};
#[cfg(not(feature = "stm32h7b0"))]
use crate::gpio::gpiok::{PK0, PK1};

use crate::gpio::{Alternate, AF5, AF6, AF7, AF8};

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

/// SPI error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    /// Calling this method is not valid in this state
    InvalidCall,
    /// Can't start a transaction because one is already started
    TransactionAlreadyStarted,
    /// A buffer is too big to be processed
    BufferTooBig { max_size: usize },
}

/// Enabled SPI peripheral (type state)
pub struct Enabled;
/// Disabled SPI peripheral (type state)
pub struct Disabled;

pub trait Pins<SPI> {
    /// States whether or not the Hardware Chip Select is present in this set of pins
    const HCS_PRESENT: bool;
}
pub trait PinSck<SPI> {}
pub trait PinMiso<SPI> {}
pub trait PinMosi<SPI> {}
pub trait PinHCS<SPI> {}

impl<SPI, SCK, MISO, MOSI> Pins<SPI> for (SCK, MISO, MOSI)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
{
    const HCS_PRESENT: bool = false;
}

impl<SPI, SCK, MISO, MOSI, HCS> Pins<SPI> for (SCK, MISO, MOSI, HCS)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
    HCS: PinHCS<SPI>,
{
    const HCS_PRESENT: bool = true;
}

/// Specifies the communication mode of the SPI interface.
#[derive(Copy, Clone)]
pub enum CommunicationMode {
    /// Both RX and TX are used.
    FullDuplex,

    /// Only the SPI TX functionality is used.
    Transmitter,

    /// Only the SPI RX functionality is used.
    Receiver,
}

/// A structure for specifying SPI configuration.
///
/// This structure uses builder semantics to generate the configuration.
///
/// `Example`
/// ```
/// use embedded_hal::spi::Mode;
///
/// let config = Config::new(Mode::MODE_0)
///     .manage_cs()
/// ```
#[derive(Copy, Clone)]
pub struct Config {
    mode: Mode,
    swap_miso_mosi: bool,
    hardware_cs: HardwareCS,
    inter_word_delay: f32,
    communication_mode: CommunicationMode,
}

impl Config {
    /// Create a default configuration for the SPI interface.
    ///
    /// Arguments:
    /// * `mode` - The SPI mode to configure.
    pub fn new(mode: Mode) -> Self {
        Config {
            mode,
            swap_miso_mosi: false,
            hardware_cs: HardwareCS {
                mode: HardwareCSMode::Disabled,
                assertion_delay: 0.0,
                polarity: Polarity::IdleHigh,
            },
            inter_word_delay: 0.0,
            communication_mode: CommunicationMode::FullDuplex,
        }
    }

    /// Specify that the SPI MISO/MOSI lines are swapped.
    ///
    /// Note:
    /// * This function updates the HAL peripheral to treat the pin provided in the MISO parameter
    /// as the MOSI pin and the pin provided in the MOSI parameter as the MISO pin.
    pub fn swap_mosi_miso(mut self) -> Self {
        self.swap_miso_mosi = true;
        self
    }

    /// Specify the behaviour of the hardware chip select.
    ///
    /// This also affects the way data is sent using [HardwareCSMode].
    /// By default the hardware cs is disabled.
    pub fn hardware_cs(mut self, hardware_cs: HardwareCS) -> Self {
        self.hardware_cs = hardware_cs;
        self
    }

    /// Specify the time in seconds that should be idled between every data word being sent.
    ///
    /// Note:
    /// * This value is converted to a number of spi peripheral clock ticks and at most 15 of those.
    pub fn inter_word_delay(mut self, inter_word_delay: f32) -> Self {
        self.inter_word_delay = inter_word_delay;
        self
    }

    /// Select the communication mode of the SPI bus.
    pub fn communication_mode(mut self, mode: CommunicationMode) -> Self {
        self.communication_mode = mode;
        self
    }
}

impl From<Mode> for Config {
    fn from(mode: Mode) -> Self {
        Self::new(mode)
    }
}

/// Object containing the settings for the hardware chip select pin
#[derive(Clone, Copy)]
pub struct HardwareCS {
    /// The value that determines the behaviour of the hardware chip select pin.
    pub mode: HardwareCSMode,
    /// The delay between CS assertion and the beginning of the SPI transaction in seconds.
    ///
    /// Note:
    /// * This value introduces a delay on SCK from the initiation of the transaction. The delay
    /// is specified as a number of SCK cycles, so the actual delay may vary.
    pub assertion_delay: f32,
    /// The polarity of the CS pin.
    pub polarity: Polarity,
}

#[derive(Debug, Clone, Copy)]
pub enum HardwareCSMode {
    /// Handling the CS is left for the user to do in software
    Disabled,
    /// The CS will assert when the first data is sent and will not de-assert,
    /// unless manually done in software using [Spi::end_transaction].
    EndlessTransaction,
    /// The CS will assert and de-assert for each word being sent
    WordTransaction,
    /// The CS will assert and only de-assert after the whole frame is sent.
    ///
    /// When this mode is active, the blocking embedded hal interface automatically
    /// sets up the frames so it will be one frame per call.
    ///
    /// Note:
    /// * This mode does require some maintenance. Before sending, you must setup
    /// the frame with [Spi::setup_transaction]. After everything has been sent,
    /// you must also clean it up with [Spi::end_transaction].
    FrameTransaction,
}

impl HardwareCS {
    fn assertion_delay(&self) -> f32 {
        self.assertion_delay
    }

    fn polarity(&self) -> Polarity {
        self.polarity
    }

    fn enabled(&self) -> bool {
        !matches!(self.mode, HardwareCSMode::Disabled)
    }

    fn interleaved_cs(&self) -> bool {
        matches!(self.mode, HardwareCSMode::WordTransaction { .. })
    }
}

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;
/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;
/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

macro_rules! pins {
    ($($SPIX:ty:
       SCK: [$($( #[ $pmeta1:meta ] )* $SCK:ty),*]
       MISO: [$($( #[ $pmeta2:meta ] )* $MISO:ty),*]
       MOSI: [$($( #[ $pmeta3:meta ] )* $MOSI:ty),*]
       HCS: [$($( #[ $pmeta4:meta ] )* $HCS:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl PinSck<$SPIX> for $SCK {}
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl PinMiso<$SPIX> for $MISO {}
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl PinMosi<$SPIX> for $MOSI {}
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl PinHCS<$SPIX> for $HCS {}
            )*
        )+
    }
}

pins! {
    SPI1:
        SCK: [
            NoSck,
            PA5<Alternate<AF5>>,
            PB3<Alternate<AF5>>,
            PG11<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PA6<Alternate<AF5>>,
            PB4<Alternate<AF5>>,
            PG9<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PA7<Alternate<AF5>>,
            PB5<Alternate<AF5>>,
            PD7<Alternate<AF5>>
        ]
        HCS: [
            PA4<Alternate<AF5>>,
            PA15<Alternate<AF5>>,
            PG10<Alternate<AF5>>
        ]
    SPI2:
        SCK: [
            NoSck,
            PA9<Alternate<AF5>>,
            PA12<Alternate<AF5>>,
            PB10<Alternate<AF5>>,
            PB13<Alternate<AF5>>,
            PD3<Alternate<AF5>>,
            #[cfg(not(feature = "rm0468"))]
            PI1<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PB14<Alternate<AF5>>,
            PC2<Alternate<AF5>>,
            #[cfg(not(feature = "rm0468"))]
            PI2<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PB15<Alternate<AF5>>,
            PC1<Alternate<AF5>>,
            PC3<Alternate<AF5>>,
            #[cfg(not(feature = "rm0468"))]
            PI3<Alternate<AF5>>
        ]
        HCS: [
            PA11<Alternate<AF5>>,
            PB4<Alternate<AF7>>,
            PB9<Alternate<AF5>>,
            PB12<Alternate<AF5>>,
            #[cfg(not(feature = "rm0468"))]
            PI0<Alternate<AF5>>
        ]
    SPI3:
        SCK: [
            NoSck,
            PB3<Alternate<AF6>>,
            PC10<Alternate<AF6>>
        ]
        MISO: [
            NoMiso,
            PB4<Alternate<AF6>>,
            PC11<Alternate<AF6>>
        ]
        MOSI: [
            NoMosi,
            PB2<Alternate<AF7>>,
            PB5<Alternate<AF7>>,
            PC12<Alternate<AF6>>,
            PD6<Alternate<AF5>>
        ]
        HCS: [
            PA4<Alternate<AF6>>,
            PA15<Alternate<AF6>>
        ]
    SPI4:
        SCK: [
            NoSck,
            PE2<Alternate<AF5>>,
            PE12<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PE5<Alternate<AF5>>,
            PE13<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PE6<Alternate<AF5>>,
            PE14<Alternate<AF5>>
        ]
        HCS: [
            PE4<Alternate<AF5>>,
            PE11<Alternate<AF5>>
        ]
    SPI5:
        SCK: [
            NoSck,
            PF7<Alternate<AF5>>,
            PH6<Alternate<AF5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PK0<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PF8<Alternate<AF5>>,
            PH7<Alternate<AF5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PJ11<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PF9<Alternate<AF5>>,
            PF11<Alternate<AF5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PJ10<Alternate<AF5>>
        ]
        HCS: [
            PF6<Alternate<AF5>>,
            PH5<Alternate<AF5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PK1<Alternate<AF5>>
        ]
    SPI6:
        SCK: [
            NoSck,
            PA5<Alternate<AF8>>,
            PB3<Alternate<AF8>>,
            #[cfg(feature = "rm0455")]
            PC12<Alternate<AF5>>,
            PG13<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PA6<Alternate<AF8>>,
            PB4<Alternate<AF8>>,
            PG12<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PA7<Alternate<AF8>>,
            PB5<Alternate<AF8>>,
            PG14<Alternate<AF5>>
        ]
        HCS: [
            PA4<Alternate<AF8>>,
            PA15<Alternate<AF7>>,
            PG8<Alternate<AF5>>
        ]
}

/// Interrupt events
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// New data has been received
    Rxp,
    /// Data can be sent
    Txp,
    /// An error occurred
    Error,
}

#[derive(Debug)]
pub struct Spi<SPI, ED, WORD = u8> {
    spi: SPI,
    hardware_cs_mode: HardwareCSMode,
    _word: PhantomData<WORD>,
    _ed: PhantomData<ED>,
}

pub trait SpiExt<SPI, WORD>: Sized {
    type Rec: ResetEnable;

    fn spi<PINS, T, CONFIG>(
        self,
        _pins: PINS,
        config: CONFIG,
        freq: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, Enabled, WORD>
    where
        PINS: Pins<SPI>,
        T: Into<Hertz>,
        CONFIG: Into<Config>;

    fn spi_unchecked<T, CONFIG>(
        self,
        config: CONFIG,
        freq: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, Enabled, WORD>
    where
        T: Into<Hertz>,
        CONFIG: Into<Config>;
}

macro_rules! spi {
    (DSIZE, $spi:ident,  u8) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(8 - 1) // 8 bit words
        });
    };
    (DSIZE, $spi:ident, u16) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(16 - 1) // 16 bit words
        });
    };
    (DSIZE, $spi:ident, u32) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(32 - 1) // 32 bit words
        });
    };
	($($SPIX:ident: ($spiX:ident, $Rec:ident, $pclkX:ident)
       => ($($TY:ident),+),)+) => {
	    $(
            // For each $TY
            $(
                impl Spi<$SPIX, Enabled, $TY> {
                    pub fn $spiX<T, CONFIG>(
                        spi: $SPIX,
                        config: CONFIG,
                        freq: T,
                        prec: rec::$Rec,
                        clocks: &CoreClocks,
                    ) -> Self
                    where
                        T: Into<Hertz>,
                        CONFIG: Into<Config>,
                    {
                        // Enable clock for SPI
                        prec.enable();

                        // Disable SS output
                        spi.cfg2.write(|w| w.ssoe().disabled());

                        let config: Config = config.into();

                        let spi_freq = freq.into().0;
	                    let spi_ker_ck = match Self::kernel_clk(clocks) {
                            Some(ker_hz) => ker_hz.0,
                            _ => panic!("$SPIX kernel clock not running!")
                        };
                        let mbr = match spi_ker_ck / spi_freq {
                            0 => unreachable!(),
                            1..=2 => MBR::DIV2,
                            3..=5 => MBR::DIV4,
                            6..=11 => MBR::DIV8,
                            12..=23 => MBR::DIV16,
                            24..=47 => MBR::DIV32,
                            48..=95 => MBR::DIV64,
                            96..=191 => MBR::DIV128,
                            _ => MBR::DIV256,
                        };
                        spi.cfg1.modify(|_, w| {
                            w.mbr()
                                .variant(mbr) // master baud rate
                        });
                        spi!(DSIZE, spi, $TY); // modify CFG1 for DSIZE

                        // ssi: select slave = master mode
                        spi.cr1.write(|w| w.ssi().slave_not_selected());

                        // Calculate the CS->transaction cycle delay bits.
                        let (assertion_delay, inter_word_delay) = {
                            let mut assertion_delay: u32 = (config.hardware_cs.assertion_delay() * spi_freq as f32) as u32;
                            let mut inter_word_delay: u32 = (config.inter_word_delay * spi_freq as f32) as u32;

                            // If a delay is specified as non-zero, add 1 to the delay cycles
                            // before truncation to an integer to ensure that we have at least as
                            // many cycles as required.
                            if config.hardware_cs.assertion_delay() > 0.0_f32 {
                                assertion_delay += 1;
                            }
                            if config.inter_word_delay > 0.0_f32 {
                                inter_word_delay += 1;
                            }

                            // If CS suspends while data is inactive, we also require an
                            // "inter-data" delay.
                            if matches!(config.hardware_cs.mode, HardwareCSMode::WordTransaction) {
                                inter_word_delay = inter_word_delay.max(1);
                            }

                            (assertion_delay.min(0xF) as u8, inter_word_delay.min(0xF) as u8)
                        };

                        // The calculated cycle delay may not be more than 4 bits wide for the
                        // configuration register.
                        let communication_mode = match config.communication_mode {
                            CommunicationMode::Transmitter => COMM::TRANSMITTER,
                            CommunicationMode::Receiver => COMM::RECEIVER,
                            CommunicationMode::FullDuplex => COMM::FULLDUPLEX,
                        };

                        let cs_polarity = match config.hardware_cs.polarity() {
                            Polarity::IdleHigh => SSIOP::ACTIVELOW,
                            Polarity::IdleLow => SSIOP::ACTIVEHIGH,
                        };

                        // mstr: master configuration
                        // lsbfrst: MSB first
                        // comm: full-duplex
                        spi.cfg2.write(|w| {
                            w.cpha()
                                .bit(config.mode.phase ==
                                     Phase::CaptureOnSecondTransition)
                                .cpol()
                                .bit(config.mode.polarity == Polarity::IdleHigh)
                                .master()
                                .master()
                                .lsbfrst()
                                .msbfirst()
                                .ssom()
                                .bit(config.hardware_cs.interleaved_cs())
                                .ssm()
                                .bit(config.hardware_cs.enabled() == false)
                                .ssoe()
                                .bit(config.hardware_cs.enabled() == true)
                                .mssi()
                                .bits(assertion_delay)
                                .midi()
                                .bits(inter_word_delay)
                                .ioswp()
                                .bit(config.swap_miso_mosi == true)
                                .comm()
                                .variant(communication_mode)
                                .ssiop()
                                .variant(cs_polarity)
                        });

                        // Reset to default (might have been set if previously used by a frame transaction)
                        // So that is 1 when it's a frame transaction and 0 when in another mode
                        spi.cr2.write(|w| w.tsize().bits(matches!(config.hardware_cs.mode, HardwareCSMode::FrameTransaction) as u16));

                        // spe: enable the SPI bus
                        spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());

                        Spi { spi, hardware_cs_mode: config.hardware_cs.mode, _word: PhantomData, _ed: PhantomData }
                    }

                    /// Disables the SPI peripheral. Any SPI operation is
                    /// stopped and disabled, the internal state machine is
                    /// reset, all the FIFOs content is flushed, the MODF
                    /// flag is cleared, the SSI flag is cleared, and the
                    /// CRC calculation is re-initialized. Clocks are not
                    /// disabled.
                    pub fn disable(self) -> Spi<$SPIX, Disabled, $TY> {
                        // Master communication must be suspended before the peripheral is disabled
                        self.spi.cr1.modify(|_, w| w.csusp().requested());
                        while self.spi.sr.read().eot().is_completed() {}
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().disabled());
                        Spi {
                            spi: self.spi,
                            hardware_cs_mode: self.hardware_cs_mode,
                            _word: PhantomData,
                            _ed: PhantomData,
                        }
                    }

                    /// Sets up a frame transaction with the given amount of data words.
                    ///
                    /// If this is called when the hardware CS mode is not [HardwareCSMode::FrameTransaction],
                    /// then an error is returned with [Error::InvalidCall].
                    ///
                    /// If this is called when a transaction has already started,
                    /// then an error is returned with [Error::TransactionAlreadyStarted].
                    pub fn setup_transaction(&mut self, words: core::num::NonZeroU16) -> Result<(), Error> {
                        if !matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            return Err(Error::InvalidCall);
                        }

                        if self.spi.cr1.read().cstart().is_started() {
                            return Err(Error::TransactionAlreadyStarted);
                        }

                        // We can only set tsize when spi is disabled
                        self.spi.cr1.modify(|_, w| w.csusp().requested());
                        while self.spi.sr.read().eot().is_completed() {}
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().disabled());

                        // Set the frame size
                        self.spi.cr2.write(|w| w.tsize().bits(words.get()));

                        // Re-enable
                        self.clear_modf(); // SPE cannot be set when MODF is set
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());

                        Ok(())
                    }

                    /// Ends the current transaction, both for endless and frame transactions.
                    ///
                    /// This method must always be called for frame transaction,
                    /// even if the full size has been sent. If this is not done,
                    /// no new data can be sent even when it looks like it should.
                    ///
                    /// If it's not either a frame or endless transaction,
                    /// an error is returned with [Error::InvalidCall].
                    pub fn end_transaction(&mut self) -> Result<(), Error> {
                        if !matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction | HardwareCSMode::EndlessTransaction) {
                            return Err(Error::InvalidCall);
                        }

                        self.spi.cr1.modify(|_, w| w.csusp().requested());
                        while(self.spi.cr1.read().cstart().is_started()) {}

                        self.spi.ifcr.write(|w| w.txtfc().clear().eotc().clear());

                        Ok(())
                    }
                }

                impl Spi<$SPIX, Disabled, $TY> {
                    /// Enables the SPI peripheral.
                    /// Clears the MODF flag, the SSI flag, and sets the SPE bit.
                    pub fn enable(mut self) -> Spi<$SPIX, Enabled, $TY> {
                        self.clear_modf(); // SPE cannot be set when MODF is set
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());
                        Spi {
                            spi: self.spi,
                            hardware_cs_mode: self.hardware_cs_mode,
                            _word: PhantomData,
                            _ed: PhantomData,
                        }
                    }

                    /// Enables the Rx DMA stream. If the DMA Rx is used, the
                    /// reference manual recommends that this is enabled before
                    /// enabling the DMA
                    pub fn enable_dma_rx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.rxdmaen().enabled());
                    }

                    pub fn disable_dma_rx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.rxdmaen().disabled());
                    }

                    /// Enables the Tx DMA stream. If the DMA Tx is used, the
                    /// reference manual recommends that this is enabled after
                    /// enabling the DMA
                    pub fn enable_dma_tx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.txdmaen().enabled());
                    }

                    pub fn disable_dma_tx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.txdmaen().disabled());
                    }

                    /// Deconstructs the SPI peripheral and returns the component parts.
                    pub fn free(self) -> ($SPIX, rec::$Rec) {
                        (self.spi, rec::$Rec { _marker: PhantomData })
                    }
                }

                impl<EN> Spi<$SPIX, EN, $TY>
                {
                    /// Returns a mutable reference to the inner peripheral
                    pub fn inner(&self) -> &$SPIX {
                        &self.spi
                    }

                    /// Returns a mutable reference to the inner peripheral
                    pub fn inner_mut(&mut self) -> &mut $SPIX {
                        &mut self.spi
                    }

                    /// Enable interrupts for the given `event`:
                    ///  - Received data ready to be read (RXP)
                    ///  - Transmit data register empty (TXP)
                    ///  - Error
                    pub fn listen(&mut self, event: Event) {
                        match event {
                            Event::Rxp => self.spi.ier.modify(|_, w|
                                                              w.rxpie().not_masked()),
                            Event::Txp => self.spi.ier.modify(|_, w|
                                                              w.txpie().not_masked()),
                            Event::Error => self.spi.ier.modify(|_, w| {
                                w.udrie() // Underrun
                                    .not_masked()
                                    .ovrie() // Overrun
                                    .not_masked()
                                    .crceie() // CRC error
                                    .not_masked()
                                    .modfie() // Mode fault
                                    .not_masked()
                            }),
                        }
                    }

                    /// Disable interrupts for the given `event`:
                    ///  - Received data ready to be read (RXP)
                    ///  - Transmit data register empty (TXP)
                    ///  - Error
                    pub fn unlisten(&mut self, event: Event) {
                        match event {
                            Event::Rxp => {
                                self.spi.ier.modify(|_, w| w.rxpie().masked());
                            }
                            Event::Txp => {
                                self.spi.ier.modify(|_, w| w.txpie().masked());
                            }
                            Event::Error => {
                                self.spi.ier.modify(|_, w| {
                                    w.udrie() // Underrun
                                        .masked()
                                        .ovrie() // Overrun
                                        .masked()
                                        .crceie() // CRC error
                                        .masked()
                                        .modfie() // Mode fault
                                        .masked()
                                })
                            }
                        }
                        let _ = self.spi.ier.read();
                        let _ = self.spi.ier.read(); // Delay 2 peripheral clocks
                    }

                    /// Return `true` if the TXP flag is set, i.e. new
                    /// data to transmit can be written to the SPI.
                    pub fn is_txp(&self) -> bool {
                        self.spi.sr.read().txp().is_not_full()
                    }

                    /// Return `true` if the RXP flag is set, i.e. new
                    /// data has been received and can be read from the
                    /// SPI.
                    pub fn is_rxp(&self) -> bool {
                        self.spi.sr.read().rxp().is_not_empty()
                    }

                    /// Return `true` if the MODF flag is set, i.e. the
                    /// SPI has experienced a mode fault
                    pub fn is_modf(&self) -> bool {
                        self.spi.sr.read().modf().is_fault()
                    }

                    /// Return `true` if the OVR flag is set, i.e. new
                    /// data has been received while the receive data
                    /// register was already filled.
                    pub fn is_ovr(&self) -> bool {
                        self.spi.sr.read().ovr().is_overrun()
                    }

                    /// Clears the MODF flag, which indicates that a
                    /// mode fault has occurred.
                    pub fn clear_modf(&mut self) {
                        self.spi.ifcr.write(|w| w.modfc().clear());
                        let _ = self.spi.sr.read();
                        let _ = self.spi.sr.read(); // Delay 2 peripheral clocks
                    }
                }

                impl SpiExt<$SPIX, $TY> for $SPIX {
                    type Rec = rec::$Rec;

	                fn spi<PINS, T, CONFIG>(self,
                                    _pins: PINS,
                                    config: CONFIG,
                                    freq: T,
                                    prec: rec::$Rec,
                                    clocks: &CoreClocks) -> Spi<$SPIX, Enabled, $TY>
	                where
	                    PINS: Pins<$SPIX>,
	                    T: Into<Hertz>,
                        CONFIG: Into<Config>,
	                {
                        let config = config.into();
                        assert_eq!(
                            config.hardware_cs.enabled(),
                            PINS::HCS_PRESENT,
                            "If the hardware cs is enabled in the config, an HCS pin must be present in the given pins"
                        );
	                    Spi::<$SPIX, Enabled, $TY>::$spiX(self, config, freq, prec, clocks)
	                }

	                fn spi_unchecked<T, CONFIG>(self,
                                        config: CONFIG,
                                        freq: T,
                                        prec: rec::$Rec,
                                        clocks: &CoreClocks) -> Spi<$SPIX, Enabled, $TY>
	                where
	                    T: Into<Hertz>,
                        CONFIG: Into<Config>,
	                {
	                    Spi::<$SPIX, Enabled, $TY>::$spiX(self, config, freq, prec, clocks)
	                }
	            }

                impl hal::spi::FullDuplex<$TY> for Spi<$SPIX, Enabled, $TY> {
                    type Error = Error;

                    fn read(&mut self) -> nb::Result<$TY, Error> {
                        let sr = self.spi.sr.read();

                        Err(if sr.ovr().is_overrun() {
                            nb::Error::Other(Error::Overrun)
                        } else if sr.modf().is_fault() {
                            nb::Error::Other(Error::ModeFault)
                        } else if sr.crce().is_error() {
                            nb::Error::Other(Error::Crc)
                        } else if sr.rxp().is_not_empty() {
                            // NOTE(read_volatile) read only 1 byte (the
                            // svd2rust API only allows reading a
                            // half-word)
                            return Ok(unsafe {
                                ptr::read_volatile(
                                    &self.spi.rxdr as *const _ as *const $TY,
                                )
                            });
                        } else {
                            nb::Error::WouldBlock
                        })
                    }

                    fn send(&mut self, byte: $TY) -> nb::Result<(), Error> {
                        let sr = self.spi.sr.read();

                        Err(if sr.ovr().is_overrun() {
                            nb::Error::Other(Error::Overrun)
                        } else if sr.modf().is_fault() {
                            nb::Error::Other(Error::ModeFault)
                        } else if sr.crce().is_error() {
                            nb::Error::Other(Error::Crc)
                        } else if sr.txp().is_not_full() {
                            // NOTE(write_volatile) see note above
                            unsafe {
                                ptr::write_volatile(
                                    &self.spi.txdr as *const _ as *mut $TY,
                                    byte,
                                )
                            }
                            // write CSTART to start a transaction in
                            // master mode
                            self.spi.cr1.modify(|_, w| w.cstart().started());

                            return Ok(());
                        } else {
                            nb::Error::WouldBlock
                        })
                    }
                }

                impl hal::blocking::spi::Transfer<$TY> for Spi<$SPIX, Enabled, $TY> {
                    type Error = Error;

                    fn transfer<'w>(&mut self, words: &'w mut [$TY]) -> Result<&'w [$TY], Self::Error> {
                        use embedded_hal::spi::FullDuplex;

                        if words.is_empty() {
                            return Ok(words);
                        }

                        // Are we in frame mode?
                        if matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            const MAX_WORDS: usize = 0xFFFF;

                            // Can we send
                            if words.len() > MAX_WORDS {
                                return Err(Error::BufferTooBig { max_size: MAX_WORDS });
                            }

                            // Setup that we're going to send this amount of bits
                            // SAFETY: We already checked that `words` is not empty
                            self.setup_transaction(unsafe{ core::num::NonZeroU16::new_unchecked(words.len() as u16) })?;
                        }

                        // Send the data
                        for word in words.iter_mut() {
                            nb::block!(self.send(word.clone()))?;
                            *word = nb::block!(self.read())?;
                        }

                        // Are we in frame mode?
                        if matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            // Clean up
                            self.end_transaction()?;
                        }

                        Ok(words)
                    }
                }

                impl hal::blocking::spi::Write<$TY> for Spi<$SPIX, Enabled, $TY> {
                    type Error = Error;

                    fn write(&mut self, words: &[$TY]) -> Result<(), Self::Error> {
                        use embedded_hal::spi::FullDuplex;

                        if words.is_empty() {
                            return Ok(());
                        }

                        // Are we in frame mode?
                        if matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            const MAX_WORDS: usize = 0xFFFF;

                            // Can we send
                            if words.len() > MAX_WORDS {
                                return Err(Error::BufferTooBig { max_size: MAX_WORDS });
                            }

                            // Setup that we're going to send this amount of bits
                            // SAFETY: We already checked that `words` is not empty
                            self.setup_transaction(unsafe{ core::num::NonZeroU16::new_unchecked(words.len() as u16) })?;
                        }

                        // Send the data
                        for word in words {
                            nb::block!(self.send(word.clone()))?;
                            nb::block!(self.read())?;
                        }

                        // Are we in frame mode?
                        if matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            // Clean up
                            self.end_transaction()?;
                        }

                        Ok(())
                    }
                }
            )+
        )+
	}
}

macro_rules! spi123sel {
	($($SPIX:ident,)+) => {
	    $(
            impl<WORD> Spi<$SPIX, Enabled, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI1, SPI2, SPI3
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi123sel().variant() {
                        Some(ccip1r::SPI123SEL_A::PLL1_Q) => clocks.pll1_q_ck(),
                        Some(ccip1r::SPI123SEL_A::PLL2_P) => clocks.pll2_p_ck(),
                        Some(ccip1r::SPI123SEL_A::PLL3_P) => clocks.pll3_p_ck(),
                        // Need a method of specifying pin clock
                        Some(ccip1r::SPI123SEL_A::I2S_CKIN) => unimplemented!(),
                        Some(ccip1r::SPI123SEL_A::PER) => clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
macro_rules! spi45sel {
	($($SPIX:ident,)+) => {
	    $(
            impl<WORD> Spi<$SPIX, Enabled, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI4, SPI5
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi45sel().variant() {
                        Some(ccip1r::SPI45SEL_A::APB) => Some(clocks.pclk2()),
                        Some(ccip1r::SPI45SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Some(ccip1r::SPI45SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Some(ccip1r::SPI45SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Some(ccip1r::SPI45SEL_A::CSI_KER) => clocks.csi_ck(),
                        Some(ccip1r::SPI45SEL_A::HSE) => clocks.hse_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
macro_rules! spi6sel {
	($($SPIX:ident,)+) => {
	    $(
            impl<WORD> Spi<$SPIX, Enabled, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI6
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).d3ccipr.read() };
                    #[cfg(feature = "rm0455")]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).srdccipr.read() };

                    match srdccipr.spi6sel().variant() {
                        Some(srdccipr::SPI6SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
                        Some(srdccipr::SPI6SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Some(srdccipr::SPI6SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Some(srdccipr::SPI6SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Some(srdccipr::SPI6SEL_A::CSI_KER) => clocks.csi_ck(),
                        Some(srdccipr::SPI6SEL_A::HSE) => clocks.hse_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}

spi! {
    SPI1: (spi1, Spi1, pclk2) => (u8, u16, u32),
    SPI2: (spi2, Spi2, pclk1) => (u8, u16, u32),
    SPI3: (spi3, Spi3, pclk1) => (u8, u16, u32),
    SPI4: (spi4, Spi4, pclk2) => (u8, u16, u32),
    SPI5: (spi5, Spi5, pclk2) => (u8, u16, u32),
    SPI6: (spi6, Spi6, pclk2) => (u8, u16, u32),
}

spi123sel! {
    SPI1, SPI2, SPI3,
}
spi45sel! {
    SPI4, SPI5,
}
spi6sel! {
    SPI6,
}
