# Changelog

## [Unreleased]
* Fixed clippy lints:
   * Added safety docs for some DMA functions
   * Implemented additional conversion utilities for `time`
   * **Breaking**: Changed I2S constructors to take less arguments

* MSRV increased to 1.51.0
* **Breaking**: Simplified API for reading device signature
  values. `VAL::get().read()` becomes `VAL::read()`
* adc: Allow parallel execution of multiple ADCs through `start_conversion()`
* Rename the PeripheralREC object for BDMA2 on 7B3, 7B0, 7A3 parts from BDMA to BDMA2
* pac: Upgrade to stm32-rs v0.14.0

## [v0.10.0] 2021-07-xx

* **Breaking**: Don't reset peripheral in DMA1/2 `StreamsTuple::new()` method #229
* adc: Add `free()` method for ADC12 #213
* dma: Fast double-buffered DMA `next_dbm_transfer_with()` method #226
* ethernet: Fix feature set for smoltcp dependency #221
* pwm: Add advanced PWM functions #175
* rtc: Set LSEBYP before enabling LSE #211
* serial: implement `fmt::Write` for `Serial<USART` #214
* spi: Add more hardware CS features #216
* timers: Better calculations for `set_timeout_ticks` #208

## [v0.9.0] 2021-03-12

* Updates `cortex-m` to v0.7.1. `cortex-m` v0.6.5+ [are forward compatible with
  v0.7.0+][cm6-changelog] except for CBP, ITM, MPU, NVIC, SCB. If you have
  problems, run `cargo update` to try to switch to `cortex-m` v0.7 in other
  dependencies. Something like `cargo tree` is very useful to track down
  remaining uses of `cortex-m` v0.6.
* **Breaking**: Add new/new_unchecked methods for USB structures, remove pin
  types from structure
* Add support for USB1_ULPI #184
* Add support for LTDC #81
* Add support for CRC #199
* Add support for DMA1, DMA2, BDMA #153
* pac: Upgrade to stm32-rs v0.13.0
* spi: Update CS management #159
* spi: Support 32-bit frames #200
* qspi: Sample on falling edges #161
* qspi: Add configuration support #176
* timers: Add `DelayFromCountDownTimer` #170
* ethernet: update smoltcp v0.6 -> v0.7
* usb: add support for external ULPI PHYs #184
* rtc: fix unlisten method
* serial: add clear_idle and is_busy methods #201

[cm6-changelog]: (https://github.com/rust-embedded/cortex-m/blob/master/CHANGELOG.md#v065---2021-01-24)

## [v0.8.0] 2020-10-26

* **Breaking**: Ethernet PHY configuration feature flags removed. The user must
  now instantiate an instance of the PHY type in order to configure the PHY.
* pac: Upgrade to stm32-rs v0.12.0
* devices: Add support for 7B3, 7B0, 7A3
* Add USB support
* Add I2S support
* Add RTC support
* Add LPTIM support
* Add DMA support, but the current API is depreciated and will be replaced
* timer: add tick_timer and set_tick_freq to configure the timer's counter frequency [#144](https://github.com/stm32-rs/stm32h7xx-hal/pull/144)
* Add RTC support [#143](https://github.com/stm32-rs/stm32h7xx-hal/pull/143)
* pwr: add PowerConfiguration to ensure VoltageScale isn't modified from pwr.freeze() to rcc.freeze() [#141](https://github.com/stm32-rs/stm32h7xx-hal/pull/141)
* impl Copy, Clone, PartialEq for enums [#139](https://github.com/stm32-rs/stm32h7xx-hal/pull/139)
* ethernet: automatically configure MDC clock based on hclk
* time: add types for microseconds and nanoseconds
* rec: add `low_power` methods for configuring peripherals

## [v0.7.1] 2020-09-04

* Update docs

## [v0.7.0] 2020-09-03

* **Breaking**: Fix PWM pin types. See #110
* Add QSPI support
* Add SDMMC support
* Add Ethernet support
* Add FMC support
* spi: add new configuration options
* i2c: fix i2c bus clock frequency calculation
* i2c: extend write duration until transaction finishes
* timer: Fix timer first cycle invalid #72
* timer: Add set_timeout method
* adc: add method to initialise ADC1 and ADC2 together, see examples
* Added method to switch to VOS0 (480MHz)
* Allow external HSE clock input (bypass mode)
* Use ACLK (AXI clock) frequency for calculating flash waitstates
* pll: Add fractional strategies
* pll: fix very subtle error in PLL Q,R frequencies
* serial: rename usart method to serial, will be depreciated in future
* examples: Added logging framework
* MSRV increased to 1.43.0

## [v0.6.0] 2020-06-25

* **Breaking:** Peripheral driver constructors now consume a peripheralREC
  singleton. Previously they inconsistently took `&mut` or `&` references
  to the CCDR struct itself. See the examples for the new constructors
* **Breaking:** Erase pins from peripheral driver types. `free()` method return
  types may have changed.
* Expose `new_unchecked` methods for instantiating peripheral drivers without
  providing pins
* Improved top-level docs
* Add DAC peripheral driver
* Added MCO pins
* Add `.free()` method for ADC
* MSRV increased to 1.41.0
* rtfm crate was renamed to rtic
* Added alternate flash size definitions to memory.x
* Bump dependency versions: cortex-m, cortex-m-rt, cast, paste, bare-metal

## [v0.5.0] 2020-04-27

* pac: Upgrade to stm32-rs v0.11.0
* pwr: Remove need for unsafe, add documentation describing Run* mode
* rcc: Add a safe interface for the user and other crates to access the RCC
* spi: 16-bit word support
* i2c: Remove reference to I2C1 in macro generic for all I2Cs
* Add HAL for Serial Audio Interface, PDM mode
* Implement default blocking write for serial

## [v0.4.0] 2020-03-20

* pac: Upgrade to stm32-rs v0.10.0
* rcc: Add iterative PLL configuration strategy
* rcc: Improve documentation
* pwr: configure core supply on parts with SMPS (dual core). Prevents
  lock-up without unsafe for certain hardware configurations
* i2c: wait for end of previous address phase before setting START
* i2c: return immediately when TXIS / RXNE is set
* timers: implement TIM3/4/5/6/7/8/12/13/14/15/16/17
* gpio: fix error in PF7 definition
* add MSRV 1.39.0

## [v0.3.0] 2019-12-27

* timer: add method to clear interrupt flag
* gpio: initialise gpio in the correct typestate
* i2c: flush the TXDR register when a NACK condition occurs
* Fix `clear_interrupt_pending_bit()`
* i2c: detect not-acknowledge error, clear errors when detected
* Run CI against stable since Rust 1.37.0
* Upgrade to stm32-rs v0.9.0 (including svd2rust v0.16)
* Started Changelog

[Unreleased]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.10.0...HEAD
[v0.10.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.9.0...v0.10.0
[v0.9.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.8.0...v0.9.0
[v0.8.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.7.1...v0.8.0
[v0.7.1]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.7.0...v0.7.1
[v0.7.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.6.0...v0.7.0
[v0.6.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.5.0...v0.6.0
[v0.5.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.2.1...v0.3.0
