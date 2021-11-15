#![no_main]
#![no_std]

use ethernet_test as _;

defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    // TODO: Change to correct time when RTIC 0.6
    let n = app::monotonics::now().duration_since_epoch().ticks() as usize;
    // COUNT.store(n + 1, AtomicOrdering::Relaxed);
    n
});

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [USART1, USART2])]
mod app {
    use ethernet_test::setup::{self, LinkLed, UserLed};
    use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;
    use systick_monotonic::*;

    #[local]
    struct Local {
        led: UserLed,
        link_led: LinkLed,
    }

    #[shared]
    struct Shared {}

    #[monotonic(binds = SysTick, default = true)]
    type DwtMono = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("pre init");

        let (mono, link_led, led) = setup::setup_leds_only(cx.core, cx.device);

        foo::spawn().ok();

        defmt::info!("init");

        (Shared {}, Local { led, link_led }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {}
    }

    #[task(local = [led, link_led])]
    fn foo(cx: foo::Context) {
        cx.local.led.toggle().ok();
        cx.local.link_led.toggle().ok();

        foo::spawn_after(1.secs()).ok();
    }
}
