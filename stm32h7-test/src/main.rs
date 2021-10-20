#![no_main]
#![no_std]

mod hardware_setup;

use core::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal::ethernet;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    // TODO: Change to correct time when RTIC 0.6
    let n = COUNT.load(AtomicOrdering::Relaxed);
    COUNT.store(n + 1, AtomicOrdering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [USART1, USART2])]
mod app {
    use rtic::rtic_monotonic::Seconds;
    use stm32h7xx_hal::ethernet;
    use stm32h7xx_hal::hal::digital::v2::OutputPin;
    use systick_monotonic::Systick;

    use crate::hardware_setup::{self, LinkLed, UserLed};

    #[local]
    struct Local {
        led: UserLed,
        link_led: LinkLed,
    }

    #[shared]
    struct Shared {
        network: hardware_setup::NetworkDevices,
    }

    #[monotonic(binds = SysTick, default = true)]
    type DwtMono = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("pre init");

        let (network, mono, link_led, led) = hardware_setup::setup(cx.core, cx.device);

        defmt::info!("init");

        foo::spawn().ok();
        ethernet_link::spawn().ok();

        (
            Shared { network },
            Local { led, link_led },
            init::Monotonics(mono),
        )
    }

    #[idle(shared = [network])]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        let mut network = cx.shared.network;

        loop {
            let time = monotonics::now().duration_since_epoch().integer();
            match network.lock(|l| l.stack.poll(time as i64)) {
                Ok(true) => {
                    // link_led.set_low().ok();
                }
                Ok(false) => {
                    // link_led.set_high().ok();
                }
                Err(e) => {
                    defmt::warn!("Poll error: {}", e);
                }
            };
        }
    }

    #[task(binds = ETH, shared = [network])]
    fn ethernet_event(_cx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }
    }

    #[task(shared = [network], local = [link_led])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        let l = c.shared.network.lock(|net| net.phy.poll_link());
        let link_led = c.local.link_led;

        match l {
            true => {
                link_led.set_low().ok();
            }
            false => {
                link_led.set_high().ok();
            }
        }

        ethernet_link::spawn_after(Seconds(1u32)).ok();
    }

    #[task(local = [led, state: bool = false])]
    fn foo(cx: foo::Context) {
        let led = cx.local.led;
        let state = cx.local.state;

        defmt::info!("blink");
        if *state {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }

        *state = !*state;
        foo::spawn_after(Seconds(1u32)).ok();
    }
}
