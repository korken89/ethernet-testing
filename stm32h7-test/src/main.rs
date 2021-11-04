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
    // COUNT.store(n + 1, AtomicOrdering::Relaxed);
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
    use crate::hardware_setup::{self, LinkLed, NetworkStorage, UserLed};
    use core::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};
    use rtic::rtic_monotonic::{Milliseconds, Seconds};
    use stm32h7xx_hal::ethernet;
    use stm32h7xx_hal::hal::digital::v2::OutputPin;
    use systick_monotonic::Systick;

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

    #[init(local = [network_storage: NetworkStorage = NetworkStorage::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("pre init");

        let network_storage = cx.local.network_storage;

        let (network, mono, link_led, led) =
            hardware_setup::setup(cx.core, cx.device, network_storage);

        foo::spawn().ok();
        ethernet_link::spawn().ok();
        cnt::spawn().ok();

        defmt::info!("init");

        (
            Shared { network },
            Local { led, link_led },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {}
    }

    #[task(capacity = 8, shared = [network])]
    fn ipstack_handler(cx: ipstack_handler::Context) {
        let mut network = cx.shared.network;
        let time = monotonics::now().duration_since_epoch().integer();
        match network.lock(|l| l.stack.poll(time as i64)) {
            Ok(true) => {
                // Something happened
                ipstack_handler::spawn().ok();
            }
            Ok(false) => {
                // Nothing happened
            }
            Err(e) => {
                defmt::warn!("Poll error: {}", e);
            }
        };
    }

    #[task(binds = ETH, priority = 2, shared = [network])]
    fn ethernet_event(_cx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }

        ipstack_handler::spawn().ok();
    }

    // This checks if the link is up from the phy
    #[task(shared = [network], local = [link_was_up: bool = false, link_led])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        let link_is_up = c.shared.network.lock(|net| net.phy.link_established());
        let ethernet_link::LocalResources {
            link_was_up,
            link_led,
        } = c.local;

        match link_is_up {
            true => {
                link_led.set_low().ok();

                if !*link_was_up {
                    c.shared.network.lock(|net| net.stack.handle_link_reset());
                }
            }
            false => {
                link_led.set_high().ok();
            }
        }

        *link_was_up = link_is_up;

        ethernet_link::spawn_after(Milliseconds(200u32)).ok();
    }

    #[task(priority = 8)]
    fn cnt(_: cnt::Context) {
        let n = crate::COUNT.load(AtomicOrdering::Relaxed);
        crate::COUNT.store(n + 1, AtomicOrdering::Relaxed);
        cnt::spawn_after(Milliseconds(1u32)).ok();
    }

    #[task(local = [led, state: bool = false])]
    fn foo(cx: foo::Context) {
        let led = cx.local.led;
        let state = cx.local.state;

        if *state {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }

        *state = !*state;
        foo::spawn_after(Seconds(1u32)).ok();
    }
}