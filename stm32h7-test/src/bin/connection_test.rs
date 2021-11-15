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
    use ethernet_test::setup::{self, LinkLed, NetworkStorage, UserLed};
    use stm32h7xx_hal::ethernet;
    use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use systick_monotonic::*;

    #[local]
    struct Local {
        led: UserLed,
        link_led: LinkLed,
    }

    #[shared]
    struct Shared {
        network: setup::NetworkDevices,
    }

    #[monotonic(binds = SysTick, default = true)]
    type DwtMono = Systick<1000>;

    #[init(local = [network_storage: NetworkStorage = NetworkStorage::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("pre init");

        let network_storage = cx.local.network_storage;

        let (network, mono, link_led, led) =
            setup::setup_with_ethernet(cx.core, cx.device, network_storage);

        foo::spawn().ok();
        ethernet_link::spawn().ok();

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
        let time = monotonics::now().duration_since_epoch().ticks();
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

        ethernet_link::spawn_after(200.millis()).ok();
    }

    #[task(local = [led])]
    fn foo(cx: foo::Context) {
        cx.local.led.toggle().ok();

        foo::spawn_after(1.secs()).ok();
    }
}
