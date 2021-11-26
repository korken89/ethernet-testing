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
    use smoltcp_nal::embedded_nal::{self, Ipv4Addr, SocketAddr, SocketAddrV4};
    use smoltcp_nal::smoltcp::wire::IpAddress;
    use smoltcp_nal::{embedded_nal::TcpClientStack, smoltcp::socket::SocketHandle};
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
        tcp_socket: Option<SocketHandle>,
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
        start_query::spawn().ok();

        defmt::info!("init");

        (
            Shared {
                network,
                tcp_socket: None,
            },
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
                    defmt::info!("Link up - resetting stack");
                    c.shared.network.lock(|net| net.stack.handle_link_reset());
                }
            }
            false => {
                if *link_was_up {
                    defmt::info!("Link down");
                }
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

    #[task(shared = [network])]
    fn start_query(mut cx: start_query::Context) {
        defmt::info!("Starting DNS query for 'server'...");

        // let name = b"\x09rust-lang\x03org\x00";
        // let name = b"\x06server\x00";
        let name = b"\x0eemifre-archpad\x00";

        let res = cx
            .shared
            .network
            .lock(|network| network.stack.start_dns_query(name));

        match res {
            Ok(()) => {
                get_dns_query::spawn().ok();
            }
            Err(e) => {
                defmt::warn!("Request IP from DNS failed: {}", e);
                start_query::spawn_after(1.secs()).ok();
            }
        }
    }

    #[task(shared = [network])]
    fn get_dns_query(mut cx: get_dns_query::Context) {
        cx.shared.network.lock(|network| {
            match network.stack.get_dns_result() {
                Ok(val) => {
                    defmt::info!("Got DNS result: {}", val);
                    open_connection_to_server::spawn(val[0]).ok();
                }
                Err(_) => {
                    // try again
                    defmt::warn!("No DNS result yet...");
                    get_dns_query::spawn_after(500.millis()).ok();
                }
            }
        });
    }

    #[task(shared = [network, tcp_socket])]
    fn open_connection_to_server(cx: open_connection_to_server::Context, ip: IpAddress) {
        let mut network = cx.shared.network;
        let mut tcp_socket = cx.shared.tcp_socket;

        let new_ip = match ip {
            IpAddress::Ipv4(addr) => addr,
            _ => panic!("Not supported"),
        };

        network.lock(|network| {
            if let Ok(mut new_tcp_socket) = network.stack.socket() {
                let ip = SocketAddr::V4(SocketAddrV4::new(
                    Ipv4Addr::new(new_ip.0[0], new_ip.0[1], new_ip.0[2], new_ip.0[3]),
                    7878,
                ));

                defmt::info!("Connecting to: {}", new_ip);

                if let Err(embedded_nal::nb::Error::Other(e)) =
                    network.stack.connect(&mut new_tcp_socket, ip)
                {
                    defmt::error!("Connection failed!: {}", e);
                }
                tcp_socket.lock(|socket| *socket = Some(new_tcp_socket));
                test_connection::spawn().ok();
            } else {
                open_connection_to_server::spawn_after(500.millis(), ip).ok();
            }
        });
    }

    #[task(shared = [network, tcp_socket])]
    fn test_connection(cx: test_connection::Context) {
        let network = cx.shared.network;
        let tcp_socket = cx.shared.tcp_socket;

        (network, tcp_socket).lock(|network, tcp_socket| {
            let tcp_socket = tcp_socket.as_mut().unwrap();

            if let Ok(true) = network.stack.is_connected(tcp_socket) {
                defmt::info!("Connected! Sending data...");

                if let Err(embedded_nal::nb::Error::Other(e)) =
                    network.stack.send(tcp_socket, &[1, 2, 3, 4, 5, 6, 7, 8, 9])
                {
                    defmt::error!("Send failed!: {}", e);
                }
            } else {
                defmt::warn!("Not connected...");
            }
        });

        test_connection::spawn_after(1.secs()).ok();
    }
}
