///! Stabilizer hardware configuration
///!
///! This file contains all of the hardware-specific configuration of Stabilizer.
use smoltcp_nal::smoltcp::{self, socket::DnsQuery};
use stm32h7xx_hal::{
    self as hal,
    ethernet::{self, PHY},
    gpio::{gpiob::PB0, gpioe::PE1, Output, PushPull, Speed::VeryHigh},
    // hal::digital::v2::OutputPin,
    prelude::*,
    signature,
};
use systick_monotonic::Systick;

pub type LinkLed = PB0<Output<PushPull>>;
pub type UserLed = PE1<Output<PushPull>>;

const NUM_TCP_SOCKETS: usize = 4;
const NUM_UDP_SOCKETS: usize = 4;
const NUM_SOCKETS: usize = NUM_UDP_SOCKETS + NUM_TCP_SOCKETS;

pub struct NetworkStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],
    pub sockets: [Option<smoltcp::socket::SocketSetItem<'static>>; NUM_SOCKETS + 2], // + 1 for DHCP, + 1 for DNS
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub udp_socket_storage: [UdpSocketStorage; NUM_UDP_SOCKETS],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
    pub dns_servers: [smoltcp::wire::IpAddress; 3],
    pub dns_queries: [Option<DnsQuery>; 3],
}

impl NetworkStorage {
    const IP_INIT: smoltcp::wire::IpCidr = smoltcp::wire::IpCidr::Ipv4(
        smoltcp::wire::Ipv4Cidr::new(smoltcp::wire::Ipv4Address::UNSPECIFIED, 0),
    );

    pub const fn new() -> Self {
        NetworkStorage {
            // Placeholder for the real IP address, which is initialized at runtime.
            ip_addrs: [Self::IP_INIT],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [None, None, None, None, None, None, None, None, None, None],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
            udp_socket_storage: [UdpSocketStorage::INIT; NUM_UDP_SOCKETS],
            dns_servers: [smoltcp::wire::IpAddress::Unspecified; 3],
            dns_queries: [None, None, None],
        }
    }
}

pub struct UdpSocketStorage {
    rx_storage: [u8; 2048],
    tx_storage: [u8; 2048],
    tx_metadata: [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
    rx_metadata: [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
}

impl UdpSocketStorage {
    pub const INIT: Self = Self::new();

    const fn new() -> Self {
        Self {
            rx_storage: [0; 2048],
            tx_storage: [0; 2048],
            tx_metadata: [smoltcp::storage::PacketMetadata::<smoltcp::wire::IpEndpoint>::EMPTY; 10],
            rx_metadata: [smoltcp::storage::PacketMetadata::<smoltcp::wire::IpEndpoint>::EMPTY; 10],
        }
    }
}

#[derive(Copy, Clone)]
pub struct TcpSocketStorage {
    rx_storage: [u8; 2048],
    tx_storage: [u8; 2048],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 2048],
            tx_storage: [0; 2048],
        }
    }
}

/// The available networking devices on Stabilizer.
pub struct NetworkDevices {
    pub stack: NetworkStack,
    pub phy: EthernetPhy,
}

pub type NetworkStack =
    smoltcp_nal::NetworkStack<'static, hal::ethernet::EthernetDMA<'static, 4, 4>>;

pub type EthernetPhy = hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>;

// #[link_section = ".sram3.eth"]
#[link_section = ".axisram.eth"]
/// Static storage for the ethernet DMA descriptor ring.
static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

#[inline(always)]
pub fn setup_with_ethernet(
    mut core: rtic::export::Peripherals,
    device: stm32h7xx_hal::stm32::Peripherals,
    network_storage: &'static mut NetworkStorage,
) -> (NetworkDevices, Systick<1_000>, LinkLed, UserLed) {
    let pwr = device.PWR.constrain();
    let pwrcfg = pwr.ldo().freeze();

    // Enable SRAM3 for the ethernet descriptor ring.
    // device.RCC.c1_ahb2enr.modify(|_, w| {
    //     w.sram1en()
    //         .set_bit()
    //         .sram2en()
    //         .set_bit()
    //         .sram3en()
    //         .set_bit()
    // });

    // Clear reset flags.
    device.RCC.rsr.write(|w| w.rmvf().set_bit());

    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.mhz())
        .hclk(200.mhz())
        .freeze(pwrcfg, &device.SYSCFG);

    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);

    // Configure ethernet pins.
    {
        let _rmii_ref_clk = gpioa.pa1.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_mdio = gpioa.pa2.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_mdc = gpioc.pc1.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_crs_dv = gpioa.pa7.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_rxd0 = gpioc.pc4.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_rxd1 = gpioc.pc5.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_tx_en = gpiog.pg11.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_txd0 = gpiog.pg13.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_txd1 = gpiob.pb13.into_alternate_af11().set_speed(VeryHigh);
    }

    let mac_addr = smoltcp::wire::EthernetAddress(create_mac());
    defmt::info!("EUI48: {:x}", mac_addr);

    let network_devices = {
        // Configure the ethernet controller
        let (eth_dma, eth_mac) = unsafe {
            ethernet::new_unchecked(
                device.ETHERNET_MAC,
                device.ETHERNET_MTL,
                device.ETHERNET_DMA,
                &mut DES_RING,
                mac_addr,
                ccdr.peripheral.ETH1MAC,
                &ccdr.clocks,
            )
        };

        // Reset and initialize the ethernet phy.
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();

        unsafe { ethernet::enable_interrupt() };

        let store = network_storage;

        store.ip_addrs[0] = smoltcp::wire::IpCidr::new(
            smoltcp::wire::IpAddress::Ipv4(smoltcp::wire::Ipv4Address::UNSPECIFIED),
            0,
        );

        let mut routes = smoltcp::iface::Routes::new(&mut store.routes_cache[..]);
        routes
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .ok();

        let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

        let interface = smoltcp::iface::InterfaceBuilder::new(eth_dma, &mut store.sockets[..])
            .hardware_addr(mac_addr.into())
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();

        let mut stack = smoltcp_nal::NetworkStack::new(interface);

        {
            for storage in store.tcp_socket_storage[..].iter_mut() {
                let rx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut storage.rx_storage[..]);
                let tx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut storage.tx_storage[..]);

                stack.add_socket(smoltcp::socket::TcpSocket::new(rx_buffer, tx_buffer).into());
            }

            for storage in store.udp_socket_storage[..].iter_mut() {
                let rx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                    &mut storage.rx_metadata[..],
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                    &mut storage.tx_metadata[..],
                    &mut storage.tx_storage[..],
                );

                stack.add_socket(smoltcp::socket::UdpSocket::new(rx_buffer, tx_buffer).into());
            }

            stack.add_socket(smoltcp::socket::Dhcpv4Socket::new().into());

            stack.add_socket(
                smoltcp::socket::DnsSocket::new(
                    &mut store.dns_servers[..],
                    &mut store.dns_queries[..],
                )
                .into(),
            );
        }

        let random_seed = {
            let mut rng = device.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
            let mut data = [0u8; 8];
            rng.fill(&mut data).ok();
            data
        };

        stack.seed_random_port(&random_seed);

        NetworkDevices {
            stack,
            phy: lan8742a,
        }
    };

    let mono = Systick::new(core.SYST, ccdr.clocks.hclk().0);

    let link_led = gpiob.pb0.into_push_pull_output();
    let led = gpioe.pe1.into_push_pull_output();

    core.SCB.invalidate_icache();
    core.SCB.enable_icache();

    (network_devices, mono, link_led, led)
}

#[inline(always)]
pub fn setup_leds_only(
    mut core: rtic::export::Peripherals,
    device: stm32h7xx_hal::stm32::Peripherals,
) -> (Systick<1_000>, LinkLed, UserLed) {
    let pwr = device.PWR.constrain();
    let pwrcfg = pwr.ldo().freeze();

    // Enable SRAM3 for the ethernet descriptor ring.
    device.RCC.c1_ahb2enr.modify(|_, w| {
        w.sram1en()
            .set_bit()
            .sram2en()
            .set_bit()
            .sram3en()
            .set_bit()
    });

    // Clear reset flags.
    device.RCC.rsr.write(|w| w.rmvf().set_bit());

    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.mhz())
        .hclk(200.mhz())
        .freeze(pwrcfg, &device.SYSCFG);

    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);

    let mono = Systick::new(core.SYST, ccdr.clocks.hclk().0);

    let link_led = gpiob.pb0.into_push_pull_output();
    let led = gpioe.pe1.into_push_pull_output();

    core.SCB.invalidate_icache();
    core.SCB.enable_icache();

    (mono, link_led, led)
}

pub fn create_mac() -> [u8; 6] {
    let data = signature::Uid::read();

    let mut crc = 0xffff_ffff_ffff_ffffu64;
    for byte in data {
        crc = crc ^ *byte as u64;
        for _ in 0..8 {
            let mask = (-((crc & 1) as i64)) as u64;
            crc = (crc >> 1) ^ (0xc96c_5795_d787_0f42 & mask);
        }
    }

    let crc = (!crc).to_ne_bytes();

    [crc[0], crc[1], crc[2], crc[3], crc[4], crc[5]]
}
