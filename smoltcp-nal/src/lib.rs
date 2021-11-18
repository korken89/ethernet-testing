#![no_std]

pub use embedded_nal;
use nanorand::Rng;
pub use smoltcp;

use embedded_nal::{TcpClientStack, UdpClientStack};
use smoltcp::iface::Interface;
use smoltcp::phy::Device;
use smoltcp::socket::{
    AnySocket, Dhcpv4Event, Dhcpv4Socket, DnsQueryHandle, DnsSocket, Socket, SocketHandle,
    TcpSocket, UdpSocket,
};
use smoltcp::wire::{IpAddress, IpCidr, IpEndpoint, Ipv4Address, Ipv4Cidr};

use heapless::Vec;
use nanorand::wyrand::WyRand;

// The start of TCP port dynamic range allocation.
const TCP_PORT_DYNAMIC_RANGE_START: u16 = 49152;

#[derive(Debug, Copy, Clone)]
pub enum NetworkError {
    NoSocket,
    ConnectionFailure,
    ReadFailure,
    WriteFailure,
    Unsupported,
    NoIpAddress,
}

#[derive(Debug)]
pub struct UdpSocketNal {
    handle: SocketHandle,
    destination: IpEndpoint,
}

struct RandWrap(core::cell::UnsafeCell<WyRand>);

impl RandWrap {
    const fn new() -> Self {
        Self(core::cell::UnsafeCell::new(WyRand::new_seed(0)))
    }

    pub fn seed(&self, seed: &[u8]) {
        critical_section::with(|_| (unsafe { &mut *self.0.get() }).reseed(seed));
    }

    pub fn rand_bytes(&self, buf: &mut [u8]) {
        buf.chunks_mut(8).for_each(|chunk| {
            let r = critical_section::with(|_| (unsafe { &mut *self.0.get() }).rand());
            chunk.copy_from_slice(&r[..chunk.len()]);
        });
    }
}

unsafe impl Sync for RandWrap {}

// Used to facilitate `smoltcp` with an RNG
struct Rand;
static RAND: RandWrap = RandWrap::new();

smoltcp::rand_custom_impl!(Rand);
impl smoltcp::Rand for Rand {
    fn rand_bytes(buf: &mut [u8]) {
        RAND.rand_bytes(buf)
    }
}

///! Network abstraction layer for smoltcp.
pub struct NetworkStack<'a, DeviceT>
where
    DeviceT: for<'c> Device<'c>,
{
    network_interface: Interface<'a, DeviceT>,
    dhcp_handle: Option<SocketHandle>,
    dns_handle: Option<SocketHandle>,
    dns_query: Option<DnsQueryHandle>,
    unused_tcp_handles: Vec<SocketHandle, 16>,
    unused_udp_handles: Vec<SocketHandle, 16>,
    name_servers: Vec<IpAddress, 3>,
}

impl<'a, DeviceT> NetworkStack<'a, DeviceT>
where
    DeviceT: for<'c> Device<'c>,
{
    /// Construct a new network stack.
    ///
    /// # Note
    /// This implementation only supports up to 16 usable sockets.
    ///
    /// Any handles provided to this function must not be used after constructing the network
    /// stack.
    ///
    /// This implementation currently only supports IPv4.
    ///
    /// # Args
    /// * `stack` - The ethernet interface to construct the network stack from.
    /// * `sockets` - The socket set to contain any socket state for the stack.
    ///
    /// # Returns
    /// A embedded-nal-compatible network stack.
    pub fn new(interface: smoltcp::iface::Interface<'a, DeviceT>) -> Self {
        defmt::info!("Hello from NAL!");

        NetworkStack {
            network_interface: interface,
            dhcp_handle: None,
            dns_handle: None,
            dns_query: None,
            unused_tcp_handles: Vec::new(),
            unused_udp_handles: Vec::new(),
            name_servers: Vec::new(),
        }
    }

    /// Add a socket to the stack.
    pub fn add_socket(&mut self, socket: Socket<'a>) {
        match socket {
            Socket::Udp(udp_socket) => {
                defmt::trace!("Added UDP socket");
                let handle = self.network_interface.add_socket(udp_socket);
                self.unused_udp_handles.push(handle).ok();
            }
            Socket::Tcp(tcp_socket) => {
                defmt::trace!("Added TCP socket");
                let handle = self.network_interface.add_socket(tcp_socket);
                self.unused_tcp_handles.push(handle).ok();
            }
            Socket::Dhcpv4(dhcp_socket) => {
                defmt::trace!("Added DHCPv4 socket");
                let handle = self.network_interface.add_socket(dhcp_socket);
                self.dhcp_handle.replace(handle);
            }
            Socket::Dns(dns_socket) => {
                defmt::trace!("Added DNS socket");
                let handle = self.network_interface.add_socket(dns_socket);
                self.dns_handle.replace(handle);
            }
            #[allow(unreachable_patterns)]
            _ => (),
        }
    }

    /// Seed the TCP port randomizer.
    ///
    /// # Args
    /// * `seed` - A seed of random data to use for randomizing local TCP port selection.
    pub fn seed_random_port(&mut self, seed: &[u8]) {
        RAND.seed(seed)
    }

    /// Poll the network stack for potential updates.
    ///
    /// # Returns
    /// A boolean indicating if the network stack updated in any way.
    pub fn poll(&mut self, time: i64) -> Result<bool, smoltcp::Error> {
        let now = smoltcp::time::Instant::from_millis(time);
        let updated = self.network_interface.poll(now)?;

        // Service the DHCP client.
        if let Some(handle) = self.dhcp_handle {
            let mut close_sockets = false;
            if let Some(event) = self
                .network_interface
                .get_socket::<Dhcpv4Socket>(handle)
                .poll()
            {
                match event {
                    Dhcpv4Event::Configured(config) => {
                        if config.address.address().is_unicast()
                            && self.network_interface.ipv4_address()
                                != Some(config.address.address())
                        {
                            close_sockets = true;
                            Self::set_ipv4_addr(&mut self.network_interface, config.address);
                            defmt::info!("DHCP address: {}", config.address);
                        }

                        // Store DNS server addresses for later read-back
                        self.name_servers.clear();
                        for server in config.dns_servers.iter() {
                            if let Some(server) = server {
                                // Note(unwrap): The name servers vector is at least as long as the
                                // number of DNS servers reported via DHCP.
                                self.name_servers.push(IpAddress::Ipv4(*server)).ok();
                                defmt::trace!("DNS server received: {}", server);
                            }
                        }

                        // Update the DNS handle with the
                        if let Some(handle) = self.dns_handle {
                            defmt::trace!(
                                "Updating DNS with servers: {}",
                                self.name_servers.as_slice()
                            );

                            self.network_interface
                                .get_socket::<DnsSocket>(handle)
                                .update_servers(self.name_servers.as_slice());
                        }

                        if let Some(route) = config.router {
                            // Note: If the user did not provide enough route storage, we may not be
                            // able to store the gateway.
                            self.network_interface
                                .routes_mut()
                                .add_default_ipv4_route(route)?;
                        } else {
                            self.network_interface
                                .routes_mut()
                                .remove_default_ipv4_route();
                        }
                    }
                    Dhcpv4Event::Deconfigured => {
                        self.network_interface
                            .routes_mut()
                            .remove_default_ipv4_route();
                        Self::set_ipv4_addr(
                            &mut self.network_interface,
                            Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0),
                        );
                    }
                }
            }

            if close_sockets {
                self.close_sockets();
            }
        }

        // Service the DNS client.
        if let (Some(query_handle), Some(dns_handle)) = (self.dns_query, self.dns_handle) {
            match self
                .network_interface
                .get_socket::<DnsSocket>(dns_handle)
                .get_query_result(query_handle)
            {
                Ok(addrs) => {
                    self.dns_query = None;
                    defmt::info!("DNS query for 'server' success: {}", addrs.as_slice());
                }
                Err(smoltcp::Error::Exhausted) => {
                    // defmt::info!("DNS query not complete...");
                } // not done yet
                Err(e) => {
                    self.dns_query = None;
                    defmt::info!("query failed: {:?}", e)
                }
            }
        }

        Ok(updated)
    }

    pub fn start_dns_query(&mut self, lookup: &[u8]) -> Result<(), ()> {
        if self.dns_query.is_some() {
            return Err(());
        }

        let dns_handle = self.dns_handle.ok_or(())?;
        let dns_socket = self.network_interface.get_socket::<DnsSocket>(dns_handle);
        let query_handle = dns_socket.start_query(lookup).map_err(|e| {
            defmt::info!("DNS query error: {}", e);
            ()
        })?;

        self.dns_query = Some(query_handle);

        Ok(())
    }

    /// Force-close all sockets.
    pub fn close_sockets(&mut self) {
        // Close all sockets.
        for socket in self.network_interface.sockets_mut() {
            if let Some(ref mut socket) = TcpSocket::downcast(socket) {
                socket.abort();
            }

            if let Some(ref mut socket) = UdpSocket::downcast(socket) {
                socket.close();
            }
        }
    }

    fn set_ipv4_addr(interface: &mut smoltcp::iface::Interface<'a, DeviceT>, address: Ipv4Cidr) {
        interface.update_ip_addrs(|addrs| {
            // Note(unwrap): This stack requires at least 1 Ipv4 Address.
            let addr = if let Some(addr) = addrs
                .iter_mut()
                .filter(|cidr| match cidr.address() {
                    IpAddress::Ipv4(_) => true,
                    _ => false,
                })
                .next()
            {
                addr
            } else {
                panic!("This stack requires at least 1 Ipv4 Address");
            };

            *addr = IpCidr::Ipv4(address);
        });
    }

    /// Handle a disconnection of the physical interface.
    pub fn handle_link_reset(&mut self) {
        // Reset the DHCP client.
        if let Some(handle) = self.dhcp_handle {
            self.network_interface
                .get_socket::<Dhcpv4Socket>(handle)
                .reset();
        }

        // Close all of the sockets and de-configure the interface.
        self.close_sockets();

        self.network_interface.update_ip_addrs(|addrs| {
            addrs.iter_mut().next().map(|addr| {
                *addr = IpCidr::Ipv4(Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
            });
        });
    }

    /// Check if a port is currently in use.
    ///
    /// # Returns
    /// True if the port is in use. False otherwise.
    fn is_port_in_use(&mut self, port: u16) -> bool {
        for socket in self.network_interface.sockets_mut() {
            if let Some(ref socket) = TcpSocket::downcast(socket) {
                let endpoint = socket.local_endpoint();
                if endpoint.is_specified() {
                    if endpoint.port == port {
                        return true;
                    }
                }
            }

            if let Some(ref socket) = UdpSocket::downcast(socket) {
                let endpoint = socket.endpoint();
                if endpoint.is_specified() {
                    if endpoint.port == port {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    // Get an ephemeral port number.
    fn get_ephemeral_port(&mut self) -> u16 {
        loop {
            // Get the next ephemeral port by generating a random, valid TCP port continuously
            // until an unused port is found.
            let random_offset = {
                let mut data = [0; 2];
                RAND.rand_bytes(&mut data);
                u16::from_be_bytes([data[0], data[1]])
            };

            let port = TCP_PORT_DYNAMIC_RANGE_START
                + random_offset % (u16::MAX - TCP_PORT_DYNAMIC_RANGE_START);
            if !self.is_port_in_use(port) {
                return port;
            }
        }
    }

    fn is_ip_unspecified(&self) -> bool {
        // Note(unwrap): This stack only supports Ipv4.
        if let Some(addr) = self.network_interface.ipv4_addr() {
            addr.is_unspecified()
        } else {
            panic!("This stack only supports Ipv4.");
        }
    }
}

impl<'a, DeviceT> TcpClientStack for NetworkStack<'a, DeviceT>
where
    DeviceT: for<'c> Device<'c>,
{
    type Error = NetworkError;
    type TcpSocket = SocketHandle;

    fn socket(&mut self) -> Result<SocketHandle, NetworkError> {
        // If we do not have a valid IP address yet, do not open the socket.
        if self.is_ip_unspecified() {
            return Err(NetworkError::NoIpAddress);
        }

        match self.unused_tcp_handles.pop() {
            Some(handle) => {
                // Abort any active connections on the handle.
                self.network_interface
                    .get_socket::<TcpSocket>(handle)
                    .abort();

                Ok(handle)
            }
            None => Err(NetworkError::NoSocket),
        }
    }

    fn connect(
        &mut self,
        socket: &mut SocketHandle,
        remote: embedded_nal::SocketAddr,
    ) -> embedded_nal::nb::Result<(), NetworkError> {
        // If there is no longer an IP address assigned to the interface, do not allow usage of the
        // socket.
        if self.is_ip_unspecified() {
            return Err(embedded_nal::nb::Error::Other(NetworkError::NoIpAddress));
        }

        {
            let internal_socket = self.network_interface.get_socket::<TcpSocket>(*socket);

            // If we're already in the process of connecting, ignore the request silently.
            if internal_socket.is_open() {
                return Ok(());
            }
        }

        match remote.ip() {
            embedded_nal::IpAddr::V4(addr) => {
                let octets = addr.octets();
                let address =
                    smoltcp::wire::Ipv4Address::new(octets[0], octets[1], octets[2], octets[3]);

                let local_port = self.get_ephemeral_port();
                self.network_interface
                    .get_socket::<TcpSocket>(*socket)
                    .connect((address, remote.port()), local_port)
                    .map_err(|_| embedded_nal::nb::Error::Other(NetworkError::ConnectionFailure))
            }

            // We only support IPv4.
            _ => Err(embedded_nal::nb::Error::Other(NetworkError::Unsupported)),
        }
    }

    fn is_connected(&mut self, socket: &SocketHandle) -> Result<bool, NetworkError> {
        // If there is no longer an IP address assigned to the interface, do not allow usage of the
        // socket.
        if self.is_ip_unspecified() {
            return Err(NetworkError::NoIpAddress);
        }

        let socket = self.network_interface.get_socket::<TcpSocket>(*socket);
        Ok(socket.may_send() && socket.may_recv())
    }

    fn send(
        &mut self,
        socket: &mut SocketHandle,
        buffer: &[u8],
    ) -> embedded_nal::nb::Result<usize, NetworkError> {
        // If there is no longer an IP address assigned to the interface, do not allow usage of the
        // socket.
        if self.is_ip_unspecified() {
            return Err(embedded_nal::nb::Error::Other(NetworkError::NoIpAddress));
        }

        self.network_interface
            .get_socket::<TcpSocket>(*socket)
            .send_slice(buffer)
            .map_err(|_| embedded_nal::nb::Error::Other(NetworkError::WriteFailure))
    }

    fn receive(
        &mut self,
        socket: &mut SocketHandle,
        buffer: &mut [u8],
    ) -> embedded_nal::nb::Result<usize, NetworkError> {
        // If there is no longer an IP address assigned to the interface, do not allow usage of the
        // socket.
        if self.is_ip_unspecified() {
            return Err(embedded_nal::nb::Error::Other(NetworkError::NoIpAddress));
        }

        self.network_interface
            .get_socket::<TcpSocket>(*socket)
            .recv_slice(buffer)
            .map_err(|_| embedded_nal::nb::Error::Other(NetworkError::ReadFailure))
    }

    fn close(&mut self, socket: SocketHandle) -> Result<(), NetworkError> {
        self.network_interface
            .get_socket::<TcpSocket>(socket)
            .close();
        self.unused_tcp_handles.push(socket).ok();
        Ok(())
    }
}

impl<'a, DeviceT> UdpClientStack for NetworkStack<'a, DeviceT>
where
    DeviceT: for<'c> Device<'c>,
{
    type Error = NetworkError;
    type UdpSocket = UdpSocketNal;

    fn socket(&mut self) -> Result<UdpSocketNal, NetworkError> {
        // If we do not have a valid IP address yet, do not open the socket.
        if self.is_ip_unspecified() {
            return Err(NetworkError::NoIpAddress);
        }

        let handle = self
            .unused_udp_handles
            .pop()
            .ok_or(NetworkError::NoSocket)?;

        // Make sure the socket is in a closed state before handing it to the user.
        self.network_interface
            .get_socket::<UdpSocket>(handle)
            .close();

        Ok(UdpSocketNal {
            handle,
            destination: IpEndpoint::UNSPECIFIED,
        })
    }

    fn connect(
        &mut self,
        socket: &mut UdpSocketNal,
        remote: embedded_nal::SocketAddr,
    ) -> Result<(), NetworkError> {
        if self.is_ip_unspecified() {
            return Err(NetworkError::NoIpAddress);
        }
        // Store the route for this socket.
        match remote {
            embedded_nal::SocketAddr::V4(addr) => {
                let octets = addr.ip().octets();
                socket.destination = IpEndpoint::new(
                    IpAddress::v4(octets[0], octets[1], octets[2], octets[3]),
                    addr.port(),
                )
            }

            // We only support IPv4.
            _ => return Err(NetworkError::Unsupported),
        }

        // Select a random port to bind to locally.
        let local_port = self.get_ephemeral_port();

        let local_address = if let Some(addr) = self
            .network_interface
            .ip_addrs()
            .iter()
            .filter(|item| matches!(item, smoltcp::wire::IpCidr::Ipv4(_)))
            .next()
        {
            addr.address()
        } else {
            panic!("No IPv4 addrees");
        };

        let local_endpoint = IpEndpoint::new(local_address, local_port);

        self.network_interface
            .get_socket::<UdpSocket>(socket.handle)
            .bind(local_endpoint)
            .map_err(|_| NetworkError::ConnectionFailure)?;

        Ok(())
    }

    fn send(
        &mut self,
        socket: &mut UdpSocketNal,
        buffer: &[u8],
    ) -> embedded_nal::nb::Result<(), NetworkError> {
        if self.is_ip_unspecified() {
            return Err(embedded_nal::nb::Error::Other(NetworkError::NoIpAddress));
        }

        self.network_interface
            .get_socket::<UdpSocket>(socket.handle)
            .send_slice(buffer, socket.destination)
            .map_err(|_| embedded_nal::nb::Error::Other(NetworkError::WriteFailure))
    }

    fn receive(
        &mut self,
        socket: &mut UdpSocketNal,
        buffer: &mut [u8],
    ) -> embedded_nal::nb::Result<(usize, embedded_nal::SocketAddr), NetworkError> {
        if self.is_ip_unspecified() {
            return Err(embedded_nal::nb::Error::Other(NetworkError::NoIpAddress));
        }

        let internal_socket = self
            .network_interface
            .get_socket::<UdpSocket>(socket.handle);
        let (size, source) = internal_socket
            .recv_slice(buffer)
            .map_err(|_| embedded_nal::nb::Error::Other(NetworkError::ReadFailure))?;

        let source = {
            let octets = source.addr.as_bytes();

            embedded_nal::SocketAddr::new(
                embedded_nal::IpAddr::V4(embedded_nal::Ipv4Addr::new(
                    octets[0], octets[1], octets[2], octets[3],
                )),
                source.port,
            )
        };

        Ok((size, source))
    }

    fn close(&mut self, socket: UdpSocketNal) -> Result<(), NetworkError> {
        self.network_interface
            .get_socket::<UdpSocket>(socket.handle)
            .close();

        // There should always be room to return the socket handle to the unused handle list.
        self.unused_udp_handles.push(socket.handle).ok();

        Ok(())
    }
}
