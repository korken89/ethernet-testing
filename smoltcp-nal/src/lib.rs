#![no_std]

pub use embedded_nal;
use nanorand::Rng;
pub use smoltcp;

use embedded_nal::{TcpClientStack, UdpClientStack};
use smoltcp::socket::{AnySocket, Dhcpv4Event, Dhcpv4Socket, SocketHandle};
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
pub struct UdpSocket {
    handle: SocketHandle,
    destination: IpEndpoint,
}

// Used to facilitate `smoltcp` with an RNG
struct Rand;
static mut RAND: WyRand = WyRand::new_seed(0);

smoltcp::rand_custom_impl!(Rand);
impl smoltcp::Rand for Rand {
    fn rand_bytes(buf: &mut [u8]) {
        buf.chunks_mut(8).for_each(|chunk| {
            let r = critical_section::with(|_| {
                let rand = unsafe { &mut RAND };
                rand.rand()
            });
            chunk.copy_from_slice(&r[..chunk.len()]);
        });
    }
}

///! Network abstraction layer for smoltcp.
pub struct NetworkStack<'a, 'b, DeviceT>
where
    DeviceT: for<'c> smoltcp::phy::Device<'c>,
{
    network_interface: smoltcp::iface::Interface<'b, DeviceT>,
    dhcp_handle: Option<SocketHandle>,
    sockets: smoltcp::socket::SocketSet<'a>,
    unused_tcp_handles: Vec<SocketHandle, 16>,
    unused_udp_handles: Vec<SocketHandle, 16>,
    name_servers: Vec<Ipv4Address, 3>,
}

impl<'a, 'b, DeviceT> NetworkStack<'a, 'b, DeviceT>
where
    DeviceT: for<'c> smoltcp::phy::Device<'c>,
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
    pub fn new(
        stack: smoltcp::iface::Interface<'b, DeviceT>,
        sockets: smoltcp::socket::SocketSet<'a>,
    ) -> Self {
        let mut unused_tcp_handles: Vec<SocketHandle, 16> = Vec::new();
        let mut unused_udp_handles: Vec<SocketHandle, 16> = Vec::new();
        let mut dhcp_handle: Option<SocketHandle> = None;

        for socket in sockets.iter() {
            match socket {
                smoltcp::socket::Socket::Tcp(sock) => {
                    unused_tcp_handles.push(sock.handle()).ok();
                }
                smoltcp::socket::Socket::Udp(sock) => {
                    unused_udp_handles.push(sock.handle()).ok();
                }
                smoltcp::socket::Socket::Dhcpv4(sock) => {
                    dhcp_handle.replace(sock.handle());
                }

                // This branch may be enabled through cargo feature unification (e.g. if an
                // application enables raw-sockets). To accomodate this, we provide a default match
                // arm.
                #[allow(unreachable_patterns)]
                _ => {}
            }
        }

        NetworkStack {
            network_interface: stack,
            sockets,
            dhcp_handle,
            unused_tcp_handles,
            unused_udp_handles,
            name_servers: Vec::new(),
        }
    }

    /// Seed the TCP port randomizer.
    ///
    /// # Args
    /// * `seed` - A seed of random data to use for randomizing local TCP port selection.
    pub fn seed_random_port(&mut self, seed: &[u8]) {
        critical_section::with(|_| {
            let randomizer = unsafe { &mut RAND };
            randomizer.reseed(seed);
        });
    }

    /// Poll the network stack for potential updates.
    ///
    /// # Returns
    /// A boolean indicating if the network stack updated in any way.
    pub fn poll(&mut self, time: i64) -> Result<bool, smoltcp::Error> {
        let now = smoltcp::time::Instant::from_millis(time);
        let updated = self.network_interface.poll(&mut self.sockets, now)?;

        // Service the DHCP client.
        if let Some(handle) = self.dhcp_handle {
            let mut close_sockets = false;

            if let Some(event) = self.sockets.get::<Dhcpv4Socket>(handle).poll() {
                match event {
                    Dhcpv4Event::Configured(config) => {
                        if config.address.address().is_unicast()
                            && self.network_interface.ipv4_address()
                                != Some(config.address.address())
                        {
                            close_sockets = true;
                            Self::set_ipv4_addr(&mut self.network_interface, config.address);
                        }

                        // Store DNS server addresses for later read-back
                        self.name_servers.clear();
                        for server in config.dns_servers.iter() {
                            if let Some(server) = server {
                                // Note(unwrap): The name servers vector is at least as long as the
                                // number of DNS servers reported via DHCP.
                                self.name_servers.push(*server).ok();
                            }
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

        Ok(updated)
    }

    /// Force-close all sockets.
    pub fn close_sockets(&mut self) {
        // Close all sockets.
        for mut socket in self.sockets.iter_mut() {
            if let Some(ref mut socket) =
                smoltcp::socket::TcpSocket::downcast(smoltcp::socket::SocketRef::new(&mut socket))
            {
                socket.abort();
            }

            if let Some(ref mut socket) =
                smoltcp::socket::UdpSocket::downcast(smoltcp::socket::SocketRef::new(&mut socket))
            {
                socket.close();
            }
        }
    }

    fn set_ipv4_addr(interface: &mut smoltcp::iface::Interface<'b, DeviceT>, address: Ipv4Cidr) {
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
            self.sockets.get::<Dhcpv4Socket>(handle).reset();
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
        for mut socket in self.sockets.iter_mut() {
            // We only explicitly can close TCP sockets because we cannot access other socket types.
            if let Some(ref socket) =
                smoltcp::socket::TcpSocket::downcast(smoltcp::socket::SocketRef::new(&mut socket))
            {
                let endpoint = socket.local_endpoint();
                if endpoint.is_specified() {
                    if endpoint.port == port {
                        return true;
                    }
                }
            }

            if let Some(ref socket) =
                smoltcp::socket::UdpSocket::downcast(smoltcp::socket::SocketRef::new(&mut socket))
            {
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
                let random_data = critical_section::with(|_| {
                    let rand = unsafe { &mut RAND };
                    rand.rand()
                });
                u16::from_be_bytes([random_data[0], random_data[1]])
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

impl<'a, 'b, DeviceT> TcpClientStack for NetworkStack<'a, 'b, DeviceT>
where
    DeviceT: for<'c> smoltcp::phy::Device<'c>,
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
                let internal_socket: &mut smoltcp::socket::TcpSocket =
                    &mut *self.sockets.get(handle);
                internal_socket.abort();

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
            let internal_socket: &mut smoltcp::socket::TcpSocket = &mut self.sockets.get(*socket);

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
                let internal_socket: &mut smoltcp::socket::TcpSocket =
                    &mut *self.sockets.get(*socket);
                internal_socket
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

        let socket: &mut smoltcp::socket::TcpSocket = &mut *self.sockets.get(*socket);
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

        let socket: &mut smoltcp::socket::TcpSocket = &mut *self.sockets.get(*socket);
        socket
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

        let socket: &mut smoltcp::socket::TcpSocket = &mut *self.sockets.get(*socket);
        socket
            .recv_slice(buffer)
            .map_err(|_| embedded_nal::nb::Error::Other(NetworkError::ReadFailure))
    }

    fn close(&mut self, socket: SocketHandle) -> Result<(), NetworkError> {
        let internal_socket: &mut smoltcp::socket::TcpSocket = &mut *self.sockets.get(socket);

        internal_socket.close();
        self.unused_tcp_handles.push(socket).ok();
        Ok(())
    }
}

impl<'a, 'b, DeviceT> UdpClientStack for NetworkStack<'a, 'b, DeviceT>
where
    DeviceT: for<'c> smoltcp::phy::Device<'c>,
{
    type Error = NetworkError;
    type UdpSocket = UdpSocket;

    fn socket(&mut self) -> Result<UdpSocket, NetworkError> {
        // If we do not have a valid IP address yet, do not open the socket.
        if self.is_ip_unspecified() {
            return Err(NetworkError::NoIpAddress);
        }

        let handle = self
            .unused_udp_handles
            .pop()
            .ok_or(NetworkError::NoSocket)?;

        // Make sure the socket is in a closed state before handing it to the user.
        let internal_socket: &mut smoltcp::socket::UdpSocket = &mut *self.sockets.get(handle);
        internal_socket.close();

        Ok(UdpSocket {
            handle,
            destination: IpEndpoint::UNSPECIFIED,
        })
    }

    fn connect(
        &mut self,
        socket: &mut UdpSocket,
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

        let internal_socket: &mut smoltcp::socket::UdpSocket =
            &mut *self.sockets.get(socket.handle);
        internal_socket
            .bind(local_endpoint)
            .map_err(|_| NetworkError::ConnectionFailure)?;

        Ok(())
    }

    fn send(
        &mut self,
        socket: &mut UdpSocket,
        buffer: &[u8],
    ) -> embedded_nal::nb::Result<(), NetworkError> {
        if self.is_ip_unspecified() {
            return Err(embedded_nal::nb::Error::Other(NetworkError::NoIpAddress));
        }

        let internal_socket: &mut smoltcp::socket::UdpSocket =
            &mut *self.sockets.get(socket.handle);
        internal_socket
            .send_slice(buffer, socket.destination)
            .map_err(|_| embedded_nal::nb::Error::Other(NetworkError::WriteFailure))
    }

    fn receive(
        &mut self,
        socket: &mut UdpSocket,
        buffer: &mut [u8],
    ) -> embedded_nal::nb::Result<(usize, embedded_nal::SocketAddr), NetworkError> {
        if self.is_ip_unspecified() {
            return Err(embedded_nal::nb::Error::Other(NetworkError::NoIpAddress));
        }

        let internal_socket: &mut smoltcp::socket::UdpSocket =
            &mut *self.sockets.get(socket.handle);
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

    fn close(&mut self, socket: UdpSocket) -> Result<(), NetworkError> {
        let internal_socket: &mut smoltcp::socket::UdpSocket =
            &mut *self.sockets.get(socket.handle);

        internal_socket.close();

        // There should always be room to return the socket handle to the unused handle list.
        self.unused_udp_handles.push(socket.handle).ok();

        Ok(())
    }
}
