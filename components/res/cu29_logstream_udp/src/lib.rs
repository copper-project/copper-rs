//! Nonblocking UDP carriers for Copper's packet-oriented log streaming traits.
//!
//! Socket setup happens once, before endpoints enter the output worker. Packet calls
//! perform one socket operation with no allocation, locking, retry, or acknowledgement.
//! FEC, pacing, and session policy belong to logstream, above this resource.

use cu29::bundle_resources;
use cu29::config::ComponentConfig;
use cu29::resource::{BundleContext, ResourceBundle, ResourceManager};
use cu29::{CuError, CuResult};
use cu29_logstream::{CuStreamRx, CuStreamRxError, CuStreamTx, CuStreamTxError};
use socket2::{Domain, MaybeUninitSlice, Protocol, SockAddr, Socket, Type};
use std::io;
use std::mem::MaybeUninit;
use std::net::SocketAddr;
use std::sync::Arc;

/// Conservative capacity for any ordinary UDP datagram (IPv6 jumbograms excluded).
/// Used for truncation reports on OSes that do not return the original packet size.
const UDP_DATAGRAM_CAPACITY: usize = 65_535;

/// Explicit socket configuration, shared by the RON bundle and standalone receivers.
///
/// `bind_addr` is required. `remote_addr` enables the `tx` slot; the `rx` slot always
/// exists. Both slots share one bound socket. Addresses must be numeric IPv4/IPv6
/// socket addresses: setup does not perform DNS resolution. Optional socket options
/// retain OS defaults when absent. Kernel buffer sizes are requests, subject to OS
/// rounding and limits; they are unrelated to the stream's MTU or FEC memory budget.
#[derive(Clone, Debug)]
pub struct CuUdpLogStreamConfig {
    pub bind_addr: SocketAddr,
    pub remote_addr: Option<SocketAddr>,
    pub send_buffer_bytes: Option<usize>,
    pub recv_buffer_bytes: Option<usize>,
    /// IPv4 unicast TTL or IPv6 unicast hop limit, in 1..=255.
    pub ttl: Option<u32>,
    /// Six-bit DSCP value, in 0..=63. ECN bits remain zero.
    pub dscp: Option<u8>,
}

impl CuUdpLogStreamConfig {
    /// Listen on this address with OS socket defaults and no transmit destination.
    pub const fn new(bind_addr: SocketAddr) -> Self {
        Self {
            bind_addr,
            remote_addr: None,
            send_buffer_bytes: None,
            recv_buffer_bytes: None,
            ttl: None,
            dscp: None,
        }
    }

    /// Parse the resource-bundle configuration, rejecting unknown or mistyped keys.
    pub fn from_component(config: &ComponentConfig) -> CuResult<Self> {
        for key in config.0.keys() {
            if !matches!(
                key.as_str(),
                "bind_addr"
                    | "remote_addr"
                    | "send_buffer_bytes"
                    | "recv_buffer_bytes"
                    | "ttl"
                    | "dscp"
            ) {
                return Err(CuError::from(format!(
                    "Unknown UDP resource option '{key}'"
                )));
            }
        }
        let bind = config
            .get::<String>("bind_addr")?
            .ok_or_else(|| CuError::from("UDP resource requires bind_addr"))?;
        let mut parsed = Self::new(parse_addr(&bind)?);
        parsed.remote_addr = config
            .get::<String>("remote_addr")?
            .map(|value| parse_addr(&value))
            .transpose()?;
        parsed.send_buffer_bytes = config.get_value("send_buffer_bytes")?;
        parsed.recv_buffer_bytes = config.get_value("recv_buffer_bytes")?;
        parsed.ttl = config.get_value("ttl")?;
        parsed.dscp = config.get_value("dscp")?;
        parsed.validate()?;
        Ok(parsed)
    }

    fn validate(&self) -> CuResult<()> {
        if let Some(remote) = self.remote_addr
            && (remote.is_ipv4() != self.bind_addr.is_ipv4() || remote.port() == 0)
        {
            return Err(CuError::from(
                "UDP remote_addr must match the bind address family and have a nonzero port",
            ));
        }
        if [self.send_buffer_bytes, self.recv_buffer_bytes]
            .into_iter()
            .flatten()
            .any(|size| size == 0 || i32::try_from(size).is_err())
        {
            return Err(CuError::from(
                "UDP socket buffer sizes must be in 1..=2147483647",
            ));
        }
        if self.ttl.is_some_and(|ttl| !(1..=255).contains(&ttl)) {
            return Err(CuError::from("UDP ttl must be in 1..=255"));
        }
        if self.dscp.is_some_and(|dscp| dscp > 63) {
            return Err(CuError::from("UDP dscp must be in 0..=63"));
        }
        #[cfg(windows)]
        if self.dscp.is_some() {
            // Winsock requires the separate QoS API; IP_TOS is not a portable
            // way to honor an explicitly requested DSCP value on Windows.
            return Err(CuError::from(
                "UDP DSCP configuration is unsupported on Windows",
            ));
        }
        Ok(())
    }

    /// Open one nonblocking socket and create its typed endpoints.
    ///
    /// Clones share the socket without a userspace mutex. Receive clones compete
    /// for datagrams; they do not subscribe to copies of the same traffic. The socket
    /// is unconnected, so reception accepts any sender and creates no reverse channel.
    pub fn open(&self) -> CuResult<(Option<CuUdpLogStreamTx>, CuUdpLogStreamRx)> {
        self.validate()?;
        let socket = self.open_socket().map_err(|error| {
            CuError::new_with_cause("Failed to configure UDP stream socket", error)
        })?;
        let socket = Arc::new(socket);
        let tx = self.remote_addr.map(|remote| CuUdpLogStreamTx {
            socket: socket.clone(),
            remote: remote.into(),
        });
        Ok((tx, CuUdpLogStreamRx { socket }))
    }

    fn open_socket(&self) -> io::Result<Socket> {
        let socket = Socket::new(
            Domain::for_address(self.bind_addr),
            Type::DGRAM,
            Some(Protocol::UDP),
        )?;
        socket.set_nonblocking(true)?;
        if self.bind_addr.is_ipv6() {
            socket.set_only_v6(true)?;
        }
        if let Some(size) = self.send_buffer_bytes {
            socket.set_send_buffer_size(size)?;
        }
        if let Some(size) = self.recv_buffer_bytes {
            socket.set_recv_buffer_size(size)?;
        }
        if let Some(ttl) = self.ttl {
            if self.bind_addr.is_ipv4() {
                socket.set_ttl_v4(ttl)?;
            } else {
                socket.set_unicast_hops_v6(ttl)?;
            }
        }
        if let Some(dscp) = self.dscp {
            let traffic_class = u32::from(dscp) << 2;
            if self.bind_addr.is_ipv4() {
                socket.set_tos_v4(traffic_class)?;
            } else {
                set_ipv6_traffic_class(&socket, traffic_class)?;
            }
        }
        socket.bind(&self.bind_addr.into())?;
        Ok(socket)
    }
}

fn parse_addr(value: &str) -> CuResult<SocketAddr> {
    value.parse().map_err(|error| {
        CuError::new_with_cause("UDP address must be a numeric IP address and port", error)
    })
}

#[cfg(any(target_os = "linux", target_os = "macos", target_os = "android"))]
fn set_ipv6_traffic_class(socket: &Socket, value: u32) -> io::Result<()> {
    socket.set_tclass_v6(value)
}

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "android")))]
fn set_ipv6_traffic_class(_socket: &Socket, _value: u32) -> io::Result<()> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "IPv6 DSCP is unsupported on this platform",
    ))
}

/// Cloneable, one-way UDP transmitter. Success means local socket acceptance only.
#[derive(Clone, Debug)]
pub struct CuUdpLogStreamTx {
    socket: Arc<Socket>,
    remote: SockAddr,
}

impl CuUdpLogStreamTx {
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        local_addr(&self.socket)
    }
}

impl CuStreamTx for CuUdpLogStreamTx {
    fn try_send(&mut self, packet: &[u8]) -> Result<(), CuStreamTxError> {
        submit_once(packet, |packet| self.socket.send_to(packet, &self.remote))
    }
}

// Keeping the one-call boundary explicit also lets tests force WouldBlock and
// Interrupted without assuming that a real UDP kernel queue reports saturation.
fn submit_once(
    packet: &[u8],
    send: impl FnOnce(&[u8]) -> io::Result<usize>,
) -> Result<(), CuStreamTxError> {
    match send(packet) {
        Ok(len) if len == packet.len() => Ok(()),
        Ok(_) => Err(CuStreamTxError::Failed(
            "UDP socket accepted an incomplete datagram",
        )),
        Err(error) if error.kind() == io::ErrorKind::WouldBlock => Err(CuStreamTxError::WouldBlock),
        Err(_) => Err(CuStreamTxError::Failed("UDP datagram submission failed")),
    }
}

/// Cloneable nonblocking receiver with no internal packet buffer.
///
/// An oversized datagram is consumed and rejected, never returned as a partial
/// packet. Linux/Android report its exact length in `BufferTooSmall::needed`;
/// other supported OSes report a conservative sufficient capacity of 65,535 bytes.
/// The caller must ignore buffer contents on error. Empty datagrams are `Some(0)`.
#[derive(Clone, Debug)]
pub struct CuUdpLogStreamRx {
    socket: Arc<Socket>,
}

impl CuUdpLogStreamRx {
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        local_addr(&self.socket)
    }
}

impl CuStreamRx for CuUdpLogStreamRx {
    fn try_recv(&mut self, packet: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
        let capacity = packet.len();
        // Darwin can return success without consuming a datagram for a zero-length
        // receive. Give it one byte of space, but validate against caller capacity.
        #[cfg(target_os = "macos")]
        let mut empty_buffer = [0u8; 1];
        #[cfg(target_os = "macos")]
        let packet = if packet.is_empty() {
            &mut empty_buffer[..]
        } else {
            packet
        };
        // SAFETY: u8 and MaybeUninit<u8> have identical layout. The exclusive borrow
        // lives only for this call. socket2 guarantees recv_from_vectored_with_flags
        // writes only initialized bytes, preserving validity even when receive fails.
        let buffer = unsafe { &mut *(packet as *mut [u8] as *mut [MaybeUninit<u8>]) };
        let mut buffers = [MaybeUninitSlice::new(buffer)];
        #[cfg(any(target_os = "linux", target_os = "android"))]
        let flags = libc::MSG_TRUNC;
        #[cfg(not(any(target_os = "linux", target_os = "android")))]
        let flags = 0;
        match self
            .socket
            .recv_from_vectored_with_flags(&mut buffers, flags)
        {
            Ok((len, flags, _)) if flags.is_truncated() || len > capacity => {
                #[cfg(any(target_os = "linux", target_os = "android"))]
                let needed = if len > capacity {
                    len
                } else {
                    UDP_DATAGRAM_CAPACITY
                };
                #[cfg(not(any(target_os = "linux", target_os = "android")))]
                let needed = UDP_DATAGRAM_CAPACITY;
                Err(CuStreamRxError::BufferTooSmall { needed })
            }
            Ok((len, _, _)) => Ok(Some(len)),
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => Ok(None),
            Err(_) => Err(CuStreamRxError::Failed("UDP datagram reception failed")),
        }
    }
}

fn local_addr(socket: &Socket) -> io::Result<SocketAddr> {
    socket
        .local_addr()?
        .as_socket()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "UDP socket has no IP address"))
}

/// Copper bundle exposing statically named `tx` and `rx` resource slots.
/// `tx` is populated only when `remote_addr` is configured; `rx` is always populated.
pub struct CuUdpLogStreamResources;

bundle_resources!(CuUdpLogStreamResources: Tx, Rx);

impl ResourceBundle for CuUdpLogStreamResources {
    fn build(
        bundle: BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let config = config.ok_or_else(|| CuError::from("UDP resource requires configuration"))?;
        let (tx, rx) = CuUdpLogStreamConfig::from_component(config)?.open()?;
        if let Some(tx) = tx {
            manager.add_owned(bundle.key(CuUdpLogStreamResourcesId::Tx), tx)?;
        }
        manager.add_owned(bundle.key(CuUdpLogStreamResourcesId::Rx), rx)
    }
}

#[cfg(test)]
mod tests;
