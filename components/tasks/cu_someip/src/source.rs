//! SOME/IP UDP Source — receives SOME/IP messages from the network.

use cu29::prelude::*;
use cu_automotive_payloads::someip::{SomeIpMessage, SOMEIP_HEADER_SIZE, SOMEIP_MAX_PAYLOAD_SIZE};

/// Receive buffer size (header + max payload).
#[allow(dead_code)]
const RX_BUF_SIZE: usize = SOMEIP_HEADER_SIZE + SOMEIP_MAX_PAYLOAD_SIZE;

/// SOME/IP UDP Source task.
///
/// Listens on a UDP socket for SOME/IP messages and outputs them.
///
/// # Config
/// - `bind_addr` (String): Address to bind to. Default: `"0.0.0.0"`.
/// - `bind_port` (i64): Port to bind to. Default: `30509`.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SomeIpSource {
    #[allow(dead_code)]
    bind_addr: alloc::string::String,
    #[allow(dead_code)]
    bind_port: u16,
    #[cfg(all(target_os = "linux", not(feature = "mock")))]
    fd: i32,
    #[cfg(feature = "mock")]
    mock_counter: u32,
    pending_msg: Option<SomeIpMessage>,
}

impl Freezable for SomeIpSource {}

impl SomeIpSource {
    /// Open a non-blocking UDP socket.
    #[cfg(all(target_os = "linux", not(feature = "mock")))]
    fn open_udp(addr: &str, port: u16) -> CuResult<i32> {
        unsafe {
            let fd = libc::socket(libc::AF_INET, libc::SOCK_DGRAM, 0);
            if fd < 0 {
                return Err(CuError::from("Failed to create UDP socket"));
            }

            // Parse address
            let ip_parts: alloc::vec::Vec<u8> = addr
                .split('.')
                .filter_map(|s| s.parse().ok())
                .collect();
            let ip_addr: u32 = if ip_parts.len() == 4 {
                u32::from_be_bytes([ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3]])
            } else {
                0 // INADDR_ANY
            };

            let sockaddr = libc::sockaddr_in {
                sin_family: libc::AF_INET as u16,
                sin_port: port.to_be(),
                sin_addr: libc::in_addr {
                    s_addr: ip_addr.to_be(),
                },
                sin_zero: [0; 8],
            };

            let ret = libc::bind(
                fd,
                &sockaddr as *const libc::sockaddr_in as *const libc::sockaddr,
                core::mem::size_of::<libc::sockaddr_in>() as u32,
            );
            if ret < 0 {
                libc::close(fd);
                return Err(CuError::from("Failed to bind UDP socket"));
            }

            // Non-blocking
            let flags = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);

            Ok(fd)
        }
    }
}

impl CuSrcTask for SomeIpSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(SomeIpMessage);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let bind_addr = match config {
            Some(cfg) => cfg
                .get::<alloc::string::String>("bind_addr")?
                .unwrap_or_else(|| alloc::string::String::from("0.0.0.0")),
            None => alloc::string::String::from("0.0.0.0"),
        };
        let bind_port = match config {
            Some(cfg) => cfg.get::<i64>("bind_port")?.unwrap_or(30509) as u16,
            None => 30509,
        };

        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        let fd = Self::open_udp(&bind_addr, bind_port)?;

        Ok(Self {
            bind_addr,
            bind_port,
            #[cfg(all(target_os = "linux", not(feature = "mock")))]
            fd,
            #[cfg(feature = "mock")]
            mock_counter: 0,
            pending_msg: None,
        })
    }

    fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        {
            let mut buf = [0u8; RX_BUF_SIZE];
            unsafe {
                let n = libc::recv(self.fd, buf.as_mut_ptr() as *mut libc::c_void, RX_BUF_SIZE, 0);
                if n > 0 {
                    self.pending_msg = SomeIpMessage::from_wire(&buf[..n as usize]);
                }
            }
        }

        #[cfg(feature = "mock")]
        {
            self.mock_counter = self.mock_counter.wrapping_add(1);
            self.pending_msg = Some(SomeIpMessage::request(
                0x0100,
                0x0001,
                self.mock_counter as u16,
                self.mock_counter as u16,
                &[self.mock_counter as u8],
            ));
        }
        Ok(())
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        if let Some(msg) = self.pending_msg.take() {
            output.set_payload(msg);
            output.tov = Tov::Time(ctx.now());
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        unsafe {
            libc::close(self.fd);
        }
        Ok(())
    }
}
