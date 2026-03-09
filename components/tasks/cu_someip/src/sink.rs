//! SOME/IP UDP Sink — sends SOME/IP messages to the network.

use cu29::prelude::*;
use cu_automotive_payloads::someip::{SomeIpMessage, SOMEIP_HEADER_SIZE, SOMEIP_MAX_PAYLOAD_SIZE};

/// Send buffer size.
#[allow(dead_code)]
const TX_BUF_SIZE: usize = SOMEIP_HEADER_SIZE + SOMEIP_MAX_PAYLOAD_SIZE;

/// SOME/IP UDP Sink task.
///
/// Sends SOME/IP messages via UDP to a remote endpoint.
///
/// # Config
/// - `remote_addr` (String): Destination IP address. Default: `"127.0.0.1"`.
/// - `remote_port` (i64): Destination port. Default: `30509`.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SomeIpSink {
    #[allow(dead_code)]
    remote_addr: alloc::string::String,
    #[allow(dead_code)]
    remote_port: u16,
    #[cfg(all(target_os = "linux", not(feature = "mock")))]
    fd: i32,
    #[cfg(all(target_os = "linux", not(feature = "mock")))]
    dest: libc::sockaddr_in,
    #[cfg(feature = "mock")]
    tx_count: u64,
}

impl Freezable for SomeIpSink {}

impl CuSinkTask for SomeIpSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(SomeIpMessage);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let remote_addr = match config {
            Some(cfg) => cfg
                .get::<alloc::string::String>("remote_addr")?
                .unwrap_or_else(|| alloc::string::String::from("127.0.0.1")),
            None => alloc::string::String::from("127.0.0.1"),
        };
        let remote_port = match config {
            Some(cfg) => cfg.get::<i64>("remote_port")?.unwrap_or(30509) as u16,
            None => 30509,
        };

        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        let (fd, dest) = {
            unsafe {
                let fd = libc::socket(libc::AF_INET, libc::SOCK_DGRAM, 0);
                if fd < 0 {
                    return Err(CuError::from("Failed to create UDP socket for SOME/IP sink"));
                }

                let ip_parts: alloc::vec::Vec<u8> = remote_addr
                    .split('.')
                    .filter_map(|s| s.parse().ok())
                    .collect();
                let ip_addr: u32 = if ip_parts.len() == 4 {
                    u32::from_be_bytes([ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3]])
                } else {
                    0x7f000001 // 127.0.0.1
                };

                let dest = libc::sockaddr_in {
                    sin_family: libc::AF_INET as u16,
                    sin_port: remote_port.to_be(),
                    sin_addr: libc::in_addr {
                        s_addr: ip_addr.to_be(),
                    },
                    sin_zero: [0; 8],
                };

                (fd, dest)
            }
        };

        Ok(Self {
            remote_addr,
            remote_port,
            #[cfg(all(target_os = "linux", not(feature = "mock")))]
            fd,
            #[cfg(all(target_os = "linux", not(feature = "mock")))]
            dest,
            #[cfg(feature = "mock")]
            tx_count: 0,
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(_msg) = input.payload() {
            #[cfg(all(target_os = "linux", not(feature = "mock")))]
            {
                let mut buf = [0u8; TX_BUF_SIZE];
                let len = _msg.to_wire(&mut buf);
                unsafe {
                    libc::sendto(
                        self.fd,
                        buf.as_ptr() as *const libc::c_void,
                        len,
                        0,
                        &self.dest as *const libc::sockaddr_in as *const libc::sockaddr,
                        core::mem::size_of::<libc::sockaddr_in>() as u32,
                    );
                }
            }

            #[cfg(feature = "mock")]
            {
                self.tx_count += 1;
            }
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
