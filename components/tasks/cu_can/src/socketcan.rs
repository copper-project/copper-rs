//! SocketCAN low-level operations for Linux.
//!
//! Uses the `libc` crate for system calls to interface with the Linux
//! kernel SocketCAN subsystem.

#[cfg(all(target_os = "linux", not(feature = "mock")))]
use cu29::prelude::*;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
use cu_automotive_payloads::{CanFlags, CanFrame, CanId};

#[cfg(all(target_os = "linux", not(feature = "mock")))]
const CAN_EFF_FLAG: u32 = 0x8000_0000;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
const CAN_RTR_FLAG: u32 = 0x4000_0000;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
const CAN_ERR_FLAG: u32 = 0x2000_0000;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
const CAN_SFF_MASK: u32 = 0x0000_07FF;

#[cfg(all(target_os = "linux", not(feature = "mock")))]
const AF_CAN: i32 = 29;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
const PF_CAN: i32 = AF_CAN;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
const CAN_RAW: i32 = 1;
#[cfg(all(target_os = "linux", not(feature = "mock")))]
const SIOCGIFINDEX: libc::c_ulong = 0x8933;

/// Linux SocketCAN frame (struct can_frame in <linux/can.h>)
#[cfg(all(target_os = "linux", not(feature = "mock")))]
#[repr(C)]
struct RawCanFrame {
    can_id: u32,
    can_dlc: u8,
    __pad: u8,
    __res0: u8,
    __res1: u8,
    data: [u8; 8],
}

/// sockaddr_can
#[cfg(all(target_os = "linux", not(feature = "mock")))]
#[repr(C)]
struct SockaddrCan {
    can_family: u16,
    ifindex: i32,
    rx_id: u32,
    tx_id: u32,
}

/// ifreq for SIOCGIFINDEX
#[cfg(all(target_os = "linux", not(feature = "mock")))]
#[repr(C)]
struct Ifreq {
    ifr_name: [u8; 16],
    ifr_ifindex: i32,
    _pad: [u8; 20],
}

/// Open a CAN raw socket bound to the given interface.
#[cfg(all(target_os = "linux", not(feature = "mock")))]
pub fn open_can_socket(interface: &str) -> CuResult<i32> {
    unsafe {
        let fd = libc::socket(PF_CAN, libc::SOCK_RAW, CAN_RAW);
        if fd < 0 {
            return Err("Failed to create CAN socket".into());
        }

        let mut ifr = core::mem::zeroed::<Ifreq>();
        let name_bytes = interface.as_bytes();
        let copy_len = name_bytes.len().min(15);
        ifr.ifr_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        if libc::ioctl(fd, SIOCGIFINDEX, &mut ifr as *mut Ifreq) < 0 {
            libc::close(fd);
            return Err(alloc::format!(
                "Failed to get interface index for '{}'", interface
            ).into());
        }

        let addr = SockaddrCan {
            can_family: AF_CAN as u16,
            ifindex: ifr.ifr_ifindex,
            rx_id: 0,
            tx_id: 0,
        };

        if libc::bind(
            fd,
            &addr as *const SockaddrCan as *const libc::sockaddr,
            core::mem::size_of::<SockaddrCan>() as u32,
        ) < 0 {
            libc::close(fd);
            return Err(alloc::format!(
                "Failed to bind CAN socket to '{}'", interface
            ).into());
        }

        // Set non-blocking
        let flags = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);

        Ok(fd)
    }
}

/// Non-blocking read of a single CAN frame.
#[cfg(all(target_os = "linux", not(feature = "mock")))]
pub fn read_frame_nonblocking(fd: i32) -> Option<CanFrame> {
    unsafe {
        let mut raw = core::mem::zeroed::<RawCanFrame>();
        let n = libc::read(
            fd,
            &mut raw as *mut RawCanFrame as *mut libc::c_void,
            core::mem::size_of::<RawCanFrame>(),
        );
        if n < core::mem::size_of::<RawCanFrame>() as isize {
            return None;
        }

        let id = if raw.can_id & CAN_EFF_FLAG != 0 {
            CanId::Extended(raw.can_id & CAN_EFF_MASK)
        } else {
            CanId::Standard((raw.can_id & CAN_SFF_MASK) as u16)
        };

        let mut flags = CanFlags::NONE;
        if raw.can_id & CAN_RTR_FLAG != 0 {
            flags = CanFlags(flags.0 | CanFlags::RTR.0);
        }
        if raw.can_id & CAN_ERR_FLAG != 0 {
            flags = CanFlags(flags.0 | CanFlags::ERR.0);
        }

        Some(CanFrame {
            id,
            dlc: raw.can_dlc.min(8),
            data: raw.data,
            flags,
        })
    }
}

/// Write a CAN frame to the socket.
#[cfg(all(target_os = "linux", not(feature = "mock")))]
pub fn write_frame(fd: i32, frame: &CanFrame) -> CuResult<()> {
    let mut raw = RawCanFrame {
        can_id: match frame.id {
            CanId::Standard(id) => id as u32,
            CanId::Extended(id) => id | CAN_EFF_FLAG,
        },
        can_dlc: frame.dlc.min(8),
        __pad: 0,
        __res0: 0,
        __res1: 0,
        data: frame.data,
    };
    if frame.flags.is_rtr() {
        raw.can_id |= CAN_RTR_FLAG;
    }

    unsafe {
        let n = libc::write(
            fd,
            &raw as *const RawCanFrame as *const libc::c_void,
            core::mem::size_of::<RawCanFrame>(),
        );
        if n < 0 {
            return Err("Failed to write CAN frame".into());
        }
    }
    Ok(())
}

/// Close the socket.
#[cfg(all(target_os = "linux", not(feature = "mock")))]
pub fn close_socket(fd: i32) {
    unsafe { libc::close(fd); }
}
