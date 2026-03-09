//! CAN frame types — ISO 11898 Classical CAN and CAN FD.
//!
//! Fixed-size, no-alloc payloads suitable for the Copper hot path.

use bincode::{Decode, Encode};
use core::fmt;
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

// ---------------------------------------------------------------------------
// CAN Identifier
// ---------------------------------------------------------------------------

/// A CAN bus identifier, either 11-bit standard or 29-bit extended.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum CanId {
    /// Standard 11-bit identifier (0x000 – 0x7FF).
    Standard(u16),
    /// Extended 29-bit identifier (0x0000_0000 – 0x1FFF_FFFF).
    Extended(u32),
}

impl Default for CanId {
    fn default() -> Self {
        CanId::Standard(0)
    }
}

impl CanId {
    /// Returns the raw numeric identifier value.
    pub fn raw(&self) -> u32 {
        match self {
            CanId::Standard(id) => *id as u32,
            CanId::Extended(id) => *id,
        }
    }

    /// Returns `true` if this is an extended (29-bit) identifier.
    pub fn is_extended(&self) -> bool {
        matches!(self, CanId::Extended(_))
    }
}

impl fmt::Display for CanId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CanId::Standard(id) => write!(f, "0x{:03X}", id),
            CanId::Extended(id) => write!(f, "0x{:08X}", id),
        }
    }
}

// ---------------------------------------------------------------------------
// CAN Frame flags
// ---------------------------------------------------------------------------

/// Bit-flags for a classical CAN frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CanFlags(pub u8);

impl CanFlags {
    pub const NONE: Self = CanFlags(0);
    /// Remote Transmission Request.
    pub const RTR: Self = CanFlags(1 << 0);
    /// Error frame indicator.
    pub const ERR: Self = CanFlags(1 << 1);

    pub fn is_rtr(self) -> bool {
        self.0 & Self::RTR.0 != 0
    }
    pub fn is_err(self) -> bool {
        self.0 & Self::ERR.0 != 0
    }
}

/// Bit-flags for a CAN FD frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CanFdFlags(pub u8);

impl CanFdFlags {
    pub const NONE: Self = CanFdFlags(0);
    /// Bit Rate Switch — data phase uses higher bitrate.
    pub const BRS: Self = CanFdFlags(1 << 0);
    /// Error State Indicator.
    pub const ESI: Self = CanFdFlags(1 << 1);

    pub fn is_brs(self) -> bool {
        self.0 & Self::BRS.0 != 0
    }
    pub fn is_esi(self) -> bool {
        self.0 & Self::ESI.0 != 0
    }
}

// ---------------------------------------------------------------------------
// Classical CAN frame (8-byte max)
// ---------------------------------------------------------------------------

/// A classical CAN 2.0 frame with a fixed 8-byte data buffer.
///
/// This payload type is zero-allocation: the data buffer is always 8 bytes on the stack.
/// The actual number of valid data bytes is indicated by [`dlc`](CanFrame::dlc).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CanFrame {
    /// CAN identifier.
    pub id: CanId,
    /// Data length code (0–8).
    pub dlc: u8,
    /// Payload data. Only `data[0..dlc]` is valid.
    pub data: [u8; 8],
    /// Frame flags (RTR, ERR).
    pub flags: CanFlags,
}

impl Default for CanFrame {
    fn default() -> Self {
        Self {
            id: CanId::default(),
            dlc: 0,
            data: [0u8; 8],
            flags: CanFlags::NONE,
        }
    }
}

impl CanFrame {
    /// Creates a new CAN frame.
    pub fn new(id: CanId, data: &[u8]) -> Self {
        let dlc = data.len().min(8) as u8;
        let mut buf = [0u8; 8];
        buf[..dlc as usize].copy_from_slice(&data[..dlc as usize]);
        Self {
            id,
            dlc,
            data: buf,
            flags: CanFlags::NONE,
        }
    }

    /// Returns the valid data slice.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

impl fmt::Display for CanFrame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} [{}]", self.id, self.dlc)?;
        for b in self.data() {
            write!(f, " {:02X}", b)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// CAN FD frame (64-byte max)
// ---------------------------------------------------------------------------

/// A CAN FD frame with a fixed 64-byte data buffer.
///
/// The [`dlc`](CanFdFrame::dlc) field uses the CAN FD DLC encoding where values above 8
/// map to 12, 16, 20, 24, 32, 48, or 64 bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CanFdFrame {
    /// CAN identifier.
    pub id: CanId,
    /// Data length code (CAN FD encoded: 0–15 mapping to 0–64 bytes).
    pub dlc: u8,
    /// Payload data. Only `data[0..data_len()]` is valid.
    #[serde(with = "BigArray")]
    pub data: [u8; 64],
    /// CAN FD flags (BRS, ESI).
    pub flags: CanFdFlags,
}

impl Default for CanFdFrame {
    fn default() -> Self {
        Self {
            id: CanId::default(),
            dlc: 0,
            data: [0u8; 64],
            flags: CanFdFlags::NONE,
        }
    }
}

impl CanFdFrame {
    /// Maps the CAN FD DLC to actual byte count.
    pub fn data_len(&self) -> usize {
        match self.dlc {
            0..=8 => self.dlc as usize,
            9 => 12,
            10 => 16,
            11 => 20,
            12 => 24,
            13 => 32,
            14 => 48,
            15 => 64,
            _ => 64,
        }
    }

    /// Returns the valid data slice.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.data_len()]
    }
}

// ---------------------------------------------------------------------------
// Batch type for high-throughput scenarios
// ---------------------------------------------------------------------------

/// A batch of CAN frames for high-throughput buses.
///
/// Use this when a single copper cycle may receive many frames (e.g. 1 Mbit/s bus).
/// The fixed-size array avoids heap allocation; `len` indicates valid entries.
#[derive(Clone, Debug, Encode, Decode)]
pub struct CanFrameBatch<const N: usize> {
    pub frames: [CanFrame; N],
    pub len: usize,
}

impl<const N: usize> Default for CanFrameBatch<N> {
    fn default() -> Self {
        Self {
            frames: [CanFrame::default(); N],
            len: 0,
        }
    }
}

impl<const N: usize> CanFrameBatch<N> {
    /// Push a frame into the batch. Returns `false` if full.
    pub fn push(&mut self, frame: CanFrame) -> bool {
        if self.len < N {
            self.frames[self.len] = frame;
            self.len += 1;
            true
        } else {
            false
        }
    }

    /// Returns the valid frames.
    pub fn as_slice(&self) -> &[CanFrame] {
        &self.frames[..self.len]
    }

    /// Clears the batch.
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

// ---------------------------------------------------------------------------
// CAN Bus Error types
// ---------------------------------------------------------------------------

/// CAN bus error state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum CanBusState {
    /// Normal operation.
    #[default]
    ErrorActive,
    /// Error-passive (TEC or REC ≥ 128).
    ErrorPassive,
    /// Bus-off (TEC ≥ 256).
    BusOff,
}

/// CAN hardware error counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CanErrorCounters {
    /// Transmit Error Counter.
    pub tec: u16,
    /// Receive Error Counter.
    pub rec: u16,
    /// Bus state derived from counters.
    pub state: CanBusState,
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{config, decode_from_slice, encode_to_vec};

    #[test]
    fn can_frame_round_trip() {
        let frame = CanFrame::new(CanId::Standard(0x123), &[0xDE, 0xAD, 0xBE, 0xEF]);
        let cfg = config::standard();
        let encoded = encode_to_vec(&frame, cfg).unwrap();
        let (decoded, _): (CanFrame, _) = decode_from_slice(&encoded, cfg).unwrap();
        assert_eq!(frame, decoded);
        assert_eq!(decoded.dlc, 4);
        assert_eq!(decoded.data(), &[0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn can_fd_dlc_mapping() {
        let mut f = CanFdFrame::default();
        for (dlc, expected) in [(0, 0), (8, 8), (9, 12), (12, 24), (13, 32), (14, 48), (15, 64)] {
            f.dlc = dlc;
            assert_eq!(f.data_len(), expected);
        }
    }

    #[test]
    fn can_frame_batch() {
        let mut batch = CanFrameBatch::<4>::default();
        assert!(batch.push(CanFrame::new(CanId::Standard(1), &[1])));
        assert!(batch.push(CanFrame::new(CanId::Standard(2), &[2])));
        assert_eq!(batch.len, 2);
        assert_eq!(batch.as_slice().len(), 2);
    }
}
