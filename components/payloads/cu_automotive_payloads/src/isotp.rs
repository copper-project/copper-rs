//! ISO 15765-2 Transport Protocol (ISO-TP) types.
//!
//! These types model the segmented/reassembled PDU and the individual N-PDU
//! frame types exchanged during multi-frame transfers.

use bincode::{Decode, Encode};
use core::fmt;
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

/// Maximum size of a single ISO-TP PDU payload (ISO 15765-2 standard limit).
pub const ISOTP_MAX_PDU_SIZE: usize = 4095;

// ---------------------------------------------------------------------------
// Addressing modes
// ---------------------------------------------------------------------------

/// ISO-TP addressing mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum IsotpAddressingMode {
    /// Normal addressing — CAN ID identifies the logical channel.
    #[default]
    Normal,
    /// Normal fixed addressing — 29-bit CAN ID embeds source/target addresses.
    NormalFixed,
    /// Extended addressing — first data byte is the address extension.
    Extended,
    /// Mixed addressing (11-bit) — first data byte is address extension.
    Mixed11,
    /// Mixed addressing (29-bit).
    Mixed29,
}

// ---------------------------------------------------------------------------
// ISO-TP N-PDU types
// ---------------------------------------------------------------------------

/// Classification of an ISO-TP protocol data unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum IsotpFrameType {
    /// Single Frame — entire message fits in one CAN frame.
    SingleFrame,
    /// First Frame — start of a multi-frame transfer.
    FirstFrame,
    /// Consecutive Frame — continuation of a multi-frame transfer.
    ConsecutiveFrame,
    /// Flow Control — receiver controls the sender's pacing.
    FlowControl,
}

impl Default for IsotpFrameType {
    fn default() -> Self {
        Self::SingleFrame
    }
}

// ---------------------------------------------------------------------------
// Flow control status
// ---------------------------------------------------------------------------

/// Flow control status field.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum FlowStatus {
    /// Continue To Send.
    #[default]
    ContinueToSend,
    /// Wait — receiver is not ready yet.
    Wait,
    /// Overflow — receiver cannot handle the message.
    Overflow,
}

/// Flow control parameters received from the peer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct FlowControlParams {
    /// Flow status.
    pub status: FlowStatus,
    /// Block size — number of CF frames before the next FC. 0 = send all at once.
    pub block_size: u8,
    /// Separation Time minimum — minimum time between consecutive frames, in milliseconds.
    /// Values 0x00-0x7F = 0–127 ms, 0xF1-0xF9 = 100–900 μs.
    pub st_min: u8,
}

impl FlowControlParams {
    /// Returns the separation time minimum as microseconds.
    pub fn st_min_us(&self) -> u64 {
        match self.st_min {
            0x00..=0x7F => self.st_min as u64 * 1000, // ms → μs
            0xF1..=0xF9 => (self.st_min - 0xF0) as u64 * 100, // 100–900 μs
            _ => 127_000, // Reserved values → use max
        }
    }
}

// ---------------------------------------------------------------------------
// ISO-TP PDU (reassembled application-layer payload)
// ---------------------------------------------------------------------------

/// A complete ISO-TP Protocol Data Unit (reassembled application message).
///
/// This is the unit exchanged between the ISO-TP layer and the UDS/application layer.
/// The fixed-size buffer avoids heap allocation in the hot path.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct IsotpPdu {
    /// Source address (physical or logical).
    pub source_addr: u32,
    /// Target address.
    pub target_addr: u32,
    /// Addressing mode used for this transfer.
    pub addressing_mode: IsotpAddressingMode,
    /// PDU payload data.
    #[serde(with = "BigArray")]
    pub data: [u8; ISOTP_MAX_PDU_SIZE],
    /// Number of valid bytes in `data`.
    pub len: u16,
}

impl Default for IsotpPdu {
    fn default() -> Self {
        Self {
            source_addr: 0,
            target_addr: 0,
            addressing_mode: IsotpAddressingMode::Normal,
            data: [0u8; ISOTP_MAX_PDU_SIZE],
            len: 0,
        }
    }
}

impl IsotpPdu {
    /// Creates a new PDU with the given data slice (full addressing).
    pub fn new(source_addr: u32, target_addr: u32, payload: &[u8]) -> Self {
        let len = payload.len().min(ISOTP_MAX_PDU_SIZE) as u16;
        let mut data = [0u8; ISOTP_MAX_PDU_SIZE];
        data[..len as usize].copy_from_slice(&payload[..len as usize]);
        Self {
            source_addr,
            target_addr,
            addressing_mode: IsotpAddressingMode::Normal,
            data,
            len,
        }
    }

    /// Creates a new PDU from raw data without addressing info.
    pub fn from_data(payload: &[u8]) -> Self {
        Self::new(0, 0, payload)
    }

    /// Returns the valid data slice.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// Clears the PDU.
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl fmt::Display for IsotpPdu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "IsotpPdu(0x{:04X}→0x{:04X}, {} bytes)",
            self.source_addr, self.target_addr, self.len
        )
    }
}

// ---------------------------------------------------------------------------
// ISO-TP Transfer State (for stateful codec tasks)
// ---------------------------------------------------------------------------

/// ISO-TP transfer direction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum IsotpDirection {
    #[default]
    Idle,
    Receiving,
    Transmitting,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn isotp_pdu_basic() {
        let pdu = IsotpPdu::new(0x7E0, 0x7E8, &[0x22, 0xF1, 0x90]);
        assert_eq!(pdu.len, 3);
        assert_eq!(pdu.data(), &[0x22, 0xF1, 0x90]);
    }

    #[test]
    fn flow_control_st_min() {
        let fc = FlowControlParams {
            status: FlowStatus::ContinueToSend,
            block_size: 0,
            st_min: 10,
        };
        assert_eq!(fc.st_min_us(), 10_000);

        let fc2 = FlowControlParams {
            st_min: 0xF3,
            ..fc
        };
        assert_eq!(fc2.st_min_us(), 300);
    }
}
