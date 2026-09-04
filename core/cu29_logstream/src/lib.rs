#![cfg_attr(not(feature = "std"), no_std)]

//! Transport-independent Copper log framing and continuous-stream recovery.
//!
//! The crate deliberately stops at datagrams. Runtime ownership handoff,
//! pacing, and concrete transports are composed outside this layer.

extern crate alloc;

mod copper;
mod error;
mod impairment;
mod record;
mod rlc;
mod wire;

pub use copper::{decode_copperlist, encode_copperlist};
pub use error::{Error, Result};
pub use impairment::{ImpairmentConfig, ImpairmentOutput, ImpairmentStats, impair};
pub use record::{DecodedRecord, RecordKind, encode_record};
pub use rlc::{
    ContinuousDecoder, ContinuousEncoder, ContinuousRecoveryStats, ReceiverLimits, RecoveredRecord,
    StreamIdentity,
};
pub use wire::{
    FecScheme, FecSymbolKind, Lane, PACKET_HEADER_LEN, WIRE_VERSION, WireHeader, WirePacket,
};
