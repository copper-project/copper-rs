//! Recover lost symbols without retransmission.
//!
//! Real-time and one-way streams often cannot wait for a lost packet to be sent
//! again. Forward erasure correction adds repair symbols that let a receiver
//! reconstruct missing source symbols from the packets that do arrive.
//!
//! This crate implements the systematic sliding-window random linear code from
//! [RFC 8681] over GF(2) and GF(2^8). It suits lossy radio and UDP telemetry,
//! logging, sensor, and media streams where recovery latency and memory use must
//! remain bounded. It is transport agnostic, allocator-free, dependency-free,
//! and usable from `no_std` targets.
//!
//! `cu-fec` was developed within the [Copper] robotics project for deterministic
//! log streaming, but has no Copper dependencies, types, or protocol assumptions.
//! Packet integrity, framing, pacing, and transport remain the caller's
//! responsibility.
//!
//! # Example
//!
//! ```
//! use cu_fec::{
//!     DensityThreshold, EncodingSymbolId, Field, RepairParameters, RlcConfig,
//!     RlcDecoder, RlcEncoder,
//! };
//!
//! # fn main() -> Result<(), cu_fec::Error> {
//! let config = RlcConfig::new(4, 16, Field::Gf256)?;
//! let mut encoder = RlcEncoder::<1200, 64>::new(config, EncodingSymbolId::new(0))?;
//! let mut decoder = RlcDecoder::<1200, 64, 64>::new(config, 16)?;
//!
//! let esi = encoder.push_source(&[1, 2, 3, 4])?;
//! let _ = decoder.receive_source(esi, &[1, 2, 3, 4])?;
//!
//! let mut repair = [0_u8; 1200];
//! let id = encoder.encode_repair(
//!     RepairParameters::new(7, DensityThreshold::FULL),
//!     &mut repair[..config.symbol_size()],
//! )?;
//! let _ = decoder.receive_repair(id, &repair[..config.symbol_size()])?;
//! # Ok(())
//! # }
//! ```
//!
//! [Copper]: https://github.com/copper-project/copper-rs
//! [RFC 8681]: https://www.rfc-editor.org/rfc/rfc8681.html

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod field;
/// Sliding-window random linear coding from RFC 8681.
pub mod rlc;
mod tinymt;
/// RFC 8681 payload identifiers and wire metadata.
pub mod wire;

pub use field::Field;
pub use rlc::{
    DecodeReport, Decoder as RlcDecoder, DensityThreshold, Encoder as RlcEncoder, EncodingSymbolId,
    Error, KnownSymbols, RepairParameters, RlcConfig, SourceReport, SourceStatus,
    generate_coding_coefficients,
};
pub use wire::{FecSchemeSpecificInfo, RepairPayloadId, SourcePayloadId, WireError};
