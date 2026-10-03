//! Native derives support `#[bincode(skip)]` for runtime-only fields, with an
//! optional `default = "path::to::function"` initializer. See the crate README.
//!
//! A default function is meaningful only on a skipped field:
//! ```compile_fail
//! # extern crate cu_bincode as bincode;
//! #[derive(bincode::Encode)]
//! struct Invalid { #[bincode(default = "Default::default")] field: u8 }
//! ```
//! Skipping and Serde delegation cannot be combined:
//! ```compile_fail
//! # extern crate cu_bincode as bincode;
//! #[derive(bincode::Encode)]
//! struct Invalid { #[bincode(skip, with_serde)] field: u8 }
//! ```
#[cfg_attr(docsrs, doc(cfg(feature = "derive")))]
pub use bincode_derive::{BorrowDecode, Decode, Encode};
#[cfg_attr(docsrs, doc(cfg(feature = "derive")))]
use cu_bincode_derive as bincode_derive;
