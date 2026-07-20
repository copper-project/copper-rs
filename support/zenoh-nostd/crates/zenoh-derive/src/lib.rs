//! # Zenoh Protocol Derive Macros
//!
//! This crate provides procedural macros for automatically implementing the Zenoh protocol
//! codec traits. These macros generate efficient encoding and decoding implementations
//! for protocol messages and extensions.
//!
//! ## Available Macros
//!
//! ### `#[derive(ZStruct)]`
//!
//! Derives codec traits for structured protocol messages. This is the primary macro for
//! implementing the `ZEncode`, `ZDecode`, `ZLen`, and related traits.
//!
//! #### Attributes
//!
//! - `#[zenoh(header = "...")]`: Defines the message header format (on struct)
//! - `#[zenoh(flatten)]`: Flattens a nested field into the parent (on field)
//! - `#[zenoh(size = prefixed|remain|header(FIELD))]`: Specifies size encoding strategy (on field)
//! - `#[zenoh(presence = prefixed|header(FIELD))]`: Marks field as optional with presence flag (on field)
//! - `#[zenoh(default = expr)]`: Provides a default value when field is absent AND avoid encoding it if it matches the default value (on field, must be used with `presence` or `ext`)
//! - `#[zenoh(ext = ID)]`: Marks field as a protocol extension with given ID (on field)
//! - `#[zenoh(mandatory)]`: Marks an extension as mandatory (on field)
//! - `#[zenoh(shift = N)]`: Bit-shifts the encoded header (on field, must be used with `flatten`)
//! - `#[zenoh(maybe_empty)]`: Allows empty collections (on field)
//!
//! #### Example
//!
//! ```ignore
//! #[derive(ZStruct, Debug, PartialEq)]
//! #[zenoh(header = "_:3|ID:5=0x01")]
//! pub struct MyMessage {
//!     pub field1: u32,
//!
//!     #[zenoh(presence = header(P))]
//!     pub optional_field: Option<u64>,
//!
//!     #[zenoh(ext = 0x1, default = false)]
//!     pub flag: bool,
//! }
//! ```
//!
//! ### `#[derive(ZExt)]`
//!
//! Derives codec traits for protocol extensions. Extensions are optional, versioned
//! additions to protocol messages that maintain backward compatibility.
//!
//! The macro automatically infers the extension kind:
//! - Empty struct → `ZExtKind::Unit`
//! - Single integer field → `ZExtKind::U64`
//! - Other structures → `ZExtKind::ZStruct`
//!
//! #### Example
//!
//! ```ignore
//! #[derive(ZExt, Debug, PartialEq)]
//! pub struct Timestamp {
//!     pub value: u64,
//! }
//! ```
//!
//! ### `#[derive(ZEnum)]`
//!
//! Derives codec traits for enums that are encoded as structured messages.
//! Each variant can have associated data that is encoded according to its type.
//!
//! #### Example
//!
//! ```ignore
//! #[derive(ZEnum, Debug, PartialEq)]
//! pub enum MessageType {
//!     Request(RequestData),
//!     Response(ResponseData),
//! }
//! ```
//!
//! ### `#[derive(ZRU8)]`
//!
//! Derives codec traits for enums represented as a single `u8` (repr(u8)).
//! This is used for simple discriminated enums without associated data.
//!
//! #### Example
//!
//! ```ignore
//! #[repr(u8)]
//! #[derive(ZRU8, Debug, PartialEq)]
//! pub enum Priority {
//!     Low = 0,
//!     Normal = 1,
//!     High = 2,
//! }
//! ```
//!
//! ## Header Format Specification
//!
//! The `header` attribute uses a DSL to specify the bit layout:
//!
//! ```text
//! #[zenoh(header = "Z|E|I|ID:5=0x01")]
//!                   ^  ^  ^  ^^^^^^
//!                   |  |  |  |    |
//!                   |  |  |  |    +-- Value (5 bits = 0x01)
//!                   |  |  |  +------- Field ID (5 bits wide)
//!                   |  |  +---------- Field I (1 bit)
//!                   |  +------------- Field E (1 bit)
//!                   +---------------- Field Z (1 bit)
//! ```
//!
//! - Named bits (e.g., `Z`, `E`): Single-bit boolean flags
//! - Underscore `_`: Reserved/unused bit
//! - `FIELD:N`: N-bit field with dynamic value
//! - `FIELD:N=VALUE`: N-bit field with constant value
//!
//! ## Implementation Details
//!
//! The macros generate implementations that:
//! - Are `no_std` compatible
//! - Support zero-copy decoding where possible
//! - Use variable-length encoding for integers
//! - Handle endianness consistently
//! - Provide detailed error reporting

pub(crate) mod codec;
pub(crate) mod link;
pub(crate) mod zerror;

/// Derives Zenoh protocol codec traits for a struct.
///
/// This macro implements `ZEncode`, `ZDecode`, `ZLen`, `ZBodyEncode`,
/// `ZBodyDecode`, and `ZBodyLen` for the annotated struct. It also implements
/// `ZHeader` if a header format is specified, and `ZExtCount` if the struct
/// contains extensions.
///
/// # Attributes
///
/// ## Struct-level
///
/// - `#[zenoh(header = "format")]`: Specifies the header byte format
///
/// ## Field-level
///
/// - `#[zenoh(flatten)]`: Flattens a nested field into the parent
/// - `#[zenoh(size = prefixed|remain|header(FIELD))]`: Specifies size encoding strategy
/// - `#[zenoh(presence = prefixed|header(FIELD))]`: Marks field as optional with presence flag
/// - `#[zenoh(default = expr)]`: Provides a default value when field is absent AND avoid encoding it if it matches the default value
/// - `#[zenoh(ext = ID)]`: Marks field as a protocol extension with given ID
/// - `#[zenoh(mandatory)]`: Marks an extension as mandatory
/// - `#[zenoh(shift = N)]`: Bit-shifts the encoded header
/// - `#[zenoh(maybe_empty)]`: Allows empty collections
///
/// # Examples
///
/// ```ignore
/// use zenoh_proto::*;
///
/// #[derive(ZStruct, Debug, PartialEq)]
/// #[zenoh(header = "Z|_:2|ID:5=0x01")]
/// pub struct MyMessage {
///     pub sequence: u32,
///
///     #[zenoh(presence = header(Z))]
///     pub optional_data: Option<u64>,
///
///     #[zenoh(ext = 0x1)]
///     pub metadata: Option<Metadata>,
/// }
/// ```
#[proc_macro_derive(ZStruct, attributes(zenoh))]
pub fn derive_zstruct(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    codec::r#struct::derive_zstruct(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

/// Derives Zenoh protocol extension traits for a struct.
///
/// This macro implements `ZExt`, `ZEncode`, `ZDecode`, and `ZLen` for
/// protocol extensions. It automatically infers the extension kind based on
/// the struct's fields:
///
/// - Empty struct → `ZExtKind::Unit` (flag-only extension)
/// - Single integer field (`u8`, `u16`, `u32`, `u64`, `usize`) → `ZExtKind::U64`
/// - Other cases → `ZExtKind::ZStruct` (structured extension)
///
/// # Examples
///
/// ```ignore
/// use zenoh_proto::*;
///
/// // Unit extension (flag only)
/// #[derive(ZExt, Debug, PartialEq)]
/// pub struct HasQoS {}
///
/// // U64 extension (single integer)
/// #[derive(ZExt, Debug, PartialEq)]
/// pub struct Timestamp {
///     pub value: u64,
/// }
///
/// // ZStruct extension (complex structure)
/// #[derive(ZExt, Debug, PartialEq)]
/// pub struct SourceInfo {
///     pub id: EntityGlobalId,
///     pub sn: u32,
/// }
/// ```
#[proc_macro_derive(ZExt, attributes(zenoh))]
pub fn derive_zext(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    codec::ext::derive_zext(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

/// Derives Zenoh protocol codec traits for an enum with structured variants.
///
/// This macro implements codec traits for enums where each variant may contain
/// associated data. Each variant is encoded with a discriminant followed by
/// its data (if any).
///
/// # Examples
///
/// ```ignore
/// use zenoh_proto::*;
///
/// #[derive(ZEnum, Debug, PartialEq)]
/// pub enum Declaration {
///     DeclareKeyExpr(DeclareKeyExpr),
///     UndeclareKeyExpr(UndeclareKeyExpr),
///     DeclareSubscriber(DeclareSubscriber),
/// }
/// ```
#[proc_macro_derive(ZEnum)]
pub fn derive_zenum(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    codec::r#enum::derive_zenum(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

/// Derives Zenoh protocol codec traits for a `repr(u8)` enum.
///
/// This macro is used for simple enums that are represented as a single byte.
/// The enum must have `#[repr(u8)]` and each variant must have an explicit
/// discriminant value.
///
/// # Examples
///
/// ```ignore
/// use zenoh_proto::*;
///
/// #[repr(u8)]
/// #[derive(ZRU8, Debug, Clone, Copy, PartialEq)]
/// pub enum Priority {
///     Control = 0,
///     RealTime = 1,
///     InteractiveHigh = 2,
///     InteractiveLow = 3,
///     DataHigh = 4,
///     Data = 5,
///     DataLow = 6,
///     Background = 7,
/// }
/// ```
#[proc_macro_derive(ZRU8)]
pub fn derive_zru8(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    codec::ru8::derive_zru8(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

#[proc_macro]
pub fn declare_zerror(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as zerror::model::DeclaredErrors);

    zerror::declare_zerror(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

#[proc_macro_derive(ZLinkInfo)]
pub fn derive_zlink_info(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    link::info::derive_zlink_info(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

#[proc_macro_derive(ZLinkTx)]
pub fn derive_zlink_tx(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    link::tx::derive_zlink_tx(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

#[proc_macro_derive(ZLinkRx)]
pub fn derive_zlink_rx(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    link::rx::derive_zlink_rx(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}

#[proc_macro_derive(ZLink, attributes(zenoh))]
pub fn derive_zlink(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);

    link::derive_zlink(&input)
        .unwrap_or_else(|err| err.to_compile_error())
        .into()
}
