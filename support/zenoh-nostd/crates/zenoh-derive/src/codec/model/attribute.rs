//! # Attribute Parsing
//!
//! This module parses `#[zenoh(...)]` attributes from struct fields.
//!
//! ## Supported Attributes
//!
//! - `flatten` - Inline field's encoding into parent struct
//! - `shift = N` - Bit-shift encoded value by N positions
//! - `size = prefixed|remain|header(FIELD)` - Size encoding strategy
//! - `maybe_empty` - Allow empty collections
//! - `mandatory` - Mark extension as mandatory
//! - `presence = prefixed|header(FIELD)` - Optional field presence indicator
//! - `header = FIELD` - Read value from header field
//! - `ext = ID` - Mark as protocol extension with given ID
//! - `default = expr` - Default value when absent
//!
//! ## Parsing Process
//!
//! 1. Iterate through all attributes on the field
//! 2. Find attributes named "zenoh"
//! 3. Parse nested meta items (the content inside `zenoh(...)`)
//! 4. Build a `ZenohAttribute` struct with all parsed values
//! 5. Validate compatibility in the type checker
//!
//! ## Examples
//!
//! ```ignore
//! #[zenoh(presence = header(P), default = 100)]
//! pub timeout: Option<u32>,
//!
//! #[zenoh(ext = 0x1, mandatory)]
//! pub metadata: Option<Metadata>,
//!
//! #[zenoh(size = remain)]
//! pub payload: &'a [u8],
//! ```

use proc_macro2::{Span, TokenStream};
use syn::{Expr, Ident, meta::ParseNestedMeta, parenthesized, spanned::Spanned};

/// Complete set of zenoh attributes parsed from a field.
///
/// This struct accumulates all `#[zenoh(...)]` attributes found on a field.
/// Multiple attributes can be combined in one `#[zenoh(...)]` or across
/// multiple `#[zenoh(...)]` annotations.
#[derive(Clone)]
pub struct ZenohAttribute {
    /// Source span for error reporting
    pub span: Span,

    /// `flatten` - Inline field into parent encoding
    pub flatten: bool,
    /// `shift = N` - Bit-shift encoded value
    pub shift: Option<usize>,
    /// `size = strategy` - How to encode collection size
    pub size: SizeAttribute,
    /// `maybe_empty` - Allow empty collections
    pub maybe_empty: bool,
    /// `mandatory` - Extension must be understood
    pub mandatory: bool,
    /// `presence = strategy` - Optional field presence
    pub presence: PresenceAttribute,
    /// `header = FIELD` - Read from header field
    pub header: HeaderAttribute,
    /// `ext = ID` - Protocol extension ID (0-15)
    pub ext: ExtAttribute,
    /// `default = expr` - Default value
    pub default: DefaultAttribute,
}

impl Default for ZenohAttribute {
    fn default() -> Self {
        ZenohAttribute {
            span: Span::call_site(),
            flatten: false,
            shift: None,
            size: SizeAttribute::default(),
            maybe_empty: false,
            mandatory: false,
            presence: PresenceAttribute::default(),
            header: HeaderAttribute::default(),
            ext: ExtAttribute::default(),
            default: DefaultAttribute::default(),
        }
    }
}

impl ZenohAttribute {
    /// Parses all zenoh attributes from a field.
    ///
    /// This function:
    /// 1. Iterates through all attributes on the field
    /// 2. Finds attributes named "zenoh"
    /// 3. Parses the nested content inside `#[zenoh(...)]`
    /// 4. Accumulates all parsed attributes into a single struct
    ///
    /// # Arguments
    ///
    /// * `field` - The syn field to parse attributes from
    ///
    /// # Returns
    ///
    /// A `ZenohAttribute` containing all parsed zenoh attributes
    ///
    /// # Errors
    ///
    /// Returns `syn::Error` if attribute syntax is invalid
    ///
    /// # Example Attributes
    ///
    /// ```ignore
    /// #[zenoh(presence = header(P))]
    /// #[zenoh(default = 100)]
    /// pub field: Option<u32>
    /// ```
    pub fn from_field(field: &syn::Field) -> syn::Result<Self> {
        let mut zattr = ZenohAttribute {
            span: field.ident.span(),
            ..Default::default()
        };

        // Iterate through all attributes on the field
        for attr in &field.attrs {
            // Look for #[zenoh(...)] attributes
            if attr.path().is_ident("zenoh") {
                // Parse the nested meta items inside zenoh(...)
                attr.parse_nested_meta(|meta| {
                    // Try to parse each possible attribute type
                    let size = SizeAttribute::from_meta(&meta)?;
                    let flatten = flatten_from_meta(&meta)?;
                    let shift = shift_from_meta(&meta)?;
                    let maybe_empty = maybe_empty_from_meta(&meta)?;
                    let mandatory = mandatory_from_meta(&meta)?;
                    let presence = PresenceAttribute::from_meta(&meta)?;
                    let header = HeaderAttribute::from_meta(&meta)?;
                    let default = DefaultAttribute::from_meta(&meta)?;
                    let ext = ExtAttribute::from_meta(&meta)?;

                    // Accumulate non-None values into zattr
                    if !matches!(size, SizeAttribute::None) {
                        zattr.size = size;
                    }
                    if flatten {
                        zattr.flatten = true;
                    }
                    if let Some(shift) = shift {
                        zattr.shift = Some(shift);
                    }
                    if maybe_empty {
                        zattr.maybe_empty = true;
                    }
                    if mandatory {
                        zattr.mandatory = true;
                    }
                    if !matches!(presence, PresenceAttribute::None) {
                        zattr.presence = presence;
                    }
                    if !matches!(header, HeaderAttribute::None) {
                        zattr.header = header;
                    }
                    if !matches!(ext, ExtAttribute::None) {
                        zattr.ext = ext;
                    }
                    if !matches!(default, DefaultAttribute::None) {
                        zattr.default = default;
                    }

                    Ok(())
                })?;
            }
        }

        Ok(zattr)
    }
}

/// Parses the `flatten` attribute.
///
/// Syntax: `#[zenoh(flatten)]`
///
/// When present, the field's encoding is inlined into the parent struct
/// instead of being encoded as a separate nested structure.
fn flatten_from_meta(meta: &ParseNestedMeta) -> syn::Result<bool> {
    if meta.path.is_ident("flatten") {
        return Ok(true);
    }

    Ok(false)
}

/// Parses the `shift = N` attribute.
///
/// Syntax: `#[zenoh(shift = 3)]`
///
/// The encoded value is bit-shifted left by N positions. This is useful
/// for packing multiple small values into a single byte when flattening.
fn shift_from_meta(meta: &ParseNestedMeta) -> syn::Result<Option<usize>> {
    if meta.path.is_ident("shift") {
        let value = meta.value()?;
        let shift: syn::LitInt = value.parse()?;
        return Ok(Some(shift.base10_parse()?));
    }

    Ok(None)
}

fn maybe_empty_from_meta(meta: &ParseNestedMeta) -> syn::Result<bool> {
    if meta.path.is_ident("maybe_empty") {
        return Ok(true);
    }

    Ok(false)
}

/// Parses the `mandatory` attribute.
///
/// Syntax: `#[zenoh(mandatory)]`
///
/// Marks a protocol extension as mandatory. Decoders that don't understand
/// this extension must fail instead of skipping it. Used with `ext`.
fn mandatory_from_meta(meta: &ParseNestedMeta) -> syn::Result<bool> {
    if meta.path.is_ident("mandatory") {
        return Ok(true);
    }

    Ok(false)
}

fn ident_to_header_path(ident: &Ident) -> TokenStream {
    let ident = syn::Ident::new(&format!("HEADER_SLOT_{}", ident), ident.span());
    quote::quote! { Self::#ident }
}

/// Specifies how the size of a collection is encoded.
///
/// Different strategies for encoding variable-length data:
///
/// - `None`: No size attribute (not applicable or fixed-size)
/// - `Prefixed`: Size is encoded as a VLE integer before the data
/// - `Remain`: Use all remaining bytes in the buffer (for last field only)
/// - `Header(FIELD)`: Size comes from a field in the header byte
#[derive(Clone, Default)]
pub enum SizeAttribute {
    /// No size attribute specified
    #[default]
    None,
    /// `size = prefixed` - Length prefix before data
    Prefixed,
    /// `size = remain` - Use remaining buffer (last field only)
    Remain,
    /// `size = header(FIELD)` - Size from header field
    Header(TokenStream),
}

impl SizeAttribute {
    /// Parses the `size` attribute.
    ///
    /// Syntax:
    /// - `#[zenoh(size = prefixed)]` - VLE length prefix
    /// - `#[zenoh(size = remain)]` - Use remaining buffer
    /// - `#[zenoh(size = header(FIELD))]` - Size from header field
    ///
    /// # Examples
    ///
    /// ```ignore
    /// #[zenoh(size = prefixed)]
    /// pub data: &'a [u8],
    ///
    /// #[zenoh(size = header(LEN))]  // LEN from header
    /// pub payload: &'a [u8],
    ///
    /// #[zenoh(size = remain)]  // Last field uses rest of buffer
    /// pub body: &'a [u8],
    /// ```
    fn from_meta(meta: &ParseNestedMeta) -> syn::Result<Self> {
        if meta.path.is_ident("size") {
            let value = meta.value()?;
            let size: syn::Ident = value.parse()?;
            if size == "prefixed" {
                return Ok(SizeAttribute::Prefixed);
            } else if size == "remain" {
                return Ok(SizeAttribute::Remain);
            } else if size == "header" {
                // Parse header(FIELD) syntax
                let content;
                parenthesized!(content in value);
                let ident: Ident = content.parse()?;
                return Ok(SizeAttribute::Header(ident_to_header_path(&ident)));
            } else {
                return Err(syn::Error::new_spanned(
                    size,
                    "Invalid size attribute value (expected: prefixed, remain, or header)",
                ));
            }
        }

        Ok(SizeAttribute::None)
    }
}

/// Specifies how the presence of an optional field is indicated.
///
/// Optional fields (Option<T>) need a way to indicate whether they're present:
///
/// - `Prefixed`: Boolean byte before the field data
/// - `Header(FIELD)`: Presence bit in the header byte
#[derive(Clone, Default)]
pub enum PresenceAttribute {
    #[default]
    None,
    /// `presence = prefixed` - Boolean byte before field
    Prefixed,
    /// `presence = header(FIELD)` - Bit in header byte
    Header(TokenStream),
}

impl PresenceAttribute {
    /// Parses the `presence` attribute.
    ///
    /// Syntax:
    /// - `#[zenoh(presence = prefixed)]` - Boolean byte before field
    /// - `#[zenoh(presence = header(FIELD))]` - Bit in header byte
    ///
    /// # Examples
    ///
    /// ```ignore
    /// #[zenoh(presence = prefixed)]
    /// pub optional: Option<u64>,
    ///
    /// // With header format "#[zenoh(header = "P|_:7")]"
    /// #[zenoh(presence = header(P))]
    /// pub maybe_present: Option<u32>,
    /// ```
    fn from_meta(meta: &ParseNestedMeta) -> syn::Result<Self> {
        if meta.path.is_ident("presence") {
            let value = meta.value()?;
            let presence: syn::Ident = value.parse()?;
            if presence == "prefixed" {
                return Ok(PresenceAttribute::Prefixed);
            } else if presence == "header" {
                // Parse header(FIELD) syntax
                let content;
                parenthesized!(content in value);
                let ident: Ident = content.parse()?;
                return Ok(PresenceAttribute::Header(ident_to_header_path(&ident)));
            } else {
                return Err(syn::Error::new_spanned(
                    presence,
                    "Invalid presence attribute value (expected: prefixed or header)",
                ));
            }
        }

        Ok(PresenceAttribute::None)
    }
}

/// Specifies that a field's value comes from the header byte.
///
/// Instead of being encoded in the body, the field's value is read from
/// a specific field in the header byte.
///
/// - `Slot(FIELD)`: Field value comes from header field
#[derive(Clone, Default)]
pub enum HeaderAttribute {
    #[default]
    None,
    /// `header = FIELD` - Value from header field
    Slot(TokenStream),
}

impl HeaderAttribute {
    /// Parses the `header` attribute.
    ///
    /// Syntax: `#[zenoh(header = FIELD)]`
    ///
    /// The field's value is read from a field in the header byte instead
    /// of being encoded in the body.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // With header format "#[zenoh(header = "F|_:7")]"
    /// #[zenoh(header = F)]
    /// pub flag: bool,  // Value comes from F bit in header
    /// ```
    fn from_meta(meta: &ParseNestedMeta) -> syn::Result<Self> {
        if meta.path.is_ident("header") {
            let ident: Ident = meta.value()?.parse()?;
            return Ok(HeaderAttribute::Slot(ident_to_header_path(&ident)));
        }

        Ok(HeaderAttribute::None)
    }
}

/// Marks a field as a protocol extension with a specific ID.
///
/// Extensions are optional, versioned additions to protocol messages.
/// Each extension has an ID (0-15) and is encoded with special headers
/// to maintain backward compatibility.
///
/// - `Expr(id)`: Extension with given ID
#[derive(Clone, Default)]
pub enum ExtAttribute {
    #[default]
    None,
    /// `ext = ID` - Extension with ID (0-15)
    Expr(Expr),
}

impl ExtAttribute {
    /// Parses the `ext` attribute.
    ///
    /// Syntax: `#[zenoh(ext = ID)]` where ID is 0-15
    ///
    /// Marks a field as a protocol extension.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// #[zenoh(ext = 0x1)]
    /// pub metadata: Option<Metadata>,
    ///
    /// #[zenoh(ext = 0x2, mandatory)]
    /// pub version: Option<Version>,
    ///
    /// #[zenoh(ext = 0x3, default = false)]
    /// pub flag: bool,
    /// ```
    fn from_meta(meta: &ParseNestedMeta) -> syn::Result<Self> {
        if meta.path.is_ident("ext") {
            let expr: Expr = meta.value()?.parse()?;
            return Ok(ExtAttribute::Expr(expr));
        }

        Ok(ExtAttribute::None)
    }
}

/// Specifies a default value for optional fields.
///
/// When a field is absent (due to `presence` or `ext`), the default
/// value is used instead.
///
/// - `Expr(expr)`: Default value expression
#[derive(Clone, Default)]
pub enum DefaultAttribute {
    #[default]
    None,
    /// `default = expr` - Default value expression
    Expr(Expr),
}

impl DefaultAttribute {
    /// Parses the `default` attribute.
    ///
    /// Syntax: `#[zenoh(default = expr)]`
    ///
    /// Provides a default value when the field is absent. Used with
    /// `presence` for optional fields or `ext` for extensions.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// #[zenoh(presence = header(P), default = 1000)]
    /// pub timeout: Option<u32>,
    ///
    /// #[zenoh(ext = 0x1, default = QoS::default())]
    /// pub qos: QoS,
    ///
    /// #[zenoh(ext = 0x2, default = false)]
    /// pub ack_required: bool,
    /// ```
    fn from_meta(meta: &ParseNestedMeta) -> syn::Result<Self> {
        if meta.path.is_ident("default") {
            let expr: Expr = meta.value()?.parse()?;
            return Ok(DefaultAttribute::Expr(expr));
        }

        Ok(DefaultAttribute::None)
    }
}
