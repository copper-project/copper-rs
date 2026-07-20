//! # Type Analysis and Validation
//!
//! This module analyzes Rust types and validates that zenoh attributes are
//! compatible with each type. It transforms `syn::Type` into our internal
//! `ZenohType` representation.
//!
//! ## Type Classification
//!
//! Types are classified into categories, each with different codec requirements:
//!
//! - **Primitive integers**: `u8`, `u16`, `u32`, `u64`, `usize`
//! - **Byte containers**: `[u8; N]`, `&[u8]`
//! - **Strings**: `&str`
//! - **Structured types**: Any other type (assumed to implement codec traits)
//! - **Optional types**: `Option<T>` (wraps any of the above)
//!
//! ## Attribute Validation
//!
//! Each type has restrictions on which attributes can be used:
//!
//! - `u8`, `u16`, `u32`, `u64`, `usize`: Limited attributes (no size, no flatten)
//! - `&[u8]`, `&str`: Require size attribute
//! - `[u8; N]`: Fixed size (no size attribute needed)
//! - `Option<T>`: Requires presence or ext attribute
//! - Structured types: Most flexible, but with logical constraints

use syn::{Type, TypeArray, TypeReference};

use crate::codec::model::attribute::{
    DefaultAttribute, ExtAttribute, HeaderAttribute, PresenceAttribute, SizeAttribute,
    ZenohAttribute,
};

/// Internal representation of a field's type after analysis.
///
/// This enum classifies types into categories that determine:
/// - How they are encoded/decoded
/// - Which attributes are valid
/// - Whether they need size prefixes
pub enum ZenohType {
    U8,
    U16,
    U32,
    U64,
    USize,

    ByteArray,

    ByteSlice,
    Str,

    ZStruct,

    Option(Box<ZenohType>),
}

impl ZenohType {
    /// Validates that the given attributes are compatible with this type.
    ///
    /// This performs comprehensive validation to catch invalid attribute combinations
    /// at compile time (during macro expansion).
    ///
    /// # Arguments
    ///
    /// * `attr` - The zenoh attributes to validate
    ///
    /// # Returns
    ///
    /// `Ok(())` if attributes are valid for this type, `Err` otherwise
    ///
    /// # Validation Rules
    ///
    /// Each type has specific rules about which attributes can be used.
    /// See the implementation for detailed rules per type.
    pub fn check_attribute(&self, attr: &ZenohAttribute) -> syn::Result<()> {
        // Extract which attributes are present (for readability below)
        let (s, f, sh, me, m, p, h, e, d) = (
            !matches!(attr.size, SizeAttribute::None),
            attr.flatten,
            attr.shift.is_some(),
            attr.maybe_empty,
            attr.mandatory,
            !matches!(attr.presence, PresenceAttribute::None),
            !matches!(attr.header, HeaderAttribute::None),
            !matches!(attr.ext, ExtAttribute::None),
            !matches!(attr.default, DefaultAttribute::None),
        );

        match self {
            // u8: Single byte encoding
            // - Can have: presence, default
            // - Cannot have: flatten, shift, size, maybe_empty, mandatory, ext
            // - Requires: if default, must have presence
            ZenohType::U8 => {
                if f || sh || s || me || m || e {
                    return Err(syn::Error::new(
                        attr.span,
                        "u8 type does not support flatten, shift, size, maybe_empty, mandatory, presence or ext attributes",
                    ));
                }
                if d && !p || p && !d {
                    return Err(syn::Error::new(
                        attr.span,
                        "types with default attribute requires a presence attribute",
                    ));
                }
                Ok(())
            }
            // Integer types (u16, u32, u64, usize) and fixed-size arrays [u8; N]
            // - Can have: presence, default
            // - Cannot have: flatten, shift, size, maybe_empty, mandatory, header, ext
            // - Requires: if default, must have presence
            ZenohType::U16
            | ZenohType::U32
            | ZenohType::U64
            | ZenohType::USize
            | ZenohType::ByteArray => {
                if f || sh || s || me || m || h || e {
                    return Err(syn::Error::new(
                        attr.span,
                        "u16, u32, u64, usize and [u8; N] types do not support flatten, shift, size, maybe_empty, mandatory, header, or ext attributes",
                    ));
                }
                if d && !p || p && !d {
                    return Err(syn::Error::new(
                        attr.span,
                        "types with default attribute requires a presence attribute",
                    ));
                }
                Ok(())
            }
            // Byte slices (&[u8]) and strings (&str)
            // - Can have: size (REQUIRED), presence, maybe_empty
            // - Cannot have: flatten, shift, mandatory, header, ext, default
            // - Requires: MUST have size attribute
            ZenohType::ByteSlice | ZenohType::Str => {
                if f || sh || m || h || e {
                    return Err(syn::Error::new(
                        attr.span,
                        "string and byte slice types do not support flatten, shift, mandatory, header, ext, or default attributes",
                    ));
                }
                if d && !p || p && !d {
                    return Err(syn::Error::new(
                        attr.span,
                        "types with default attribute requires a presence attribute",
                    ));
                }
                if !s {
                    return Err(syn::Error::new(
                        attr.span,
                        "string and byte slice types require a size attribute",
                    ));
                }
                Ok(())
            }
            // Structured types (any type implementing codec traits)
            // Complex validation rules due to flexibility:
            // - Can be regular fields, flattened fields, or extensions
            // - Various attribute combinations allowed depending on usage
            ZenohType::ZStruct => {
                // If has presence, must have default (or be an extension)
                if p && !d || (d && !p && !e) {
                    return Err(syn::Error::new(
                        attr.span,
                        "ZStruct types with default attribute requires a presence attribute",
                    ));
                }
                // If has default without ext, must have presence
                if d && !e && !p {
                    return Err(syn::Error::new(
                        attr.span,
                        "structs with default attribute requires an ext attribute",
                    ));
                }
                // Extensions must have default values
                if e && !d {
                    return Err(syn::Error::new(
                        attr.span,
                        "ZStruct type with ext attribute requires a default attribute",
                    ));
                }
                // Extensions cannot have size attribute (size is part of ext encoding)
                // Extensions encode their own size
                if e && s {
                    return Err(syn::Error::new(
                        attr.span,
                        "ZStruct type that are extensions cannot have a size attribute",
                    ));
                }
                // Cannot both read from header and flatten
                if h && f {
                    return Err(syn::Error::new(
                        attr.span,
                        "ZStruct type with header attribute cannot be flattened",
                    ));
                }
                // Shift only makes sense for flattened fields
                if sh && !f {
                    return Err(syn::Error::new(
                        attr.span,
                        "ZStruct type with shift attribute must be flattened",
                    ));
                }
                Ok(())
            }
            // Option<T>: Wraps another type in optional encoding
            // - Cannot have: default (Option itself is the optionality), header
            // - Must have: EITHER presence OR ext (one way to know if present)
            // - Cannot have: both presence AND ext (only one optionality mechanism)
            // - Validates inner type with adjusted attributes
            ZenohType::Option(inner_ty) => {
                if d || h {
                    return Err(syn::Error::new(
                        attr.span,
                        "Option type does not support default or header attributes",
                    ));
                }

                // Must have either presence or ext to know if value is present
                if !e && !p {
                    return Err(syn::Error::new(
                        attr.span,
                        "Option type that are not extensions must have a presence attribute",
                    ));
                }

                // Cannot have both presence and ext (conflicting optionality)
                if e && p {
                    return Err(syn::Error::new(
                        attr.span,
                        "Option type that are extensions cannot have a presence attribute",
                    ));
                }

                if e && s {
                    return Err(syn::Error::new(
                        attr.span,
                        "Option type that are extensions cannot have a size attribute",
                    ));
                }

                // Recursively validate the inner type with appropriate attributes
                // Remove presence/ext/default since they apply to the Option itself
                let attr = ZenohAttribute {
                    size: attr.size.clone(),
                    flatten: attr.flatten,
                    shift: attr.shift,
                    maybe_empty: attr.maybe_empty,
                    mandatory: attr.mandatory,
                    presence: PresenceAttribute::None,
                    header: HeaderAttribute::None,
                    ext: ExtAttribute::None,
                    default: DefaultAttribute::None,
                    span: attr.span,
                };

                inner_ty.check_attribute(&attr)
            }
        }
    }

    /// Analyzes a `syn::Type` and classifies it into a `ZenohType`.
    ///
    /// This function recursively analyzes Rust types to determine:
    /// - What codec strategy to use
    /// - What attributes are valid
    ///
    /// # Arguments
    ///
    /// * `ty` - The syn type to analyze
    ///
    /// # Returns
    ///
    /// A `ZenohType` classification of the input type
    ///
    /// # Errors
    ///
    /// Returns `syn::Error` if the type is unsupported
    ///
    /// # Supported Types
    ///
    /// - Path types: `u8`, `u16`, `u32`, `u64`, `usize`, `Option<T>`, custom types
    /// - Reference types: `&str`, `&[u8]`
    /// - Array types: `[u8; N]`
    pub fn from_type(ty: &Type) -> syn::Result<Self> {
        match ty {
            Type::Path(type_path) => {
                if type_path.path.segments.first().unwrap().ident == "Option" {
                    if let syn::PathArguments::AngleBracketed(args) =
                        &type_path.path.segments[0].arguments
                        && args.args.len() == 1
                        && let syn::GenericArgument::Type(inner_ty) = &args.args[0]
                    {
                        let zenoh_type = ZenohType::from_type(inner_ty)?;
                        return Ok(ZenohType::Option(Box::new(zenoh_type)));
                    }
                    return Err(syn::Error::new_spanned(
                        ty,
                        "Option must have exactly one type argument",
                    ));
                }

                let ident = &type_path.path.segments.last().unwrap().ident;
                match ident.to_string().as_str() {
                    "u8" => Ok(ZenohType::U8),
                    "u16" => Ok(ZenohType::U16),
                    "u32" => Ok(ZenohType::U32),
                    "u64" => Ok(ZenohType::U64),
                    "usize" => Ok(ZenohType::USize),
                    _ => Ok(ZenohType::ZStruct),
                }
            }
            Type::Reference(TypeReference { elem, .. }) => match &**elem {
                Type::Path(type_path) => {
                    let ident = &type_path.path.segments.last().unwrap().ident;
                    if ident == "str" {
                        Ok(ZenohType::Str)
                    } else {
                        Err(syn::Error::new_spanned(ty, "Unsupported reference type"))
                    }
                }
                Type::Slice(syn::TypeSlice { elem, .. }) => match &**elem {
                    Type::Path(type_path) => {
                        let ident = &type_path.path.segments.last().unwrap().ident;
                        if ident == "u8" {
                            Ok(ZenohType::ByteSlice)
                        } else {
                            Err(syn::Error::new_spanned(
                                ty,
                                "Unsupported slice element type",
                            ))
                        }
                    }
                    _ => Err(syn::Error::new_spanned(
                        ty,
                        "Unsupported slice element type",
                    )),
                },
                _ => Err(syn::Error::new_spanned(ty, "Unsupported reference type")),
            },
            Type::Array(TypeArray { elem, .. }) => match &**elem {
                Type::Path(type_path) => {
                    let ident = &type_path.path.segments.last().unwrap().ident;
                    if ident == "u8" {
                        Ok(ZenohType::ByteArray)
                    } else {
                        Err(syn::Error::new_spanned(
                            ty,
                            "Unsupported array element type",
                        ))
                    }
                }
                _ => Err(syn::Error::new_spanned(
                    ty,
                    "Unsupported array element type",
                )),
            },
            _ => Err(syn::Error::new_spanned(ty, "Unsupported type")),
        }
    }
}
