//! # Internal Model Representation
//!
//! This module defines the internal representation of structs after parsing.
//! It transforms `syn` AST types into our domain-specific types that are
//! easier to work with during code generation.
//!
//! ## Main Types
//!
//! - [`ZenohStruct`]: Complete representation of a `#[derive(ZStruct)]` struct
//! - [`ZenohField`]: Enum representing either a regular field or extension block
//! - [`ZenohFieldInner`]: Parsed field with attributes and type information
//! - [`HeaderDeclaration`]: Parsed header format string

use proc_macro2::TokenStream;
use syn::{DeriveInput, Generics, Ident, LitStr};

use crate::codec::model::{
    attribute::{ExtAttribute, ZenohAttribute},
    ty::ZenohType,
};

pub mod attribute;
pub mod ty;

/// Internal representation of a single field after parsing.
///
/// Contains all information needed to generate codec code for this field:
/// - Parsed attributes (`presence`, `ext`, `size`, etc.)
/// - Analyzed type information
/// - How to access the field in generated code
pub struct ZenohFieldInner {
    pub attr: ZenohAttribute,
    pub ty: ZenohType,
    pub access: TokenStream,
}

impl ZenohFieldInner {
    pub fn from_field(field: &syn::Field) -> syn::Result<Self> {
        let attr = ZenohAttribute::from_field(field)?;

        let ident = field
            .ident
            .as_ref()
            .ok_or_else(|| syn::Error::new_spanned(field, "Expected named field"))?;

        let access = quote::quote! { #ident };

        let ty = ZenohType::from_type(&field.ty)?;
        ty.check_attribute(&attr)?;

        Ok(Self { attr, access, ty })
    }
}

/// Represents a parsed header declaration from `#[zenoh(header = "...")]`
pub struct HeaderDeclaration {
    /// The header format string literal (e.g., "Z|E|ID:5=0x01")
    pub expr: LitStr,
}

/// Represents a field or group of extension fields in the struct.
///
/// Fields are classified into two categories:
/// - Regular fields: Normal struct fields without `ext` attribute
/// - Extension block: Contiguous group of fields with `ext` attribute
///
/// Extension fields must be grouped together in the struct definition,
/// and this is enforced during parsing.
pub enum ZenohField {
    Regular {
        field: Box<ZenohFieldInner>,
    },
    /// A block of extension fields (must be contiguous)
    ExtBlock {
        exts: Vec<ZenohFieldInner>,
    },
}

/// Complete internal representation of a struct for `#[derive(ZStruct)]`.
///
/// This is the main data structure used throughout code generation.
/// It contains all information extracted from the input struct.
pub struct ZenohStruct {
    pub ident: Ident,
    pub generics: Generics,
    pub header: Option<HeaderDeclaration>,
    pub fields: Vec<ZenohField>,
}

impl ZenohStruct {
    /// Parses a [`DeriveInput`] into a [`ZenohStruct`].
    ///
    /// This method extracts struct-level attributes (like `header`), processes each field,
    /// and builds the internal representation used for code generation.
    ///
    /// # Errors
    ///
    /// Returns a [`syn::Error`] if:
    /// - The input is not a struct
    /// - Attributes are malformed
    /// - Field types cannot be analyzed
    pub(crate) fn from_derive_input(input: &DeriveInput) -> syn::Result<Self> {
        let fields = match &input.data {
            syn::Data::Struct(data_struct) => &data_struct.fields,
            _ => {
                return Err(syn::Error::new_spanned(
                    input,
                    "ZStruct can only be derived for structs",
                ));
            }
        };

        let mut fields_vec = Vec::new();
        let mut found_ext_block = false;
        let mut in_ext_block = false;
        for field in fields {
            let field = ZenohFieldInner::from_field(field)?;
            let is_ext = !matches!(field.attr.ext, ExtAttribute::None);

            if is_ext {
                if !found_ext_block {
                    found_ext_block = true;
                    in_ext_block = true;
                    fields_vec.push(ZenohField::ExtBlock { exts: vec![] });
                } else if !in_ext_block {
                    return Err(syn::Error::new_spanned(
                        field.access.clone(),
                        "Fields with 'ext' attribute must be grouped in a single contiguous block",
                    ));
                }
            } else {
                in_ext_block = false;
            }

            if is_ext {
                match fields_vec
                    .last_mut()
                    .expect("Expected ext block, something went wrong")
                {
                    ZenohField::ExtBlock { exts } => {
                        exts.push(field);
                    }
                    _ => unreachable!("Expected ext block, something went wrong"),
                }
            } else {
                fields_vec.push(ZenohField::Regular {
                    field: Box::new(field),
                });
            }
        }

        let mut header = Option::<HeaderDeclaration>::None;

        for attr in &input.attrs {
            if attr.path().is_ident("zenoh") {
                attr.parse_nested_meta(|meta| {
                    if meta.path.is_ident("header") {
                        let value = meta.value()?;
                        let expr: LitStr = value.parse()?;
                        header.replace(HeaderDeclaration { expr });
                    }

                    Ok(())
                })?;
            }
        }

        Ok(Self {
            ident: input.ident.clone(),
            generics: input.generics.clone(),
            header,
            fields: fields_vec,
        })
    }
}
