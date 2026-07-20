//! # Header Implementation Generator
//!
//! This module generates constant definitions from the header format string.
//!
//! ## Purpose
//!
//! Given a header format like `"Z|E|I|ID:5=0x01"`, this module:
//! 1. Parses the format string into individual fields
//! 2. Assigns bit positions to each field (working right-to-left)
//! 3. Generates constants for accessing these fields
//! 4. Calculates the base header value from constant fields
//!
//! ## Generated Constants
//!
//! For each field in the header, constants are generated:
//!
//! - **Single-bit flags** (e.g., `Z`, `E`, `I`):
//!   - `HEADER_SLOT_Z: u8 = 0b10000000` (bit mask)
//!
//! - **Multi-bit dynamic fields** (e.g., `ID:5`):
//!   - `HEADER_SLOT_ID: u8 = 0b00011111` (bit mask for 5 bits)
//!
//! - **Multi-bit constant fields** (e.g., `ID:5=0x01`):
//!   - `ID: u8 = 0x01` (the constant value)
//!   - `HEADER_SLOT_ID: u8 = 0b00011111 << shift` (bit mask)
//!
//! - **Base header** (`HEADER_BASE`):
//!   - Combination of all constant field values shifted to their positions
//!
//! ## Header Format Syntax
//!
//! - `FIELD` - Single-bit boolean (generates `HEADER_SLOT_FIELD`)
//! - `_` - Reserved/unused bit (no constant generated)
//! - `FIELD:N` - N-bit dynamic field (generates `HEADER_SLOT_FIELD`)
//! - `FIELD:N=VALUE` - N-bit constant field (generates `FIELD` and `HEADER_SLOT_FIELD`)
//! - `|` - Visual separator (ignored)
//!
//! ## Bit Allocation
//!
//! Bits are allocated from left to right (MSB to LSB):
//! - Start with shift=8 (bit 7, the MSB)
//! - For each field, decrement shift by the field's width
//! - Must use exactly 8 bits total
//!
//! ## Example
//!
//! Input: `#[zenoh(header = "Z|E|_|ID:5=0x1e")]`
//!
//! Generated:
//! ```ignore
//! impl MyStruct {
//!     const HEADER_BASE: u8 = 0b00011110;  // 0x1e shifted to bits 0-4
//!     const HEADER_SLOT_FULL: u8 = 0b11111111;
//!     const HEADER_SLOT_Z: u8 = 0b10000000;  // bit 7
//!     const HEADER_SLOT_E: u8 = 0b01000000;  // bit 6
//!     // bit 5 is unused (_)
//!     pub const ID: u8 = 0x1e;
//!     const HEADER_SLOT_ID: u8 = 0b00011111;  // bits 0-4
//! }
//! ```

use proc_macro2::{Span, TokenStream};
use syn::Ident;

use crate::codec::model::ZenohStruct;

pub fn parse(r#struct: &ZenohStruct) -> syn::Result<TokenStream> {
    if let Some(header) = &r#struct.header {
        let ident = &r#struct.ident;
        let generics = &r#struct.generics;
        let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

        let mut shift = 8u8;
        let content = header.expr.value();
        let mut const_defs = Vec::new();

        // Collect constant field values to compute base header
        let mut base_header = Vec::new();

        // Parse each part of the header format (separated by |)
        for part in content.split('|') {
            let part = part.trim();
            if part.is_empty() {
                continue; // Skip empty parts (e.g., between "||")
            }
            // Parse this field and generate its constants
            const_defs.push(parse_part(
                part,
                &mut shift,
                &mut base_header,
                header.expr.span(),
            )?);
        }

        // Compute HEADER_BASE by OR-ing all constant field values
        let base_header = if base_header.is_empty() {
            quote::quote! { 0u8 }
        } else {
            // Combine all constant values: (val1 << shift1) | (val2 << shift2) | ...
            base_header
                .into_iter()
                .reduce(|acc, expr| {
                    quote::quote! { (#acc) | (#expr) }
                })
                .unwrap()
        };

        // Verify that all 8 bits have been allocated
        if shift != 0 {
            return Err(syn::Error::new(
                header.expr.span(),
                "Header declaration does not use all 8 bits",
            ));
        }

        Ok(quote::quote! {
            impl #impl_generics #ident #ty_generics #where_clause {
                const HEADER_BASE: u8 = #base_header;
                const HEADER_SLOT_FULL: u8 = 0b1111_1111;

                #(#const_defs)*
            }
        })
    } else {
        Ok(quote::quote! {})
    }
}

fn parse_part(
    part: &str,
    shift: &mut u8,
    base_header: &mut Vec<TokenStream>,
    span: Span,
) -> syn::Result<TokenStream> {
    // Special case: single underscore (1 unused bit)
    if part == "_" {
        *shift = shift.saturating_sub(1);
        return Ok(quote::quote! {});
    }

    // Special validation: 'Z' must be the first field (MSB)
    if part == "Z" && *shift != 8 {
        return Err(syn::Error::new(
            span,
            "The special 'Z' placeholder must be the first part in header declaration (bit 7)",
        ));
    }

    // Parse field specification: "FIELD:SIZE=VALUE" or "FIELD:SIZE" or "FIELD"
    let mut split = part.split('=');
    let left = split.next().unwrap(); // Everything before '='
    let value_opt = split.next(); // Optional value after '='

    // Parse left side: "FIELD:SIZE" or "FIELD"
    let mut left_split = left.split(':');
    let name_str = left_split.next().unwrap();
    let size_opt = left_split.next();
    let name = Ident::new(name_str, proc_macro2::Span::call_site());

    if let Some(size_str) = size_opt {
        let size: u8 = size_str.parse().map_err(|_| {
            syn::Error::new(
                span,
                format!("Invalid size '{}' in header declaration", size_str),
            )
        })?;
        if let Some(value_str) = value_opt {
            // Constant field: "FIELD:N=VALUE"
            // Parse the value (hex or decimal)
            let value: u8 = if let Some(stripped) = value_str.strip_prefix("0x") {
                u8::from_str_radix(stripped, 16).map_err(|_| {
                    syn::Error::new(
                        span,
                        format!("Invalid hex value '{}' in header declaration", value_str),
                    )
                })?
            } else {
                value_str.parse().map_err(|_| {
                    syn::Error::new(
                        span,
                        format!("Invalid value '{}' in header declaration", value_str),
                    )
                })?
            };

            // Generate bit mask: ((1 << size) - 1) creates a mask of 'size' bits
            // Example: size=5 -> (1 << 5) - 1 = 0b11111
            let x = syn::LitInt::new(&format!("0b{:b}", (1 << size) - 1), Span::call_site());
            let y = *shift - size;

            let slot = quote::quote! { #x << #y };
            let shifted_value = quote::quote! { #value << #y };
            let value = quote::quote! { #value };

            *shift = shift.checked_sub(size).ok_or_else(|| {
                syn::Error::new(span, "Not enough bits left in header declaration")
            })?;

            // Add to base header calculation
            base_header.push(shifted_value);

            // Skip constant generation for underscore fields
            if name == "_" {
                return Ok(quote::quote! {});
            }

            // Generate constants
            let name_slot = Ident::new(&format!("HEADER_SLOT_{}", name_str), Span::call_site());
            Ok(quote::quote! {
                pub const #name: u8 = #value;
                const #name_slot: u8 = #slot;
            })
        } else {
            let x = syn::LitInt::new(&format!("0b{:b}", (1 << size) - 1), Span::call_site());
            let y = *shift - size;

            *shift = shift.checked_sub(size).ok_or_else(|| {
                syn::Error::new(span, "Not enough bits left in header declaration")
            })?;

            if name == "_" {
                return Ok(quote::quote! {});
            }

            let name = Ident::new(&format!("HEADER_SLOT_{}", name_str), Span::call_site());
            Ok(quote::quote! {
                const #name: u8 = #x << #y;
            })
        }
    } else if value_opt.is_some() {
        Err(syn::Error::new(
            span,
            "Affectation without size is not allowed in header declaration",
        ))
    } else {
        *shift = shift
            .checked_sub(1)
            .ok_or_else(|| syn::Error::new(span, "Not enough bits left in header declaration"))?;

        let name = Ident::new(&format!("HEADER_SLOT_{}", name_str), Span::call_site());
        Ok(quote::quote! {
            const #name: u8 = 0b1 << #shift;
        })
    }
}
