//! # Header Runtime Computation Generator
//!
//! This module generates code that computes the header byte at runtime.
//!
//! ## Purpose
//!
//! While `header_impl.rs` generates constants from the header format string,
//! this module generates the actual code that computes the header byte value
//! when encoding a message.
//!
//! ## Generated Code Pattern
//!
//! The generated code follows this pattern:
//!
//! ```ignore
//! let Self { field1, field2, .. } = self;  // Destructure relevant fields
//! let mut header: u8 = Self::HEADER_BASE;  // Start with constant parts
//!
//! // Set presence flags
//! if field1.is_some() {
//!     header |= Self::HEADER_SLOT_FIELD1;
//! }
//!
//! // Set size fields
//! header |= {
//!     let shift = Self::HEADER_SLOT_SIZE.trailing_zeros();
//!     let len = field2.len() as u8;
//!     ((len - offset) << shift) & Self::HEADER_SLOT_SIZE
//! };
//!
//! // Set value fields
//! header |= {
//!     let v: u8 = (*field3).into();
//!     (v << shift) & Self::HEADER_SLOT_FIELD3
//! };
//!
//! // Set extension flag
//! header |= if self.z_ext_count() > 0 {
//!     Self::HEADER_SLOT_Z
//! } else {
//!     0
//! };
//!
//! header
//! ```
//!
//! ## Field Processing
//!
//! Different field attributes generate different header code:
//!
//! - **`presence = header(P)`**: Sets presence bit if field is present
//! - **`size = header(S)`**: Encodes size in header field
//! - **`header = H`**: Writes field value directly to header
//! - **`flatten`**: Combines nested struct's header into parent
//! - **Extension block**: Sets Z flag if extensions present

use proc_macro2::TokenStream;

use crate::{
    codec::model::{
        ZenohField, ZenohStruct,
        attribute::{DefaultAttribute, HeaderAttribute, PresenceAttribute, SizeAttribute},
        ty::ZenohType,
    },
    codec::r#struct::enc_len_modifier,
};

/// Generates code that computes the header byte at runtime.
///
/// This function analyzes all fields in the struct and generates code to:
/// 1. Initialize header with constant base value
/// 2. Set presence flags for optional fields
/// 3. Encode sizes in header fields
/// 4. Write field values directly to header
/// 5. Combine flattened struct headers
/// 6. Set extension flag if extensions are present
///
/// # Arguments
///
/// * `r#struct` - The parsed struct representation
///
/// # Returns
///
/// A tuple of:
/// - `TokenStream`: The generated header computation code
/// - `bool`: Whether the struct has a header (true) or not (false)
///
/// # Generated Structure
///
/// The code destructures relevant fields, computes the header byte by
/// OR-ing various components, and returns it. Fields that don't affect
/// the header are not destructured.
pub fn parse(r#struct: &ZenohStruct) -> syn::Result<(TokenStream, bool)> {
    let mut body = Vec::<TokenStream>::new();
    let mut s = Vec::<TokenStream>::new();

    if r#struct.header.is_some() {
        body.push(quote::quote! {
            let mut header: u8 = Self::HEADER_BASE;
        });
    }

    for field in &r#struct.fields {
        match field {
            ZenohField::Regular { field } => {
                let access = &field.access;
                let ty = &field.ty;
                let attr = &field.attr;

                let default = match &attr.default {
                    DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                    _ => quote::quote! {},
                };

                if let HeaderAttribute::Slot(slot) = &attr.header {
                    s.push(access.clone());

                    body.push(quote::quote! {
                        header  |= {
                            let v: u8 = (*#access).into();
                            (v << (#slot .trailing_zeros())) & #slot
                        };
                    });

                    continue;
                }

                // Check if field affects the header in other ways

                if matches!(attr.presence, PresenceAttribute::Header(_))
                    || matches!(attr.size, SizeAttribute::Header(_))
                    || attr.flatten
                {
                    s.push(access.clone());
                }

                let len = if attr.flatten {
                    let shift = attr.shift.unwrap_or(0);

                    body.push(enc_len_modifier(
                        attr,
                        &quote::quote! {
                            header |= < _ as crate::ZHeader>::z_header(#access) << #shift;
                        },
                        access,
                        &default,
                        false,
                    ));

                    // Use body length (header is handled separately)
                    quote::quote! { < _ as crate::ZBodyLen>::z_body_len(#access) }
                } else {
                    quote::quote! { < _ as crate::ZLen>::z_len(#access) }
                };

                let check = match ty {
                    ZenohType::Option(_) => quote::quote! { #access.is_some() },
                    _ => quote::quote! { #access  != &#default },
                };

                if let PresenceAttribute::Header(slot) = &attr.presence {
                    body.push(quote::quote! {
                        if #check {
                            header |= #slot;
                        }
                    });
                }

                if let SizeAttribute::Header(slot) = &attr.size {
                    let e: u8 = (!attr.maybe_empty) as u8;

                    // Generate code to encode size in header:
                    // 1. Calculate bit shift from mask
                    // 2. Get length and adjust if non-empty required
                    // 3. Shift to position and apply mask
                    // Wrapped with optional/default checks
                    body.push(enc_len_modifier(
                        attr,
                        &quote::quote! {
                            header |= {
                                let shift = #slot .trailing_zeros();
                                let len = #len as u8;

                                ((len - #e) << shift) & #slot
                            };
                        },
                        access,
                        &default,
                        false,
                    ));
                }
            }
            ZenohField::ExtBlock { .. } => {
                // Extension block: set Z flag if any extensions are present
                // The Z flag is conventionally bit 7 (MSB)
                body.push(quote::quote! {
                    header |= if <_ as crate::ZExtCount>::z_ext_count(self) > 0 {
                        Self::HEADER_SLOT_Z
                    } else {
                        0
                    };
                });
            }
        }
    }

    let expand = if s.is_empty() {
        quote::quote! { .. }
    } else {
        quote::quote! { , .. }
    };

    if body.is_empty() {
        return Ok((quote::quote! {}, false));
    }

    Ok((
        quote::quote! {
            let Self {
                #(#s),*
                #expand
            } = self;

            // Execute all header computation statements
            #(#body)*

            header
        },
        true,
    ))
}
