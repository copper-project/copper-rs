//! # Length Calculation Generator
//!
//! This module generates code that calculates the encoded length of a struct.
//!
//! ## Purpose
//!
//! This module generates implementations of:
//! - `ZBodyLen::z_body_len()` - calculates body size only (without header)
//! - `ZLen::z_len()` - calculates total size (header + body)
//!
//! ## Generated Code Pattern
//!
//! ```ignore
//! // ZBodyLen implementation
//! fn z_body_len(&self) -> usize {
//!     let Self { field1, field2, .. } = self;
//!     0 + presence_flag_size
//!       + field1_size
//!       + field2_size_prefix
//!       + field2_size
//!       + extension_sizes
//! }
//!
//! // ZLen implementation
//! fn z_len(&self) -> usize {
//!     1 + self.z_body_len()  // 1 = header byte
//! }
//! ```
//!
//! ## Size Components
//!
//! The total size is computed by summing:
//!
//! 1. **Header byte**: 1 byte (if struct has a header)
//! 2. **Presence flags**: 1 byte each for `presence = prefixed` fields
//! 3. **Size prefixes**: VLE encoded length for `size = prefixed` fields
//! 4. **Field data**: The actual encoded field data
//! 5. **Extension headers**: Extension header + optional size prefix + data
//!
//! ## Field Processing
//!
//! - **`header = H`**: Field in header, contributes 0 bytes to body
//! - **`presence = prefixed`**: Adds 1 byte for presence flag
//! - **`presence = header(P)`**: No extra bytes (flag in header)
//! - **`size = prefixed`**: Adds VLE length prefix + data
//! - **`size = header(S)`**: No size prefix (size in header)
//! - **`size = remain`**: Just the data (no prefix)
//! - **`flatten`**: Uses body length only (header combined separately)
//! - **Extensions**: Uses `zext_len()` which includes ext header
//!
//! ## Optional Fields
//!
//! Optional fields (`Option<T>`) and fields with defaults only contribute
//! to the size when present/non-default. This is handled by wrapping size
//! calculations with conditional checks via `enc_len_modifier`.

use proc_macro2::TokenStream;

use crate::{
    codec::model::{
        ZenohField, ZenohStruct,
        attribute::{DefaultAttribute, HeaderAttribute, PresenceAttribute, SizeAttribute},
    },
    codec::r#struct::enc_len_modifier,
};

pub fn parse(r#struct: &ZenohStruct) -> syn::Result<(TokenStream, TokenStream)> {
    let mut body = Vec::<TokenStream>::new();
    let mut full = quote::quote! { <_ as crate::ZBodyLen>::z_body_len(self) };
    let mut s = Vec::<TokenStream>::new();

    if r#struct.header.is_some() {
        full = quote::quote! { 1 + #full };
    }

    for field in &r#struct.fields {
        match field {
            ZenohField::Regular { field } => {
                let access = &field.access;
                let attr = &field.attr;

                if let HeaderAttribute::Slot(_) = &attr.header {
                    continue;
                }

                s.push(access.clone());

                let default = match &attr.default {
                    DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                    _ => quote::quote! {},
                };

                let len = if attr.flatten {
                    // Flattened: use body length only (header combined separately)
                    quote::quote! { < _ as crate::ZBodyLen>::z_body_len(#access) }
                } else {
                    // Normal: use total length (includes any nested headers)
                    quote::quote! { < _ as crate::ZLen>::z_len(#access) }
                };

                // Add size for presence flag (if needed)
                match &attr.presence {
                    PresenceAttribute::Prefixed => {
                        // Prefixed presence: 1 byte boolean before field
                        body.push(quote::quote! { 1usize });
                    }
                    PresenceAttribute::Header(_) => {
                        // Presence in header: no extra bytes
                    }
                    _ => {
                        // No presence flag needed
                    }
                }

                // Add size for length prefix (if needed)
                match &attr.size {
                    SizeAttribute::Prefixed => {
                        // Size prefixed: VLE encoded length before data
                        // Calculate the size of the VLE encoded length itself
                        body.push(enc_len_modifier(
                            attr,
                            &quote::quote! {
                                <usize as crate::ZLen>::z_len(&#len)  // VLE size of length
                            },
                            access,
                            &default,
                            true,
                        ));
                    }
                    SizeAttribute::Header(_) => {
                        // Size in header: no prefix bytes needed
                    }
                    _ => {
                        // No size prefix (fixed size or remain)
                    }
                }

                // Add size of the field data itself
                // Wrapped with conditional for optional/default fields
                body.push(enc_len_modifier(
                    attr, &len, // Field's encoded size
                    access, &default, true, // Append "else 0" for optional fields
                ));
            }
            ZenohField::ExtBlock { exts } => {
                // Extension block: each extension has header + optional size + data
                for field in exts {
                    let access = &field.access;
                    s.push(access.clone()); // Need to access extension field
                    let attr = &field.attr;

                    // Get default value for comparison (if any)
                    let default = match &attr.default {
                        DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                        _ => quote::quote! {},
                    };

                    // Add extension size: header + optional size prefix + data
                    // The `zext_len` function calculates the complete extension size:
                    // - 1 byte for extension header (ID, kind, flags)
                    // - For ZStruct kind: VLE size prefix
                    // - Extension data size
                    body.push(enc_len_modifier(
                        attr,
                        &quote::quote! {
                            crate::zext_len::<_>(#access)  // Complete extension size
                        },
                        access,
                        &default,
                        true, // Append "else 0" for optional extensions
                    ));
                }
            }
        }
    }

    // Generate field destructuring syntax
    let expand = if s.is_empty() {
        quote::quote! { .. } // "Self { .. }" - no fields needed
    } else {
        quote::quote! { , .. } // "Self { field1, .. }" - some fields + rest
    };

    Ok((
        quote::quote! {
            let Self {
                #(#s),*
                #expand
            } = self;

            0 #(+ #body )*
        },
        quote::quote! { #full },
    ))
}
