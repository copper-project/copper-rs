//! # ZStruct Derive Implementation
//!
//! This module orchestrates the code generation for `#[derive(ZStruct)]`.
use proc_macro2::TokenStream;
use syn::DeriveInput;

use crate::codec::model::{
    ZenohStruct,
    attribute::{DefaultAttribute, ExtAttribute, PresenceAttribute, ZenohAttribute},
};

pub mod header_impl;

pub mod decode;
pub mod encode;
pub mod ext_count;
pub mod header;
pub mod len;

/// Main entry point for `#[derive(ZStruct)]` macro.
///
/// This function orchestrates the entire code generation process:
/// 1. Parses the input struct into a [`ZenohStruct`] representation
/// 2. Generates implementations for all codec traits
/// 3. Returns the generated token stream
///
/// # Arguments
///
/// * `input` - The parsed `DeriveInput` from the macro invocation
///
/// # Returns
///
/// A `TokenStream` containing all generated trait implementations
///
/// # Generated Traits
///
/// - `ZHeader` (if struct has a header)
/// - `ZExtCount` (if struct has extensions)
/// - `ZBodyLen` - calculates body size
/// - `ZLen` - calculates total size (header + body)
/// - `ZBodyEncode` - encodes body data
/// - `ZEncode` - encodes complete message (header + body)
/// - `ZBodyDecode` - decodes body data
/// - `ZDecode` - decodes complete message (header + body)
pub fn derive_zstruct(input: &DeriveInput) -> syn::Result<TokenStream> {
    let r#struct = ZenohStruct::from_derive_input(input)?;
    let ident = &r#struct.ident;

    let generics = &r#struct.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    let header_impl = header_impl::parse(&r#struct)?;
    let (header, h) = header::parse(&r#struct)?;

    let (header, ctx, ctx_p) = if h {
        (
            quote::quote! {
                impl #impl_generics crate::ZHeader for #ident #ty_generics #where_clause {
                    fn z_header(&self) -> u8 {
                        #header
                    }
                }
            },
            quote::quote! { u8 },
            quote::quote! { header: u8 },
        )
    } else {
        (header, quote::quote! { () }, quote::quote! { _: () })
    };

    let (ext_count, e) = ext_count::parse(&r#struct)?;
    let ext_count = if e {
        quote::quote! {
            impl #impl_generics crate::ZExtCount for #ident #ty_generics #where_clause {
                fn z_ext_count(&self) -> usize {
                    #ext_count
                }
            }
        }
    } else {
        quote::quote! {}
    };

    let (len_body, len) = len::parse(&r#struct)?;
    let (encode_body, encode) = encode::parse(&r#struct)?;
    let (decode_body, decode) = decode::parse(&r#struct)?;

    Ok(quote::quote! {
        #header_impl
        #header

        #ext_count

        impl #impl_generics crate::ZBodyLen for #ident #ty_generics #where_clause {
            fn z_body_len(&self) -> usize {
                #len_body
            }
        }

        impl #impl_generics crate::ZLen for #ident #ty_generics #where_clause {
            fn z_len(&self) -> usize {
                #len
            }
        }

        impl #impl_generics crate::ZBodyEncode for #ident #ty_generics #where_clause {
            fn z_body_encode(&self, w: &mut impl crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                #encode_body

                Ok(())
            }
        }

        impl #impl_generics crate::ZEncode for #ident #ty_generics #where_clause {
            fn z_encode(&self, w: &mut impl crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                #encode

                Ok(())
            }
        }

        impl<'a> crate::ZBodyDecode<'a> for #ident #ty_generics #where_clause {
            type Ctx = #ctx;

            fn z_body_decode(r: &mut impl crate::ZReadable<'a>, #ctx_p) -> core::result::Result<Self, crate::CodecError> {
                #decode_body
            }
        }

        impl<'a> crate::ZDecode<'a> for #ident #ty_generics #where_clause {
            fn z_decode(r: &mut impl crate::ZReadable<'a>) -> core::result::Result<Self, crate::CodecError> {
                #decode
            }
        }
    })
}

/// Wraps encoding or length calculation code with optional/default checks.
///
/// This helper function generates conditional code that handles:
/// - Optional fields (`Option<T>`) with presence flags
/// - Fields with default values
/// - Extension fields
///
/// # Arguments
///
/// * `attr` - The field's attributes (presence, default, ext)
/// * `tk` - The encoding/length token stream to wrap
/// * `access` - How to access the field (e.g., `self.field`)
/// * `default` - Default value expression (if any)
/// * `append` - If true, append `else { 0usize }` for length calculations
///
/// # Logic
///
/// The function generates different code based on attribute combinations:
///
/// - **No attributes**: Returns `tk` as-is
/// - **presence + default** OR **ext + default**:
///   ```ignore
///   if field != &default { tk } else { 0 }  // if append
///   if field != &default { tk }              // if !append
///   ```
/// - **presence only** OR **ext only**:
///   ```ignore
///   if let Some(field) = field { tk } else { 0 }  // if append
///   if let Some(field) = field { tk }              // if !append
///   ```
#[allow(clippy::nonminimal_bool)]
pub fn enc_len_modifier(
    attr: &ZenohAttribute,
    tk: &TokenStream,
    access: &TokenStream,
    default: &TokenStream,
    append: bool,
) -> TokenStream {
    let (p, e, d) = (
        !matches!(attr.presence, PresenceAttribute::None),
        !matches!(attr.ext, ExtAttribute::None),
        !matches!(attr.default, DefaultAttribute::None),
    );

    if !p && !d && !e {
        quote::quote! { #tk }
    } else if (p && d && !e) || (e && !p && d) {
        let res = quote::quote! {
            if #access  != &#default {
                #tk
            }
        };

        if append {
            quote::quote! { #res else { 0usize } }
        } else {
            res
        }
    } else if (p && !d && !e) || (e && !p && !d) {
        let res = quote::quote! {
            if let Some(#access) = #access {
                #tk
            }
        };

        if append {
            quote::quote! { #res else { 0usize } }
        } else {
            res
        }
    } else {
        unreachable!("All cases have been covered, this panic should have been caught earlier.");
    }
}

/// Wraps decoding code with optional/default value handling.
///
/// This helper function generates conditional decoding code that handles:
/// - Optional fields (`Option<T>`) with presence flags
/// - Fields with default values
///
/// # Arguments
///
/// * `attr` - The field's attributes (presence, default)
/// * `tk` - The decoding token stream to wrap
/// * `access` - Variable name to bind the decoded value to
/// * `default` - Default value expression (if any)
///
/// # Logic
///
/// The function generates different code based on attribute combinations:
///
/// - **No attributes**: Decode unconditionally
///   ```ignore
///   let field = { decode_expr };
///   ```
/// - **presence + default**: Use default if not present
///   ```ignore
///   let field = if present { decode_expr } else { default };
///   ```
/// - **presence only**: Wrap in Option
///   ```ignore
///   let field = if present { Some(decode_expr) } else { None };
///   ```
pub fn dec_modifier(
    attr: &ZenohAttribute,
    tk: &TokenStream,
    access: &TokenStream,
    default: &TokenStream,
) -> TokenStream {
    let (p, d) = (
        !matches!(attr.presence, PresenceAttribute::None),
        !matches!(attr.default, DefaultAttribute::None),
    );

    if !p && !d {
        quote::quote! {
            let #access = {
                #tk
            };
        }
    } else if p && d {
        quote::quote! {
            let #access = if #access {
                #tk
            } else {
                #default
            };
        }
    } else if p && !d {
        quote::quote! {
            let #access = if #access {
                Some( { #tk })
            } else {
                None
            };
        }
    } else {
        unreachable!("All cases have been covered, this panic should have been caught earlier.");
    }
}
