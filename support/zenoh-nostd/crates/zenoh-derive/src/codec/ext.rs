use proc_macro2::{Span, TokenStream};
use quote::ToTokens;
use syn::DeriveInput;

use crate::codec::model::{ZenohField, ZenohStruct, ty::ZenohType};

mod r#u64;

pub fn derive_zext(input: &DeriveInput) -> syn::Result<TokenStream> {
    let r#struct = ZenohStruct::from_derive_input(input)?;
    let ident = &r#struct.ident;

    let generics = &r#struct.generics;
    let (_, ty_generics, where_clause) = generics.split_for_impl();

    let kind = infer_kind(&r#struct)?;
    let struct_impl = match kind {
        InferredKind::U64 => r#u64::parse(&r#struct, ident),
        _ => crate::codec::r#struct::derive_zstruct(input)?,
    };

    Ok(quote::quote! {
        #struct_impl

        impl<'a> crate::ZExt<'a> for #ident #ty_generics #where_clause {
            const KIND: crate::ZExtKind = #kind;
        }
    })
}

enum InferredKind {
    Unit,
    U64,
    ZStruct,
}

impl ToTokens for InferredKind {
    fn to_tokens(&self, tokens: &mut TokenStream) {
        let kind_token = match self {
            InferredKind::Unit => quote::quote! { crate::ZExtKind::Unit },
            InferredKind::U64 => quote::quote! { crate::ZExtKind::U64 },
            InferredKind::ZStruct => quote::quote! { crate::ZExtKind::ZStruct },
        };

        tokens.extend(kind_token);
    }
}

fn infer_kind(ext: &ZenohStruct) -> syn::Result<InferredKind> {
    if ext.fields.is_empty() {
        Ok(InferredKind::Unit)
    } else if ext.fields.len() == 1 {
        let field = &ext.fields.first().unwrap();

        match field {
            ZenohField::ExtBlock { .. } => Err(syn::Error::new(
                Span::call_site(),
                "Cannot infer ZExtKind from only one ext block field",
            )),
            ZenohField::Regular { field } => match field.ty {
                ZenohType::U8
                | ZenohType::U16
                | ZenohType::U32
                | ZenohType::U64
                | ZenohType::USize => Ok(InferredKind::U64),
                _ => Ok(InferredKind::ZStruct),
            },
        }
    } else {
        Ok(InferredKind::ZStruct)
    }
}
