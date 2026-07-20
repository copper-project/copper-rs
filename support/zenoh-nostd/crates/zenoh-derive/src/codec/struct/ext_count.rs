use proc_macro2::TokenStream;

use crate::{
    codec::model::{ZenohField, ZenohStruct, attribute::DefaultAttribute},
    codec::r#struct::enc_len_modifier,
};

pub fn parse(r#struct: &ZenohStruct) -> syn::Result<(TokenStream, bool)> {
    let mut body = Vec::<TokenStream>::new();
    let mut s = Vec::<TokenStream>::new();

    for field in &r#struct.fields {
        if let ZenohField::ExtBlock { exts } = field {
            for field in exts {
                let access = &field.access;
                s.push(access.clone());
                let attr = &field.attr;

                let default = match &attr.default {
                    DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                    _ => quote::quote! {},
                };

                body.push(enc_len_modifier(
                    attr,
                    &quote::quote! {
                        let _ = #access;
                        n_exts += 1;
                    },
                    access,
                    &default,
                    false,
                ));
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
            let mut n_exts = 0;

            let Self {
                #(#s),*
                #expand
            } = self;

            #(#body)*

            n_exts
        },
        true,
    ))
}
