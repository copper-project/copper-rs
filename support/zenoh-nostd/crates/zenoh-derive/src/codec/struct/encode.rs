use proc_macro2::TokenStream;

use crate::{
    codec::model::{
        ZenohField, ZenohStruct,
        attribute::{
            DefaultAttribute, ExtAttribute, HeaderAttribute, PresenceAttribute, SizeAttribute,
        },
        ty::ZenohType,
    },
    codec::r#struct::enc_len_modifier,
};

pub fn parse(r#struct: &ZenohStruct) -> syn::Result<(TokenStream, TokenStream)> {
    let mut body = Vec::<TokenStream>::new();
    let mut full = quote::quote! { <_ as crate::ZBodyEncode>::z_body_encode(self, w)?; };
    let mut s = Vec::<TokenStream>::new();

    if r#struct.header.is_some() {
        full = quote::quote! {
            let h = <Self as crate::ZHeader>::z_header(self);
            <u8 as crate::ZEncode>::z_encode(&h, w)?;
            #full
        };
    }

    for field in &r#struct.fields {
        match field {
            ZenohField::Regular { field } => {
                let access = &field.access;
                let ty = &field.ty;
                let attr = &field.attr;

                if let HeaderAttribute::Slot(_) = &attr.header {
                    continue;
                }

                s.push(access.clone());

                let default = match &attr.default {
                    DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                    _ => quote::quote! {},
                };

                let check = match ty {
                    ZenohType::Option(_) => quote::quote! { #access.is_some() },
                    _ => quote::quote! { #access  != &#default },
                };

                let len = if attr.flatten {
                    quote::quote! { < _ as crate::ZBodyLen>::z_body_len(#access) }
                } else {
                    quote::quote! { < _ as crate::ZLen>::z_len(#access) }
                };

                let enc = if attr.flatten {
                    quote::quote! { < _ as crate::ZBodyEncode>::z_body_encode(#access, w)?; }
                } else {
                    quote::quote! { < _ as crate::ZEncode>::z_encode(#access, w)?; }
                };

                match &attr.presence {
                    PresenceAttribute::Prefixed => {
                        body.push(quote::quote! {
                            <u8 as crate::ZEncode>::z_encode(&((#check) as u8), w)?;
                        });
                    }
                    PresenceAttribute::Header(_) => {}
                    _ => {}
                }

                match &attr.size {
                    SizeAttribute::Prefixed => {
                        body.push(enc_len_modifier(
                            attr,
                            &quote::quote! {
                                <usize as crate::ZEncode>::z_encode(&#len, w)?;
                            },
                            access,
                            &default,
                            false,
                        ));
                    }
                    SizeAttribute::Header(_) => {}
                    _ => {}
                }

                body.push(enc_len_modifier(attr, &enc, access, &default, false));
            }
            ZenohField::ExtBlock { exts } => {
                body.push(
                    quote::quote! { let mut n_exts = <_ as crate::ZExtCount>::z_ext_count(self); },
                );

                for field in exts {
                    let access = &field.access;
                    s.push(access.clone());
                    let attr = &field.attr;

                    let id = match &attr.ext {
                        ExtAttribute::Expr(id) => id,
                        _ => unreachable!(
                            "ExtBlock fields must have an ext attribute, this should have been caught earlier"
                        ),
                    };

                    let mandatory = match &field.attr.mandatory {
                        true => quote::quote! { true },
                        false => quote::quote! { false },
                    };

                    let default = match &attr.default {
                        DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                        _ => quote::quote! {},
                    };

                    body.push(enc_len_modifier(
                        attr,
                        &quote::quote! {
                            n_exts -= 1;
                            crate::zext_encode::<_, #id, #mandatory>(#access, w, n_exts != 0)?;
                        },
                        access,
                        &default,
                        false,
                    ));
                }
            }
        }
    }

    let expand = if s.is_empty() {
        quote::quote! { .. }
    } else {
        quote::quote! { , .. }
    };

    Ok((
        quote::quote! {
            let Self {
                #(#s),*
                #expand
            } = self;

            #(#body)*
        },
        quote::quote! { #full },
    ))
}
