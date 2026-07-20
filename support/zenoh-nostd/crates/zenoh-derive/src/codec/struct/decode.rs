use proc_macro2::TokenStream;

use crate::{
    codec::model::{
        ZenohField, ZenohStruct,
        attribute::{
            DefaultAttribute, ExtAttribute, HeaderAttribute, PresenceAttribute, SizeAttribute,
        },
        ty::ZenohType,
    },
    codec::r#struct::dec_modifier,
};

pub fn parse(r#struct: &ZenohStruct) -> syn::Result<(TokenStream, TokenStream)> {
    let mut body = Vec::<TokenStream>::new();
    let mut full = quote::quote! { <_ as crate::ZBodyDecode>::z_body_decode(r, ()) };
    let mut d = Vec::<TokenStream>::new();

    if r#struct.header.is_some() {
        full = quote::quote! {
            let h = <u8 as crate::ZDecode>::z_decode(r)?;
            <_ as crate::ZBodyDecode>::z_body_decode(r, h)
        };
    }

    for field in &r#struct.fields {
        match field {
            ZenohField::Regular { field } => {
                let access = &field.access;
                d.push(access.clone());
                let attr = &field.attr;

                if let HeaderAttribute::Slot(slot) = &attr.header {
                    body.push(quote::quote! {
                        let #access = {
                            let v = header & #slot;
                            <_ as TryFrom<u8>>::try_from(v >> #slot.trailing_zeros()).map_err(|_| crate::CodecError::CouldNotParseHeader)?
                        };
                    });

                    continue;
                }

                let default = match &attr.default {
                    DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                    _ => quote::quote! {},
                };

                let (dec1, dec2) = if attr.flatten {
                    let shift = attr.shift.unwrap_or(0);

                    (
                        quote::quote! { < _ as crate::ZBodyDecode>::z_body_decode(&mut crate::ZReadable::read_slice(r, #access)?)?, header >> #shift },
                        quote::quote! { < _ as crate::ZBodyDecode>::z_body_decode(r, header >> #shift)? },
                    )
                } else {
                    (
                        quote::quote! { < _ as crate::ZDecode>::z_decode(&mut crate::ZReadable::read_slice(r, #access)?)? },
                        quote::quote! { < _ as crate::ZDecode>::z_decode(r)? },
                    )
                };

                match &attr.presence {
                    PresenceAttribute::Prefixed => {
                        body.push(quote::quote! {
                            let #access: bool = <u8 as crate::ZDecode>::z_decode(r)? != 0;
                        });
                    }
                    PresenceAttribute::Header(slot) => {
                        body.push(quote::quote! {
                            let #access: bool = (header & #slot) != 0;
                        });
                    }
                    _ => {}
                }

                match &attr.size {
                    SizeAttribute::Prefixed => {
                        body.push(dec_modifier(
                            attr,
                            &quote::quote! {
                                let #access = < usize as crate::ZDecode>::z_decode(r)?;
                                #dec1
                            },
                            access,
                            &default,
                        ));
                    }
                    SizeAttribute::Header(slot) => {
                        let e: u8 = (!attr.maybe_empty) as u8;
                        body.push(dec_modifier(
                            attr,
                            &quote::quote! {
                                let #access = (((header & #slot) >> #slot.trailing_zeros()) + #e) as usize;
                                #dec1
                            },
                            access,
                            &default,
                        ));
                    }
                    _ => {
                        body.push(dec_modifier(attr, &dec2, access, &default));
                    }
                }
            }
            ZenohField::ExtBlock { exts } => {
                body.push(
                    quote::quote! { let mut has_ext: bool = header & Self::HEADER_SLOT_Z != 0; },
                );

                let mut body_ext = Vec::<TokenStream>::new();

                for field in exts {
                    let access = &field.access;
                    d.push(access.clone());
                    let ty = &field.ty;
                    let attr = &field.attr;

                    let id = match &attr.ext {
                        ExtAttribute::Expr(id) => id,
                        _ => unreachable!(
                            "ExtBlock fields must have an ext attribute, this should have been caught earlier"
                        ),
                    };

                    let default = match &attr.default {
                        DefaultAttribute::Expr(expr) => quote::quote! { #expr },
                        _ => quote::quote! {},
                    };

                    match ty {
                        ZenohType::ZStruct => {
                            body.push(quote::quote! {
                                let mut #access = #default;
                            });

                            body_ext.push(quote::quote! {
                                #id => {
                                    #access = crate::zext_decode::< _ >(r)?;
                                }
                            });
                        }
                        ZenohType::Option(_) => {
                            body.push(quote::quote! {
                                let mut #access: _ = None;
                            });

                            body_ext.push(quote::quote! {
                                #id if ext_kind == < _ as crate::ZExtResolveKind>::ext_kind(& #access) => {
                                    #access = Some(crate::zext_decode::< _ >(r)?);
                                }
                            });
                        }
                        _ => unreachable!(
                            "ExtBlock fields must be ZStruct or Option<ZStruct>, this should have been caught earlier"
                        ),
                    }
                }

                body.push(quote::quote! {
                    while has_ext {
                        let (ext_id, ext_kind, mandatory, more) = crate::decode_ext_header(r)?;
                        has_ext = more;

                        match ext_id {
                            #(#body_ext,)*
                            _ => {
                                if mandatory {
                                    return Err(crate::CodecError::CouldNotReadExtension);
                                }

                                crate::skip_ext(r, ext_kind)?;
                            }
                        }
                    }
                });
            }
        }
    }

    Ok((
        quote::quote! {
            #(#body)*

            Ok(Self { #(#d),* })
        },
        quote::quote! { #full },
    ))
}
