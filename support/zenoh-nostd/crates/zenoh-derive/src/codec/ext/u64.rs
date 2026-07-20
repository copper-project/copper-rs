use proc_macro2::TokenStream;
use syn::Ident;

use crate::codec::model::{ZenohField, ZenohStruct, ty::ZenohType};

pub fn parse(r#struct: &ZenohStruct, ident: &Ident) -> TokenStream {
    let field = r#struct
        .fields
        .first()
        .expect("At least one field is expected, this should have been caught earlier");

    let field = match field {
        ZenohField::Regular { field } => field,
        ZenohField::ExtBlock { .. } => unreachable!(
            "The single field cannot be an ext block, this should have been caught earlier"
        ),
    };

    let access = &field.access;
    let ty = &field.ty;
    let ty = match ty {
        ZenohType::U8 => quote::quote! { u8 },
        ZenohType::U16 => quote::quote! { u16 },
        ZenohType::U32 => quote::quote! { u32 },
        ZenohType::U64 => quote::quote! { u64 },
        ZenohType::USize => quote::quote! { usize },
        _ => unreachable!(),
    };

    quote::quote! {
        impl crate::ZBodyLen for #ident {
            fn z_body_len(&self) -> usize {
                < _ as crate::ZLen>::z_len(&(self. #access as u64))
            }
        }

        impl crate::ZLen for #ident  {
            fn z_len(&self) -> usize {
                < _ as crate::ZBodyLen>::z_body_len(self)
            }
        }

        impl crate::ZBodyEncode for #ident {
            fn z_body_encode(&self, w: &mut impl crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                < _ as crate::ZEncode>::z_encode(&(self. #access as u64), w)
            }
        }

        impl crate::ZEncode for #ident {
            fn z_encode(&self, w: &mut impl crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                < _ as crate::ZBodyEncode>::z_body_encode(self, w)
            }
        }

        impl<'a> crate::ZBodyDecode<'a> for #ident  {
            type Ctx = ();

            fn z_body_decode(r: &mut impl crate::ZReadable<'a>, _: ()) -> core::result::Result<Self, crate::CodecError> {
                let #access = < u64 as crate::ZDecode>::z_decode(r)? as #ty;

                Ok(Self {
                    #access
                })
            }
        }

        impl<'a> crate::ZDecode<'a> for #ident {
            fn z_decode(r: &mut impl crate::ZReadable<'a>) -> core::result::Result<Self, crate::CodecError> {
                < _ as crate::ZBodyDecode>::z_body_decode(r, ())
            }
        }
    }
}
