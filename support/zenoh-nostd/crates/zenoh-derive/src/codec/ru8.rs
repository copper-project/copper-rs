use proc_macro2::TokenStream;
use syn::DeriveInput;

/// Derives codec traits for a `repr(u8)` enum.
///
/// This macro is used for simple enums that are represented as a single byte.
/// The enum must have `#[repr(u8)]` and each variant must have an explicit
/// discriminant value.
///
/// The encoding is simply the discriminant value as a single byte.
///
/// # Errors
///
/// Returns a [`syn::Error`] if:
/// - The input is not an enum
/// - The enum doesn't have `#[repr(u8)]`
/// - Variants have associated data (not supported)
pub fn derive_zru8(input: &DeriveInput) -> syn::Result<TokenStream> {
    let ident = &input.ident;
    let variants = match &input.data {
        syn::Data::Enum(data_enum) => &data_enum.variants,
        _ => {
            return Err(syn::Error::new_spanned(
                input,
                "ZRU8 can only be derived for enums represented as u8",
            ));
        }
    }
    .iter()
    .map(|variant| {
        (
            &variant.ident,
            variant.discriminant.as_ref().map(|(_, expr)| expr),
        )
    });

    if variants.clone().any(|(_, discr)| discr.is_none()) && variants.clone().count() > 1 {
        return Err(syn::Error::new_spanned(
            input,
            "All enum variants must have discriminants",
        ));
    }

    let variants_map = variants.clone().map(|(ident, discr)| {
        let expr = discr.expect("All variants have discriminants");

        quote::quote! {
            #expr => Ok(Self:: #ident),
        }
    });

    let ids = variants.clone().map(|(_, discr)| {
        let expr = discr.expect("All variants have discriminants");

        quote::quote! {
            #expr,
        }
    });

    let rand = variants.map(|(ident, discr)| {
        let expr = discr.expect("All variants have discriminants");

        quote::quote! {
            #expr => Self:: #ident,
        }
    });

    let try_from_map = variants_map.clone();

    Ok(quote::quote! {
        impl From<#ident> for u8 {
            fn from(value: #ident) -> Self {
                value as u8
            }
        }

        impl TryFrom<u8> for #ident {
            type Error = crate::CodecError;

            fn try_from(value: u8) -> Result<Self, Self::Error> {
                match value {
                    #(#try_from_map)*
                    _ => Err(crate::CodecError::CouldNotParseField),
                }
            }
        }

        impl crate::ZBodyLen for #ident {
            fn z_body_len(&self) -> usize {
                <u64 as crate::ZLen>::z_len(&((*self as u8) as u64))
            }
        }

        impl crate::ZLen for #ident {
            fn z_len(&self) -> usize {
                <Self as crate::ZBodyLen>::z_body_len(self)
            }
        }

        impl crate::ZBodyEncode for #ident {
            fn z_body_encode(&self, w: &mut impl crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                <u64 as ZEncode>::z_encode(&((*self as u8) as u64), w)
            }
        }

        impl crate::ZEncode for #ident {
            fn z_encode(&self, w: &mut impl crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                <Self as crate::ZBodyEncode>::z_body_encode(self, w)
            }
        }

        impl<'a> crate::ZBodyDecode<'a> for #ident {
            type Ctx = ();

            fn z_body_decode(r: &mut impl crate::ZReadable<'a>, _: ()) -> core::result::Result<Self, crate::CodecError> {
                let value = <u64 as ZDecode>::z_decode(r)?;
                match value as u8 {
                    #(#variants_map)*
                    _ => Err(crate::CodecError::CouldNotParseField),
                }
            }
        }

        impl<'a> crate::ZDecode<'a> for #ident {
            fn z_decode(r: &mut impl crate::ZReadable<'a>) -> core::result::Result<Self, crate::CodecError> {
                <Self as crate::ZBodyDecode<'a>>::z_body_decode(r, ())
            }
        }

        impl<'a> crate::ZExt<'a> for #ident {
            const KIND: ZExtKind = ZExtKind::U64;
        }

        impl #ident {
            #[cfg(test)]
            pub(crate) fn rand<'a>(_: &mut impl crate::ZStoreable<'a>) -> Self {
                use rand::seq::SliceRandom;
                let mut rng = rand::thread_rng();
                let choices = [#(#ids)*];

                match *choices.choose(&mut rng).unwrap() {
                    #(#rand)*
                    _ => unreachable!(),
                }
            }
        }
    })
}
