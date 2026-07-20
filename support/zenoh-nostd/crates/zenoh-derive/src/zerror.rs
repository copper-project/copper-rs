use proc_macro2::TokenStream;

use crate::zerror::model::DeclaredErrors;

pub(crate) mod model;

pub(crate) mod children;

pub fn declare_zerror(input: &DeclaredErrors) -> syn::Result<TokenStream> {
    let variants_declare = input.values().flat_map(|error_enum| {
        error_enum.variants.iter().map(|variant| {
            let name = &variant.name;
            let code = variant.code;
            let doc = &variant.doc;

            quote::quote! {
                #[doc = #doc]
                #name = #code,
            }
        })
    });

    let variants_display = input.values().flat_map(|error_enum| {
        error_enum.variants.iter().map(|variant| {
            let ename = &error_enum.name;
            let name = &variant.name;
            let err = &variant.err;
            let code = variant.code;

            quote::quote! {
                Error::#name => write!(f, "[{}({})]: {}", stringify!(#ename), #code, #err),
            }
        })
    });

    let children = children::declare_children(input);

    Ok(quote::quote! {
        #[doc = "Base error enum for Zenoh. It contains all possible error codes."]
        #[repr(u8)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum Error {
            #(#variants_declare)*
        }

        impl core::fmt::Display for Error {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match self {
                    #(#variants_display)*
                    _ => Ok(())
                }
            }
        }

        impl core::error::Error for Error {}

        #[cfg(feature = "defmt")]
        impl defmt::Format for Error {
            fn format(&self, f: defmt::Formatter) {
                defmt::write!(f, "{}", defmt::Display2Format(self));
            }
        }

        #children
    })
}
