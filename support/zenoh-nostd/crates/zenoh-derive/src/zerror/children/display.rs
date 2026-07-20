use proc_macro2::TokenStream;

use crate::zerror::model::{DeclaredErrors, ErrorEnum, ErrorVariant};

pub fn impl_display(error_enum: &ErrorEnum, input: &DeclaredErrors) -> TokenStream {
    let name = &error_enum.name;

    let variants = super::map_variants(
        error_enum,
        input,
        |error_enum: &ErrorEnum, variant: &ErrorVariant| {
            let ename = &error_enum.name;
            let vname = &variant.name;
            let code = variant.code;
            let err = &variant.err;

            quote::quote! {
                #name:: #vname => write!(f, "[{}({})]: {}", stringify!(#ename), #code, #err),
            }
        },
    );

    quote::quote! {
        impl core::fmt::Display for #name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match self {
                    #(#variants)*
                    _ => Ok(())
                }
            }
        }
    }
}
