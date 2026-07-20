use proc_macro2::TokenStream;

use crate::zerror::model::{DeclaredErrors, ErrorEnum, ErrorVariant};

pub fn impls_from(error_enum: &ErrorEnum, input: &DeclaredErrors) -> TokenStream {
    let name = &error_enum.name;

    let to_zerror = {
        let variants = super::map_variants(
            error_enum,
            input,
            |_: &ErrorEnum, variant: &ErrorVariant| {
                let vname = &variant.name;

                quote::quote! {
                    #name:: #vname => Error:: #vname,
                }
            },
        );

        quote::quote! {
            impl From<#name> for Error {
                fn from(value: #name) -> Self {
                    match value {
                        #(#variants)*
                        _ => unreachable!(),
                    }
                }
            }
        }
    };

    let from_children = error_enum.children.iter().map(|child_ident| {
        let child_error = input.get(child_ident).unwrap();
        let child_name = &child_error.name;

        let variants = super::map_variants(child_error, input, |_, variant: &ErrorVariant| {
            let vname = &variant.name;

            quote::quote! {
                #child_name:: #vname => #name:: #vname,
            }
        });

        quote::quote! {
            impl From<#child_name> for #name {
                fn from(value: #child_name) -> Self {
                    match value {
                        #(#variants)*
                        _ => unreachable!(),
                    }
                }
            }
        }
    });

    quote::quote! {
        #to_zerror

        #(#from_children)*
    }
}
