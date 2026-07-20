use std::collections::HashSet;

use proc_macro2::TokenStream;

use crate::zerror::model::{DeclaredErrors, ErrorEnum, ErrorVariant};

pub(crate) mod display;
pub(crate) mod impls;

fn map_variants(
    error: &ErrorEnum,
    input: &DeclaredErrors,
    map: impl Fn(&ErrorEnum, &ErrorVariant) -> TokenStream,
) -> Vec<TokenStream> {
    let mut variants = Vec::new();
    let mut stack = vec![error];
    let mut seen = HashSet::new();

    while let Some(current_error) = stack.pop() {
        for variant in &current_error.variants {
            if seen.insert(&variant.name) {
                variants.push(map(current_error, variant));
            }
        }

        for child_ident in &current_error.children {
            if let Some(child_error) = input.get(child_ident) {
                stack.push(child_error);
            }
        }
    }

    variants
}

pub fn declare_children(input: &DeclaredErrors) -> TokenStream {
    let children = input.values().flat_map(|error_enum| {
        let name = &error_enum.name;
        let doc = &error_enum.doc;

        let variants = map_variants(error_enum, input, |_, error_variant: &ErrorVariant| {
            let name = &error_variant.name;
            let doc = format!("See [`Error::{}`]", name);
            quote::quote! {
                #[doc = #doc]
                #name = Error:: #name as u8,
            }
        });

        let display = display::impl_display(error_enum, input);
        let impls = impls::impls_from(error_enum, input);

        quote::quote! {
            #[repr(u8)]
            #[derive(Debug, Clone, Copy, PartialEq, Eq)]
            #[doc = #doc]
            pub enum #name {
                #(#variants)*
            }

            #impls

            #display

            impl core::error::Error for #name {}


            #[cfg(feature = "defmt")]
            impl defmt::Format for #name {
                fn format(&self, f: defmt::Formatter) {
                    defmt::write!(f, "{}", defmt::Display2Format(self));
                }
            }
        }
    });

    quote::quote! {
        #(#children)*
    }
}
