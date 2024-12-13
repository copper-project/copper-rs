use proc_macro::TokenStream;
use quote::{quote, ToTokens};
use syn::{parse_macro_input, DeriveInput};

pub fn derive_schema(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    let schema = match &input.data {
        syn::Data::Struct(data) => {
            let fields: Vec<_> = data
                .fields
                .iter()
                .map(|f| {
                    let field_name = f.ident.as_ref().unwrap().to_string();
                    let field_type = f.ty.to_token_stream().to_string();
                    quote! {
                        schema.push((#field_name.to_string(), #field_type.to_string()));
                    }
                })
                .collect();

            quote! {
                let mut schema = Vec::new();
                #(#fields)*
                schema
            }
        }
        _ => {
            return syn::Error::new_spanned(name, "Schema derive macro only supports structs")
                .to_compile_error()
                .into();
        }
    };

    let expanded = quote! {
        impl #name {
            pub fn schema() -> Vec<(String, String)> {
                #schema
            }
        }
    };

    TokenStream::from(expanded)
}
