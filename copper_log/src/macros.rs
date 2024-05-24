mod index;

extern crate proc_macro;

use crate::index::check_and_insert;
use proc_macro::TokenStream;
use proc_macro2::{Span, TokenTree};
use quote::quote;
use syn::{custom_keyword, parse_macro_input, LitStr};

#[proc_macro]
pub fn debug(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as LitStr);
    let message = input.value();
    // We put dummy locations here because we cannot get the actual location of the macro invocation
    let index = check_and_insert("dummy", 0, &message).expect("Failed to insert log string.");

    let expanded = quote! {
        {
            let log_message = #message;
            println!("#{}:{}:{} Log: {:?}", #index, file!(), line!(), log_message);
            log_message
        }
    };
    println!("#{} -> [{}]", index, message);

    TokenStream::from(expanded)
}
