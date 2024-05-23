extern crate proc_macro;

use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;
use syn::{parse_macro_input, LitStr};

#[proc_macro]
pub fn debug(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as LitStr);
    let message = input.value();

    let expanded = quote! {
        {
            let log_message = #message;
            println!("Log: {:?}", log_message);
            log_message
        }
    };
    println!("Found logging string {}", message);

    // print the OUT env vairable
    println!("OUT: {}", std::env::var("OUT_DIR").unwrap());

    TokenStream::from(expanded)
}
