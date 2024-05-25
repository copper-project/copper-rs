mod index;

extern crate proc_macro;

use crate::index::check_and_insert;
use proc_macro::TokenStream;
use proc_macro2::{Span, TokenTree};
use quote::quote;
use syn::parse::Parser;
use syn::Token;
use syn::{custom_keyword, parse_macro_input, Expr, ExprAssign, ExprLit, Lit, LitStr};

#[proc_macro]
pub fn debug(input: TokenStream) -> TokenStream {
    let parser = syn::punctuated::Punctuated::<Expr, Token![,]>::parse_terminated;
    let exprs = parser.parse(input).expect("Failed to parse input");

    let mut exprs_iter = exprs.iter();

    let msg_expr = exprs_iter.next().expect("Expected at least one expression");
    let (index, msg) = if let Expr::Lit(ExprLit {
        lit: Lit::Str(msg), ..
    }) = msg_expr
    {
        let msg = msg.value();
        let index = check_and_insert("dummy", 0, &msg).expect("Failed to insert log string.");
        (index, msg)
    } else {
        panic!("The first parameter of the argument needs to be a string literal.");
    };
    println!("{} -> [{}]", index, msg);

    let mut unnamed_params = vec![];
    let mut named_params = vec![];

    for expr in exprs_iter {
        if let Expr::Assign(ExprAssign { left, right, .. }) = expr {
            named_params.push((left, right));
        } else {
            unnamed_params.push(expr);
        }
    }

    let unnamed_prints = unnamed_params.iter().map(|param| {
        quote! {
            println!("{:?}", #param);
        }
    });

    let named_prints = named_params.iter().map(|(name, value)| {
        quote! {
            println!("{} = {:?}", stringify!(#name), #value);
        }
    });

    let expanded = quote! {
        println!("Message: {}", #msg);
        #(#unnamed_prints)*
        #(#named_prints)*
    };

    expanded.into()
}
