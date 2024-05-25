mod index;

extern crate proc_macro;

use crate::index::{check_and_insert, intern_string};
use proc_macro::TokenStream;
use quote::quote;
use syn::parse::Parser;
use syn::Token;
use syn::{Expr, ExprAssign, ExprLit, Lit};

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
    let prefix = quote! {
        use copper_log_runtime::value::Value;
        use copper_log_runtime::value::to_value;
        use copper_log_runtime::ANONYMOUS;
        let msg = #msg;
        let index = #index;
        println!("{} -> [{}]", index, msg);
        let mut params = Vec::<(Value, Value)>::new();

    };

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
            // 0 means an unnamed parameter
            let param = (ANONYMOUS, to_value(#param).expect("Failed to convert a parameter to a Value"));
            println!("anon param: {:?}", &param);
            params.push(param);
        }
    });

    let named_prints = named_params.iter().map(|(name, value)| {
        let index = intern_string(quote!(#name).to_string().as_str())
            .expect("Failed to insert log string.");
        quote! {
            let param = (to_value(#index).unwrap(), to_value(#value).expect("Failed to convert a parameter to a Value"));
            println!("named param: {:?}", &param);
            params.push(param);
        }
    });

    let postfix = quote! {
        println!("Would send {:?}", (index, params));
    };

    let expanded = quote! {
        {
            #prefix
            #(#unnamed_prints)*
            #(#named_prints)*
            #postfix
        }
    };

    expanded.into()
}
