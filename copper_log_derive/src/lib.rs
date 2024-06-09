mod index;

extern crate proc_macro;

use crate::index::{intern_string, record_callsite};
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
        let index = intern_string(&msg).expect("Failed to insert log string.");
        (index, msg)
    } else {
        panic!("The first parameter of the argument needs to be a string literal.");
    };
    let prefix = quote! {
        use copper_log::value::Value;
        use copper_log::value::to_value;
        use copper_log::CuLogEntry;
        use copper_log::ANONYMOUS;
        let msg = #msg;
        let mut log_entry = CuLogEntry::new(#index);
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

    let unnamed_prints = unnamed_params.iter().map(|value| {
        quote! {
            let param = to_value(#value).expect("Failed to convert a parameter to a Value");
            log_entry.add_param(ANONYMOUS, param);
        }
    });

    let named_prints = named_params.iter().map(|(name, value)| {
        let index = intern_string(quote!(#name).to_string().as_str())
            .expect("Failed to insert log string.");
        quote! {
            let param = to_value(#value).expect("Failed to convert a parameter to a Value");
            log_entry.add_param(#index, param);
        }
    });
    let postfix = quote! {
        let r = copper_log_runtime::log(log_entry);
        if let Err(e) = r {
            eprintln!("Warning: Failed to log: {}", e);
            let backtrace = std::backtrace::Backtrace::capture();
            eprintln!("{:?}", backtrace);
        }
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
