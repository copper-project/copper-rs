mod index;

extern crate proc_macro;

use crate::index::intern_string;
use proc_macro::TokenStream;
use quote::quote;
use syn::parse::Parser;
use syn::Token;
use syn::{Expr, ExprAssign, ExprLit, Lit};

/// This macro is used to log a message with parameters.
/// The first parameter is a string literal that represents the message to be logged.
/// Only `{}` is supported as a placeholder for parameters.
/// The rest of the parameters are the values to be logged.
/// The parameters can be named or unnamed.
/// Named parameters are specified as `name = value`.
/// Unnamed parameters are specified as `value`.
/// # Example
/// ```
/// use cu29_log_derive::debug;
/// let a = 1;
/// let b = 2;
/// debug!("a = {}, b = {}", my_value = a, b); // named and unnamed parameters
/// ```
///
/// You can retreive this data using the log_reader generated with your project and giving it the
/// unified .copper log file and the string index file generated at compile time.
/// 
/// Note: In debug mode, the log will also be printed to the console. (ie slooow).
/// In release mode, the log will be only be written to the unified logger.
#[proc_macro]
pub fn debug(input: TokenStream) -> TokenStream {
    let parser = syn::punctuated::Punctuated::<Expr, Token![,]>::parse_terminated;
    let exprs = parser.parse(input).expect("Failed to parse input");

    let mut exprs_iter = exprs.iter();

    let msg_expr = exprs_iter.next().expect("Expected at least one expression");
    let (index, _msg) = if let Expr::Lit(ExprLit {
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
        use cu29_log::value::Value;
        use cu29_log::value::to_value;
        use cu29_log::CuLogEntry;
        use cu29_log::ANONYMOUS;
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
        let r = cu29_log_runtime::log(log_entry);
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
