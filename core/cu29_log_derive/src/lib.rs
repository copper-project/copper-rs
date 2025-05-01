extern crate proc_macro;
mod index;

use crate::index::intern_string;
use proc_macro::TokenStream;
use quote::quote;
use syn::parse::Parser;
#[cfg(debug_assertions)]
use syn::spanned::Spanned;
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
/// ```ignore
/// use cu29_log_derive::debug;
/// let a = 1;
/// let b = 2;
/// debug!("a = {}, b = {}", my_value = a, b); // named and unnamed parameters
/// ```
///
/// You can retrieve this data using the log_reader generated with your project and giving it the
/// unified .copper log file and the string index file generated at compile time.
///
/// Note: In debug mode, the log will also be printed to the console. (ie slooow).
/// In release mode, the log will be only be written to the unified logger.
#[proc_macro]
pub fn debug(input: TokenStream) -> TokenStream {
    log_with_level(input, LogLevel::Debug)
}

/// This macro is used to log a message with parameters at INFO level.
#[proc_macro]
pub fn info(input: TokenStream) -> TokenStream {
    log_with_level(input, LogLevel::Info)
}

/// This macro is used to log a message with parameters at WARN level.
#[proc_macro]
pub fn warn(input: TokenStream) -> TokenStream {
    log_with_level(input, LogLevel::Warn)
}

/// This macro is used to log a message with parameters at ERROR level.
#[proc_macro]
pub fn error(input: TokenStream) -> TokenStream {
    log_with_level(input, LogLevel::Error)
}

/// This macro is used to log a message with parameters at TRACE level.
#[proc_macro]
pub fn trace(input: TokenStream) -> TokenStream {
    log_with_level(input, LogLevel::Trace)
}

/// Enum to represent log levels
#[derive(Copy, Clone)]
enum LogLevel {
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

/// Implementation for all log level macros - creates appropriate log entry based on level
fn log_with_level(input: TokenStream, level: LogLevel) -> TokenStream {
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

    // Map LogLevel to CuLogLevel directly
    let level_token = match level {
        LogLevel::Error => quote! { CuLogLevel::Error },
        LogLevel::Warn => quote! { CuLogLevel::Warn },
        LogLevel::Info => quote! { CuLogLevel::Info },
        LogLevel::Debug => quote! { CuLogLevel::Debug },
        LogLevel::Trace => quote! { CuLogLevel::Trace },
    };

    // Generate the log entry creation with the appropriate level
    let prefix = quote! {
        let mut log_entry = CuLogEntry::with_level(#index, #level_token);
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

    #[cfg(not(debug_assertions))]
    let log_stmt = quote! {
        let r = log(&mut log_entry);
    };

    #[cfg(debug_assertions)]
    let log_stmt = {
        let keys: Vec<_> = named_params
            .iter()
            .map(|(name, _)| {
                let name_str = quote!(#name).to_string(); // Convert the expression to a token stream, then to a string
                let lit_str = syn::LitStr::new(&name_str, name.span()); // Create a string literal with the original span
                quote!(#lit_str)
            })
            .collect();
        quote! {
            let r = log_debug_mode(&mut log_entry, #_msg, &[#(#keys),*]);
        }
    };

    let postfix = quote! {
        #log_stmt
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
