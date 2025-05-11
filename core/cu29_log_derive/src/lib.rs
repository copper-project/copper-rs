extern crate proc_macro;
mod index;

use crate::index::intern_string;
use cu29_log::CuLogLevel;
use proc_macro::TokenStream;
use quote::quote;
use syn::parse::Parser;
#[cfg(debug_assertions)]
use syn::spanned::Spanned;
use syn::Token;
use syn::{Expr, ExprAssign, ExprLit, Lit};

/// Create reference of unused_variables to avoid warnings
/// ex: let _ = &tmp;
#[allow(unused)]
fn reference_unused_variables(input: TokenStream) -> TokenStream {
    // Attempt to parse the expressions to "use" them.
    // This ensures variables passed to the macro are considered used by the compiler.
    let parser = syn::punctuated::Punctuated::<syn::Expr, syn::Token![,]>::parse_terminated;
    if let Ok(exprs) = parser.parse(input.clone()) {
        let mut var_usages = Vec::new();
        // Skip the first expression, which is assumed to be the format string literal.
        // We only care about "using" the subsequent variable arguments.
        for expr in exprs.iter().skip(1) {
            match expr {
                // If the argument is an assignment (e.g., `foo = bar`),
                // we need to ensure `bar` (the right-hand side) is "used".
                syn::Expr::Assign(assign_expr) => {
                    let value_expr = &assign_expr.right;
                    var_usages.push(quote::quote! { let _ = &#value_expr; });
                }
                // Otherwise, for any other expression, ensure it's "used".
                _ => {
                    var_usages.push(quote::quote! { let _ = &#expr; });
                }
            }
        }
        // Return a block that contains these dummy "usages".
        // If only a format string was passed, var_usages will be empty,
        // resulting in an empty block `{}`, which is fine.
        return quote::quote! { { #(#var_usages;)* } }.into();
    }

    // Fallback: if parsing fails for some reason, return an empty TokenStream.
    // This might still lead to warnings if parsing failed but is better than panicking.
    proc_macro::TokenStream::new()
}

/// Create a log entry at the specified log level.
///
/// This is the internal macro implementation used by all the logging macros.
/// Users should use the public-facing macros: `debug!`, `info!`, `warning!`, `error!`, or `critical!`.
#[allow(unused)]
fn create_log_entry(input: TokenStream, level: CuLogLevel) -> TokenStream {
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
    let level_str = match level {
        CuLogLevel::Debug => quote! { Debug },
        CuLogLevel::Info => quote! { Info },
        CuLogLevel::Warning => quote! { Warning },
        CuLogLevel::Error => quote! { Error },
        CuLogLevel::Critical => quote! { Critical },
    };
    let prefix = quote! {
        let mut log_entry = CuLogEntry::new(#index, CuLogLevel::#level_str);
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

/// This macro is used to log a debug message with parameters.
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
///
/// This macro will be compiled out if the max log level is set to a level higher than Debug.
#[cfg(feature = "log-level-debug")]
#[proc_macro]
pub fn debug(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Debug)
}

/// This macro is used to log an info message with parameters.
/// The first parameter is a string literal that represents the message to be logged.
/// Only `{}` is supported as a placeholder for parameters.
/// The rest of the parameters are the values to be logged.
///
/// This macro will be compiled out if the max log level is set to a level higher than Info.
#[cfg(any(feature = "log-level-debug", feature = "log-level-info",))]
#[proc_macro]
pub fn info(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Info)
}

/// This macro is used to log a warning message with parameters.
/// The first parameter is a string literal that represents the message to be logged.
/// Only `{}` is supported as a placeholder for parameters.
/// The rest of the parameters are the values to be logged.
///
/// This macro will be compiled out if the max log level is set to a level higher than Warning.
#[cfg(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
))]
#[proc_macro]
pub fn warning(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Warning)
}

/// This macro is used to log an error message with parameters.
/// The first parameter is a string literal that represents the message to be logged.
/// Only `{}` is supported as a placeholder for parameters.
/// The rest of the parameters are the values to be logged.
///
/// This macro will be compiled out if the max log level is set to a level higher than Error.
#[cfg(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    feature = "log-level-error",
))]
#[proc_macro]
pub fn error(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Error)
}

/// This macro is used to log a critical message with parameters.
/// The first parameter is a string literal that represents the message to be logged.
/// Only `{}` is supported as a placeholder for parameters.
/// The rest of the parameters are the values to be logged.
///
/// This macro is always compiled in, regardless of the max log level setting.
#[cfg(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    feature = "log-level-error",
    feature = "log-level-critical",
))]
#[proc_macro]
pub fn critical(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Critical)
}

// Provide empty implementations for macros that are compiled out
#[cfg(not(any(feature = "log-level-debug",)))]
#[proc_macro]
pub fn debug(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    reference_unused_variables(input)
}

#[cfg(not(any(feature = "log-level-debug", feature = "log-level-info",)))]
#[proc_macro]
pub fn info(input: TokenStream) -> TokenStream {
    reference_unused_variables(input)
}

#[cfg(not(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
)))]
#[proc_macro]
pub fn warning(input: TokenStream) -> TokenStream {
    reference_unused_variables(input)
}

#[cfg(not(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    feature = "log-level-error",
)))]
#[proc_macro]
pub fn error(input: TokenStream) -> TokenStream {
    reference_unused_variables(input)
}

#[cfg(not(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    feature = "log-level-error",
    feature = "log-level-critical",
)))]
#[proc_macro]
pub fn critical(input: TokenStream) -> TokenStream {
    reference_unused_variables(input)
}
