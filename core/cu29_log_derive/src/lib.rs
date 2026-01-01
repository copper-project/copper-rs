extern crate proc_macro;

use cu29_intern_strs::intern_string;
use cu29_log::CuLogLevel;
use proc_macro::TokenStream;
use proc_macro_crate::{FoundCrate, crate_name};
use proc_macro2::{Span, TokenStream as TokenStream2};
use quote::quote;
#[allow(unused)]
use syn::Token;
use syn::parse::Parser;
use syn::spanned::Spanned;
use syn::{Expr, ExprLit, Lit};

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
    TokenStream::new()
}

/// Create a log entry at the specified log level.
///
/// This is the internal macro implementation used by all the logging macros.
/// Users should use the public-facing macros: `debug!`, `info!`, `warning!`, `error!`, or `critical!`.
#[allow(unused)]
fn create_log_entry(input: TokenStream, level: CuLogLevel) -> TokenStream {
    use quote::quote;
    use syn::{Expr, ExprAssign, ExprLit, Lit, Token};

    let parser = syn::punctuated::Punctuated::<Expr, Token![,]>::parse_terminated;
    let exprs = parser.parse(input).expect("Failed to parse input");
    let mut exprs_iter = exprs.iter();

    #[cfg(not(feature = "std"))]
    const STD: bool = false;
    #[cfg(feature = "std")]
    const STD: bool = true;

    let msg_expr = exprs_iter.next().expect("Expected at least one expression");
    let (index, msg_str) = if let Expr::Lit(ExprLit {
        lit: Lit::Str(msg), ..
    }) = msg_expr
    {
        let s = msg.value();
        let index = intern_string(&s).expect("Failed to insert log string.");
        (index, s)
    } else {
        panic!("The first parameter of the argument needs to be a string literal.");
    };

    let level_ident = match level {
        CuLogLevel::Debug => quote! { Debug },
        CuLogLevel::Info => quote! { Info },
        CuLogLevel::Warning => quote! { Warning },
        CuLogLevel::Error => quote! { Error },
        CuLogLevel::Critical => quote! { Critical },
    };

    // Partition unnamed vs named args (a = b treated as named)
    let mut unnamed_params = Vec::<&Expr>::new();
    let mut named_params = Vec::<(&Expr, &Expr)>::new();

    for expr in exprs_iter {
        if let Expr::Assign(ExprAssign { left, right, .. }) = expr {
            named_params.push((left, right));
        } else {
            unnamed_params.push(expr);
        }
    }

    // Build the CuLogEntry population tokens
    let unnamed_prints = unnamed_params.iter().map(|value| {
        quote! {
            let param = to_value(#value).expect("Failed to convert a parameter to a Value");
            log_entry.add_param(ANONYMOUS, param);
        }
    });

    let named_prints = named_params.iter().map(|(name, value)| {
        let idx = intern_string(quote!(#name).to_string().as_str())
            .expect("Failed to insert log string.");
        quote! {
            let param = to_value(#value).expect("Failed to convert a parameter to a Value");
            log_entry.add_param(#idx, param);
        }
    });

    // ---------- For baremetal: build a defmt format literal and arg list ----------
    // defmt line: "<msg> | a={:?}, b={:?}, arg0={:?} ..."
    let defmt_fmt_lit = {
        let mut s = msg_str.clone();
        if !unnamed_params.is_empty() || !named_params.is_empty() {
            s.push_str(" |");
        }
        for (i, _) in unnamed_params.iter().enumerate() {
            use std::fmt::Write as _;
            let _ = write!(&mut s, " arg{}={:?}", i, ());
        }
        for (name, _) in named_params.iter() {
            let name_str = quote!(#name).to_string();
            s.push(' ');
            s.push_str(&name_str);
            s.push_str("={:?}");
        }
        syn::LitStr::new(&s, msg_expr.span())
    };

    let defmt_args_unnamed_ts: Vec<TokenStream2> =
        unnamed_params.iter().map(|e| quote! { #e }).collect();
    let defmt_args_named_ts: Vec<TokenStream2> = named_params
        .iter()
        .map(|(_, rhs)| quote! { #rhs })
        .collect();

    let defmt_available = crate_name("defmt").is_ok();

    fn defmt_macro_path(level: CuLogLevel) -> TokenStream2 {
        let macro_ident = match level {
            CuLogLevel::Debug => quote! { defmt_debug },
            CuLogLevel::Info => quote! { defmt_info },
            CuLogLevel::Warning => quote! { defmt_warn },
            CuLogLevel::Error => quote! { defmt_error },
            CuLogLevel::Critical => quote! { defmt_error },
        };

        let (base, use_prelude) = match crate_name("cu29") {
            Ok(FoundCrate::Name(name)) => {
                let ident = proc_macro2::Ident::new(&name, Span::call_site());
                (quote! { ::#ident }, true)
            }
            Ok(FoundCrate::Itself) => (quote! { crate }, true),
            Err(_) => match crate_name("cu29-log") {
                Ok(FoundCrate::Name(name)) => {
                    let ident = proc_macro2::Ident::new(&name, Span::call_site());
                    (quote! { ::#ident }, false)
                }
                Ok(FoundCrate::Itself) => (quote! { crate }, false),
                Err(_) => (quote! { ::cu29_log }, false),
            },
        };

        if use_prelude {
            quote! { #base::prelude::#macro_ident }
        } else {
            quote! { #base::#macro_ident }
        }
    }

    // Runtime logging path (unchanged)
    #[cfg(not(debug_assertions))]
    let log_stmt = quote! { let r = log(&mut log_entry); };

    #[cfg(debug_assertions)]
    let log_stmt: TokenStream2 = {
        let keys: Vec<_> = named_params
            .iter()
            .map(|(name, _)| {
                let name_str = quote!(#name).to_string();
                syn::LitStr::new(&name_str, name.span())
            })
            .collect();
        quote! {
            let r = log_debug_mode(&mut log_entry, #msg_str, &[#(#keys),*]);
        }
    };

    let error_handling: Option<TokenStream2> = Some(quote! {
        if let Err(_e) = r {
            let _ = &_e;
        }
    });

    #[cfg(debug_assertions)]
    let defmt_macro: TokenStream2 = defmt_macro_path(level);

    #[cfg(debug_assertions)]
    let maybe_inject_defmt: Option<TokenStream2> = if STD || !defmt_available {
        None // defmt is only emitted in no-std builds.
    } else {
        Some(quote! {
             #[cfg(debug_assertions)]
             {
                 #defmt_macro!(#defmt_fmt_lit, #(#defmt_args_unnamed_ts,)* #(#defmt_args_named_ts,)*);
             }
        })
    };

    #[cfg(not(debug_assertions))]
    let maybe_inject_defmt: Option<TokenStream2> = None; // ... neither in release mode

    // Emit both: defmt (conditionally) + Copper structured logging
    quote! {{
        let mut log_entry = CuLogEntry::new(#index, CuLogLevel::#level_ident);
        #(#unnamed_prints)*
        #(#named_prints)*

        #maybe_inject_defmt

        #log_stmt
        #error_handling
    }}
    .into()
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
#[cfg(any(feature = "log-level-debug", cu29_default_log_level_debug))]
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
#[cfg(not(any(feature = "log-level-debug", cu29_default_log_level_debug)))]
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

/// Interns a string
/// For example:
///
/// let string_number: u32 = intern!("my string");
///
/// will store "my string" in the interned string db at compile time and return the index of the string.
#[proc_macro]
pub fn intern(input: TokenStream) -> TokenStream {
    let expr = syn::parse::<Expr>(input).expect("Failed to parse input as expression");
    let (index, _msg) = if let Expr::Lit(ExprLit {
        lit: Lit::Str(msg), ..
    }) = expr
    {
        let msg = msg.value();
        let index = intern_string(&msg).expect("Failed to insert log string.");
        (index, msg)
    } else {
        panic!("The first parameter of the argument needs to be a string literal.");
    };
    quote! { #index }.into()
}
