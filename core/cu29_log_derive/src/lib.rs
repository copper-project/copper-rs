extern crate proc_macro;

use cu29_intern_strs::intern_string;
use cu29_log::CuLogLevel;
use proc_macro::TokenStream;
#[cfg(feature = "textlogs")]
use proc_macro_crate::{FoundCrate, crate_name};
#[cfg(feature = "textlogs")]
use proc_macro2::Span;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
#[allow(unused)]
use syn::Token;
use syn::parse::Parser;
#[cfg(any(feature = "textlogs", debug_assertions))]
use syn::spanned::Spanned;
use syn::{Expr, ExprLit, Lit};

struct ParsedLogArgs<'a> {
    context_expr: Option<&'a Expr>,
    msg_expr: &'a Expr,
    param_exprs: Vec<&'a Expr>,
}

fn parse_log_exprs(
    input: TokenStream,
) -> syn::Result<syn::punctuated::Punctuated<Expr, syn::Token![,]>> {
    let parser = syn::punctuated::Punctuated::<Expr, syn::Token![,]>::parse_terminated;
    parser.parse(input)
}

fn parse_log_args<'a>(
    exprs: &'a syn::punctuated::Punctuated<Expr, syn::Token![,]>,
) -> syn::Result<ParsedLogArgs<'a>> {
    let Some(first_expr) = exprs.first() else {
        return Err(syn::Error::new(
            proc_macro2::Span::call_site(),
            "Expected at least one expression",
        ));
    };

    let (context_expr, msg_expr, msg_index) = if matches!(
        first_expr,
        Expr::Lit(ExprLit {
            lit: Lit::Str(_),
            ..
        })
    ) {
        (None, first_expr, 0)
    } else {
        let Some(second_expr) = exprs.iter().nth(1) else {
            return Err(syn::Error::new_spanned(
                first_expr,
                "Expected a string literal as the first argument, or as the second argument after a CuContext expression.",
            ));
        };
        if matches!(
            second_expr,
            Expr::Lit(ExprLit {
                lit: Lit::Str(_),
                ..
            })
        ) {
            (Some(first_expr), second_expr, 1)
        } else {
            return Err(syn::Error::new_spanned(
                second_expr,
                "Expected a string literal as the first argument, or as the second argument after a CuContext expression.",
            ));
        }
    };

    Ok(ParsedLogArgs {
        context_expr,
        msg_expr,
        param_exprs: exprs.iter().skip(msg_index + 1).collect(),
    })
}

/// Create reference of unused_variables to avoid warnings
/// ex: let _ = &tmp;
#[allow(unused)]
fn reference_unused_variables(input: TokenStream) -> TokenStream {
    // Attempt to parse the expressions to "use" them.
    // This ensures variables passed to the macro are considered used by the compiler.
    if let Ok(exprs) = parse_log_exprs(input.clone())
        && let Ok(parsed) = parse_log_args(&exprs)
    {
        let mut var_usages = Vec::new();
        if let Some(context_expr) = parsed.context_expr {
            var_usages.push(quote::quote! { let _ = &#context_expr; });
        }
        for expr in parsed.param_exprs {
            match expr {
                syn::Expr::Assign(assign_expr) => {
                    let value_expr = &assign_expr.right;
                    var_usages.push(quote::quote! { let _ = &#value_expr; });
                }
                _ => {
                    var_usages.push(quote::quote! { let _ = &#expr; });
                }
            }
        }
        return quote::quote! { { #(#var_usages;)* } }.into();
    }

    if let Ok(exprs) = parse_log_exprs(input.clone()) {
        let mut var_usages = Vec::new();
        for expr in exprs.iter() {
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

/// Returns `true` if `s` is a valid Rust identifier (used to tell an implicit
/// capture like `{hash}` apart from a positional `{}` / `{0}` placeholder).
fn is_ident(s: &str) -> bool {
    let mut chars = s.chars();
    match chars.next() {
        Some(c) if c == '_' || c.is_alphabetic() => {}
        _ => return false,
    }
    chars.all(|c| c == '_' || c.is_alphanumeric())
}

/// Extract identifiers implicitly captured by inline format placeholders,
/// mirroring std `format!` capture (Rust 1.58+): `{ident}` (or `{ident:spec}`)
/// captures `ident` from the caller's scope. Returns names in order of first
/// appearance. Escaped braces (`{{`, `}}`) and positional placeholders
/// (`{}`, `{0}`) are ignored.
fn extract_captured_idents(fmt: &str) -> Vec<String> {
    let mut names = Vec::new();
    let mut rest = fmt;
    while let Some(open) = rest.find('{') {
        if rest[open + 1..].starts_with('{') {
            rest = &rest[open + 2..]; // escaped "{{"
            continue;
        }
        let after = &rest[open + 1..];
        let Some(close) = after.find('}') else { break };
        // Only the part before a `:` format spec is the identifier.
        let name = after[..close].split(':').next().unwrap_or("").trim();
        if is_ident(name) && !names.iter().any(|n| n == name) {
            names.push(name.to_string());
        }
        rest = &after[close + 1..];
    }
    names
}

/// Create a log entry at the specified log level.
///
/// This is the internal macro implementation used by all the logging macros.
/// Users should use the public-facing macros: `debug!`, `info!`, `warning!`, `error!`, or `critical!`.
#[allow(unused)]
fn create_log_entry(input: TokenStream, level: CuLogLevel) -> TokenStream {
    use quote::quote;
    use syn::{Expr, ExprAssign, ExprLit, Lit, Token};

    let exprs = match parse_log_exprs(input) {
        Ok(exprs) => exprs,
        Err(err) => return err.to_compile_error().into(),
    };
    let parsed = match parse_log_args(&exprs) {
        Ok(parsed) => parsed,
        Err(err) => return err.to_compile_error().into(),
    };

    #[cfg(not(feature = "std"))]
    const STD: bool = false;
    #[cfg(feature = "std")]
    const STD: bool = true;

    let msg_expr = parsed.msg_expr;
    let (index, msg_str) = if let Expr::Lit(ExprLit {
        lit: Lit::Str(msg), ..
    }) = msg_expr
    {
        let s = msg.value();
        let index = match intern_string(&s) {
            Some(index) => index,
            None => {
                return syn::Error::new_spanned(msg_expr, "Failed to intern log string.")
                    .to_compile_error()
                    .into();
            }
        };
        (index, s)
    } else {
        return syn::Error::new_spanned(
            msg_expr,
            "The first parameter of the argument needs to be a string literal.",
        )
        .to_compile_error()
        .into();
    };

    let level_ident = match level {
        CuLogLevel::Debug => quote! { Debug },
        CuLogLevel::Info => quote! { Info },
        CuLogLevel::Warning => quote! { Warning },
        CuLogLevel::Error => quote! { Error },
        CuLogLevel::Critical => quote! { Critical },
    };

    // Implicit inline capture (std `format!` style, Rust 1.58+): `{ident}` in the
    // message captures `ident` from the caller's scope as a named param. Declared
    // before `named_params` so the owned exprs outlive the borrows below.
    let captured_exprs: Vec<Expr> = extract_captured_idents(&msg_str)
        .iter()
        .filter_map(|name| syn::parse_str::<Expr>(name).ok())
        .collect();

    // Partition unnamed vs named args (a = b treated as named)
    let mut unnamed_params = Vec::<&Expr>::new();
    let mut named_params = Vec::<(&Expr, &Expr)>::new();

    for expr in parsed.param_exprs {
        if let Expr::Assign(ExprAssign { left, right, .. }) = expr {
            named_params.push((left, right));
        } else {
            unnamed_params.push(expr);
        }
    }

    // Append implicitly-captured idents as named params. An explicit `name = value`
    // for the same name wins and suppresses the capture (no duplicate param).
    for expr in &captured_exprs {
        let name = quote!(#expr).to_string();
        let already_named = named_params
            .iter()
            .any(|(n, _)| quote!(#n).to_string() == name);
        if !already_named {
            named_params.push((expr, expr));
        }
    }

    // Build the CuLogEntry population tokens
    let unnamed_prints = unnamed_params.iter().map(|value| {
        quote! {
            let param = to_value(#value).expect("Failed to convert a parameter to a Value");
            log_entry.add_param(ANONYMOUS, param);
        }
    });

    let mut named_prints = Vec::with_capacity(named_params.len());
    for (name, value) in &named_params {
        let name_str = quote!(#name).to_string();
        let idx = match intern_string(&name_str) {
            Some(idx) => idx,
            None => {
                return syn::Error::new_spanned(name, "Failed to intern log parameter name.")
                    .to_compile_error()
                    .into();
            }
        };
        named_prints.push(quote! {
            let param = to_value(#value).expect("Failed to convert a parameter to a Value");
            log_entry.add_param(#idx, param);
        });
    }

    // ---------- For baremetal: build a defmt format literal and arg list ----------
    // defmt line: "<msg> | a={:?}, b={:?}, arg0={:?} ..."
    #[cfg(feature = "textlogs")]
    let (defmt_fmt_lit, defmt_args_unnamed_ts, defmt_args_named_ts, defmt_available) = {
        let defmt_fmt_lit = {
            let mut s = msg_str.clone();
            if !named_params.is_empty() {
                s.push_str(" |");
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

        (
            defmt_fmt_lit,
            defmt_args_unnamed_ts,
            defmt_args_named_ts,
            defmt_available,
        )
    };

    #[cfg(feature = "textlogs")]
    fn defmt_macro_path(level: CuLogLevel) -> TokenStream2 {
        let macro_ident = match level {
            CuLogLevel::Debug => quote! { defmt_debug },
            CuLogLevel::Info => quote! { defmt_info },
            CuLogLevel::Warning => quote! { defmt_warn },
            CuLogLevel::Error => quote! { defmt_error },
            CuLogLevel::Critical => quote! { defmt_error },
        };

        let (base, use_prelude) = match crate_name("cu29-log") {
            Ok(FoundCrate::Name(name)) => {
                let ident = proc_macro2::Ident::new(&name, Span::call_site());
                (quote! { ::#ident }, false)
            }
            Ok(FoundCrate::Itself) => (quote! { crate }, false),
            Err(_) => match crate_name("cu29") {
                Ok(FoundCrate::Name(name)) => {
                    let ident = proc_macro2::Ident::new(&name, Span::call_site());
                    (quote! { ::#ident }, true)
                }
                Ok(FoundCrate::Itself) => (quote! { crate }, true),
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

    #[cfg(feature = "textlogs")]
    let defmt_macro: TokenStream2 = defmt_macro_path(level);

    #[cfg(feature = "textlogs")]
    let maybe_inject_defmt: Option<TokenStream2> = if STD || !defmt_available {
        None // defmt is only emitted in no-std builds.
    } else {
        Some(quote! {
            #defmt_macro!(#defmt_fmt_lit, #(#defmt_args_unnamed_ts,)* #(#defmt_args_named_ts,)*);
        })
    };

    #[cfg(not(feature = "textlogs"))]
    let maybe_inject_defmt: Option<TokenStream2> = None; // defmt emission disabled

    let maybe_inject_origin: Option<TokenStream2> = parsed.context_expr.map(|ctx_expr| {
        quote! {
            let __cu29_log_ctx = &#ctx_expr;
            log_entry.set_origin(
                Some(__cu29_log_ctx.cl_id()),
                __cu29_log_ctx
                    .current_component_id()
                    .map(|component_id| component_id as u32),
                __cu29_log_ctx.task_index().map(|task_index| task_index as u32),
            );
        }
    });

    // Emit both: defmt (conditionally) + Copper structured logging
    quote! {{
        let mut log_entry = CuLogEntry::new(#index, CuLogLevel::#level_ident);
        #maybe_inject_origin
        #(#unnamed_prints)*
        #(#named_prints)*

        #maybe_inject_defmt

        #log_stmt
        #error_handling
    }}
    .into()
}

/// Log a debug message as a Copper structured log entry.
///
/// Accepted forms:
/// - `debug!("message {}", value)`
/// - `debug!(ctx, "message {}", value)`
///
/// When a `CuContext` is passed as the first argument, the
/// emitted structured log entry also captures the current Copper callback origin:
/// - `culistid`
/// - `component_id`
/// - `task_index` when the callback is running inside a task
///
/// The message argument must be a string literal. Only `{}` placeholders are supported.
/// Remaining arguments can be unnamed expressions or named fields written as `name = value`.
///
/// # Example
/// ```ignore
/// use cu29_log_derive::debug;
/// let a = 1;
/// let b = 2;
/// debug!("a = {}, b = {}", my_value = a, b); // named and unnamed parameters
///
/// # fn run(ctx: &cu29::context::CuContext) {
/// debug!(ctx, "processing {}", b); // same log plus runtime origin metadata
/// # }
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

/// Log an info message as a Copper structured log entry.
///
/// Accepted forms:
/// - `info!("message {}", value)`
/// - `info!(ctx, "message {}", value)`
///
/// Passing a `CuContext` as the first argument records the
/// current `culistid`, `component_id`, and `task_index` (when present) on the structured log
/// entry. The message argument must be a string literal, and remaining arguments may be unnamed
/// expressions or named fields written as `name = value`.
///
/// This macro will be compiled out if the max log level is set to a level higher than Info.
#[cfg(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
))]
#[proc_macro]
pub fn info(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Info)
}

/// Log a warning message as a Copper structured log entry.
///
/// Accepted forms:
/// - `warning!("message {}", value)`
/// - `warning!(ctx, "message {}", value)`
///
/// Passing a `CuContext` as the first argument records the
/// current `culistid`, `component_id`, and `task_index` (when present) on the structured log
/// entry. The message argument must be a string literal, and remaining arguments may be unnamed
/// expressions or named fields written as `name = value`.
///
/// This macro will be compiled out if the max log level is set to a level higher than Warning.
#[cfg(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
))]
#[proc_macro]
pub fn warning(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Warning)
}

/// Log an error message as a Copper structured log entry.
///
/// Accepted forms:
/// - `error!("message {}", value)`
/// - `error!(ctx, "message {}", value)`
///
/// Passing a `CuContext` as the first argument records the
/// current `culistid`, `component_id`, and `task_index` (when present) on the structured log
/// entry. The message argument must be a string literal, and remaining arguments may be unnamed
/// expressions or named fields written as `name = value`.
///
/// This macro will be compiled out if the max log level is set to a level higher than Error.
#[cfg(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    feature = "log-level-error",
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
))]
#[proc_macro]
pub fn error(input: TokenStream) -> TokenStream {
    create_log_entry(input, CuLogLevel::Error)
}

/// Log a critical message as a Copper structured log entry.
///
/// Accepted forms:
/// - `critical!("message {}", value)`
/// - `critical!(ctx, "message {}", value)`
///
/// Passing a `CuContext` as the first argument records the
/// current `culistid`, `component_id`, and `task_index` (when present) on the structured log
/// entry. The message argument must be a string literal, and remaining arguments may be unnamed
/// expressions or named fields written as `name = value`.
///
/// This macro is always compiled in, regardless of the max log level setting.
#[cfg(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    feature = "log-level-error",
    feature = "log-level-critical",
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
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

#[cfg(not(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
)))]
#[proc_macro]
pub fn info(input: TokenStream) -> TokenStream {
    reference_unused_variables(input)
}

#[cfg(not(any(
    feature = "log-level-debug",
    feature = "log-level-info",
    feature = "log-level-warning",
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
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
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
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
    cu29_default_log_level_debug,
    cu29_default_log_level_info,
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
    let expr = match syn::parse::<Expr>(input) {
        Ok(expr) => expr,
        Err(err) => return err.to_compile_error().into(),
    };
    let index = if let Expr::Lit(ExprLit {
        lit: Lit::Str(msg), ..
    }) = &expr
    {
        let msg = msg.value();
        match intern_string(&msg) {
            Some(index) => index,
            None => {
                return syn::Error::new_spanned(&expr, "Failed to intern log string.")
                    .to_compile_error()
                    .into();
            }
        }
    } else {
        return syn::Error::new_spanned(
            &expr,
            "The first parameter of the argument needs to be a string literal.",
        )
        .to_compile_error()
        .into();
    };
    quote! { #index }.into()
}

#[cfg(test)]
mod tests {
    use super::{extract_captured_idents, is_ident};

    #[test]
    fn captures_simple_named_placeholder() {
        assert_eq!(extract_captured_idents("Hash: {hash}"), vec!["hash"]);
    }

    #[test]
    fn captures_identifier_before_format_spec() {
        assert_eq!(
            extract_captured_idents("{hash:?} {size:>5}"),
            vec!["hash", "size"]
        );
    }

    #[test]
    fn ignores_positional_and_escaped_braces() {
        assert!(extract_captured_idents("{} {0} {{not_a_capture}}").is_empty());
    }

    #[test]
    fn mixes_positional_and_named_and_dedupes() {
        assert_eq!(
            extract_captured_idents("{} {hash} and again {hash}"),
            vec!["hash"]
        );
    }

    #[test]
    fn is_ident_rejects_non_identifiers() {
        assert!(is_ident("hash"));
        assert!(is_ident("_x1"));
        assert!(!is_ident("0"));
        assert!(!is_ident(""));
        assert!(!is_ident(":?"));
    }
}
