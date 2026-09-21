//! Safety-case attribute expansion.

use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;
use std::collections::BTreeSet;
use syn::parse::Parser;
use syn::punctuated::Punctuated;
use syn::{Block, Expr, ItemFn, Lit, LitStr, Stmt, Token, parse_macro_input};

#[derive(Debug, Clone)]
struct ParsedSafetyCheck {
    check_id: String,
    requirement_id: String,
    kind: &'static str,
}

pub(super) fn expand(args: TokenStream, input: TokenStream) -> TokenStream {
    let case_id = parse_macro_input!(args as LitStr).value();
    if let Err(err) = validate_case_id(&case_id) {
        return err.to_compile_error().into();
    }

    let function = parse_macro_input!(input as ItemFn);
    let checks = match collect_safety_checks(&case_id, &function.block) {
        Ok(checks) => checks,
        Err(err) => return err.to_compile_error().into(),
    };

    if checks.is_empty() {
        return syn::Error::new_spanned(
            &function.sig.ident,
            format!("safety case '{case_id}' must contain at least one safety_check! or safety_check_eq!"),
        )
        .to_compile_error()
        .into();
    }

    let function_ident = &function.sig.ident;
    let checks_tokens = checks.iter().map(|check| {
        let check_id = &check.check_id;
        let requirement_id = &check.requirement_id;
        let kind = check.kind;
        quote! {
            ::cu29::safety::SafetyCheckRef {
                check_id: #check_id,
                requirement_id: #requirement_id,
                kind: #kind,
            }
        }
    });

    quote! {
        #function

        #[cfg(feature = "safety-ids")]
        ::cu29::safety::inventory::submit! {
            ::cu29::safety::SafetyCaseRef {
                package: env!("CARGO_PKG_NAME"),
                case_id: #case_id,
                function: stringify!(#function_ident),
                module_path: module_path!(),
                file: file!(),
                checks: &[#(#checks_tokens),*],
            }
        }
    }
    .into()
}

fn collect_safety_checks(
    case_id: &str,
    block: &Block,
) -> Result<Vec<ParsedSafetyCheck>, syn::Error> {
    let mut checks = Vec::new();
    collect_safety_checks_from_block(block, &mut checks)?;

    let mut ids = BTreeSet::new();
    for check in &checks {
        validate_check_id(case_id, &check.check_id)?;
        validate_requirement_id(&check.requirement_id)?;
        if !ids.insert(check.check_id.clone()) {
            return Err(syn::Error::new(
                Span::call_site(),
                format!("duplicate safety check ID '{}'", check.check_id),
            ));
        }
    }

    Ok(checks)
}

fn collect_safety_checks_from_block(
    block: &Block,
    checks: &mut Vec<ParsedSafetyCheck>,
) -> Result<(), syn::Error> {
    for stmt in &block.stmts {
        collect_safety_checks_from_stmt(stmt, checks)?;
    }
    Ok(())
}

fn collect_safety_checks_from_stmt(
    stmt: &Stmt,
    checks: &mut Vec<ParsedSafetyCheck>,
) -> Result<(), syn::Error> {
    match stmt {
        Stmt::Local(local) => {
            if let Some(init) = &local.init {
                collect_safety_checks_from_expr(&init.expr, checks)?;
                if let Some((_else, expr)) = &init.diverge {
                    collect_safety_checks_from_expr(expr, checks)?;
                }
            }
        }
        Stmt::Item(_) => {}
        Stmt::Expr(expr, _) => collect_safety_checks_from_expr(expr, checks)?,
        Stmt::Macro(stmt_macro) => {
            if let Some(check) = parse_safety_check_macro(&stmt_macro.mac)? {
                checks.push(check);
            }
        }
    }
    Ok(())
}

fn collect_safety_checks_from_expr(
    expr: &Expr,
    checks: &mut Vec<ParsedSafetyCheck>,
) -> Result<(), syn::Error> {
    match expr {
        Expr::Array(expr) => {
            for elem in &expr.elems {
                collect_safety_checks_from_expr(elem, checks)?;
            }
        }
        Expr::Assign(expr) => {
            collect_safety_checks_from_expr(&expr.left, checks)?;
            collect_safety_checks_from_expr(&expr.right, checks)?;
        }
        Expr::Async(expr) => collect_safety_checks_from_block(&expr.block, checks)?,
        Expr::Await(expr) => collect_safety_checks_from_expr(&expr.base, checks)?,
        Expr::Binary(expr) => {
            collect_safety_checks_from_expr(&expr.left, checks)?;
            collect_safety_checks_from_expr(&expr.right, checks)?;
        }
        Expr::Block(expr) => collect_safety_checks_from_block(&expr.block, checks)?,
        Expr::Break(expr) => {
            if let Some(value) = &expr.expr {
                collect_safety_checks_from_expr(value, checks)?;
            }
        }
        Expr::Call(expr) => {
            collect_safety_checks_from_expr(&expr.func, checks)?;
            for arg in &expr.args {
                collect_safety_checks_from_expr(arg, checks)?;
            }
        }
        Expr::Cast(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::Closure(expr) => collect_safety_checks_from_expr(&expr.body, checks)?,
        Expr::Field(expr) => collect_safety_checks_from_expr(&expr.base, checks)?,
        Expr::ForLoop(expr) => {
            collect_safety_checks_from_expr(&expr.expr, checks)?;
            collect_safety_checks_from_block(&expr.body, checks)?;
        }
        Expr::Group(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::If(expr) => {
            collect_safety_checks_from_expr(&expr.cond, checks)?;
            collect_safety_checks_from_block(&expr.then_branch, checks)?;
            if let Some((_else, else_expr)) = &expr.else_branch {
                collect_safety_checks_from_expr(else_expr, checks)?;
            }
        }
        Expr::Index(expr) => {
            collect_safety_checks_from_expr(&expr.expr, checks)?;
            collect_safety_checks_from_expr(&expr.index, checks)?;
        }
        Expr::Let(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::Loop(expr) => collect_safety_checks_from_block(&expr.body, checks)?,
        Expr::Macro(expr_macro) => {
            if let Some(check) = parse_safety_check_macro(&expr_macro.mac)? {
                checks.push(check);
            }
        }
        Expr::Match(expr) => {
            collect_safety_checks_from_expr(&expr.expr, checks)?;
            for arm in &expr.arms {
                if let syn::Pat::Guard(pat_guard) = &arm.pat {
                    collect_safety_checks_from_expr(&pat_guard.guard, checks)?;
                }
                collect_safety_checks_from_expr(&arm.body, checks)?;
            }
        }
        Expr::MethodCall(expr) => {
            collect_safety_checks_from_expr(&expr.receiver, checks)?;
            for arg in &expr.args {
                collect_safety_checks_from_expr(arg, checks)?;
            }
        }
        Expr::Paren(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::Reference(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::Repeat(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::Return(expr) => {
            if let Some(value) = &expr.expr {
                collect_safety_checks_from_expr(value, checks)?;
            }
        }
        Expr::Struct(expr) => {
            for field in &expr.fields {
                collect_safety_checks_from_expr(&field.expr, checks)?;
            }
            if let Some(rest) = &expr.rest {
                collect_safety_checks_from_expr(rest, checks)?;
            }
        }
        Expr::Try(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::TryBlock(expr) => collect_safety_checks_from_block(&expr.block, checks)?,
        Expr::Tuple(expr) => {
            for elem in &expr.elems {
                collect_safety_checks_from_expr(elem, checks)?;
            }
        }
        Expr::Unary(expr) => collect_safety_checks_from_expr(&expr.expr, checks)?,
        Expr::Unsafe(expr) => collect_safety_checks_from_block(&expr.block, checks)?,
        Expr::While(expr) => {
            collect_safety_checks_from_expr(&expr.cond, checks)?;
            collect_safety_checks_from_block(&expr.body, checks)?;
        }
        Expr::Yield(expr) => {
            if let Some(value) = &expr.expr {
                collect_safety_checks_from_expr(value, checks)?;
            }
        }
        _ => {}
    }
    Ok(())
}

fn parse_safety_check_macro(mac: &syn::Macro) -> Result<Option<ParsedSafetyCheck>, syn::Error> {
    let Some(segment) = mac.path.segments.last() else {
        return Ok(None);
    };

    let kind = match segment.ident.to_string().as_str() {
        "safety_check" => "assert",
        "safety_check_eq" => "assert_eq",
        _ => return Ok(None),
    };

    let args = Punctuated::<Expr, Token![,]>::parse_terminated.parse2(mac.tokens.clone())?;
    let min_args = if kind == "assert_eq" { 4 } else { 3 };
    if args.len() < min_args {
        return Err(syn::Error::new_spanned(
            mac,
            format!(
                "{}! expects at least {} arguments: check ID, requirement ID, and assertion inputs",
                segment.ident, min_args
            ),
        ));
    }

    let check_id = string_literal_from_expr(&args[0], "safety check ID")?;
    let requirement_id = string_literal_from_expr(&args[1], "requirement ID")?;

    Ok(Some(ParsedSafetyCheck {
        check_id,
        requirement_id,
        kind,
    }))
}

fn string_literal_from_expr(expr: &Expr, label: &str) -> Result<String, syn::Error> {
    let Expr::Lit(expr_lit) = expr else {
        return Err(syn::Error::new_spanned(
            expr,
            format!("{label} must be a string literal"),
        ));
    };
    let Lit::Str(value) = &expr_lit.lit else {
        return Err(syn::Error::new_spanned(
            expr,
            format!("{label} must be a string literal"),
        ));
    };
    Ok(value.value())
}

fn validate_case_id(case_id: &str) -> Result<(), syn::Error> {
    validate_id(case_id, "TEST", false)
}

fn validate_check_id(case_id: &str, check_id: &str) -> Result<(), syn::Error> {
    validate_id(check_id, "TEST", true)?;
    if !check_id.starts_with(case_id) {
        return Err(syn::Error::new(
            Span::call_site(),
            format!("safety check ID '{check_id}' must start with case ID '{case_id}'"),
        ));
    }
    Ok(())
}

fn validate_requirement_id(requirement_id: &str) -> Result<(), syn::Error> {
    validate_id(requirement_id, "REQ", false)
}

fn validate_id(value: &str, kind: &str, allow_check_suffix: bool) -> Result<(), syn::Error> {
    let mut parts = value.split('-');
    let Some(prefix) = parts.next() else {
        return invalid_id(value, kind, allow_check_suffix);
    };
    let Some(actual_kind) = parts.next() else {
        return invalid_id(value, kind, allow_check_suffix);
    };
    let Some(number) = parts.next() else {
        return invalid_id(value, kind, allow_check_suffix);
    };

    if !prefix.chars().all(|ch| ch.is_ascii_uppercase())
        || actual_kind != kind
        || number.len() != 3
        || !number.chars().all(|ch| ch.is_ascii_digit())
    {
        return invalid_id(value, kind, allow_check_suffix);
    }

    match parts.next() {
        None => Ok(()),
        Some(check_suffix) if allow_check_suffix && check_suffix.starts_with('C') => {
            let digits = &check_suffix[1..];
            if digits.is_empty() || !digits.chars().all(|ch| ch.is_ascii_digit()) {
                return invalid_id(value, kind, allow_check_suffix);
            }
            if parts.next().is_some() {
                return invalid_id(value, kind, allow_check_suffix);
            }
            Ok(())
        }
        _ => invalid_id(value, kind, allow_check_suffix),
    }
}

fn invalid_id(value: &str, kind: &str, allow_check_suffix: bool) -> Result<(), syn::Error> {
    let suffix = if allow_check_suffix {
        " or PREFIX-TEST-001-C1"
    } else {
        ""
    };
    Err(syn::Error::new(
        Span::call_site(),
        format!("invalid ID '{value}', expected PREFIX-{kind}-001{suffix}"),
    ))
}
