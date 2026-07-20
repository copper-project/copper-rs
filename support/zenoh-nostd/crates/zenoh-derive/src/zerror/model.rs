use std::collections::HashMap;

use quote::quote;
use syn::{
    Attribute, Error, Expr, Ident, Lit, Meta, MetaNameValue, Result, Token, Variant,
    parse::{Parse, ParseStream},
    punctuated::Punctuated,
};

#[derive(Debug, Clone)]
pub(crate) struct ErrorEnum {
    pub(crate) name: Ident,
    pub(crate) doc: String,
    pub(crate) children: Vec<Ident>,
    pub(crate) variants: Vec<ErrorVariant>,
}

#[derive(Debug, Clone)]
pub(crate) struct ErrorVariant {
    pub(crate) name: Ident,
    pub(crate) doc: String,
    pub(crate) err: String,
    pub(crate) code: u8,
}

fn extract_attr_string(attrs: &[Attribute], name: &str) -> Result<String> {
    for attr in attrs {
        if !attr.path().is_ident(name) {
            continue;
        }

        if let Meta::NameValue(MetaNameValue {
            value:
                Expr::Lit(syn::ExprLit {
                    lit: Lit::Str(lit_str),
                    ..
                }),
            ..
        }) = attr.meta.clone()
        {
            return Ok(lit_str.value());
        } else {
            return Err(Error::new_spanned(
                attr,
                format!("expected #[{name} = \"...\"]"),
            ));
        }
    }

    Err(Error::new_spanned(
        quote! { #[#name = "..."] },
        format!("missing required attribute #[{name} = \"...\"]"),
    ))
}

fn extract_variant_code(variant: &syn::Variant) -> Result<u8> {
    if let Some((_, expr)) = &variant.discriminant
        && let Expr::Lit(syn::ExprLit {
            lit: Lit::Int(int), ..
        }) = expr
    {
        let val = int
            .base10_parse::<u8>()
            .map_err(|_| Error::new_spanned(int, "code must fit in u8"))?;
        return Ok(val);
    }
    Err(Error::new_spanned(
        &variant.ident,
        "each variant must have an explicit u8 discriminant, e.g. `VariantName = 3`",
    ))
}

fn extract_children(input: ParseStream) -> Result<Vec<Ident>> {
    if input.peek(Token![:]) {
        let _colon: Token![:] = input.parse()?;
        let punctuated: Punctuated<Ident, Token![+]> = Punctuated::parse_separated_nonempty(input)?;
        Ok(punctuated.into_iter().collect())
    } else {
        Ok(Vec::new())
    }
}

impl Parse for ErrorEnum {
    fn parse(input: ParseStream) -> Result<Self> {
        let attrs = input.call(Attribute::parse_outer)?;
        let _vis: syn::Visibility = input.parse()?;
        let _enum_token: Token![enum] = input.parse()?;

        let name: Ident = input.parse()?;

        let children = extract_children(input)?;

        let content;
        syn::braced!(content in input);
        let variants = content.parse_terminated(Variant::parse, Token![,])?;

        let doc = extract_attr_string(&attrs, "doc")?;

        let variants = variants
            .into_iter()
            .map(|variant| {
                let name = variant.ident.clone();
                let doc = extract_attr_string(&variant.attrs, "doc")?;
                let err = extract_attr_string(&variant.attrs, "err")?;
                let code = extract_variant_code(&variant)?;
                Ok(ErrorVariant {
                    name,
                    doc,
                    err,
                    code,
                })
            })
            .collect::<Result<Vec<_>>>()?;

        Ok(ErrorEnum {
            name,
            doc,
            children,
            variants,
        })
    }
}

#[derive(Debug)]
pub struct DeclaredErrors {
    errors: HashMap<Ident, ErrorEnum>,
}

impl DeclaredErrors {
    pub fn new() -> Self {
        Self {
            errors: HashMap::new(),
        }
    }

    pub fn check(&self, error: &ErrorEnum) -> Result<()> {
        for child in &error.children {
            if !self.contains_key(child) {
                return Err(Error::new_spanned(
                    child,
                    format!("error enum `{}` used as child but not declared", child),
                ));
            }
        }

        for variant in &error.variants {
            for existing_error in self.values() {
                for existing_variant in &existing_error.variants {
                    if variant.name == existing_variant.name {
                        return Err(Error::new_spanned(
                            &variant.name,
                            format!(
                                "variant name `{}` is already used in error enum `{}`",
                                variant.name, existing_error.name
                            ),
                        ));
                    }
                    if variant.code == existing_variant.code {
                        return Err(Error::new_spanned(
                            &variant.name,
                            format!(
                                "variant code `{}` is already used in error enum `{}`",
                                variant.code, existing_error.name
                            ),
                        ));
                    }
                }
            }
        }

        Ok(())
    }

    pub fn insert(&mut self, error: ErrorEnum) -> Result<()> {
        if self.errors.contains_key(&error.name) {
            return Err(Error::new_spanned(
                &error.name,
                format!("error enum `{}` is already declared", error.name),
            ));
        }

        self.check(&error)?;
        self.errors.insert(error.name.clone(), error);

        Ok(())
    }

    pub fn get(&self, name: &Ident) -> Option<&ErrorEnum> {
        self.errors.get(name)
    }

    pub fn values(&self) -> impl Iterator<Item = &ErrorEnum> {
        self.errors.values()
    }

    pub fn contains_key(&self, name: &Ident) -> bool {
        self.errors.contains_key(name)
    }
}

impl Parse for DeclaredErrors {
    fn parse(input: ParseStream) -> Result<Self> {
        let mut declared_errors = DeclaredErrors::new();

        while !input.is_empty() {
            let error_enum: ErrorEnum = input.parse()?;
            declared_errors.insert(error_enum)?;
        }

        Ok(declared_errors)
    }
}
