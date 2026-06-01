use proc_macro::TokenStream;
use quote::{format_ident, quote};
use syn::parse::{Parse, ParseStream};
use syn::{Error, Ident, Path, Token, parse_macro_input, punctuated::Punctuated};

use convert_case::{Case, Casing};

pub fn bundle_resources(input: TokenStream) -> TokenStream {
    let BundleResourcesMacro { bundle, ids } = parse_macro_input!(input as BundleResourcesMacro);

    if ids.is_empty() {
        return Error::new(
            proc_macro2::Span::call_site(),
            "bundle_resources! requires at least one id",
        )
        .to_compile_error()
        .into();
    }

    let bundle_ident = bundle
        .segments
        .last()
        .map(|seg| seg.ident.clone())
        .unwrap_or_else(|| Ident::new("Bundle", proc_macro2::Span::call_site()));
    let enum_ident = format_ident!("{}Id", bundle_ident);
    let count = ids.len();
    let variants = ids.iter().map(|entry| &entry.ident);
    let names = ids.iter().map(|entry| {
        entry.name.clone().unwrap_or_else(|| {
            let name = entry.ident.to_string().to_case(Case::Snake);
            syn::LitStr::new(&name, entry.ident.span())
        })
    });
    let names_ident_str = format!("{}_NAMES", enum_ident.to_string().to_case(Case::UpperSnake));
    let names_ident = format_ident!("{}", names_ident_str);

    let expanded = quote! {
        #[derive(Copy, Clone, Debug, Eq, PartialEq)]
        #[repr(usize)]
        pub enum #enum_ident {
            #(#variants),*
        }

        impl ::cu29::resource::ResourceId for #enum_ident {
            const COUNT: usize = #count;

            fn index(self) -> usize {
                self as usize
            }
        }

        impl ::cu29::resource::ResourceBundleDecl for #bundle {
            type Id = #enum_ident;
        }

        impl ::cu29::resource::NamedResourceBundleDecl for #bundle {
            const NAMES: &'static [&'static str] = #names_ident;
        }

        #[allow(dead_code)]
        pub const #names_ident: &[&str] = &[#(#names),*];
    };

    TokenStream::from(expanded)
}

struct BundleResourcesMacro {
    bundle: Path,
    ids: Vec<BundleResourceEntry>,
}

struct BundleResourceEntry {
    ident: Ident,
    name: Option<syn::LitStr>,
}

impl Parse for BundleResourcesMacro {
    fn parse(input: ParseStream<'_>) -> syn::Result<Self> {
        let bundle: Path = input.parse()?;
        input.parse::<Token![:]>()?;
        let ids: Punctuated<BundleResourceEntry, Token![,]> =
            input.parse_terminated(BundleResourceEntry::parse, Token![,])?;
        Ok(BundleResourcesMacro {
            bundle,
            ids: ids.into_iter().collect(),
        })
    }
}

impl Parse for BundleResourceEntry {
    fn parse(input: ParseStream<'_>) -> syn::Result<Self> {
        let ident: Ident = input.parse()?;
        let name = if input.peek(Token![=]) {
            input.parse::<Token![=]>()?;
            Some(input.parse()?)
        } else {
            None
        };
        Ok(Self { ident, name })
    }
}
