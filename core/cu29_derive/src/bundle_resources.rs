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
    let variants = ids.iter();
    let names = ids.iter().map(|ident| {
        let name = ident.to_string().to_case(Case::Snake);
        syn::LitStr::new(&name, ident.span())
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

        #[allow(dead_code)]
        pub const #names_ident: &[&str] = &[#(#names),*];
    };

    TokenStream::from(expanded)
}

struct BundleResourcesMacro {
    bundle: Path,
    ids: Vec<Ident>,
}

impl Parse for BundleResourcesMacro {
    fn parse(input: ParseStream<'_>) -> syn::Result<Self> {
        let bundle: Path = input.parse()?;
        input.parse::<Token![:]>()?;
        let ids: Punctuated<Ident, Token![,]> = input.parse_terminated(Ident::parse, Token![,])?;
        Ok(BundleResourcesMacro {
            bundle,
            ids: ids.into_iter().collect(),
        })
    }
}
