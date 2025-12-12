use proc_macro::TokenStream;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{
    braced, parenthesized, parse_macro_input, punctuated::Punctuated, Error, Ident, Token, Type,
    WherePredicate,
};

pub fn resources(input: TokenStream) -> TokenStream {
    let ResourcesMacro {
        generics,
        where_preds,
        entries,
    } = parse_macro_input!(input as ResourcesMacro);

    if entries.is_empty() {
        return Error::new(
            proc_macro2::Span::call_site(),
            "resources! requires at least one entry",
        )
        .to_compile_error()
        .into();
    }

    let mut needs_lifetime = false;
    let mut fields = Vec::new();
    let mut inits = Vec::new();

    for entry in entries {
        let name = entry.name;
        let binding = name.to_string();
        let ty = entry.ty;
        let (field_ty, access) = match entry.kind {
            ResourceKind::Owned => (
                quote! { ::cu29::resource::Owned<#ty> },
                quote! { take::<#ty> },
            ),
            ResourceKind::Borrowed | ResourceKind::Shared => {
                needs_lifetime = true;
                (
                    quote! { ::cu29::resource::Borrowed<'r, #ty> },
                    quote! { borrow::<#ty> },
                )
            }
        };
        fields.push(quote! { pub #name: #field_ty });
        inits.push(quote! {
            #name: {
                let key = mapping
                    .get(#binding)
                    .ok_or_else(|| ::cu29::CuError::from(concat!("missing `", #binding, "` resource binding")))?;
                manager.#access(key.typed())?
            }
        });
    }

    let where_clause = if where_preds.is_empty() {
        quote! {}
    } else {
        quote! { where #(#where_preds),* }
    };

    if !generics.is_empty() {
        let marker_ty = quote! { ::core::marker::PhantomData<(#(#generics),*)> };
        fields.push(quote! { _marker: #marker_ty });
        inits.push(quote! { _marker: ::core::marker::PhantomData });
    }

    let generic_params = if generics.is_empty() {
        quote! {}
    } else {
        quote! { <#(#generics),*> }
    };

    let lifetime_params = if needs_lifetime {
        if generics.is_empty() {
            quote! { <'r> }
        } else {
            quote! { <'r, #(#generics),*> }
        }
    } else {
        generic_params.clone()
    };

    let expanded = if needs_lifetime {
        quote! {
            pub struct Resources #lifetime_params {
                #(#fields),*
            }

            impl #lifetime_params ::cu29::resource::ResourceBindings<'r> for Resources #lifetime_params
            #where_clause
            {
                fn from_bindings(
                    manager: &'r mut ::cu29::resource::ResourceManager,
                    mapping: Option<&::cu29::resource::ResourceMapping>,
                ) -> ::cu29::CuResult<Self> {
                    let mapping = mapping.ok_or_else(|| ::cu29::CuError::from("missing resource bindings"))?;
                    Ok(Self { #(#inits),* })
                }
            }
        }
    } else {
        quote! {
            pub struct Resources #lifetime_params {
                #(#fields),*
            }

            impl #lifetime_params ::cu29::resource::ResourceBindings<'_> for Resources #lifetime_params
            #where_clause
            {
                fn from_bindings(
                    manager: &mut ::cu29::resource::ResourceManager,
                    mapping: Option<&::cu29::resource::ResourceMapping>,
                ) -> ::cu29::CuResult<Self> {
                    let mapping = mapping.ok_or_else(|| ::cu29::CuError::from("missing resource bindings"))?;
                    Ok(Self { #(#inits),* })
                }
            }
        }
    };

    TokenStream::from(expanded)
}

struct ResourcesMacro {
    generics: Vec<Ident>,
    where_preds: Vec<WherePredicate>,
    entries: Vec<ResourceEntry>,
}

impl Parse for ResourcesMacro {
    fn parse(input: ParseStream<'_>) -> syn::Result<Self> {
        let mut generics = Vec::new();
        if input.peek(Token![for]) {
            input.parse::<Token![for]>()?;
            input.parse::<Token![<]>()?;
            loop {
                if input.peek(Token![>]) {
                    input.parse::<Token![>]>()?;
                    break;
                }
                let ident: Ident = input.parse()?;
                generics.push(ident);
                if input.peek(Token![,]) {
                    input.parse::<Token![,]>()?;
                } else {
                    input.parse::<Token![>]>()?;
                    break;
                }
            }
        } else if input.peek(syn::token::Paren) {
            // legacy form: resources!((S, E) { ... })
            let content;
            parenthesized!(content in input);
            let params: Punctuated<Ident, Token![,]> =
                content.parse_terminated(Ident::parse, Token![,])?;
            generics.extend(params.into_iter());
        }

        let mut where_preds = Vec::new();
        if input.peek(Token![where]) {
            input.parse::<Token![where]>()?;
            loop {
                if input.peek(syn::token::Brace) {
                    break;
                }
                let pred: WherePredicate = input.parse()?;
                where_preds.push(pred);
                if input.peek(Token![,]) {
                    input.parse::<Token![,]>()?;
                } else {
                    break;
                }
            }
        }

        let content;
        braced!(content in input);
        let entries: Punctuated<ResourceEntry, Token![,]> =
            content.parse_terminated(ResourceEntry::parse, Token![,])?;

        Ok(ResourcesMacro {
            generics,
            where_preds,
            entries: entries.into_iter().collect(),
        })
    }
}

struct ResourceEntry {
    name: Ident,
    kind: ResourceKind,
    ty: Type,
}

impl Parse for ResourceEntry {
    fn parse(input: ParseStream<'_>) -> syn::Result<Self> {
        let name: Ident = input.parse()?;
        input.parse::<Token![=>]>()?;
        let kind_ident: Ident = input.parse()?;
        let kind = match kind_ident.to_string().as_str() {
            "Owned" => ResourceKind::Owned,
            "Shared" => ResourceKind::Shared,
            "Borrowed" => ResourceKind::Borrowed,
            other => {
                return Err(Error::new(
                    kind_ident.span(),
                    format!("unknown kind `{}`", other),
                ))
            }
        };
        let _lt = input.parse::<Token![<]>()?;
        let ty: Type = input.parse()?;
        input.parse::<Token![>]>()?;
        Ok(ResourceEntry { name, kind, ty })
    }
}

enum ResourceKind {
    Owned,
    Shared,
    Borrowed,
}
