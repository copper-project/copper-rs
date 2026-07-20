use proc_macro2::TokenStream;
use syn::DeriveInput;

pub mod info;
pub mod rx;
pub mod tx;

pub fn derive_zlink(input: &DeriveInput) -> syn::Result<TokenStream> {
    let ident = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();
    let (tx_type, rx_type) = extract_zlink_types(input)?;
    let variants = match &input.data {
        syn::Data::Enum(data_enum) => &data_enum.variants,
        _ => {
            return Err(syn::Error::new_spanned(
                input,
                "ZLInk can only be derived for enums",
            ));
        }
    }
    .iter()
    .map(|variant| &variant.ident);

    let variants_split = variants.clone().map(|ident| {
        quote::quote! {
            Self:: #ident (link) => {
                let (tx, rx) = zenoh_nostd::platform::ZLink::split(link);
                (Self::Tx:: #ident (tx), Self::Rx:: #ident (rx))
            },
        }
    });

    Ok(quote::quote! {
        impl #impl_generics zenoh_nostd::platform::ZLink for #ident #ty_generics #where_clause {
            type Tx<'link> = #tx_type where Self: 'link;
            type Rx<'link> = #rx_type where Self: 'link;

            fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
                match self {
                    #(#variants_split)*
                }
            }
        }
    })
}

fn extract_zlink_types(input: &DeriveInput) -> syn::Result<(syn::Type, syn::Type)> {
    for attr in &input.attrs {
        if !attr.path().is_ident("zenoh") {
            continue;
        }

        let result: Result<(syn::Type, syn::Type), syn::Error> =
            attr.parse_args_with(|input: syn::parse::ParseStream| {
                let name: syn::Ident = input.parse()?;
                if name != "ZLink" {
                    return Err(syn::Error::new(name.span(), "Expected 'ZLink'"));
                }

                input.parse::<syn::Token![=]>()?;

                let content;
                syn::parenthesized!(content in input);

                let tx: syn::Type = content.parse()?;
                content.parse::<syn::Token![,]>()?;
                let rx: syn::Type = content.parse()?;

                Ok((tx, rx))
            });

        if let Ok(types) = result {
            return Ok(types);
        }
    }

    Err(syn::Error::new_spanned(
        input,
        "Missing #[zenoh(ZLink = (TxType, RxType))] attribute",
    ))
}
