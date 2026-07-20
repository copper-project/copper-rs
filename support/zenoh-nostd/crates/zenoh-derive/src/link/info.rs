use proc_macro2::TokenStream;
use syn::DeriveInput;

pub fn derive_zlink_info(input: &DeriveInput) -> syn::Result<TokenStream> {
    let ident = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();
    let variants = match &input.data {
        syn::Data::Enum(data_enum) => &data_enum.variants,
        _ => {
            return Err(syn::Error::new_spanned(
                input,
                "ZLInkInfo can only be derived for enums",
            ));
        }
    }
    .iter()
    .map(|variant| &variant.ident);

    let variants_mtu = variants.clone().map(|ident| {
        quote::quote! {
            Self:: #ident (link) => zenoh_nostd::platform::ZLinkInfo::mtu(link),
        }
    });

    let variants_streamed = variants.clone().map(|ident| {
        quote::quote! {
            Self:: #ident (link) => zenoh_nostd::platform::ZLinkInfo::is_streamed(link),
        }
    });

    Ok(quote::quote! {
        impl #impl_generics zenoh_nostd::platform::ZLinkInfo for #ident #ty_generics #where_clause {
            fn mtu(&self) -> u16 {
                match self {
                    #(#variants_mtu)*
                }
            }

            fn is_streamed(&self) -> bool {
                match self {
                    #(#variants_streamed)*
                }
            }
        }
    })
}
