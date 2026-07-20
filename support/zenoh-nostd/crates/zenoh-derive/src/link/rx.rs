use proc_macro2::TokenStream;
use syn::DeriveInput;

pub fn derive_zlink_rx(input: &DeriveInput) -> syn::Result<TokenStream> {
    let ident = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();
    let variants = match &input.data {
        syn::Data::Enum(data_enum) => &data_enum.variants,
        _ => {
            return Err(syn::Error::new_spanned(
                input,
                "ZLInkRx can only be derived for enums",
            ));
        }
    }
    .iter()
    .map(|variant| &variant.ident);

    let variants_read = variants.clone().map(|ident| {
        quote::quote! {
            Self:: #ident (link) => zenoh_nostd::platform::ZLinkRx::read(link, buffer).await,
        }
    });

    let variants_read_exact = variants.clone().map(|ident| {
        quote::quote! {
            Self:: #ident (link) => zenoh_nostd::platform::ZLinkRx::read_exact(link, buffer).await,
        }
    });

    Ok(quote::quote! {
        impl #impl_generics zenoh_nostd::platform::ZLinkRx for #ident #ty_generics #where_clause {
            async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, zenoh_nostd::platform::LinkError> {
                match self {
                    #(#variants_read)*
                }
            }

            async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), zenoh_nostd::platform::LinkError> {
                match self {
                    #(#variants_read_exact)*
                }
            }
        }
    })
}
