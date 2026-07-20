use proc_macro2::TokenStream;
use syn::DeriveInput;

pub fn derive_zlink_tx(input: &DeriveInput) -> syn::Result<TokenStream> {
    let ident = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();
    let variants = match &input.data {
        syn::Data::Enum(data_enum) => &data_enum.variants,
        _ => {
            return Err(syn::Error::new_spanned(
                input,
                "ZLInkTx can only be derived for enums",
            ));
        }
    }
    .iter()
    .map(|variant| &variant.ident);

    let variants_write_all = variants.clone().map(|ident| {
        quote::quote! {
            Self:: #ident (link) => zenoh_nostd::platform::ZLinkTx::write_all(link, buffer).await,
        }
    });

    Ok(quote::quote! {
        impl #impl_generics zenoh_nostd::platform::ZLinkTx for #ident #ty_generics #where_clause {
            async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), zenoh_nostd::platform::LinkError> {
                match self {
                    #(#variants_write_all)*
                }
            }
        }
    })
}
