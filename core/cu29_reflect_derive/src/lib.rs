use proc_macro::TokenStream;
use quote::{ToTokens, quote};
use syn::{Attribute, DeriveInput, parse_macro_input};

/// Lightweight fallback derive used when Copper reflect is disabled.
///
/// This keeps accepting `#[reflect(...)]` helper attributes without pulling
/// `bevy_reflect`, while still emitting the `TypePath` impl that runtime code
/// generation now relies on for schema/introspection metadata.
#[proc_macro_derive(Reflect, attributes(reflect))]
pub fn derive_reflect(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    derive_reflect_impl(&input).into()
}

fn derive_reflect_impl(input: &DeriveInput) -> proc_macro2::TokenStream {
    if reflect_disables_type_path(&input.attrs) {
        return proc_macro2::TokenStream::new();
    }

    let ident = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();

    quote! {
        impl #impl_generics cu29::reflect::TypePath for #ident #ty_generics #where_clause {}
    }
}

fn reflect_disables_type_path(attrs: &[Attribute]) -> bool {
    attrs
        .iter()
        .filter(|attr| attr.path().is_ident("reflect"))
        .any(|attr| {
            attr.meta
                .to_token_stream()
                .to_string()
                .chars()
                .filter(|ch| !ch.is_whitespace())
                .collect::<String>()
                .contains("type_path=false")
        })
}

#[cfg(test)]
mod tests {
    use super::{derive_reflect_impl, reflect_disables_type_path};
    use syn::parse_quote;

    #[test]
    fn emits_type_path_impl_by_default() {
        let input = parse_quote! {
            struct Payload;
        };

        let output = derive_reflect_impl(&input).to_string();

        assert!(output.contains("impl cu29 :: reflect :: TypePath for Payload"));
    }

    #[test]
    fn respects_type_path_false() {
        let input = parse_quote! {
            #[reflect(from_reflect = false, type_path = false)]
            struct Payload<T>(T);
        };

        assert!(derive_reflect_impl(&input).is_empty());
        assert!(reflect_disables_type_path(&input.attrs));
    }
}
