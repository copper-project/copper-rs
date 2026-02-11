use proc_macro::TokenStream;

/// No-op fallback derive used when Copper reflect is disabled.
///
/// This intentionally emits no code while accepting `#[reflect(...)]` helper attributes,
/// so existing derives compile without pulling `bevy_reflect`.
#[proc_macro_derive(Reflect, attributes(reflect))]
pub fn derive_reflect(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}
