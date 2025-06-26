use cu29_intern_strs::intern_string;
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, LitStr};

/// A procedural macro that generates a `CuError` interning the given string.
///
/// Usage:
///
/// ```rust
/// use cu29_base_derive::cu_error;
///
/// let my_error = cu_error!("This is an error message");
///
/// // If you want to add a cause to the error (any core::error::Error), you can do so like this:
/// let my_error_with_cause = cu_error!("This is an error message").with_cause(my_cause);
///  ```
///

#[proc_macro]
pub fn cu_error(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as LitStr);
    let s = input.value();

    let index = match intern_string(s.as_str()) {
        Some(idx) => idx,
        None => panic!("Failed to intern string: {}", s),
    };
    let expanded = quote! {
        CuError::new(#index)
    };
    expanded.into()
}
