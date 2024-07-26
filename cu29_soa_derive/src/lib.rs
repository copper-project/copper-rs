use proc_macro::TokenStream;
use quote::{format_ident, quote};
use syn::{parse_macro_input, DeriveInput, Fields, Lit, Data, LitInt};

#[proc_macro_attribute]
pub fn soa(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as DeriveInput);

    let name = &input.ident;
    let data = match &input.data {
        Data::Struct(data) => data,
        _ => panic!("Only structs are supported"),
    };
    let fields = match &data.fields {
        Fields::Named(fields) => &fields.named,
        _ => panic!("Only named fields are supported"),
    };

    let mut field_names = vec![];
    let mut field_names_mut = vec![];
    let mut field_names_range = vec![];
    let mut field_names_range_mut = vec![];
    let mut field_types = vec![];

    for field in fields {
        let field_name = field.ident.as_ref().unwrap();
        let field_type = &field.ty;
        field_names.push(field_name);
        field_types.push(field_type);
        field_names_mut.push(format_ident!("{}_mut", field_name));
        field_names_range.push(format_ident!("{}_range", field_name));
        field_names_range_mut.push(format_ident!("{}_range_mut", field_name));

    }

    let soa_struct_name = format_ident!("{}Soa", name);

    let expanded = quote! {
        #input

        pub struct #soa_struct_name<const N: usize> {
            #(pub #field_names: [#field_types; N]),*
        }

        impl<const N: usize> #soa_struct_name<N> {
            pub fn new(default: #name) -> Self {
                Self {
                    #( #field_names: [default.#field_names; N] ),*
                }
            }

            pub fn set(&mut self, index: usize, value: #name) {
                assert!(index < N, "Index out of bounds");
                #( self.#field_names[index] = value.#field_names; )*
            }

            pub fn get(&self, index: usize) -> #name {
                assert!(index < N, "Index out of bounds");
                #name {
                    #( #field_names: self.#field_names[index], )*
                }
            }

             #(
                pub fn #field_names(&self) -> &[#field_types] {
                    &self.#field_names
                }

                pub fn #field_names_mut(&mut self) -> &mut [#field_types] {
                    &mut self.#field_names
                }

                pub fn #field_names_range(&self, range: std::ops::Range<usize>) -> &[#field_types] {
                    &self.#field_names[range]
                }

                pub fn #field_names_range_mut(&mut self, range: std::ops::Range<usize>) -> &mut [#field_types] {
                    &mut self.#field_names[range]
                }
            )*

        }
    };
    expanded.into()
}


