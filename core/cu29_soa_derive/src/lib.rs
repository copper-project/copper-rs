mod format;
#[cfg(feature = "macro_debug")]
use format::{highlight_rust_code, rustfmt_generated_code};
use proc_macro::TokenStream;
use quote::{format_ident, quote, ToTokens};
use syn::{parse_macro_input, Data, DeriveInput, Fields, Type};

/// Build a fixed sized SoA (Structure of Arrays) from a struct.
/// The outputted SoA will be suitable for in place storage in messages and should be
/// easier for the compiler to vectorize.
///
/// for example:
///
/// ```ignore
/// #[derive(Soa)]
/// struct MyStruct {
///    a: i32,
///    b: f32,
/// }
/// ```
///
/// will generate:
/// ```ignore
/// pub struct MyStructSoa<const N: usize> {
///     pub a: [i32; N],
///     pub b: [f32; N],
/// }
/// ```
///
/// You can then use the generated struct to store multiple
/// instances of the original struct in an SoA format.
///
/// ```ignore
/// // makes an SOA with a default value
/// let soa1: MyStructSoa<8> = XyzSoa::new(MyStruct{ a: 1, b: 2.3 });
/// ```
///
/// Then you can access the fields of the SoA as slices:
/// ```ignore
/// let a = soa1.a();
/// let b = soa1.b();
/// ```
///
/// You can also access a range of the fields:
/// ```ignore
/// let a = soa1.a_range(0..4);
/// let b = soa1.b_range(0..4);
/// ```
///
/// You can also modify the fields of the SoA:
/// ```ignore
/// soa1.a_mut()[0] = 42;
/// soa1.b_mut()[0] = 42.0;
/// ```
///
/// You can also modify a range of the fields:
/// ```ignore
/// soa1.a_range_mut(0..4)[0] = 42;
/// soa1.b_range_mut(0..4)[0] = 42.0;
/// ```
///
/// You can also apply a function to all the fields of the SoA:
/// ```ignore
/// soa1.apply(|a, b| {
///    (a + 1, b + 1.0)
/// });
/// ```
#[proc_macro_derive(Soa)]
pub fn derive_soa(input: TokenStream) -> TokenStream {
    use syn::TypePath;

    let input = parse_macro_input!(input as DeriveInput);
    let visibility = &input.vis;

    let name = &input.ident;
    let module_name = format_ident!("{}_soa", name.to_string().to_lowercase());
    let soa_struct_name = format_ident!("{}Soa", name);

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
    let mut unique_imports = vec![];
    let mut unique_import_names = vec![];

    fn is_primitive(type_name: &str) -> bool {
        matches!(
            type_name,
            "i8" | "i16"
                | "i32"
                | "i64"
                | "i128"
                | "u8"
                | "u16"
                | "u32"
                | "u64"
                | "u128"
                | "f32"
                | "f64"
                | "bool"
                | "char"
                | "str"
                | "usize"
                | "isize"
        )
    }

    for field in fields {
        let field_name = field.ident.as_ref().unwrap();
        let field_type = &field.ty;
        field_names.push(field_name);
        field_names_mut.push(format_ident!("{}_mut", field_name));
        field_names_range.push(format_ident!("{}_range", field_name));
        field_names_range_mut.push(format_ident!("{}_range_mut", field_name));
        field_types.push(field_type);

        if let Type::Path(TypePath { path, .. }) = field_type {
            let type_name = path.segments.last().unwrap().ident.to_string();
            let path_str = path.to_token_stream().to_string();

            if !is_primitive(&type_name) && !unique_import_names.contains(&path_str) {
                unique_imports.push(path.clone());
                unique_import_names.push(path_str);
            }
        }
    }

    let soa_struct_name_iterator = format_ident!("{}Iterator", name);

    let iterator = quote! {
        pub struct #soa_struct_name_iterator<'a, const N: usize> {
            soa_struct: &'a #soa_struct_name<N>,
            current: usize,
        }

        impl<'a, const N: usize> #soa_struct_name_iterator<'a, N> {
            pub fn new(soa_struct: &'a #soa_struct_name<N>) -> Self {
                Self {
                    soa_struct,
                    current: 0,
                }
            }
        }

        impl<'a, const N: usize> Iterator for #soa_struct_name_iterator<'a, N> {
            type Item = super::#name;

            fn next(&mut self) -> Option<Self::Item> {
                if self.current < self.soa_struct.len {
                    let item = self.soa_struct.get(self.current); // Reuse `get` method
                    self.current += 1;
                    Some(item)
                } else {
                    None
                }
            }
        }
    };

    let expanded = quote! {
        #visibility mod #module_name {
            use bincode::{Decode, Encode};
            use bincode::enc::Encoder;
            use bincode::de::Decoder;
            use bincode::error::{DecodeError, EncodeError};
            use std::ops::{Index, IndexMut};
            #( use super::#unique_imports; )*
            use core::array::from_fn;

            #[derive(Debug)]
            #visibility struct #soa_struct_name<const N: usize> {
                pub len: usize,
                #(pub #field_names: [#field_types; N], )*
            }

            impl<const N: usize> #soa_struct_name<N> {
                pub fn new(default: super::#name) -> Self {
                    Self {
                        #( #field_names: from_fn(|_| default.#field_names.clone()), )*
                        len: 0,
                    }
                }

                pub fn len(&self) -> usize {
                    self.len
                }

                pub fn is_empty(&self) -> bool {
                    self.len == 0
                }

                pub fn push(&mut self, value: super::#name) {
                    if self.len < N {
                        #( self.#field_names[self.len] = value.#field_names.clone(); )*
                        self.len += 1;
                    } else {
                        panic!("Capacity exceeded")
                    }
                }

                pub fn pop(&mut self) -> Option<super::#name> {
                    if self.len == 0 {
                        None
                    } else {
                        self.len -= 1;
                        Some(super::#name {
                            #( #field_names: self.#field_names[self.len].clone(), )*
                        })
                    }
                }

                pub fn set(&mut self, index: usize, value: super::#name) {
                    assert!(index < self.len, "Index out of bounds");
                    #( self.#field_names[index] = value.#field_names.clone(); )*
                }

                pub fn get(&self, index: usize) -> super::#name {
                    assert!(index < self.len, "Index out of bounds");
                    super::#name {
                        #( #field_names: self.#field_names[index].clone(), )*
                    }
                }

                pub fn apply<F>(&mut self, mut f: F)
                where
                    F: FnMut(#(#field_types),*) -> (#(#field_types),*)
                {
                    // don't use something common like i here.
                    for _idx in 0..self.len {
                        let result = f(#(self.#field_names[_idx].clone()),*);
                        let (#(#field_names),*) = result;
                        #(
                            self.#field_names[_idx] = #field_names;
                        )*
                    }
                }

                pub fn iter(&self) -> #soa_struct_name_iterator<N> {
                    #soa_struct_name_iterator::new(self)
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

            impl<const N: usize> Encode for #soa_struct_name<N> {
                fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                    self.len.encode(encoder)?;
                    #( self.#field_names[..self.len].encode(encoder)?; )*
                    Ok(())
                }
            }

            impl<const N: usize> Decode<()> for #soa_struct_name<N> {
                fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                    let mut result = Self::default();
                    result.len = Decode::decode(decoder)?;
                    #(
                        for _idx in 0..result.len {
                            result.#field_names[_idx] = Decode::decode(decoder)?;
                        }
                    )*
                    Ok(result)
                }
            }

            impl<const N: usize> Default for #soa_struct_name<N> {
                fn default() -> Self {
                    Self {
                        #( #field_names: from_fn(|_| #field_types::default()), )*
                        len: 0,
                    }
                }
            }

            impl<const N: usize> Clone for #soa_struct_name<N>
            where
                #(
                    #field_types: Clone,
                )*
            {
                fn clone(&self) -> Self {
                    Self {
                        #( #field_names: self.#field_names.clone(), )*
                        len: self.len,
                    }
                }
            }

            #iterator

        }
        #visibility use #module_name::#soa_struct_name;
    };

    let tokens: TokenStream = expanded.into();

    #[cfg(feature = "macro_debug")]
    {
        let formatted_code = rustfmt_generated_code(tokens.to_string());
        eprintln!("\n     ===    Gen. SOA     ===\n");
        eprintln!("{}", highlight_rust_code(formatted_code));
        eprintln!("\n     === === === === === ===\n");
    }
    tokens
}
