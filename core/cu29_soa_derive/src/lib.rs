mod format;
#[cfg(feature = "macro_debug")]
use format::{highlight_rust_code, rustfmt_generated_code};
use proc_macro::TokenStream;
use quote::{ToTokens, format_ident, quote};
use syn::{Attribute, Data, DeriveInput, Fields, Path, PathArguments, Type, parse_macro_input};

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
///
/// You can also compose nested SoAs by annotating fields with `#[soa(nested)]`:
/// ```ignore
/// #[derive(Soa)]
/// struct ColoredPoint {
///     #[soa(nested)]
///     position: Xyz,
///     #[soa(nested)]
///     color: Color,
/// }
/// // ColoredPointSoa<N> stores position as XyzSoaStorage<N> and color as ColorSoaStorage<N>.
/// ```
/// For nested types from other crates, use absolute paths like `::other_crate::Type`
/// so the generated storage path resolves correctly.
///
/// Memory layout
/// - Flat fields: `MyStructSoa<N>` stores `len` plus one `[T; N]` per field.
/// - Nested fields (`#[soa(nested)]`): the field is stored inline as `<Field>SoaStorage<N>`,
///   so the top-level struct contains `len` plus nested storage(s) and the leaf arrays live
///   in those nested storages.
/// - `*SoaStorage<N>` has the same layout as `*Soa<N>` without the `len` field.
///
/// ASCII layouts
/// ```text
/// Flat:
///   MyStructSoa<N>
///   +-----+------------------+------------------+
///   | len | a: [i32; N]       | b: [f32; N]      |
///   +-----+------------------+------------------+
///
/// Nested:
///   ColoredPointSoa<N>
///   +-----+---------------------+---------------------+
///   | len | position: XyzSoaStorage<N> | color: ColorSoaStorage<N> |
///   +-----+---------------------+---------------------+
///                |                               |
///                v                               v
///        XyzSoaStorage<N>                 ColorSoaStorage<N>
///        +------------------+             +------------------+
///        | x: [f32; N]       |             | r: [f32; N]       |
///        | y: [f32; N]       |             | g: [f32; N]       |
///        | z: [f32; N]       |             | b: [f32; N]       |
///        | i: [i32; N]       |             +------------------+
///        +------------------+
/// ```
/// ```ignore
/// struct ColoredPointSoa<const N: usize> {
///     len: usize,
///     position: XyzSoaStorage<N>,
///     color: ColorSoaStorage<N>,
/// }
/// struct XyzSoaStorage<const N: usize> {
///     x: [f32; N],
///     y: [f32; N],
///     z: [f32; N],
///     i: [i32; N],
/// }
/// ```
#[proc_macro_derive(Soa, attributes(soa))]
pub fn derive_soa(input: TokenStream) -> TokenStream {
    use syn::TypePath;

    let input = parse_macro_input!(input as DeriveInput);
    let visibility = &input.vis;

    let name = &input.ident;
    let module_name = format_ident!("{}_soa", name.to_string().to_lowercase());
    let soa_struct_name = format_ident!("{}Soa", name);
    let soa_storage_name = format_ident!("{}SoaStorage", name);
    let soa_storage_wire_name = format_ident!("{}SoaStorageWire", name);
    let soa_storage_serde_name = format_ident!("{}SoaStorageSerde", name);
    let soa_struct_wire_name = format_ident!("{}SoaWire", name);

    let data = match &input.data {
        Data::Struct(data) => data,
        _ => panic!("Only structs are supported"),
    };
    let fields = match &data.fields {
        Fields::Named(fields) => &fields.named,
        _ => panic!("Only named fields are supported"),
    };

    struct FieldInfo {
        name: syn::Ident,
        ty: syn::Type,
        nested: bool,
        storage_path: Option<syn::Path>,
        storage_wire_path: Option<syn::Path>,
    }

    let mut field_infos = Vec::new();
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

    fn parse_soa_nested(attrs: &[Attribute]) -> Result<bool, syn::Error> {
        let mut nested = false;
        for attr in attrs {
            if attr.path().is_ident("soa") {
                attr.parse_nested_meta(|meta| {
                    if meta.path.is_ident("nested") {
                        nested = true;
                        Ok(())
                    } else {
                        Err(meta.error("unsupported #[soa] option, expected `nested`"))
                    }
                })?;
            }
        }
        Ok(nested)
    }

    fn qualify_path(path: &Path) -> Path {
        if path.leading_colon.is_some() {
            return path.clone();
        }
        if let Some(first) = path.segments.first() {
            if first.ident == "crate" {
                return path.clone();
            }
            if first.ident == "self" {
                let mut path = path.clone();
                if let Some(segment) = path.segments.first_mut() {
                    segment.ident = format_ident!("super");
                }
                return path;
            }
            if first.ident == "std" || first.ident == "core" || first.ident == "alloc" {
                return path.clone();
            }
        }
        syn::parse_quote!(super::#path)
    }

    fn storage_paths(field_type: &Type) -> Result<(Path, Path), syn::Error> {
        let Type::Path(type_path) = field_type else {
            return Err(syn::Error::new_spanned(
                field_type,
                "expected a path type for #[soa(nested)]",
            ));
        };

        let mut storage_path = type_path.path.clone();
        let last_segment = storage_path
            .segments
            .last_mut()
            .ok_or_else(|| syn::Error::new_spanned(field_type, "expected a non-empty type path"))?;
        if !matches!(last_segment.arguments, PathArguments::None) {
            return Err(syn::Error::new_spanned(
                field_type,
                "generic types are not supported with #[soa(nested)]",
            ));
        }
        let base_ident = last_segment.ident.clone();
        last_segment.ident = format_ident!("{}SoaStorage", base_ident);

        let mut storage_wire_path = type_path.path.clone();
        let last_wire_segment = storage_wire_path
            .segments
            .last_mut()
            .ok_or_else(|| syn::Error::new_spanned(field_type, "expected a non-empty type path"))?;
        last_wire_segment.ident = format_ident!("{}SoaStorageWire", base_ident);

        Ok((
            qualify_path(&storage_path),
            qualify_path(&storage_wire_path),
        ))
    }

    for field in fields {
        let field_name = field.ident.as_ref().unwrap().clone();
        let field_type = field.ty.clone();
        let nested = match parse_soa_nested(&field.attrs) {
            Ok(value) => value,
            Err(err) => return err.to_compile_error().into(),
        };
        let (storage_path, storage_wire_path) = if nested {
            match storage_paths(&field_type) {
                Ok((storage_path, storage_wire_path)) => {
                    (Some(storage_path), Some(storage_wire_path))
                }
                Err(err) => return err.to_compile_error().into(),
            }
        } else {
            (None, None)
        };

        if let Type::Path(TypePath { path, .. }) = &field_type {
            let type_name = path.segments.last().unwrap().ident.to_string();
            let path_str = path.to_token_stream().to_string();

            if !is_primitive(&type_name) && !unique_import_names.contains(&path_str) {
                unique_imports.push(path.clone());
                unique_import_names.push(path_str);
            }
        }

        field_infos.push(FieldInfo {
            name: field_name,
            ty: field_type,
            nested,
            storage_path,
            storage_wire_path,
        });
    }

    let field_names: Vec<_> = field_infos.iter().map(|info| &info.name).collect();
    let field_types: Vec<_> = field_infos.iter().map(|info| &info.ty).collect();

    let soa_struct_name_iterator = format_ident!("{}Iterator", name);
    let storage_field_count = field_names.len();
    let field_count = storage_field_count + 1; // +1 for the len field

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

    // Shared between storage and soa (identical generated code)
    let mut field_decls = Vec::new();
    let mut new_inits = Vec::new();
    let mut default_inits = Vec::new();
    let mut get_fields = Vec::new();
    let mut set_fields = Vec::new();
    let mut accessors = Vec::new();

    // Soa-only fields
    let mut soa_push_fields = Vec::new();
    let mut soa_pop_fields = Vec::new();
    let mut soa_apply_args = Vec::new();
    let mut soa_apply_sets = Vec::new();
    let mut soa_encode_fields = Vec::new();
    let mut soa_decode_fields = Vec::new();
    let mut soa_serialize_fields = Vec::new();
    let mut soa_serialize_bounds = Vec::new();
    let mut soa_deserialize_bounds = Vec::new();
    let mut soa_wire_fields = Vec::new();
    let mut soa_wire_checks = Vec::new();
    let mut soa_wire_assignments = Vec::new();

    // Storage-only fields
    let mut storage_encode_fields = Vec::new();
    let mut storage_decode_fields = Vec::new();
    let mut storage_serialize_fields = Vec::new();
    let mut storage_serialize_bounds = Vec::new();
    let mut storage_clone_bounds = Vec::new();
    let mut storage_wire_fields = Vec::new();
    let mut storage_wire_checks = Vec::new();
    let mut storage_wire_assignments = Vec::new();

    for info in &field_infos {
        let name = &info.name;
        let ty = &info.ty;
        let name_mut = format_ident!("{}_mut", name);

        if info.nested {
            let storage_path = info
                .storage_path
                .as_ref()
                .expect("nested field missing storage path");
            let storage_wire_path = info
                .storage_wire_path
                .as_ref()
                .expect("nested field missing storage wire path");
            let serde_name = format_ident!("{}_serde", name);

            field_decls.push(quote!(pub #name: #storage_path<N>));
            new_inits.push(quote!(#name: #storage_path::<N>::new(default.#name.clone())));
            default_inits.push(quote!(#name: #storage_path::<N>::default()));
            storage_clone_bounds.push(quote!(#storage_path<N>: Clone,));

            accessors.push(quote! {
                pub fn #name(&self) -> &#storage_path<N> {
                    &self.#name
                }

                pub fn #name_mut(&mut self) -> &mut #storage_path<N> {
                    &mut self.#name
                }
            });

            get_fields.push(quote!(#name: self.#name.get(index),));
            set_fields.push(quote!(self.#name.set(index, value.#name.clone());));

            soa_push_fields.push(quote!(self.#name.set(self.len, value.#name.clone());));
            soa_pop_fields.push(quote!(#name: self.#name.get(self.len),));

            soa_apply_args.push(quote!(self.#name.get(_idx)));
            soa_apply_sets.push(quote!(self.#name.set(_idx, #name);));

            storage_encode_fields.push(quote!(self.#name.encode_len(encoder, len)?;));
            storage_decode_fields
                .push(quote!(result.#name = #storage_path::<N>::decode_len(decoder, len)?;));

            soa_encode_fields.push(quote!(self.#name.encode_len(encoder, self.len)?;));
            soa_decode_fields
                .push(quote!(result.#name = #storage_path::<N>::decode_len(decoder, result.len)?;));

            storage_serialize_fields.push(quote! {
                {
                    let #serde_name = self.storage.#name.serialize_len(self.len);
                    state.serialize_field(stringify!(#name), &#serde_name)?;
                }
            });
            soa_serialize_fields.push(quote! {
                {
                    let #serde_name = self.#name.serialize_len(self.len);
                    state.serialize_field(stringify!(#name), &#serde_name)?;
                }
            });

            storage_wire_fields.push(quote!(#name: #storage_wire_path,));
            storage_wire_assignments.push(quote!(
                result.#name = #storage_path::<N>::from_wire(#name, len)
                    .map_err(|err| format!("field {}: {}", stringify!(#name), err))?;
            ));

            soa_wire_fields.push(quote!(#name: #storage_wire_path,));
            soa_wire_assignments.push(quote!(
                result.#name = #storage_path::<N>::from_wire(#name, len)
                    .map_err(|err| serde::de::Error::custom(format!(
                        "field {}: {}",
                        stringify!(#name),
                        err
                    )))?;
            ));
        } else {
            let name_range = format_ident!("{}_range", name);
            let name_range_mut = format_ident!("{}_range_mut", name);

            field_decls.push(quote!(pub #name: [#ty; N]));
            new_inits.push(quote!(#name: from_fn(|_| default.#name.clone())));
            default_inits.push(quote!(#name: from_fn(|_| #ty::default())));
            storage_clone_bounds.push(quote!(#ty: Clone,));

            accessors.push(quote! {
                pub fn #name(&self) -> &[#ty] {
                    &self.#name
                }

                pub fn #name_mut(&mut self) -> &mut [#ty] {
                    &mut self.#name
                }

                pub fn #name_range(&self, range: std::ops::Range<usize>) -> &[#ty] {
                    &self.#name[range]
                }

                pub fn #name_range_mut(&mut self, range: std::ops::Range<usize>) -> &mut [#ty] {
                    &mut self.#name[range]
                }
            });

            get_fields.push(quote!(#name: self.#name[index].clone(),));
            set_fields.push(quote!(self.#name[index] = value.#name.clone();));

            soa_push_fields.push(quote!(self.#name[self.len] = value.#name.clone();));
            soa_pop_fields.push(quote!(#name: self.#name[self.len].clone(),));

            soa_apply_args.push(quote!(self.#name[_idx].clone()));
            soa_apply_sets.push(quote!(self.#name[_idx] = #name;));

            storage_encode_fields.push(quote! {
                for _idx in 0..len {
                    self.#name[_idx].encode(encoder)?;
                }
            });
            storage_decode_fields.push(quote! {
                for _idx in 0..len {
                    result.#name[_idx] = Decode::decode(decoder)?;
                }
            });

            soa_encode_fields.push(quote! {
                for _idx in 0..self.len {
                    self.#name[_idx].encode(encoder)?;
                }
            });
            soa_decode_fields.push(quote! {
                for _idx in 0..result.len {
                    result.#name[_idx] = Decode::decode(decoder)?;
                }
            });

            storage_serialize_fields.push(quote! {
                state.serialize_field(stringify!(#name), &self.storage.#name[..self.len])?;
            });
            soa_serialize_fields.push(quote! {
                state.serialize_field(stringify!(#name), &self.#name[..self.len])?;
            });

            storage_serialize_bounds.push(quote!(#ty: Serialize,));
            soa_serialize_bounds.push(quote!(#ty: Serialize,));
            soa_deserialize_bounds.push(quote!(#ty: Deserialize<'de> + Default,));

            storage_wire_fields.push(quote!(#name: Vec<#ty>,));
            storage_wire_checks.push(quote! {
                if #name.len() != len {
                    return Err(format!(
                        "field {} has length {} but len is {}",
                        stringify!(#name),
                        #name.len(),
                        len
                    ));
                }
            });
            storage_wire_assignments.push(quote! {
                for (idx, value) in #name.into_iter().enumerate() {
                    result.#name[idx] = value;
                }
            });

            soa_wire_fields.push(quote!(#name: Vec<#ty>,));
            soa_wire_checks.push(quote! {
                if #name.len() != len {
                    return Err(serde::de::Error::custom(format!(
                        "field {} has length {} but len is {}",
                        stringify!(#name),
                        #name.len(),
                        len
                    )));
                }
            });
            soa_wire_assignments.push(quote! {
                for (idx, value) in #name.into_iter().enumerate() {
                    result.#name[idx] = value;
                }
            });
        }
    }

    let storage_clone_where = if storage_clone_bounds.is_empty() {
        quote!()
    } else {
        quote!(where #(#storage_clone_bounds)*)
    };
    let storage_serialize_where = if storage_serialize_bounds.is_empty() {
        quote!()
    } else {
        quote!(where #(#storage_serialize_bounds)*)
    };
    let soa_serialize_where = if soa_serialize_bounds.is_empty() {
        quote!()
    } else {
        quote!(where #(#soa_serialize_bounds)*)
    };
    let soa_deserialize_where = if soa_deserialize_bounds.is_empty() {
        quote!()
    } else {
        quote!(where #(#soa_deserialize_bounds)*)
    };

    let expanded = quote! {
        #visibility mod #module_name {
            use bincode::{Decode, Encode};
            use bincode::enc::Encoder;
            use bincode::de::Decoder;
            use bincode::error::{DecodeError, EncodeError};
            use serde::Deserialize;
            use serde::Serialize;
            use serde::Serializer;
            use serde::ser::SerializeStruct;
            use std::ops::{Index, IndexMut};
            #( use super::#unique_imports; )*
            use core::array::from_fn;

            #[derive(Debug)]
            #visibility struct #soa_storage_name<const N: usize> {
                #(#field_decls,)*
            }

            #[doc(hidden)]
            #[derive(Deserialize)]
            #visibility struct #soa_storage_wire_name {
                #(#storage_wire_fields)*
            }

            #[doc(hidden)]
            #visibility struct #soa_storage_serde_name<'a, const N: usize> {
                storage: &'a #soa_storage_name<N>,
                len: usize,
            }

            impl<const N: usize> #soa_storage_name<N> {
                pub fn new(default: super::#name) -> Self {
                    Self {
                        #(#new_inits,)*
                    }
                }

                pub fn set(&mut self, index: usize, value: super::#name) {
                    assert!(index < N, "Index out of bounds");
                    #(#set_fields)*
                }

                pub fn get(&self, index: usize) -> super::#name {
                    assert!(index < N, "Index out of bounds");
                    super::#name {
                        #(#get_fields)*
                    }
                }

                pub fn encode_len<E: Encoder>(
                    &self,
                    encoder: &mut E,
                    len: usize,
                ) -> Result<(), EncodeError> {
                    #(#storage_encode_fields)*
                    Ok(())
                }

                pub fn decode_len<D: Decoder<Context = ()>>(
                    decoder: &mut D,
                    len: usize,
                ) -> Result<Self, DecodeError> {
                    let mut result = Self::default();
                    #(#storage_decode_fields)*
                    Ok(result)
                }

                pub fn serialize_len(&self, len: usize) -> #soa_storage_serde_name<'_, N> {
                    #soa_storage_serde_name {
                        storage: self,
                        len,
                    }
                }

                pub fn from_wire(wire: #soa_storage_wire_name, len: usize) -> Result<Self, String> {
                    let #soa_storage_wire_name { #( #field_names ),* } = wire;

                    if len > N {
                        return Err(format!(
                            "len {} exceeds capacity {}",
                            len,
                            N
                        ));
                    }

                    #(#storage_wire_checks)*

                    let mut result = Self::default();
                    #(#storage_wire_assignments)*
                    Ok(result)
                }

                #(#accessors)*
            }

            impl<'a, const N: usize> Serialize for #soa_storage_serde_name<'a, N>
            #storage_serialize_where
            {
                fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
                where
                    S: Serializer,
                {
                    let mut state = serializer.serialize_struct(
                        stringify!(#soa_storage_name),
                        #storage_field_count,
                    )?;
                    #(#storage_serialize_fields)*
                    state.end()
                }
            }

            impl<const N: usize> Default for #soa_storage_name<N> {
                fn default() -> Self {
                    Self {
                        #(#default_inits,)*
                    }
                }
            }

            impl<const N: usize> Clone for #soa_storage_name<N>
            #storage_clone_where
            {
                fn clone(&self) -> Self {
                    Self {
                        #( #field_names: self.#field_names.clone(), )*
                    }
                }
            }

            #[derive(Debug)]
            #visibility struct #soa_struct_name<const N: usize> {
                pub len: usize,
                #(#field_decls,)*
            }

            impl<const N: usize> #soa_struct_name<N> {
                pub fn new(default: super::#name) -> Self {
                    Self {
                        #(#new_inits,)*
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
                        #(#soa_push_fields)*
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
                            #(#soa_pop_fields)*
                        })
                    }
                }

                pub fn set(&mut self, index: usize, value: super::#name) {
                    assert!(index < self.len, "Index out of bounds");
                    #(#set_fields)*
                }

                pub fn get(&self, index: usize) -> super::#name {
                    assert!(index < self.len, "Index out of bounds");
                    super::#name {
                        #(#get_fields)*
                    }
                }

                pub fn apply<F>(&mut self, mut f: F)
                where
                    F: FnMut(#(#field_types),*) -> (#(#field_types),*)
                {
                    // don't use something common like i here.
                    for _idx in 0..self.len {
                        let result = f(#(#soa_apply_args),*);
                        let (#(#field_names),*) = result;
                        #(#soa_apply_sets)*
                    }
                }

                pub fn iter(&self) -> #soa_struct_name_iterator<N> {
                    #soa_struct_name_iterator::new(self)
                }

                #(#accessors)*
            }

            impl<const N: usize> Encode for #soa_struct_name<N> {
                fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                    self.len.encode(encoder)?;
                    #(#soa_encode_fields)*
                    Ok(())
                }
            }

            impl<const N: usize> Decode<()> for #soa_struct_name<N> {
                fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                    let mut result = Self::default();
                    result.len = Decode::decode(decoder)?;
                    #(#soa_decode_fields)*
                    Ok(result)
                }
            }

            impl<const N: usize> Default for #soa_struct_name<N> {
                fn default() -> Self {
                    Self {
                        #(#default_inits,)*
                        len: 0,
                    }
                }
            }

            impl<const N: usize> Clone for #soa_struct_name<N>
            #storage_clone_where
            {
                fn clone(&self) -> Self {
                    Self {
                        #( #field_names: self.#field_names.clone(), )*
                        len: self.len,
                    }
                }
            }

            impl<const N: usize> Serialize for #soa_struct_name<N>
            #soa_serialize_where
            {
                fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
                where
                    S: Serializer,
                {
                    let mut state =
                        serializer.serialize_struct(stringify!(#soa_struct_name), #field_count)?;
                    state.serialize_field("len", &self.len)?;
                    #(#soa_serialize_fields)*
                    state.end()
                }
            }

            impl<'de, const N: usize> Deserialize<'de> for #soa_struct_name<N>
            #soa_deserialize_where
            {
                fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
                where
                    D: serde::Deserializer<'de>,
                {
                    #[derive(Deserialize)]
                    struct #soa_struct_wire_name {
                        len: usize,
                        #(#soa_wire_fields)*
                    }

                    let wire = #soa_struct_wire_name::deserialize(deserializer)?;
                    let #soa_struct_wire_name { len, #( #field_names ),* } = wire;

                    if len > N {
                        return Err(serde::de::Error::custom(format!(
                            "len {} exceeds capacity {}",
                            len,
                            N
                        )));
                    }

                    #(#soa_wire_checks)*

                    let mut result = Self::default();
                    result.len = len;
                    #(#soa_wire_assignments)*
                    Ok(result)
                }
            }

            #iterator
        }
        #visibility use #module_name::#soa_struct_name;
        #visibility use #module_name::#soa_storage_name;
        #visibility use #module_name::#soa_storage_wire_name;
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
