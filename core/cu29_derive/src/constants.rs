//! Compile-time constant code generation.

use cu29_runtime::config::{ConstantConfig, ConstantNumber, ConstantStorage};
use cu29_traits::{CuError, CuResult};
use proc_macro2::Ident;
use quote::quote;
use std::collections::{BTreeMap, BTreeSet};
use syn::ext::IdentExt;
use syn::{Expr, Path as SynPath, Type, parse_str};

fn primitive_constant_number_tokens(
    storage: ConstantStorage,
    number: ConstantNumber,
) -> CuResult<proc_macro2::TokenStream> {
    if storage == ConstantStorage::F32 {
        let bits = proc_macro2::Literal::u32_suffixed((number.as_f64() as f32).to_bits());
        return Ok(quote! { ::core::primitive::f32::from_bits(#bits) });
    }
    if storage == ConstantStorage::F64 {
        let bits = proc_macro2::Literal::u64_suffixed(number.as_f64().to_bits());
        return Ok(quote! { ::core::primitive::f64::from_bits(#bits) });
    }
    let source = match (storage, number) {
        (storage, ConstantNumber::Signed(value)) => {
            format!("{value}{}", storage.rust_type())
        }
        (storage, ConstantNumber::Unsigned(value)) => {
            format!("{value}{}", storage.rust_type())
        }
        (storage, ConstantNumber::Float(value)) => {
            return Err(CuError::from(format!(
                "Floating-point value {value:?} cannot be emitted as {}",
                storage.rust_type()
            )));
        }
    };
    let expression = parse_str::<Expr>(&source).map_err(|error| {
        CuError::from(format!(
            "Could not generate constant expression '{source}': {error}"
        ))
    })?;
    Ok(quote! { #expression })
}

fn build_constant_def(constant: &ConstantConfig) -> CuResult<proc_macro2::TokenStream> {
    let id = parse_str::<Ident>(constant.id()).map_err(|error| {
        CuError::from(format!(
            "Constant id '{}' is not a valid Rust identifier: {error}",
            constant.id()
        ))
    })?;

    if let Some((rust_type, expression)) = constant.expression_definition() {
        let constant_type = parse_str::<Type>(rust_type).map_err(|error| {
            CuError::from(format!(
                "Constant '{}' type '{}' is not a valid Rust type: {error}",
                constant.id(),
                rust_type
            ))
        })?;
        let expression = parse_str::<Expr>(expression).map_err(|error| {
            CuError::from(format!(
                "Constant '{}' expression is not a valid Rust expression: {error}",
                constant.id()
            ))
        })?;
        return Ok(quote! {
            pub const #id: #constant_type = #expression;
        });
    }

    if let Some(quantity) = constant.quantity() {
        let definition = cu29_units::constant::definition(quantity).ok_or_else(|| {
            CuError::from(format!(
                "Constant '{}' quantity '{}' is missing from the unit catalogue",
                constant.id(),
                quantity.name()
            ))
        })?;
        let (constant_type, values): (Type, Vec<proc_macro2::TokenStream>) =
            match constant.storage() {
                ConstantStorage::F32 => {
                    let ty = parse_str::<Type>(definition.rust_type_f32).map_err(|error| {
                        CuError::from(format!(
                            "Invalid f32 type metadata for quantity '{}': {error}",
                            quantity.name()
                        ))
                    })?;
                    let (_, values) = constant.normalized_f32().map_err(CuError::from)?;
                    let values = values
                        .into_iter()
                        .map(|value| {
                            let bits = proc_macro2::Literal::u32_suffixed(value.to_bits());
                            quote! {
                                #ty {
                                    value: ::core::primitive::f32::from_bits(#bits),
                                }
                            }
                        })
                        .collect();
                    (ty, values)
                }
                ConstantStorage::F64 => {
                    let ty = parse_str::<Type>(definition.rust_type_f64).map_err(|error| {
                        CuError::from(format!(
                            "Invalid f64 type metadata for quantity '{}': {error}",
                            quantity.name()
                        ))
                    })?;
                    let (_, values) = constant.normalized_f64().map_err(CuError::from)?;
                    let values = values
                        .into_iter()
                        .map(|value| {
                            let bits = proc_macro2::Literal::u64_suffixed(value.to_bits());
                            quote! {
                                #ty {
                                    value: ::core::primitive::f64::from_bits(#bits),
                                }
                            }
                        })
                        .collect();
                    (ty, values)
                }
                storage => {
                    return Err(CuError::from(format!(
                        "Constant '{}' quantity '{}' cannot use storage {}",
                        constant.id(),
                        quantity.name(),
                        storage.rust_type()
                    )));
                }
            };
        let (is_array, _) = constant.numbers().map_err(CuError::from)?;
        if is_array {
            let length = values.len();
            Ok(quote! {
                pub const #id: [#constant_type; #length] = [#(#values),*];
            })
        } else {
            let value = values.into_iter().next().ok_or_else(|| {
                CuError::from(format!("Constant '{}' has no scalar value", constant.id()))
            })?;
            Ok(quote! {
                pub const #id: #constant_type = #value;
            })
        }
    } else {
        let constant_type = parse_str::<Type>(constant.storage().rust_type()).map_err(|error| {
            CuError::from(format!(
                "Invalid primitive storage type '{}': {error}",
                constant.storage().rust_type()
            ))
        })?;
        let (is_array, numbers) = constant.numbers().map_err(CuError::from)?;
        let values = numbers
            .into_iter()
            .map(|number| primitive_constant_number_tokens(constant.storage(), number))
            .collect::<CuResult<Vec<_>>>()?;
        if is_array {
            let length = values.len();
            Ok(quote! {
                pub const #id: [#constant_type; #length] = [#(#values),*];
            })
        } else {
            let value = values.into_iter().next().ok_or_else(|| {
                CuError::from(format!("Constant '{}' has no scalar value", constant.id()))
            })?;
            Ok(quote! {
                pub const #id: #constant_type = #value;
            })
        }
    }
}

#[derive(Default)]
pub(super) struct ConstantModuleTree {
    ident: Option<Ident>,
    constants: Vec<proc_macro2::TokenStream>,
    constant_names: BTreeSet<String>,
    children: BTreeMap<String, ConstantModuleTree>,
}

impl ConstantModuleTree {
    pub(super) fn insert(
        &mut self,
        module_path: &[Ident],
        constant_id: &Ident,
        definition: proc_macro2::TokenStream,
        qualified_id: &str,
    ) -> CuResult<()> {
        let mut module = self;
        for segment in module_path {
            let canonical = segment.unraw().to_string();
            if module.constant_names.contains(&canonical) {
                return Err(CuError::from(format!(
                    "Constant module '{}' conflicts with constant '{}'",
                    module_path
                        .iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>()
                        .join("::"),
                    canonical
                )));
            }
            module = module
                .children
                .entry(canonical)
                .or_insert_with(|| ConstantModuleTree {
                    ident: Some(segment.clone()),
                    ..Self::default()
                });
        }

        let canonical_id = constant_id.unraw().to_string();
        if module.children.contains_key(&canonical_id) {
            return Err(CuError::from(format!(
                "Constant '{qualified_id}' conflicts with a generated module of the same name"
            )));
        }
        if !module.constant_names.insert(canonical_id) {
            return Err(CuError::from(format!(
                "Duplicate constant '{qualified_id}'. Constant ids must be unique within a module."
            )));
        }
        module.constants.push(definition);
        Ok(())
    }

    pub(super) fn contents(&self) -> proc_macro2::TokenStream {
        let constants = &self.constants;
        let children = self.children.values().map(Self::module_tokens);
        quote! {
            #(#constants)*
            #(#children)*
        }
    }

    pub(super) fn module_tokens(&self) -> proc_macro2::TokenStream {
        let ident = self
            .ident
            .as_ref()
            .expect("only non-root constant module nodes are rendered");
        let contents = self.contents();
        quote! {
            pub mod #ident {
                #contents
            }
        }
    }

    pub(super) fn child_contents(&self, ident: &Ident) -> proc_macro2::TokenStream {
        self.children
            .get(&ident.unraw().to_string())
            .map(Self::contents)
            .unwrap_or_default()
    }

    pub(super) fn root_modules_except(
        &self,
        excluded: &BTreeSet<String>,
    ) -> proc_macro2::TokenStream {
        let modules = self
            .children
            .iter()
            .filter(|(name, _)| !excluded.contains(*name))
            .map(|(_, module)| module.module_tokens());
        quote! { #(#modules)* }
    }
}

fn parse_constant_module_path(constant: &ConstantConfig) -> CuResult<Vec<Ident>> {
    let source = constant.module_path();
    let path = parse_str::<SynPath>(source).map_err(|error| {
        CuError::from(format!(
            "Constant '{}' module path '{}' is not a valid Rust module path: {error}",
            constant.id(),
            source
        ))
    })?;
    if path.leading_colon.is_some() {
        return Err(CuError::from(format!(
            "Constant '{}' module path '{}' must be relative",
            constant.id(),
            source
        )));
    }

    let mut segments = Vec::with_capacity(path.segments.len());
    for segment in path.segments {
        if !segment.arguments.is_none() {
            return Err(CuError::from(format!(
                "Constant '{}' module path '{}' cannot contain generic arguments",
                constant.id(),
                source
            )));
        }
        let canonical = segment.ident.unraw().to_string();
        if matches!(canonical.as_str(), "crate" | "self" | "super") {
            return Err(CuError::from(format!(
                "Constant '{}' module path '{}' must not contain '{canonical}'",
                constant.id(),
                source
            )));
        }
        segments.push(segment.ident);
    }
    if segments.is_empty() {
        return Err(CuError::from(format!(
            "Constant '{}' module path cannot be empty",
            constant.id()
        )));
    }
    Ok(segments)
}

pub(super) fn build_constant_modules(constants: &[ConstantConfig]) -> CuResult<ConstantModuleTree> {
    let mut modules = ConstantModuleTree::default();
    for constant in constants {
        let module_path = parse_constant_module_path(constant)?;
        let constant_id = parse_str::<Ident>(constant.id()).map_err(|error| {
            CuError::from(format!(
                "Constant id '{}' is not a valid Rust identifier: {error}",
                constant.id()
            ))
        })?;
        let definition = build_constant_def(constant)?;
        modules.insert(
            &module_path,
            &constant_id,
            definition,
            &constant.qualified_id(),
        )?;
    }
    Ok(modules)
}
