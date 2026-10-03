//! Companion recipes use Encode's parsed fields and attribute handling.

use crate::attribute::{ContainerAttributes, FieldAttributes};
use virtue::prelude::*;

struct EncodedField {
    selector: String,
    declaration_index: usize,
    ty: String,
    with_serde: bool,
}

fn encoded_fields(fields: Option<&Fields>, codec: &str) -> Result<Vec<EncodedField>> {
    let mut result = Vec::new();
    let Some(fields) = fields else {
        return Ok(result);
    };
    let declarations: Vec<_> = match fields {
        Fields::Struct(fields) => fields
            .iter()
            .enumerate()
            .map(|(index, (name, field))| (index, Some(name), field))
            .collect(),
        Fields::Tuple(fields) => fields
            .iter()
            .enumerate()
            .map(|(index, field)| (index, None, field))
            .collect(),
    };
    for (index, name, field) in &declarations {
        let attributes = field
            .attributes
            .get_attribute::<FieldAttributes>()?
            .unwrap_or_default();
        if attributes.skip {
            continue;
        }
        let mut ty = field
            .r#type
            .iter()
            .cloned()
            .collect::<TokenStream>()
            .to_string();
        if attributes.with_serde {
            ty = format!("{codec}::serde::Compat<{ty}>");
        }
        let selector = if let Some(name) = name {
            let name = name.to_string().trim_start_matches("r#").to_string();
            format!("{codec}::value_decode::FieldSelector::Named({name:?})")
        } else {
            format!(
                "{codec}::value_decode::FieldSelector::Index {{ index: {index}, declared_fields: {} }}",
                declarations.len()
            )
        };
        result.push(EncodedField {
            selector,
            declaration_index: *index,
            ty,
            with_serde: attributes.with_serde,
        });
    }
    Ok(result)
}

fn fields_expression(fields: &[EncodedField], codec: &str) -> String {
    let fields = fields.iter().map(|field| format!(
        "{codec}::value_decode::ValueDecodeField {{ selector: {}, declaration_index: {}, value: <{} as {codec}::ValueDecode>::__DECODE_REF }}",
        field.selector, field.declaration_index, field.ty
    )).collect::<Vec<_>>().join(",");
    format!("&[{fields}]")
}

fn shape(fields: Option<&Fields>, encoded_len: usize, codec: &str) -> String {
    let shape = match fields {
        None => "Unit",
        Some(Fields::Struct(_)) => "Struct",
        Some(Fields::Tuple(fields)) if fields.len() == 1 && encoded_len == 1 => "Newtype",
        Some(Fields::Tuple(_)) => "Tuple",
    };
    format!("{codec}::value_decode::RecordShape::{shape}")
}

fn generate(
    generator: &mut Generator,
    codec: &str,
    fields: &[EncodedField],
    expression: String,
) -> Result<()> {
    let name = generator.target_name().to_string();
    generator
        .impl_for(format!("{codec}::ValueDecode"))
        .modify_generic_constraints(|generics, constraints| {
            // Unlike push_parsed_constraint, push_constraint handles an existing
            // trailing comma in the declaration's where clause.
            let (placeholder, _, _) =
                Parse::new("struct Placeholder<T>;".parse().unwrap())?.into_generator();
            let mut self_generic = virtue::generate::Parent::generics(&placeholder)
                .unwrap()
                .iter_generics()
                .next()
                .unwrap()
                .clone();
            self_generic.ident = Ident::new("Self", Span::call_site());
            constraints.push_constraint(&self_generic, "'static")?;
            for field in fields {
                // Recursive concrete fields must not create a cyclic where-clause obligation.
                if field
                    .ty
                    .split(|c: char| !c.is_alphanumeric() && c != '_')
                    .any(|token| token == name)
                {
                    for generic in generics.iter_generics() {
                        constraints.push_constraint(generic, format!("{codec}::ValueDecode"))?;
                    }
                } else {
                    constraints
                        .push_parsed_constraint(format!("{}: {codec}::ValueDecode", field.ty))?;
                }
            }
            Ok(())
        })?
        .generate_const("DECODE", format!("&'static {codec}::ValueDecodeSpec"))
        .with_value(|value| {
            value.push_parsed(format!("&{expression}"))?;
            Ok(())
        })?;
    Ok(())
}

pub fn generate_struct(
    generator: &mut Generator,
    attributes: &ContainerAttributes,
    declarations: Option<&Fields>,
) -> Result<()> {
    let codec = &attributes.crate_name;
    let fields = encoded_fields(declarations, codec)?;
    // Serde's serialization can differ from the native field representation.
    // Keep Encode available; these types need a handwritten recipe.
    if fields.iter().any(|field| field.with_serde) {
        return Ok(());
    }
    let expression = format!(
        "{codec}::ValueDecodeSpec::Record {{ shape: {}, fields: {} }}",
        shape(declarations, fields.len(), codec),
        fields_expression(&fields, codec)
    );
    generate(generator, codec, &fields, expression)
}

pub fn generate_enum(
    generator: &mut Generator,
    attributes: &ContainerAttributes,
    declarations: &[EnumVariant],
) -> Result<()> {
    let codec = &attributes.crate_name;
    let mut variants = Vec::new();
    let mut all_fields = Vec::new();
    for (tag, variant) in declarations.iter().enumerate() {
        let fields = encoded_fields(variant.fields.as_ref(), codec)?;
        let name = variant
            .name
            .to_string()
            .trim_start_matches("r#")
            .to_string();
        variants.push(format!("{codec}::value_decode::ValueDecodeVariant {{ tag: {tag}, name: {name:?}, shape: {}, fields: {} }}", shape(variant.fields.as_ref(), fields.len(), codec), fields_expression(&fields, codec)));
        all_fields.extend(fields);
    }
    let expression = format!(
        "{codec}::ValueDecodeSpec::Enum {{ tag: {codec}::value_decode::Scalar::U32, variants: &[{}] }}",
        variants.join(",")
    );
    if all_fields.iter().any(|field| field.with_serde) {
        return Ok(());
    }
    generate(generator, codec, &all_fields, expression)
}
