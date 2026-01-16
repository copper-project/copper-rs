//! Converter from serde-reflection format to JSON Schema.
//!
//! This module provides functions to convert serde-reflection's `Format` and
//! `ContainerFormat` types into JSON Schema (draft-07 compatible).

use serde::Serialize;
use serde::de::DeserializeOwned;
use serde_json::{Map, Value, json};
use serde_reflection::{
    ContainerFormat, Format, Named, Registry, Tracer, TracerConfig, VariantFormat,
};
use std::collections::BTreeMap;

/// Trace a type using serde-reflection and convert to JSON Schema.
///
/// This function uses serde-reflection to analyze the structure of type `T`
/// and produces a complete JSON Schema that describes it.
///
/// # Type Parameters
/// * `T` - The type to generate a schema for. Must implement Serialize and Deserialize.
///
/// # Returns
/// A JSON Schema string describing the type.
pub fn trace_type_to_jsonschema<T>() -> String
where
    T: Serialize + DeserializeOwned,
{
    let mut tracer = Tracer::new(TracerConfig::default());

    // Trace the type - this discovers all variants, fields, etc.
    if let Err(e) = tracer.trace_simple_type::<T>() {
        // If tracing fails, return a permissive schema
        return json!({
            "$schema": "https://json-schema.org/draft-07/schema#",
            "description": format!("Schema generation failed: {}", e)
        })
        .to_string();
    }

    let registry = match tracer.registry() {
        Ok(r) => r,
        Err(e) => {
            return json!({
                "$schema": "https://json-schema.org/draft-07/schema#",
                "description": format!("Registry extraction failed: {}", e)
            })
            .to_string();
        }
    };

    // Get the root type name
    let type_name = std::any::type_name::<T>();
    let short_name = type_name.rsplit("::").next().unwrap_or(type_name);

    registry_to_jsonschema(&registry, short_name)
}

/// Convert a serde-reflection Registry to a JSON Schema string.
///
/// # Arguments
/// * `registry` - The serde-reflection registry containing all type definitions
/// * `root_type` - The name of the root type to use as the schema's main definition
///
/// # Returns
/// A JSON Schema string with definitions for all types in the registry.
pub fn registry_to_jsonschema(registry: &Registry, root_type: &str) -> String {
    let mut schema = json!({
        "$schema": "https://json-schema.org/draft-07/schema#"
    });

    // Build definitions for all container types
    let mut defs = Map::new();
    for (name, container_format) in registry {
        let type_schema = container_format_to_schema(container_format, registry);
        defs.insert(name.clone(), type_schema);
    }

    // If we have definitions, add them
    if !defs.is_empty() {
        schema["$defs"] = Value::Object(defs);
    }

    // Set the root reference or inline the schema
    if let Some(root_container) = registry.get(root_type) {
        // Inline the root type schema
        let root_schema = container_format_to_schema(root_container, registry);
        if let Value::Object(root_obj) = root_schema {
            for (k, v) in root_obj {
                if k != "$defs" {
                    schema[k] = v;
                }
            }
        }
    } else {
        // Try to find a matching type by short name
        for (name, container_format) in registry {
            if name.ends_with(root_type) || root_type.ends_with(name) {
                let root_schema = container_format_to_schema(container_format, registry);
                if let Value::Object(root_obj) = root_schema {
                    for (k, v) in root_obj {
                        if k != "$defs" {
                            schema[k] = v;
                        }
                    }
                }
                break;
            }
        }
    }

    serde_json::to_string_pretty(&schema).unwrap_or_else(|_| "{}".to_string())
}

/// Convert a ContainerFormat to a JSON Schema Value.
fn container_format_to_schema(container: &ContainerFormat, registry: &Registry) -> Value {
    match container {
        ContainerFormat::UnitStruct => {
            json!({ "type": "null" })
        }
        ContainerFormat::NewTypeStruct(format) => format_to_schema(format, registry),
        ContainerFormat::TupleStruct(formats) => {
            let items: Vec<Value> = formats
                .iter()
                .map(|f| format_to_schema(f, registry))
                .collect();
            json!({
                "type": "array",
                "prefixItems": items,
                "items": false,
                "minItems": formats.len(),
                "maxItems": formats.len()
            })
        }
        ContainerFormat::Struct(fields) => struct_to_schema(fields, registry),
        ContainerFormat::Enum(variants) => enum_to_schema(variants, registry),
    }
}

/// Convert a struct (list of named fields) to JSON Schema.
fn struct_to_schema(fields: &[Named<Format>], registry: &Registry) -> Value {
    let mut properties = Map::new();
    let mut required = Vec::new();

    for field in fields {
        let field_schema = format_to_schema(&field.value, registry);
        let is_optional = matches!(&field.value, Format::Option(_));

        properties.insert(field.name.clone(), field_schema);

        if !is_optional {
            required.push(Value::String(field.name.clone()));
        }
    }

    let mut schema = json!({
        "type": "object",
        "properties": properties
    });

    if !required.is_empty() {
        schema["required"] = Value::Array(required);
    }

    schema
}

/// Convert an enum to JSON Schema using oneOf.
fn enum_to_schema(variants: &BTreeMap<u32, Named<VariantFormat>>, registry: &Registry) -> Value {
    let variant_schemas: Vec<Value> = variants
        .values()
        .map(|named_variant| variant_to_schema(&named_variant.name, &named_variant.value, registry))
        .collect();

    if variant_schemas.len() == 1 {
        variant_schemas.into_iter().next().unwrap()
    } else {
        json!({ "oneOf": variant_schemas })
    }
}

/// Convert a single enum variant to JSON Schema.
fn variant_to_schema(name: &str, variant: &VariantFormat, registry: &Registry) -> Value {
    match variant {
        VariantFormat::Unit => {
            // Unit variant: { "type": "string", "const": "VariantName" }
            // Or as object: { "VariantName": null }
            json!({ "const": name })
        }
        VariantFormat::NewType(format) => {
            // NewType variant: { "VariantName": value }
            let inner = format_to_schema(format, registry);
            json!({
                "type": "object",
                "properties": {
                    name: inner
                },
                "required": [name],
                "additionalProperties": false
            })
        }
        VariantFormat::Tuple(formats) => {
            // Tuple variant: { "VariantName": [values] }
            let items: Vec<Value> = formats
                .iter()
                .map(|f| format_to_schema(f, registry))
                .collect();
            json!({
                "type": "object",
                "properties": {
                    name: {
                        "type": "array",
                        "prefixItems": items,
                        "items": false
                    }
                },
                "required": [name],
                "additionalProperties": false
            })
        }
        VariantFormat::Struct(fields) => {
            // Struct variant: { "VariantName": { fields } }
            let inner = struct_to_schema(fields, registry);
            json!({
                "type": "object",
                "properties": {
                    name: inner
                },
                "required": [name],
                "additionalProperties": false
            })
        }
        VariantFormat::Variable(_) => {
            // Unknown format - use permissive schema
            json!({})
        }
    }
}

/// Convert a Format to a JSON Schema Value.
fn format_to_schema(format: &Format, registry: &Registry) -> Value {
    match format {
        Format::Unit => json!({ "type": "null" }),
        Format::Bool => json!({ "type": "boolean" }),

        // Signed integers
        Format::I8 => json!({ "type": "integer", "minimum": -128, "maximum": 127 }),
        Format::I16 => json!({ "type": "integer", "minimum": -32768, "maximum": 32767 }),
        Format::I32 => json!({ "type": "integer" }),
        Format::I64 => json!({ "type": "integer" }),
        Format::I128 => json!({ "type": "integer" }),

        // Unsigned integers
        Format::U8 => json!({ "type": "integer", "minimum": 0, "maximum": 255 }),
        Format::U16 => json!({ "type": "integer", "minimum": 0, "maximum": 65535 }),
        Format::U32 => json!({ "type": "integer", "minimum": 0 }),
        Format::U64 => json!({ "type": "integer", "minimum": 0 }),
        Format::U128 => json!({ "type": "integer", "minimum": 0 }),

        // Floating point
        Format::F32 => json!({ "type": "number" }),
        Format::F64 => json!({ "type": "number" }),

        // String types
        Format::Char => json!({ "type": "string", "minLength": 1, "maxLength": 1 }),
        Format::Str => json!({ "type": "string" }),

        // Bytes - represent as array of integers or base64 string
        Format::Bytes => json!({
            "oneOf": [
                { "type": "array", "items": { "type": "integer", "minimum": 0, "maximum": 255 } },
                { "type": "string", "contentEncoding": "base64" }
            ]
        }),

        // Option - nullable type
        Format::Option(inner) => {
            let inner_schema = format_to_schema(inner, registry);
            json!({
                "oneOf": [
                    { "type": "null" },
                    inner_schema
                ]
            })
        }

        // Sequence (Vec, etc.)
        Format::Seq(inner) => {
            let inner_schema = format_to_schema(inner, registry);
            json!({
                "type": "array",
                "items": inner_schema
            })
        }

        // Map
        Format::Map { key, value } => {
            let value_schema = format_to_schema(value, registry);
            // JSON only supports string keys
            if matches!(key.as_ref(), Format::Str) {
                json!({
                    "type": "object",
                    "additionalProperties": value_schema
                })
            } else {
                // Non-string keys - represent as array of tuples
                let key_schema = format_to_schema(key, registry);
                json!({
                    "type": "array",
                    "items": {
                        "type": "array",
                        "prefixItems": [key_schema, value_schema],
                        "items": false,
                        "minItems": 2,
                        "maxItems": 2
                    }
                })
            }
        }

        // Tuple
        Format::Tuple(formats) => {
            let items: Vec<Value> = formats
                .iter()
                .map(|f| format_to_schema(f, registry))
                .collect();
            json!({
                "type": "array",
                "prefixItems": items,
                "items": false,
                "minItems": formats.len(),
                "maxItems": formats.len()
            })
        }

        // Fixed-size array
        Format::TupleArray { content, size } => {
            let content_schema = format_to_schema(content, registry);
            json!({
                "type": "array",
                "items": content_schema,
                "minItems": size,
                "maxItems": size
            })
        }

        // Reference to named type
        Format::TypeName(name) => {
            // Check if the type is in the registry
            if registry.contains_key(name) {
                json!({ "$ref": format!("#/$defs/{}", name) })
            } else {
                // Unknown type - inline what we can
                json!({ "description": format!("Reference to type: {}", name) })
            }
        }

        // Variable (placeholder during tracing)
        Format::Variable(_) => {
            json!({})
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize)]
    struct SimpleStruct {
        name: String,
        value: i32,
        active: bool,
    }

    #[derive(Serialize, Deserialize)]
    struct NestedStruct {
        id: u64,
        data: SimpleStruct,
        tags: Vec<String>,
    }

    #[derive(Serialize, Deserialize)]
    enum Status {
        Active,
        Inactive,
        Pending { reason: String },
    }

    #[derive(Serialize, Deserialize)]
    struct WithOptional {
        required_field: String,
        optional_field: Option<i32>,
    }

    #[test]
    fn test_simple_struct_schema() {
        let schema = trace_type_to_jsonschema::<SimpleStruct>();
        println!("SimpleStruct schema:\n{}", schema);

        let parsed: Value = serde_json::from_str(&schema).unwrap();
        assert_eq!(parsed["type"], "object");
        assert!(parsed["properties"]["name"].is_object());
        assert!(parsed["properties"]["value"].is_object());
        assert!(parsed["properties"]["active"].is_object());
    }

    #[test]
    fn test_nested_struct_schema() {
        let schema = trace_type_to_jsonschema::<NestedStruct>();
        println!("NestedStruct schema:\n{}", schema);

        let parsed: Value = serde_json::from_str(&schema).unwrap();
        assert_eq!(parsed["type"], "object");
        assert!(parsed["$defs"].is_object());
    }

    #[test]
    fn test_enum_schema() {
        let schema = trace_type_to_jsonschema::<Status>();
        println!("Status enum schema:\n{}", schema);

        let parsed: Value = serde_json::from_str(&schema).unwrap();
        // Should have oneOf for the variants
        assert!(parsed["oneOf"].is_array() || parsed["type"].is_string());
    }

    #[test]
    fn test_optional_field_schema() {
        let schema = trace_type_to_jsonschema::<WithOptional>();
        println!("WithOptional schema:\n{}", schema);

        let parsed: Value = serde_json::from_str(&schema).unwrap();
        assert_eq!(parsed["type"], "object");
        // optional_field should NOT be in required array
        if let Some(required) = parsed["required"].as_array() {
            assert!(required.contains(&json!("required_field")));
            assert!(!required.contains(&json!("optional_field")));
        }
    }
}
