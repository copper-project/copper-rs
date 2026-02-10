//! JSON Schema generation for Copper payload types using reflection metadata.

use core::any::TypeId;
use std::collections::BTreeSet;

use cu29::reflect::{
    ArrayInfo, EnumInfo, GetTypeRegistration, ListInfo, MapInfo, NamedField, SetInfo, StructInfo,
    TupleInfo, TupleStructInfo, Type, TypeInfo, TypeRegistry, VariantInfo,
    serde::SerializationData,
};
use serde_json::{Map, Value, json};

const DRAFT_07: &str = "https://json-schema.org/draft-07/schema#";

/// Trace a type using reflected type metadata and convert it to JSON Schema.
pub fn trace_type_to_jsonschema<T>() -> String
where
    T: GetTypeRegistration + 'static,
{
    let mut registry = TypeRegistry::new();
    registry.register::<T>();

    match registry.get_type_info(TypeId::of::<T>()) {
        Some(root) => registry_to_jsonschema(&registry, root.type_path()),
        None => fallback_schema("Root reflected type is not registered"),
    }
}

/// Convert a reflected `TypeRegistry` to a JSON Schema string.
pub fn registry_to_jsonschema(registry: &TypeRegistry, root_type: &str) -> String {
    let Some(root) = find_root_type_info(registry, root_type) else {
        return fallback_schema(&format!(
            "Could not find reflected root type `{root_type}` in TypeRegistry"
        ));
    };

    let mut builder = ReflectSchemaBuilder::new(registry);
    builder.ensure_definition(root);

    let mut schema = json!({
        "$schema": DRAFT_07,
        "$ref": builder.ref_for(root.type_path()),
    });

    if !builder.defs.is_empty() {
        schema["$defs"] = Value::Object(builder.defs);
    }

    serde_json::to_string_pretty(&schema).unwrap_or_else(|_| "{}".to_string())
}

fn find_root_type_info(registry: &TypeRegistry, root_type: &str) -> Option<&'static TypeInfo> {
    registry
        .get_with_type_path(root_type)
        .map(|registration| registration.type_info())
        .or_else(|| {
            registry.iter().find_map(|registration| {
                let path = registration.type_info().type_path();
                (path.ends_with(root_type) || root_type.ends_with(path))
                    .then_some(registration.type_info())
            })
        })
}

fn fallback_schema(reason: &str) -> String {
    json!({
        "$schema": DRAFT_07,
        "description": format!("Schema generation failed: {reason}"),
    })
    .to_string()
}

struct ReflectSchemaBuilder<'a> {
    registry: &'a TypeRegistry,
    defs: Map<String, Value>,
}

impl<'a> ReflectSchemaBuilder<'a> {
    fn new(registry: &'a TypeRegistry) -> Self {
        Self {
            registry,
            defs: Map::new(),
        }
    }

    fn ref_for(&self, type_path: &str) -> String {
        format!("#/$defs/{}", escape_json_pointer(type_path))
    }

    fn ensure_definition(&mut self, type_info: &'static TypeInfo) {
        let key = type_info.type_path().to_string();
        if self.defs.contains_key(&key) {
            return;
        }

        // Insert placeholder first to support recursive types.
        self.defs.insert(key.clone(), json!({}));
        let schema = self.inline_schema_for_type(type_info);
        self.defs.insert(key, schema);
    }

    fn schema_for_type(&mut self, type_info: &'static TypeInfo) -> Value {
        self.ensure_definition(type_info);
        json!({ "$ref": self.ref_for(type_info.type_path()) })
    }

    fn inline_schema_for_type(&mut self, type_info: &'static TypeInfo) -> Value {
        match type_info {
            TypeInfo::Struct(info) => self.schema_for_struct(info),
            TypeInfo::TupleStruct(info) => self.schema_for_tuple_struct(info),
            TypeInfo::Tuple(info) => self.schema_for_tuple(info),
            TypeInfo::List(info) => self.schema_for_list(info),
            TypeInfo::Array(info) => self.schema_for_array(info),
            TypeInfo::Map(info) => self.schema_for_map(info),
            TypeInfo::Set(info) => self.schema_for_set(info),
            TypeInfo::Enum(info) => self.schema_for_enum(info),
            TypeInfo::Opaque(info) => primitive_schema_for_type_path(info.type_path())
                .unwrap_or_else(|| {
                    json!({ "description": format!("Opaque reflected type: {}", info.type_path()) })
                }),
        }
    }

    fn schema_for_struct(&mut self, info: &StructInfo) -> Value {
        let mut properties = Map::new();
        let mut required = Vec::new();
        let skipped = self.skipped_indices(info.type_id());

        for (index, field) in info.iter().enumerate() {
            if skipped.contains(&index) {
                continue;
            }

            let field_schema = if is_option_field(field) {
                self.schema_for_option_field(field)
            } else {
                self.schema_for_reflected_or_path(field.type_info(), field.ty())
            };

            properties.insert(field.name().to_string(), field_schema);
            if !is_option_field(field) {
                required.push(Value::String(field.name().to_string()));
            }
        }

        let mut schema = json!({
            "type": "object",
            "properties": properties,
            "additionalProperties": false,
        });

        if !required.is_empty() {
            schema["required"] = Value::Array(required);
        }

        schema
    }

    fn schema_for_tuple_struct(&mut self, info: &TupleStructInfo) -> Value {
        let has_serialization_data = self.serialization_data(info.type_id()).is_some();
        let skipped = self.skipped_indices(info.type_id());

        // Reflect serializes single-field tuple structs as newtypes unless there is
        // extra serialization metadata registered on the type.
        if info.field_len() == 1
            && !has_serialization_data
            && let Some(field) = info.field_at(0)
        {
            return self.schema_for_reflected_or_path(field.type_info(), field.ty());
        }

        let fields: Vec<_> = info
            .iter()
            .enumerate()
            .filter_map(|(index, field)| {
                if skipped.contains(&index) {
                    None
                } else {
                    Some(self.schema_for_reflected_or_path(field.type_info(), field.ty()))
                }
            })
            .collect();

        json!({
            "type": "array",
            "prefixItems": fields,
            "items": false,
            "minItems": fields.len(),
            "maxItems": fields.len(),
        })
    }

    fn schema_for_tuple(&mut self, info: &TupleInfo) -> Value {
        let fields: Vec<_> = info
            .iter()
            .map(|field| self.schema_for_reflected_or_path(field.type_info(), field.ty()))
            .collect();

        json!({
            "type": "array",
            "prefixItems": fields,
            "items": false,
            "minItems": fields.len(),
            "maxItems": fields.len(),
        })
    }

    fn schema_for_list(&mut self, info: &ListInfo) -> Value {
        let item_schema = self.schema_for_reflected_or_path(info.item_info(), &info.item_ty());
        json!({
            "type": "array",
            "items": item_schema,
        })
    }

    fn schema_for_array(&mut self, info: &ArrayInfo) -> Value {
        let item_schema = self.schema_for_reflected_or_path(info.item_info(), &info.item_ty());
        json!({
            "type": "array",
            "items": item_schema,
            "minItems": info.capacity(),
            "maxItems": info.capacity(),
        })
    }

    fn schema_for_map(&mut self, info: &MapInfo) -> Value {
        let value_schema = self.schema_for_reflected_or_path(info.value_info(), &info.value_ty());

        let mut schema = json!({
            "type": "object",
            "additionalProperties": value_schema,
        });

        if !is_string_like_type_path(info.key_ty().path()) {
            schema["description"] = Value::String(format!(
                "JSON map keys are strings; reflected key type is `{}`",
                info.key_ty().path()
            ));
        }

        schema
    }

    fn schema_for_set(&mut self, info: &SetInfo) -> Value {
        let value_type = info.value_ty();
        let item_schema = self.registry.get_type_info(value_type.id()).map_or_else(
            || primitive_schema_for_type_path(value_type.path()).unwrap_or_else(|| json!({})),
            |ti| self.schema_for_type(ti),
        );

        json!({
            "type": "array",
            "items": item_schema,
        })
    }

    fn schema_for_enum(&mut self, info: &EnumInfo) -> Value {
        if let Some(option_inner) = self.schema_for_option_enum(info) {
            return option_inner;
        }

        let variants: Vec<_> = info
            .iter()
            .map(|variant| self.schema_for_variant(variant))
            .collect();

        if variants.len() == 1 {
            variants.into_iter().next().unwrap_or_else(|| json!({}))
        } else {
            json!({ "oneOf": variants })
        }
    }

    fn schema_for_option_enum(&mut self, info: &EnumInfo) -> Option<Value> {
        let none_variant = info.variant("None")?;
        let some_variant = info.variant("Some")?;

        if !matches!(none_variant, VariantInfo::Unit(_)) {
            return None;
        }

        let VariantInfo::Tuple(tuple_variant) = some_variant else {
            return None;
        };

        if tuple_variant.field_len() != 1 {
            return None;
        }

        let inner_field = tuple_variant.field_at(0)?;
        let inner_schema =
            self.schema_for_reflected_or_path(inner_field.type_info(), inner_field.ty());

        Some(json!({
            "oneOf": [
                { "type": "null" },
                inner_schema,
            ]
        }))
    }

    fn schema_for_variant(&mut self, variant: &VariantInfo) -> Value {
        match variant {
            VariantInfo::Unit(unit) => {
                json!({ "type": "string", "const": unit.name() })
            }
            VariantInfo::Tuple(tuple) => {
                if tuple.field_len() == 1 {
                    let inner = tuple
                        .field_at(0)
                        .map(|field| {
                            self.schema_for_reflected_or_path(field.type_info(), field.ty())
                        })
                        .unwrap_or_else(|| json!({}));
                    json!({
                        "type": "object",
                        "properties": {
                            tuple.name(): inner,
                        },
                        "required": [tuple.name()],
                        "additionalProperties": false,
                    })
                } else {
                    let fields: Vec<_> = tuple
                        .iter()
                        .map(|field| {
                            self.schema_for_reflected_or_path(field.type_info(), field.ty())
                        })
                        .collect();

                    json!({
                        "type": "object",
                        "properties": {
                            tuple.name(): {
                                "type": "array",
                                "prefixItems": fields,
                                "items": false,
                                "minItems": fields.len(),
                                "maxItems": fields.len(),
                            }
                        },
                        "required": [tuple.name()],
                        "additionalProperties": false,
                    })
                }
            }
            VariantInfo::Struct(struct_variant) => {
                let mut properties = Map::new();
                let mut required = Vec::new();

                for field in struct_variant.iter() {
                    let field_schema = if is_option_field(field) {
                        self.schema_for_option_field(field)
                    } else {
                        self.schema_for_reflected_or_path(field.type_info(), field.ty())
                    };

                    properties.insert(field.name().to_string(), field_schema);

                    if !is_option_field(field) {
                        required.push(Value::String(field.name().to_string()));
                    }
                }

                let mut inner = json!({
                    "type": "object",
                    "properties": properties,
                    "additionalProperties": false,
                });
                if !required.is_empty() {
                    inner["required"] = Value::Array(required);
                }

                json!({
                    "type": "object",
                    "properties": {
                        struct_variant.name(): inner,
                    },
                    "required": [struct_variant.name()],
                    "additionalProperties": false,
                })
            }
        }
    }

    fn schema_for_option_field(&mut self, field: &NamedField) -> Value {
        if let Some(type_info) = field.type_info()
            && let TypeInfo::Enum(enum_info) = type_info
            && let Some(schema) = self.schema_for_option_enum(enum_info)
        {
            return schema;
        }

        json!({
            "oneOf": [
                { "type": "null" },
                {}
            ]
        })
    }

    fn schema_for_reflected_or_path(
        &mut self,
        type_info: Option<&'static TypeInfo>,
        ty: &Type,
    ) -> Value {
        type_info.map_or_else(
            || {
                primitive_schema_for_type_path(ty.path()).unwrap_or_else(|| {
                    json!({
                        "description": format!("Unknown reflected field type: {}", ty.path())
                    })
                })
            },
            |info| self.schema_for_type(info),
        )
    }

    fn serialization_data(&self, type_id: TypeId) -> Option<&SerializationData> {
        self.registry.get_type_data::<SerializationData>(type_id)
    }

    fn skipped_indices(&self, type_id: TypeId) -> BTreeSet<usize> {
        self.serialization_data(type_id)
            .map(|data| data.iter_skipped().map(|(index, _)| *index).collect())
            .unwrap_or_default()
    }
}

fn is_option_field(field: &NamedField) -> bool {
    if field.ty().module_path() == Some("core::option") && field.ty().ident() == Some("Option") {
        return true;
    }

    if let Some(type_info) = field.type_info()
        && let TypeInfo::Enum(enum_info) = type_info
    {
        return enum_info.variant("None").is_some() && enum_info.variant("Some").is_some();
    }

    false
}

fn is_string_like_type_path(type_path: &str) -> bool {
    matches!(
        type_path,
        "alloc::string::String" | "std::string::String" | "str" | "&str"
    )
}

fn primitive_schema_for_type_path(type_path: &str) -> Option<Value> {
    match type_path {
        "()" => Some(json!({ "type": "null" })),
        "bool" | "core::primitive::bool" => Some(json!({ "type": "boolean" })),

        "i8" | "core::primitive::i8" => {
            Some(json!({ "type": "integer", "minimum": -128, "maximum": 127 }))
        }
        "i16" | "core::primitive::i16" => Some(json!({
            "type": "integer",
            "minimum": -32768,
            "maximum": 32767
        })),
        "i32" | "core::primitive::i32" => Some(json!({ "type": "integer" })),
        "i64" | "core::primitive::i64" => Some(json!({ "type": "integer" })),
        "i128" | "core::primitive::i128" => Some(json!({ "type": "integer" })),
        "isize" | "core::primitive::isize" => Some(json!({ "type": "integer" })),

        "u8" | "core::primitive::u8" => {
            Some(json!({ "type": "integer", "minimum": 0, "maximum": 255 }))
        }
        "u16" | "core::primitive::u16" => Some(json!({
            "type": "integer",
            "minimum": 0,
            "maximum": 65535
        })),
        "u32" | "core::primitive::u32" => Some(json!({ "type": "integer", "minimum": 0 })),
        "u64" | "core::primitive::u64" => Some(json!({ "type": "integer", "minimum": 0 })),
        "u128" | "core::primitive::u128" => Some(json!({ "type": "integer", "minimum": 0 })),
        "usize" | "core::primitive::usize" => Some(json!({ "type": "integer", "minimum": 0 })),

        "f32" | "core::primitive::f32" => Some(json!({ "type": "number" })),
        "f64" | "core::primitive::f64" => Some(json!({ "type": "number" })),

        "char" | "core::primitive::char" => {
            Some(json!({ "type": "string", "minLength": 1, "maxLength": 1 }))
        }

        "alloc::string::String" | "std::string::String" | "str" | "&str" => {
            Some(json!({ "type": "string" }))
        }

        _ => None,
    }
}

fn escape_json_pointer(segment: &str) -> String {
    segment.replace('~', "~0").replace('/', "~1")
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::bevy_reflect;
    use cu29::prelude::Reflect;

    #[derive(Reflect)]
    struct SimpleStruct {
        name: String,
        value: i32,
        active: bool,
    }

    #[derive(Reflect)]
    struct NestedStruct {
        id: u64,
        data: SimpleStruct,
        tags: Vec<String>,
    }

    #[derive(Reflect)]
    enum Status {
        Active,
        Inactive,
        Pending { reason: String },
    }

    #[derive(Reflect)]
    struct WithOptional {
        required_field: String,
        optional_field: Option<i32>,
    }

    #[derive(Reflect)]
    struct WithSkipped {
        kept: i32,
        #[reflect(skip_serializing)]
        skipped: i32,
    }

    fn find_def<'a>(parsed: &'a Value, suffix: &str) -> &'a Value {
        parsed["$defs"]
            .as_object()
            .and_then(|defs| {
                defs.iter()
                    .find_map(|(k, v)| k.ends_with(suffix).then_some(v))
            })
            .expect("Expected schema definition")
    }

    #[test]
    fn test_simple_struct_schema() {
        let schema = trace_type_to_jsonschema::<SimpleStruct>();
        let parsed: Value = serde_json::from_str(&schema).expect("Invalid JSON schema");
        let root = find_def(&parsed, "SimpleStruct");

        assert_eq!(root["type"], "object");
        assert!(root["properties"]["name"].is_object());
        assert!(root["properties"]["value"].is_object());
        assert!(root["properties"]["active"].is_object());
    }

    #[test]
    fn test_nested_struct_schema() {
        let schema = trace_type_to_jsonschema::<NestedStruct>();
        let parsed: Value = serde_json::from_str(&schema).expect("Invalid JSON schema");

        assert!(parsed["$defs"].is_object());
        assert!(find_def(&parsed, "NestedStruct").is_object());
        assert!(find_def(&parsed, "SimpleStruct").is_object());
    }

    #[test]
    fn test_enum_schema() {
        let schema = trace_type_to_jsonschema::<Status>();
        let parsed: Value = serde_json::from_str(&schema).expect("Invalid JSON schema");
        let root = find_def(&parsed, "Status");

        assert!(root["oneOf"].is_array() || root["const"].is_string());
    }

    #[test]
    fn test_optional_field_schema() {
        let schema = trace_type_to_jsonschema::<WithOptional>();
        let parsed: Value = serde_json::from_str(&schema).expect("Invalid JSON schema");
        let root = find_def(&parsed, "WithOptional");

        assert_eq!(root["type"], "object");
        let required = root["required"].as_array().expect("Missing required");
        assert!(required.contains(&json!("required_field")));
        assert!(!required.contains(&json!("optional_field")));
    }

    #[test]
    fn test_skip_serializing_field_schema() {
        let schema = trace_type_to_jsonschema::<WithSkipped>();
        let parsed: Value = serde_json::from_str(&schema).expect("Invalid JSON schema");
        let root = find_def(&parsed, "WithSkipped");

        assert!(root["properties"]["kept"].is_object());
        assert!(root["properties"].get("skipped").is_none());
    }
}
