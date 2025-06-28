use cu29::prelude::*;
use cu29_derive::Schema as SchemaDer;

#[derive(SchemaDer)]
struct TestStruct {
    id: u32,
    name: String,
    value: f64,
    active: bool,
}

#[derive(SchemaDer)]
struct ComplexStruct {
    numbers: Vec<i32>,
    optional: Option<String>,
    nested: TestStruct,
}

#[test]
fn test_basic_struct_schema() {
    let schema = TestStruct::schema();
    assert_eq!(schema.len(), 4);
    assert_eq!(schema.get("id"), Some(&SchemaType::U32));
    assert_eq!(schema.get("name"), Some(&SchemaType::String));
    assert_eq!(schema.get("value"), Some(&SchemaType::F64));
    assert_eq!(schema.get("active"), Some(&SchemaType::Bool));
}

#[test]
fn test_struct_type_name() {
    assert_eq!(TestStruct::type_name(), "TestStruct");
}

#[test]
fn test_complex_struct_schema() {
    let complex_schema = ComplexStruct::schema();
    assert_eq!(complex_schema.len(), 3);
    assert_eq!(
        complex_schema.get("numbers"),
        Some(&SchemaType::Vec(Box::new(SchemaType::I32)))
    );
    assert_eq!(
        complex_schema.get("optional"),
        Some(&SchemaType::Option(Box::new(SchemaType::String)))
    );
    assert_eq!(
        complex_schema.get("nested"),
        Some(&SchemaType::Struct {
            name: "TestStruct".to_string(),
            fields: TestStruct::schema()
        })
    );
}
