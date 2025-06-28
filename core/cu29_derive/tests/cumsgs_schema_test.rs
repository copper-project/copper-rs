use cu29::prelude::*;

// Generate CuMsgs from a test config
gen_cumsgs!("tests/test_config.ron");

#[test]
fn test_cumsgs_schema_implementation() {
    // Test that CuMsgs implements Schema
    let schema = CuMsgs::schema();

    // Verify that schema is not empty (assuming the config has some tasks)
    assert!(!schema.is_empty(), "CuMsgs schema should not be empty");

    // Test type name
    assert_eq!(CuMsgs::type_name(), "CuMsgs");

    // Verify that all fields have the expected naming pattern (field_0, field_1, etc.)
    for field_name in schema.keys() {
        assert!(
            field_name.starts_with("field_"),
            "Field name '{field_name}' should start with 'field_'"
        );
    }

    // Test that we have exactly 2 fields (matching test_config.ron)
    assert_eq!(
        schema.len(),
        2,
        "Should have exactly 2 fields matching test_config.ron"
    );

    // Test that the fields correspond to the expected CuMsg types
    assert!(schema.contains_key("field_0"), "Should contain field_0");
    assert!(schema.contains_key("field_1"), "Should contain field_1");

    // Verify the schema types are CuMsg structs and test their payload fields
    for (field_name, field_type) in &schema {
        match field_type {
            cu29::prelude::SchemaType::Struct { name, fields } => {
                assert_eq!(name, "CuMsg", "Field {field_name} should be a CuMsg struct");

                // Check that the CuMsg has payload and metadata fields
                assert!(
                    fields.contains_key("payload"),
                    "CuMsg should have a payload field"
                );
                assert!(
                    fields.contains_key("metadata"),
                    "CuMsg should have a metadata field"
                );

                // Test the payload field type
                match fields.get("payload").unwrap() {
                    cu29::prelude::SchemaType::Option(inner_type) => {
                        match field_name.as_str() {
                            "field_0" => {
                                // field_0.payload should be i32
                                match inner_type.as_ref() {
                                    cu29::prelude::SchemaType::I32 => {
                                        println!("✓ field_0.payload is i32 as expected");
                                    }
                                    _ => panic!(
                                        "field_0.payload should be i32, but got: {inner_type:?}"
                                    ),
                                }
                            }
                            "field_1" => {
                                // field_1.payload should be () (unit type)
                                match inner_type.as_ref() {
                                    cu29::prelude::SchemaType::UNIT => {
                                        println!("✓ field_1.payload is () as expected");
                                    }
                                    _ => panic!(
                                        "field_1.payload should be (), but got: {:?}",
                                        inner_type
                                    ),
                                }
                            }
                            _ => panic!("Unexpected field name: {field_name}"),
                        }
                    }
                    _ => panic!("CuMsg payload field should be an Option type"),
                }
            }
            _ => panic!("Field {field_name} should be a struct type"),
        }
    }

    // Test metadata schema by checking CuMsgMetadata structure
    // Since CuMsgMetadata doesn't implement Schema directly, we test its known structure
    println!("Testing metadata schema structure...");

    // CuMsgMetadata should have: process_time, tov, status_txt fields
    // We can't directly test the schema since it doesn't implement Schema,
    // but we can verify the structure exists by checking the generated code works

    // Test payload type matching from test_config.ron
    // test_config.ron defines:
    // - test_task1 with type "i32"
    // - test_task2 with type "f64"
    // - connection from test_task1 to test_task2 with message type "i32"

    println!("Testing payload type matching with test_config.ron...");

    // Verify that i32 and f64 implement Schema (they should as primitive types)
    assert_eq!(i32::type_name(), "i32", "i32 should have correct type name");
    assert_eq!(f64::type_name(), "f64", "f64 should have correct type name");

    // For primitive types, schema() returns empty HashMap but schema_type() returns the correct type
    let i32_schema_type = i32::schema_type();
    let f64_schema_type = f64::schema_type();

    match i32_schema_type {
        cu29::prelude::SchemaType::I32 => println!("✓ i32 schema type is correct"),
        _ => panic!("i32 should have I32 schema type"),
    }

    match f64_schema_type {
        cu29::prelude::SchemaType::F64 => println!("✓ f64 schema type is correct"),
        _ => panic!("f64 should have F64 schema type"),
    }

    // Print schema for debugging
    println!("CuMsgs schema:");
    for (field_name, field_type) in &schema {
        println!("  {field_name}: {field_type}");
    }

    // Test that print_schema works
    CuMsgs::print_schema();

    println!("✓ All schema tests passed!");
}

fn main() {
    test_cumsgs_schema_implementation();
    println!("✅ CuMsgs Schema implementation test passed!");
}
