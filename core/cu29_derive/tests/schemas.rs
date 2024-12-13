use bincode::{Decode, Encode};
use cu29_derive::Schema;

#[derive(Encode, Decode, Schema)]
struct TestStruct {
    id: u32,
    name: String,
    active: bool,
}

#[test]
fn test_schema_macro() {
    let schema = TestStruct::schema();
    assert_eq!(
        schema,
        vec![
            ("id".to_string(), "u32".to_string()),
            ("name".to_string(), "String".to_string()),
            ("active".to_string(), "bool".to_string()),
        ]
    );
}
