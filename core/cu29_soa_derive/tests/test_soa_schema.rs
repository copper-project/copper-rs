#[derive(cu29_soa_derive::Soa)]
pub struct TestStruct {
    a: i32,
    b: f64,
    c: bool,
}

fn main() {
    use cu29_schema::Schema;

    // Test that the SoA type implements Schema
    let schema = TestStructSoa::<10>::schema();
    println!("Schema: {:?}", schema);

    let type_name = TestStructSoa::<10>::type_name();
    println!("Type name: {}", type_name);

    let schema_type = TestStructSoa::<10>::schema_type();
    println!("Schema type: {:?}", schema_type);
}
