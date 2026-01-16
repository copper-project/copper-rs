use cu29::prelude::*;
use cu29_export::run_cli;
use cu29_export::serde_to_jsonschema::trace_type_to_jsonschema;

gen_cumsgs!("copperconfig.ron");

// Implement PayloadSchemas for MCAP export with proper JSON schemas
impl PayloadSchemas for cumsgs::CuStampedDataSet {
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        let i32_schema = trace_type_to_jsonschema::<i32>();
        let unit_schema = trace_type_to_jsonschema::<()>();
        vec![
            ("task0", i32_schema.clone()),
            ("task1", i32_schema),
            ("task2", unit_schema),
        ]
    }
}

fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the export CLI");
}
