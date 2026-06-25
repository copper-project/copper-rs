mod tasks;
use crate::tasks::DoraPayload;
use cu29::prelude::*;
use cu29_export::run_cli;
use cu29_export::serde_to_jsonschema::trace_type_to_jsonschema;

gen_cumsgs!("copperconfig.ron");

// Implement PayloadSchemas for MCAP export with proper JSON schemas
impl PayloadSchemas for cumsgs::CuStampedDataSet {
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        vec![
            ("src", trace_type_to_jsonschema::<DoraPayload>()),
            ("dst", trace_type_to_jsonschema::<()>()),
        ]
    }
}

fn main() {
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
