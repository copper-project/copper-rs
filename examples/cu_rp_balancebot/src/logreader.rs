pub mod tasks;

use cu29::prelude::*;
use cu29_export::run_cli;
use cu29_export::serde_to_jsonschema::trace_type_to_jsonschema;
use cu_ads7883_new::ADSReadingPayload;
use cu_pid::PIDControlOutputPayload;
use cu_rp_encoder::EncoderPayload;
use cu_rp_sn754410_new::MotorPayload;

gen_cumsgs!("copperconfig.ron");

// Implement PayloadSchemas for MCAP export with proper JSON schemas
impl PayloadSchemas for cumsgs::CuStampedDataSet {
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        vec![
            ("balpos", trace_type_to_jsonschema::<ADSReadingPayload>()),
            ("railpos", trace_type_to_jsonschema::<EncoderPayload>()),
            ("balpos_pid", trace_type_to_jsonschema::<PIDControlOutputPayload>()),
            ("railpos_pid", trace_type_to_jsonschema::<PIDControlOutputPayload>()),
            ("merge_pids", trace_type_to_jsonschema::<MotorPayload>()),
        ]
    }
}

fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the export CLI");
}
