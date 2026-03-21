use cu29::prelude::*;
use cu29_export::{PayloadSchemas, run_cli, trace_type_to_jsonschema};

pub mod payloads;

gen_cumsgs!("copperconfig.ron");

impl PayloadSchemas for cumsgs::log_only::CuStampedDataSet {
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        let task_ids = <cumsgs::log_only::CuStampedDataSet as MatchingTasks>::get_all_task_ids();
        let schema = trace_type_to_jsonschema::<payloads::MandelbrotStripe>();
        task_ids.iter().map(|&id| (id, schema.clone())).collect()
    }
}

fn main() {
    run_cli::<cumsgs::log_only::CuStampedDataSet>()
        .expect("Failed to run the Mandelbrot export CLI");
}
