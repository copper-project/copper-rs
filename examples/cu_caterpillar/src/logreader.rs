use cu_rp_gpio::RPGpioPayload;
use cu29::prelude::*;
use cu29_export::run_cli;
use cu29_export::serde_to_jsonschema::trace_type_to_jsonschema;

gen_cumsgs!("copperconfig.ron");

// Implement PayloadSchemas for MCAP export with proper JSON schemas
impl PayloadSchemas for cumsgs::CuStampedDataSet {
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        // All tasks in this example use RPGpioPayload
        let task_ids = <cumsgs::CuStampedDataSet as MatchingTasks>::get_all_task_ids();
        let schema = trace_type_to_jsonschema::<RPGpioPayload>();
        task_ids.iter().map(|&id| (id, schema.clone())).collect()
    }
}

fn main() {
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
