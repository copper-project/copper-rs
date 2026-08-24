extern crate cu29 as bevy;

mod messages;

use cu29::prelude::*;
use cu29_export::{PayloadSchemas, run_cli, trace_type_to_jsonschema};
use messages::*;

// The simulator and deployed compute runtime use the same auto-flying graph.
gen_cumsgs!("compute_config.ron");

/// Schema for tasks whose payload type is not traced here (see logreader.rs).
/// `cu_zed::ZedDepthMap` as serialized: 16-bit samples in `handle`, scaled by
/// `encoding.units_per_meter`, with `encoding.invalid` marking no-return pixels.
fn depth_map_schema() -> String {
    r#"{"$schema":"https://json-schema.org/draft-07/schema#","type":"object","properties":{
"format":{"type":"object","properties":{"width":{"type":"integer"},"height":{"type":"integer"},"stride":{"type":"integer"}}},
"encoding":{"type":"object","properties":{"units_per_meter":{"type":"number"},"invalid":{"type":"object","properties":{"unsigned":{"type":"integer"}}}}},
"handle":{"type":"array","items":{"type":"integer"}}}}"#
        .to_string()
}

fn untyped_schema() -> String {
    r#"{"$schema":"https://json-schema.org/draft-07/schema#","type":"object","properties":{}}"#
        .to_string()
}

impl PayloadSchemas for CuStampedDataSet {
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        <CuStampedDataSet as MatchingTasks>::get_all_task_ids()
            .iter()
            .map(|&id| {
                let schema = match id {
                    // The reflected schema of the depth map stops at `format`;
                    // spell out the whole payload so exported depth samples are typed.
                    "vitfly_depth_rate" => depth_map_schema(),
                    "zed" => untyped_schema(),
                    "vitfly" => trace_type_to_jsonschema::<cu_vitfly::VitFlyVelocity>(),
                    "vitfly_command" => trace_type_to_jsonschema::<AutonomyVelocityCommand>(),
                    _ => untyped_schema(),
                };
                (id, schema)
            })
            .collect()
    }
}

fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the compute export CLI");
}
