//! Standard export CLI over this app's `.copper` logs: fsck, extract-copperlists,
//! extract-text-log, export-mcap.

use cu_autoware::payload;
use cu29::prelude::*;
use cu29_export::run_cli;
use cu29_export::serde_to_jsonschema::trace_type_to_jsonschema;

gen_cumsgs!("copperconfig.ron");

impl PayloadSchemas for cumsgs::CuStampedDataSet {
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        let ids = <cumsgs::CuStampedDataSet as MatchingTasks>::get_all_task_ids();
        let sample = trace_type_to_jsonschema::<payload::RefSample>();
        let lane = trace_type_to_jsonschema::<payload::RefLaneSample>();
        let unit = trace_type_to_jsonschema::<()>();
        // Slots repeat the id for a second port: the detector's second slot is the lane.
        let mut detector_slots = 0;
        ids.iter()
            .map(|&id| {
                let schema = match id {
                    "vehicle_dbw" | "intersection_output" => unit.clone(),
                    "euclidean_cluster_detector" => {
                        detector_slots += 1;
                        if detector_slots > 1 {
                            lane.clone()
                        } else {
                            sample.clone()
                        }
                    }
                    _ => sample.clone(),
                };
                (id, schema)
            })
            .collect()
    }
}

fn main() {
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
