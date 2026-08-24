extern crate cu29 as bevy;

mod messages;

use cu29::prelude::*;
use cu29_export::{PayloadSchemas, run_cli, trace_type_to_jsonschema};
use messages::*;

// The simulator logs the MCU side of the closed-loop autonomy graph.
// Keep this schema aligned with `sim.rs` rather than the legacy mission config.
gen_cumsgs!("mcu_config.ron");

/// Schema for tasks whose payload type is not traced here: an empty object, so
/// MCAP consumers still see the channel (with Copper's tov/process_time
/// envelope) without a typed payload.
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
                    "bmi088" => trace_type_to_jsonschema::<cu_sensor_payloads::ImuPayload>(),
                    "ahrs" => trace_type_to_jsonschema::<cu_ahrs::AhrsPose>(),
                    "mapper" | "mode_supervisor" => trace_type_to_jsonschema::<ControlInputs>(),
                    "mag_heading" => trace_type_to_jsonschema::<GeographicHeading>(),
                    "navigation" => trace_type_to_jsonschema::<NavigationState>(),
                    "auto_mission" => trace_type_to_jsonschema::<AutoMissionTarget>(),
                    "autonomy_context" => trace_type_to_jsonschema::<AutonomyContext>(),
                    "auto_controller" => trace_type_to_jsonschema::<AutonomyControl>(),
                    "attitude" => trace_type_to_jsonschema::<BodyRateSetpoint>(),
                    "rate" => trace_type_to_jsonschema::<BodyCommand>(),
                    "battery_adc" => trace_type_to_jsonschema::<BatteryVoltage>(),
                    _ => untyped_schema(),
                };
                (id, schema)
            })
            .collect()
    }
}

fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the export CLI");
}
