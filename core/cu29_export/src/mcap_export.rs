//! MCAP export support for Copper logs.
//!
//! This module provides functionality to export Copper CopperLists to the MCAP format,
//! which is compatible with Foxglove visualization tools.
//!
//! Payload schemas are provided by `PayloadSchemas` and wrapped into the full
//! exported MCAP message envelope (`payload`, `tov`, `process_time`, `status_txt`).

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Read};
use std::path::Path;

use cu29::prelude::*;
use cu29_clock;
use mcap::records::MessageHeader;
use mcap::write::Writer as McapWriter;
use serde_json;

// Re-export PayloadSchemas from cu29 prelude for convenience
pub use cu29::prelude::PayloadSchemas;

/// Wrapper for serializing CuMsg data to JSON for MCAP messages.
/// This captures the full message including payload, tov, and metadata.
#[derive(serde::Serialize)]
struct McapMessageData<'a> {
    payload: &'a dyn erased_serde::Serialize,
    tov: McapTovData,
    process_time: PartialCuTimeRange,
    status_txt: String,
}

/// Metadata-only record for messages where payload is absent.
#[derive(serde::Serialize)]
struct McapMetadataOnlyData {
    tov: McapTovData,
    process_time: PartialCuTimeRange,
    status_txt: String,
    payload_missing: bool,
}

/// Foxglove-friendly serialization for TOV without JSON Schema oneOf.
#[derive(serde::Serialize)]
struct McapTovData {
    kind: &'static str,
    time_ns: u64,
    start_ns: u64,
    end_ns: u64,
}

/// Information about a channel.
struct ChannelInfo {
    channel_id: u16,
    metadata_channel_id: u16,
}

/// Convert a Tov (Time of Validity) to nanoseconds for MCAP timestamps.
fn tov_to_nanos(tov: &Tov) -> u64 {
    match tov {
        Tov::None => 0,
        Tov::Time(t) => (*t).into(),
        Tov::Range(range) => range.start.into(),
    }
}

/// Convert an OptionCuTime to nanoseconds, returning 0 if None.
fn option_cutime_to_nanos(opt: &cu29_clock::OptionCuTime) -> u64 {
    let opt_cutime: Option<CuTime> = (*opt).into();
    opt_cutime.map(|t: CuTime| -> u64 { t.into() }).unwrap_or(0)
}

fn mcap_tov_data(tov: &Tov) -> McapTovData {
    match tov {
        Tov::None => McapTovData {
            kind: "none",
            time_ns: 0,
            start_ns: 0,
            end_ns: 0,
        },
        Tov::Time(t) => {
            let time_ns: u64 = (*t).into();
            McapTovData {
                kind: "time",
                time_ns,
                start_ns: time_ns,
                end_ns: time_ns,
            }
        }
        Tov::Range(range) => McapTovData {
            kind: "range",
            time_ns: 0,
            start_ns: range.start.into(),
            end_ns: range.end.into(),
        },
    }
}

fn mcap_tov_schema() -> serde_json::Value {
    serde_json::json!({
        "type": "object",
        "properties": {
            "kind": {
                "type": "string",
                "enum": ["none", "time", "range"]
            },
            "time_ns": { "type": "integer", "minimum": 0 },
            "start_ns": { "type": "integer", "minimum": 0 },
            "end_ns": { "type": "integer", "minimum": 0 }
        },
        "required": ["kind", "time_ns", "start_ns", "end_ns"],
        "additionalProperties": false
    })
}

fn metadata_only_message_schema() -> String {
    let process_time_schema =
        inline_schema_local_refs(parse_schema_or_unknown(&crate::trace_type_to_jsonschema::<
            cu29_clock::PartialCuTimeRange,
        >()));
    let (process_time_root, _) = split_schema_root_and_defs(process_time_schema);

    let schema = serde_json::json!({
        "$schema": "https://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "tov": mcap_tov_schema(),
            "process_time": process_time_root,
            "status_txt": { "type": "string" },
            "payload_missing": { "type": "boolean", "const": true }
        },
        "required": ["tov", "process_time", "status_txt", "payload_missing"],
        "additionalProperties": false
    });

    serde_json::to_string_pretty(&schema).unwrap_or_else(|_| "{}".to_string())
}

fn schema_is_full_mcap_message(schema: &serde_json::Value) -> bool {
    let Some(properties) = schema.get("properties").and_then(|value| value.as_object()) else {
        return false;
    };

    properties.contains_key("payload")
        && properties.contains_key("tov")
        && properties.contains_key("process_time")
        && properties.contains_key("status_txt")
}

fn split_schema_root_and_defs(
    mut schema: serde_json::Value,
) -> (
    serde_json::Value,
    serde_json::Map<String, serde_json::Value>,
) {
    let mut defs = serde_json::Map::new();

    if let Some(schema_obj) = schema.as_object_mut() {
        schema_obj.remove("$schema");
        if let Some(serde_json::Value::Object(found_defs)) = schema_obj.remove("$defs") {
            defs = found_defs;
        }
    }

    (schema, defs)
}

fn parse_schema_or_unknown(schema: &str) -> serde_json::Value {
    serde_json::from_str::<serde_json::Value>(schema).unwrap_or_else(|_| serde_json::json!({}))
}

fn unescape_json_pointer(segment: &str) -> String {
    segment.replace("~1", "/").replace("~0", "~")
}

fn resolve_local_ref(
    reference: &str,
    defs: &serde_json::Map<String, serde_json::Value>,
    stack: &mut Vec<String>,
) -> Option<serde_json::Value> {
    let key = unescape_json_pointer(reference.strip_prefix("#/$defs/")?);

    if stack.iter().any(|seen| seen == &key) {
        return Some(serde_json::json!({
            "description": format!("Recursive schema reference omitted for `{key}`")
        }));
    }

    let target = defs.get(&key)?.clone();
    stack.push(key);
    let resolved = inline_local_refs_in_value(target, defs, stack);
    stack.pop();
    Some(resolved)
}

fn inline_local_refs_in_value(
    value: serde_json::Value,
    defs: &serde_json::Map<String, serde_json::Value>,
    stack: &mut Vec<String>,
) -> serde_json::Value {
    match value {
        serde_json::Value::Object(mut map) => {
            if let Some(reference) = map.get("$ref").and_then(|v| v.as_str()) {
                if let Some(mut resolved) = resolve_local_ref(reference, defs, stack) {
                    map.remove("$ref");

                    if !map.is_empty() {
                        let mut merged = resolved.as_object().cloned().unwrap_or_default();
                        for (k, v) in map {
                            merged.insert(k, inline_local_refs_in_value(v, defs, stack));
                        }
                        resolved = serde_json::Value::Object(merged);
                    }

                    return resolved;
                }
            }

            let mut resolved = serde_json::Map::new();
            for (k, v) in map {
                resolved.insert(k, inline_local_refs_in_value(v, defs, stack));
            }
            serde_json::Value::Object(resolved)
        }
        serde_json::Value::Array(values) => serde_json::Value::Array(
            values
                .into_iter()
                .map(|v| inline_local_refs_in_value(v, defs, stack))
                .collect(),
        ),
        primitive => primitive,
    }
}

fn inline_schema_local_refs(mut schema: serde_json::Value) -> serde_json::Value {
    let mut defs = serde_json::Map::new();

    if let Some(schema_obj) = schema.as_object_mut() {
        schema_obj.remove("$schema");
        if let Some(serde_json::Value::Object(found_defs)) = schema_obj.remove("$defs") {
            defs = found_defs;
        }
    }

    inline_local_refs_in_value(schema, &defs, &mut Vec::new())
}

/// Wrap a payload-only schema into the full MCAP JSON message envelope.
///
/// If `schema_json` already describes the MCAP envelope, this returns it unchanged.
fn wrap_payload_schema_for_mcap_message(schema_json: &str) -> String {
    let parsed_payload = inline_schema_local_refs(parse_schema_or_unknown(schema_json));
    if schema_is_full_mcap_message(&parsed_payload) {
        return serde_json::to_string_pretty(&parsed_payload).unwrap_or_else(|_| "{}".to_string());
    }

    let (payload_root, mut defs) = split_schema_root_and_defs(parsed_payload);

    let tov_root = mcap_tov_schema();

    let process_time_schema =
        inline_schema_local_refs(parse_schema_or_unknown(&crate::trace_type_to_jsonschema::<
            cu29_clock::PartialCuTimeRange,
        >()));
    let (process_time_root, process_time_defs) = split_schema_root_and_defs(process_time_schema);
    defs.extend(process_time_defs);

    let mut schema = serde_json::json!({
        "$schema": "https://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "payload": payload_root,
            "tov": tov_root,
            "process_time": process_time_root,
            "status_txt": { "type": "string" }
        },
        "required": ["payload", "tov", "process_time", "status_txt"],
        "additionalProperties": false
    });

    if !defs.is_empty() {
        schema["$defs"] = serde_json::Value::Object(defs);
    }

    serde_json::to_string_pretty(&schema).unwrap_or_else(|_| "{}".to_string())
}

/// Export CopperLists to MCAP format with payload schemas.
///
/// This function uses `PayloadSchemas` for per-task payload schemas and wraps each
/// payload schema into the full JSON envelope emitted in MCAP messages.
///
/// # Arguments
/// * `src` - Reader for the CopperList log data
/// * `output_path` - Path to write the MCAP file
///
/// # Type Parameters
/// * `P` - The CopperListTuple type generated from the Copper configuration. Must implement `PayloadSchemas` to provide compile-time schema info.
pub fn export_to_mcap<P, R>(src: R, output_path: &Path) -> CuResult<McapExportStats>
where
    P: CopperListTuple + PayloadSchemas,
    R: Read,
{
    // Get payload schemas for all task outputs
    let schemas = P::get_payload_schemas();
    let task_ids = P::get_all_task_ids();

    export_to_mcap_with_schemas::<P, R>(src, output_path, task_ids, &schemas)
}

/// Export CopperLists to MCAP format with explicitly provided schemas.
///
/// This is the lower-level export function that takes explicit schemas.
/// Use `export_to_mcap` for the simpler interface with payload schemas.
///
/// # Arguments
/// * `src` - Reader for the CopperList log data
/// * `output_path` - Path to write the MCAP file
/// * `task_ids` - Slice of task IDs corresponding to the messages in CopperLists
/// * `schemas` - Vector of (task_id, schema_json) pairs
pub fn export_to_mcap_with_schemas<P, R>(
    mut src: R,
    output_path: &Path,
    task_ids: &[&str],
    schemas: &[(&str, String)],
) -> CuResult<McapExportStats>
where
    P: CopperListTuple,
    R: Read,
{
    // Create MCAP file
    let file = File::create(output_path)
        .map_err(|e| CuError::new_with_cause("Failed to create MCAP output file", e))?;
    let writer = BufWriter::new(file);

    let mut mcap_writer = McapWriter::new(writer)
        .map_err(|e| CuError::new_with_cause("Failed to create MCAP writer", e))?;

    // Build a map from task_id to schema
    let schema_map: BTreeMap<&str, &str> = schemas
        .iter()
        .map(|(task_id, schema)| (*task_id, schema.as_str()))
        .collect();

    // Register schemas and channels
    let mut channel_infos: Vec<Option<ChannelInfo>> = Vec::with_capacity(task_ids.len());
    let metadata_schema_id = mcap_writer
        .add_schema(
            "copper.meta",
            "jsonschema",
            metadata_only_message_schema().as_bytes(),
        )
        .map_err(|e| CuError::new_with_cause("Failed to add metadata schema", e))?;

    for task_id in task_ids.iter() {
        if let Some(schema_json) = schema_map.get(task_id) {
            let wrapped_schema = wrap_payload_schema_for_mcap_message(schema_json);
            let schema_id = mcap_writer
                .add_schema(
                    &format!("copper.{}", task_id),
                    "jsonschema",
                    wrapped_schema.as_bytes(),
                )
                .map_err(|e| {
                    CuError::new_with_cause(&format!("Failed to add schema for {}", task_id), e)
                })?;

            let channel_id = mcap_writer
                .add_channel(
                    schema_id,
                    &format!("/{}", task_id),
                    "json",
                    &BTreeMap::new(),
                )
                .map_err(|e| {
                    CuError::new_with_cause(&format!("Failed to add channel for {}", task_id), e)
                })?;
            let metadata_channel_id = mcap_writer
                .add_channel(
                    metadata_schema_id,
                    &format!("/{task_id}/__meta"),
                    "json",
                    &BTreeMap::new(),
                )
                .map_err(|e| {
                    CuError::new_with_cause(
                        &format!("Failed to add metadata channel for {}", task_id),
                        e,
                    )
                })?;

            channel_infos.push(Some(ChannelInfo {
                channel_id,
                metadata_channel_id,
            }));
        } else {
            // No schema for this task - create channel without schema
            let channel_id = mcap_writer
                .add_channel(0, &format!("/{}", task_id), "json", &BTreeMap::new())
                .map_err(|e| {
                    CuError::new_with_cause(&format!("Failed to add channel for {}", task_id), e)
                })?;
            let metadata_channel_id = mcap_writer
                .add_channel(
                    metadata_schema_id,
                    &format!("/{task_id}/__meta"),
                    "json",
                    &BTreeMap::new(),
                )
                .map_err(|e| {
                    CuError::new_with_cause(
                        &format!("Failed to add metadata channel for {}", task_id),
                        e,
                    )
                })?;

            channel_infos.push(Some(ChannelInfo {
                channel_id,
                metadata_channel_id,
            }));
        }
    }

    // Write messages
    let mut sequence: u32 = 0;
    let mut messages_written: u64 = 0;
    let mut copperlists_read: u64 = 0;

    for copperlist in crate::copperlists_reader::<P>(&mut src) {
        copperlists_read += 1;
        let msgs = copperlist.cumsgs();

        for (idx, msg) in msgs.iter().enumerate() {
            if let Some(channel_info) = channel_infos.get(idx).and_then(|c| c.as_ref()) {
                let msg_tov = msg.tov();
                let msg_tov_data = mcap_tov_data(&msg_tov);
                let process_time = msg.metadata().process_time();
                let status_txt = msg.metadata().status_txt().0.to_string();

                let (target_channel, json_data) = if let Some(payload) = msg.payload() {
                    let msg_data = McapMessageData {
                        payload,
                        tov: msg_tov_data,
                        process_time,
                        status_txt,
                    };
                    let json_data = serde_json::to_vec(&msg_data).map_err(|e| {
                        CuError::new_with_cause("Failed to serialize message to JSON", e)
                    })?;
                    (channel_info.channel_id, json_data)
                } else {
                    let msg_data = McapMetadataOnlyData {
                        tov: msg_tov_data,
                        process_time,
                        status_txt,
                        payload_missing: true,
                    };
                    let json_data = serde_json::to_vec(&msg_data).map_err(|e| {
                        CuError::new_with_cause("Failed to serialize metadata message to JSON", e)
                    })?;
                    (channel_info.metadata_channel_id, json_data)
                };

                // Calculate timestamps
                let publish_time = tov_to_nanos(&msg_tov);
                let log_time = option_cutime_to_nanos(&process_time.start);

                // Write message
                mcap_writer
                    .write_to_known_channel(
                        &MessageHeader {
                            channel_id: target_channel,
                            sequence,
                            log_time,
                            publish_time,
                        },
                        &json_data,
                    )
                    .map_err(|e| CuError::new_with_cause("Failed to write MCAP message", e))?;

                sequence += 1;
                messages_written += 1;
            }
        }
    }

    // Finish writing
    mcap_writer
        .finish()
        .map_err(|e| CuError::new_with_cause("Failed to finish MCAP file", e))?;

    Ok(McapExportStats {
        copperlists_read,
        messages_written,
        channels_created: channel_infos.iter().filter(|c| c.is_some()).count(),
        schemas_generated: schemas.len(),
    })
}

/// Statistics from an MCAP export operation.
#[derive(Debug, Clone)]
pub struct McapExportStats {
    /// Number of CopperLists read from the source
    pub copperlists_read: u64,
    /// Number of messages written to MCAP
    pub messages_written: u64,
    /// Number of channels created in the MCAP file
    pub channels_created: usize,
    /// Number of schemas generated
    pub schemas_generated: usize,
}

impl std::fmt::Display for McapExportStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "MCAP Export: {} CopperLists → {} messages, {} channels, {} schemas",
            self.copperlists_read,
            self.messages_written,
            self.channels_created,
            self.schemas_generated
        )
    }
}

/// Inspect an MCAP file and print metadata, schemas, and statistics.
///
/// # Arguments
/// * `mcap_path` - Path to the MCAP file to inspect
/// * `show_schemas` - If true, print full schema content
/// * `sample_messages` - Number of sample messages to show per channel (0 = none)
pub fn mcap_info(mcap_path: &Path, show_schemas: bool, sample_messages: usize) -> CuResult<()> {
    use mcap::MessageStream;
    use std::collections::BTreeMap;

    // Read the entire file into memory (required by mcap crate)
    let data = std::fs::read(mcap_path)
        .map_err(|e| CuError::new_with_cause("Failed to read MCAP file", e))?;

    println!("=== MCAP File Info ===");
    println!("File: {}", mcap_path.display());
    println!(
        "Size: {} bytes ({:.2} MB)",
        data.len(),
        data.len() as f64 / 1_048_576.0
    );
    println!();

    // Track statistics
    let mut total_messages: u64 = 0;
    let mut messages_per_channel: BTreeMap<String, u64> = BTreeMap::new();
    let mut schemas: BTreeMap<String, (String, String)> = BTreeMap::new(); // name -> (encoding, data)
    let mut channels: BTreeMap<String, (String, Option<String>)> = BTreeMap::new(); // topic -> (encoding, schema_name)
    let mut sample_data: BTreeMap<String, Vec<String>> = BTreeMap::new();

    // Parse messages
    let stream = MessageStream::new(&data)
        .map_err(|e| CuError::new_with_cause("Failed to parse MCAP file", e))?;

    for msg_result in stream {
        let msg = msg_result.map_err(|e| CuError::new_with_cause("Failed to read message", e))?;

        total_messages += 1;
        *messages_per_channel
            .entry(msg.channel.topic.clone())
            .or_insert(0) += 1;

        // Track channel info
        let schema_name = msg.channel.schema.as_ref().map(|s| s.name.clone());
        channels
            .entry(msg.channel.topic.clone())
            .or_insert_with(|| (msg.channel.message_encoding.clone(), schema_name.clone()));

        // Track schema info
        if let Some(schema) = &msg.channel.schema {
            schemas.entry(schema.name.clone()).or_insert_with(|| {
                (
                    schema.encoding.clone(),
                    String::from_utf8_lossy(&schema.data).to_string(),
                )
            });
        }

        // Collect sample messages
        if sample_messages > 0 {
            let samples = sample_data.entry(msg.channel.topic.clone()).or_default();
            if samples.len() < sample_messages
                && let Ok(json_str) = String::from_utf8(msg.data.to_vec())
            {
                samples.push(json_str);
            }
        }
    }

    // Print statistics
    println!("=== Statistics ===");
    println!("Total messages: {}", total_messages);
    println!("Channels: {}", channels.len());
    println!("Schemas: {}", schemas.len());
    println!();

    // Print channels
    println!("=== Channels ===");
    for (topic, (encoding, schema_name)) in &channels {
        let msg_count = messages_per_channel.get(topic).unwrap_or(&0);
        let schema_info = schema_name
            .as_ref()
            .map(|s| format!(" (schema: {})", s))
            .unwrap_or_default();
        println!(
            "  {} [{}]{}: {} messages",
            topic, encoding, schema_info, msg_count
        );
    }
    println!();

    // Print schemas
    println!("=== Schemas ===");
    for (name, (encoding, data)) in &schemas {
        println!("Schema: {} (encoding: {})", name, encoding);
        if show_schemas {
            // Pretty print JSON if possible
            if encoding == "jsonschema" {
                if let Ok(parsed) = serde_json::from_str::<serde_json::Value>(data) {
                    println!(
                        "{}",
                        serde_json::to_string_pretty(&parsed).unwrap_or_else(|_| data.clone())
                    );
                } else {
                    println!("{}", data);
                }
            } else {
                println!("{}", data);
            }
        }
        println!();
    }

    // Print sample messages
    if sample_messages > 0 && !sample_data.is_empty() {
        println!("=== Sample Messages ===");
        for (topic, samples) in &sample_data {
            println!("Channel: {}", topic);
            for (i, sample) in samples.iter().enumerate() {
                // Pretty print JSON if possible
                if let Ok(parsed) = serde_json::from_str::<serde_json::Value>(sample) {
                    println!(
                        "  [{}] {}",
                        i + 1,
                        serde_json::to_string_pretty(&parsed)
                            .unwrap_or_else(|_| sample.clone())
                            .lines()
                            .collect::<Vec<_>>()
                            .join("\n      ")
                    );
                } else {
                    println!("  [{}] {}", i + 1, sample);
                }
            }
            println!();
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{Decode, Encode, config::standard, encode_into_slice};
    use cu29::prelude::{
        CopperList, CuMsgMetadata, CuStampedData, ErasedCuStampedData, ErasedCuStampedDataSet,
        MatchingTasks, Reflect,
    };
    use cu29_clock::OptionCuTime;
    use serde::{Deserialize, Serialize};
    use std::io::Cursor;
    use tempfile::tempdir;

    // Test payload types
    #[derive(Debug, Clone, Default, Serialize, Deserialize, Encode, Decode, PartialEq, Reflect)]
    struct TestPayloadA {
        value: i32,
        name: String,
    }

    #[derive(Debug, Clone, Default, Serialize, Deserialize, Encode, Decode, PartialEq, Reflect)]
    struct TestPayloadB {
        temperature: f64,
        active: bool,
    }

    // Test CopperList tuple type with two messages
    #[derive(Debug, Default, Encode, Decode, Serialize, Deserialize)]
    struct TestMsgs(
        CuStampedData<TestPayloadA, CuMsgMetadata>,
        CuStampedData<TestPayloadB, CuMsgMetadata>,
    );

    impl ErasedCuStampedDataSet for TestMsgs {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            vec![&self.0, &self.1]
        }
    }

    impl MatchingTasks for TestMsgs {
        fn get_all_task_ids() -> &'static [&'static str] {
            &["task_a", "task_b"]
        }
    }

    // Implement PayloadSchemas for test type
    impl PayloadSchemas for TestMsgs {
        fn get_payload_schemas() -> Vec<(&'static str, String)> {
            vec![
                (
                    "task_a",
                    crate::serde_to_jsonschema::trace_type_to_jsonschema::<TestPayloadA>(),
                ),
                (
                    "task_b",
                    crate::serde_to_jsonschema::trace_type_to_jsonschema::<TestPayloadB>(),
                ),
            ]
        }
    }

    #[test]
    fn test_tov_to_nanos() {
        assert_eq!(tov_to_nanos(&Tov::None), 0);

        let time = CuTime::from(1_000_000_000u64);
        assert_eq!(tov_to_nanos(&Tov::Time(time)), 1_000_000_000);
    }

    #[test]
    fn test_option_cutime_to_nanos() {
        let none = OptionCuTime::none();
        assert_eq!(option_cutime_to_nanos(&none), 0);

        let some_time = OptionCuTime::from(CuTime::from(500_000_000u64));
        assert_eq!(option_cutime_to_nanos(&some_time), 500_000_000);
    }

    #[test]
    fn test_mcap_export_with_compile_time_schemas() {
        let dir = tempdir().expect("Failed to create temp dir");
        let mcap_path = dir.path().join("test_output.mcap");

        // Create test CopperLists
        let mut buffer = vec![0u8; 10000];
        let mut offset = 0;

        // Create a few CopperLists with test data
        for i in 0..3 {
            let mut msgs = TestMsgs::default();

            // Set payload A
            msgs.0.set_payload(TestPayloadA {
                value: i * 10,
                name: format!("test_{}", i),
            });
            msgs.0.tov = Tov::Time(CuTime::from((i as u64) * 1_000_000_000));
            msgs.0.metadata.process_time.start =
                OptionCuTime::from(CuTime::from((i as u64) * 1_000_000_000));
            msgs.0.metadata.process_time.end =
                OptionCuTime::from(CuTime::from((i as u64) * 1_000_000_000 + 1000));

            // Set payload B
            msgs.1.set_payload(TestPayloadB {
                temperature: 25.0 + (i as f64),
                active: i % 2 == 0,
            });
            msgs.1.tov = Tov::Time(CuTime::from((i as u64) * 1_000_000_000 + 500));
            msgs.1.metadata.process_time.start =
                OptionCuTime::from(CuTime::from((i as u64) * 1_000_000_000 + 100));
            msgs.1.metadata.process_time.end =
                OptionCuTime::from(CuTime::from((i as u64) * 1_000_000_000 + 600));

            let cl = CopperList::new(i as u32, msgs);
            offset += encode_into_slice(&cl, &mut buffer[offset..], standard()).unwrap();
        }

        // Export to MCAP using compile-time schemas
        let cursor = Cursor::new(&buffer[..offset]);
        let stats =
            export_to_mcap::<TestMsgs, _>(cursor, &mcap_path).expect("Failed to export MCAP");

        // Verify stats
        assert_eq!(stats.copperlists_read, 3);
        assert_eq!(stats.messages_written, 6); // 2 messages per CopperList * 3
        assert_eq!(stats.channels_created, 2);
        assert_eq!(stats.schemas_generated, 2);

        // Verify MCAP file was created
        assert!(mcap_path.exists());

        // Read back the MCAP file and verify
        let mcap_data = std::fs::read(&mcap_path).expect("Failed to read MCAP file");
        let mapped = mcap_data.as_slice();

        // Parse with mcap library
        let messages: Vec<_> = mcap::MessageStream::new(mapped)
            .expect("Failed to parse MCAP")
            .collect::<Result<Vec<_>, _>>()
            .expect("Failed to read messages");

        // Verify message count
        assert_eq!(messages.len(), 6);

        // Verify channel topics
        let topics: std::collections::HashSet<_> =
            messages.iter().map(|m| m.channel.topic.as_str()).collect();
        assert!(topics.contains("/task_a"));
        assert!(topics.contains("/task_b"));

        // Verify schemas were attached
        for msg in &messages {
            assert!(msg.channel.schema.is_some(), "Channel should have a schema");
            let schema = msg.channel.schema.as_ref().unwrap();
            assert_eq!(schema.encoding, "jsonschema");
            assert!(!schema.data.is_empty(), "Schema data should not be empty");
        }

        // Verify message encoding
        for msg in &messages {
            assert_eq!(msg.channel.message_encoding, "json");
        }

        // Verify we can parse the JSON data
        for msg in &messages {
            let json: serde_json::Value =
                serde_json::from_slice(&msg.data).expect("Failed to parse message JSON");
            assert!(json.get("tov").is_some());
            assert!(json.get("process_time").is_some());
        }
    }

    #[test]
    fn test_generated_schemas_are_complete() {
        // Get payload schemas
        let schemas = TestMsgs::get_payload_schemas();

        assert_eq!(schemas.len(), 2);

        // Parse and validate schema for TestPayloadA
        let (task_a_id, schema_a) = &schemas[0];
        assert_eq!(*task_a_id, "task_a");
        let parsed_a: serde_json::Value =
            serde_json::from_str(schema_a).expect("Invalid JSON schema A");
        let root_a = parsed_a["$defs"]
            .as_object()
            .and_then(|defs| {
                defs.iter()
                    .find_map(|(k, v)| k.ends_with("TestPayloadA").then_some(v))
            })
            .expect("Missing TestPayloadA definition");
        assert_eq!(root_a["type"], "object");
        assert!(root_a["properties"]["value"].is_object());
        assert!(root_a["properties"]["name"].is_object());

        // Parse and validate schema for TestPayloadB
        let (task_b_id, schema_b) = &schemas[1];
        assert_eq!(*task_b_id, "task_b");
        let parsed_b: serde_json::Value =
            serde_json::from_str(schema_b).expect("Invalid JSON schema B");
        let root_b = parsed_b["$defs"]
            .as_object()
            .and_then(|defs| {
                defs.iter()
                    .find_map(|(k, v)| k.ends_with("TestPayloadB").then_some(v))
            })
            .expect("Missing TestPayloadB definition");
        assert_eq!(root_b["type"], "object");
        assert!(root_b["properties"]["temperature"].is_object());
        assert!(root_b["properties"]["active"].is_object());
    }

    #[test]
    fn test_payload_schema_is_wrapped_into_message_schema() {
        fn contains_key_recursive(value: &serde_json::Value, key: &str) -> bool {
            match value {
                serde_json::Value::Object(map) => {
                    map.contains_key(key)
                        || map.values().any(|inner| contains_key_recursive(inner, key))
                }
                serde_json::Value::Array(values) => values
                    .iter()
                    .any(|inner| contains_key_recursive(inner, key)),
                _ => false,
            }
        }

        let payload_schema = crate::trace_type_to_jsonschema::<TestPayloadA>();
        let wrapped = wrap_payload_schema_for_mcap_message(&payload_schema);
        let parsed: serde_json::Value =
            serde_json::from_str(&wrapped).expect("Invalid wrapped schema");

        assert_eq!(parsed["type"], "object");
        assert!(parsed["properties"]["payload"].is_object());
        assert!(parsed["properties"]["tov"].is_object());
        assert!(parsed["properties"]["process_time"].is_object());
        assert_eq!(parsed["properties"]["status_txt"]["type"], "string");
        assert!(parsed["properties"]["payload"].get("oneOf").is_none());
        assert!(parsed["properties"]["tov"].get("oneOf").is_none());
        assert_eq!(parsed["properties"]["tov"]["type"], "object");
        assert_eq!(
            parsed["properties"]["tov"]["properties"]["kind"]["enum"],
            serde_json::json!(["none", "time", "range"])
        );
        assert!(!contains_key_recursive(&parsed, "$ref"));
        assert!(!contains_key_recursive(&parsed, "$defs"));
    }

    #[test]
    fn test_exported_messages_match_channel_json_schema() {
        let dir = tempdir().expect("Failed to create temp dir");
        let mcap_path = dir.path().join("validated_output.mcap");

        let mut buffer = vec![0u8; 4096];
        let mut offset = 0;

        for i in 0..2 {
            let mut msgs = TestMsgs::default();
            msgs.0.set_payload(TestPayloadA {
                value: i * 5,
                name: format!("schema_check_{}", i),
            });
            msgs.1.set_payload(TestPayloadB {
                temperature: 20.0 + (i as f64),
                active: i % 2 == 0,
            });
            let cl = CopperList::new(i as u32, msgs);
            offset += encode_into_slice(&cl, &mut buffer[offset..], standard()).unwrap();
        }

        let cursor = Cursor::new(&buffer[..offset]);
        export_to_mcap::<TestMsgs, _>(cursor, &mcap_path).expect("Failed to export MCAP");

        let mcap_data = std::fs::read(&mcap_path).expect("Failed to read MCAP file");
        let messages: Vec<_> = mcap::MessageStream::new(mcap_data.as_slice())
            .expect("Failed to parse MCAP")
            .collect::<Result<Vec<_>, _>>()
            .expect("Failed to read messages");

        for msg in messages {
            let schema_data = &msg
                .channel
                .schema
                .as_ref()
                .expect("Channel should contain schema")
                .data;

            let schema: serde_json::Value =
                serde_json::from_slice(schema_data).expect("Invalid schema JSON");
            let instance: serde_json::Value =
                serde_json::from_slice(&msg.data).expect("Invalid message JSON");

            let validator = jsonschema::validator_for(&schema).expect("Invalid JSON Schema");
            if let Err(error) = validator.validate(&instance) {
                panic!("Exported message did not match channel schema: {error}");
            }
        }
    }

    #[test]
    fn test_export_routes_missing_payload_to_metadata_channel() {
        let dir = tempdir().expect("Failed to create temp dir");
        let mcap_path = dir.path().join("metadata_only_payloads.mcap");

        let mut buffer = vec![0u8; 4096];
        let mut offset = 0;

        // One list with only task_a payload, task_b left as None.
        let mut msgs = TestMsgs::default();
        msgs.0.set_payload(TestPayloadA {
            value: 1,
            name: "only_a".to_string(),
        });
        let cl = CopperList::new(0, msgs);
        offset += encode_into_slice(&cl, &mut buffer[offset..], standard()).unwrap();

        // One list with no payloads at all.
        let cl_empty = CopperList::new(1, TestMsgs::default());
        offset += encode_into_slice(&cl_empty, &mut buffer[offset..], standard()).unwrap();

        let cursor = Cursor::new(&buffer[..offset]);
        export_to_mcap::<TestMsgs, _>(cursor, &mcap_path).expect("Failed to export MCAP");

        let mcap_data = std::fs::read(&mcap_path).expect("Failed to read MCAP file");
        let messages: Vec<_> = mcap::MessageStream::new(mcap_data.as_slice())
            .expect("Failed to parse MCAP")
            .collect::<Result<Vec<_>, _>>()
            .expect("Failed to read messages");

        assert_eq!(messages.len(), 4);

        let mut main_count = 0usize;
        let mut meta_count = 0usize;

        for message in &messages {
            let instance: serde_json::Value =
                serde_json::from_slice(&message.data).expect("Invalid message JSON");
            if message.channel.topic.ends_with("/__meta") {
                meta_count += 1;
                assert_eq!(instance["payload_missing"], serde_json::json!(true));
                assert!(instance.get("payload").is_none());
            } else {
                main_count += 1;
                assert!(instance.get("payload").is_some());
                assert!(!instance["payload"].is_null(), "payload must never be null");
            }
        }

        assert_eq!(main_count, 1);
        assert_eq!(meta_count, 3);
    }
}
