//! MCAP export support for Copper logs.
//!
//! This module provides functionality to export Copper CopperLists to the MCAP format,
//! which is compatible with Foxglove visualization tools.
//!
//! Schemas are generated at compile time using serde-reflection, which traces the
//! complete type structure including all enum variants and optional fields.

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
    payload: Option<&'a dyn erased_serde::Serialize>,
    tov: &'a Tov,
    process_time: PartialCuTimeRange,
    status_txt: String,
}

/// Information about a channel.
struct ChannelInfo {
    channel_id: u16,
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

/// Export CopperLists to MCAP format with compile-time schemas.
///
/// This function exports CopperLists using schemas that were generated at compile
/// time via serde-reflection. This ensures complete and accurate schema coverage
/// including all enum variants and optional fields.
///
/// # Arguments
/// * `src` - Reader for the CopperList log data
/// * `output_path` - Path to write the MCAP file
///
/// # Type Parameters
/// * `P` - The CopperListTuple type generated from the Copper configuration.
///         Must implement `PayloadSchemas` to provide compile-time schema info.
pub fn export_to_mcap<P, R>(src: R, output_path: &Path) -> CuResult<McapExportStats>
where
    P: CopperListTuple + PayloadSchemas,
    R: Read,
{
    // Get compile-time schemas for all payload types
    let schemas = P::get_payload_schemas();
    let task_ids = P::get_all_task_ids();

    export_to_mcap_with_schemas::<P, R>(src, output_path, task_ids, &schemas)
}

/// Export CopperLists to MCAP format with explicitly provided schemas.
///
/// This is the lower-level export function that takes explicit schemas.
/// Use `export_to_mcap` for the simpler interface with compile-time schemas.
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

    for task_id in task_ids.iter() {
        if let Some(schema_json) = schema_map.get(task_id) {
            let schema_id = mcap_writer
                .add_schema(
                    &format!("copper.{}", task_id),
                    "jsonschema",
                    schema_json.as_bytes(),
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

            channel_infos.push(Some(ChannelInfo { channel_id }));
        } else {
            // No schema for this task - create channel without schema
            let channel_id = mcap_writer
                .add_channel(0, &format!("/{}", task_id), "json", &BTreeMap::new())
                .map_err(|e| {
                    CuError::new_with_cause(&format!("Failed to add channel for {}", task_id), e)
                })?;

            channel_infos.push(Some(ChannelInfo { channel_id }));
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
            if let Some(ref channel_info) = channel_infos.get(idx).and_then(|c| c.as_ref()) {
                // Create message data structure
                let msg_data = McapMessageData {
                    payload: msg.payload(),
                    tov: &msg.tov(),
                    process_time: msg.metadata().process_time(),
                    status_txt: msg.metadata().status_txt().0.to_string(),
                };

                // Serialize to JSON
                let json_data = serde_json::to_vec(&msg_data).map_err(|e| {
                    CuError::new_with_cause("Failed to serialize message to JSON", e)
                })?;

                // Calculate timestamps
                let publish_time = tov_to_nanos(&msg.tov());
                let log_time = option_cutime_to_nanos(&msg.metadata().process_time().start);

                // Write message
                mcap_writer
                    .write_to_known_channel(
                        &MessageHeader {
                            channel_id: channel_info.channel_id,
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
            if samples.len() < sample_messages {
                if let Ok(json_str) = String::from_utf8(msg.data.to_vec()) {
                    samples.push(json_str);
                }
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
        MatchingTasks,
    };
    use cu29_clock::OptionCuTime;
    use serde::{Deserialize, Serialize};
    use std::io::Cursor;
    use tempfile::tempdir;

    // Test payload types
    #[derive(Debug, Clone, Default, Serialize, Deserialize, Encode, Decode, PartialEq)]
    struct TestPayloadA {
        value: i32,
        name: String,
    }

    #[derive(Debug, Clone, Default, Serialize, Deserialize, Encode, Decode, PartialEq)]
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
        // Get compile-time schemas
        let schemas = TestMsgs::get_payload_schemas();

        assert_eq!(schemas.len(), 2);

        // Parse and validate schema for TestPayloadA
        let (task_a_id, schema_a) = &schemas[0];
        assert_eq!(*task_a_id, "task_a");
        let parsed_a: serde_json::Value =
            serde_json::from_str(schema_a).expect("Invalid JSON schema A");
        assert_eq!(parsed_a["type"], "object");
        assert!(parsed_a["properties"]["value"].is_object());
        assert!(parsed_a["properties"]["name"].is_object());

        // Parse and validate schema for TestPayloadB
        let (task_b_id, schema_b) = &schemas[1];
        assert_eq!(*task_b_id, "task_b");
        let parsed_b: serde_json::Value =
            serde_json::from_str(schema_b).expect("Invalid JSON schema B");
        assert_eq!(parsed_b["type"], "object");
        assert!(parsed_b["properties"]["temperature"].is_object());
        assert!(parsed_b["properties"]["active"].is_object());
    }
}
