use crate::{copperlists_reader, keyframes_reader, structlog_reader};
use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use cu29::prelude::UnifiedLoggerRead;
use cu29::prelude::*;
use cu29::{CopperListTuple, CuResult};
use num_format::{Locale, ToFormattedString};
use std::io::Cursor;

fn print_runtime_lifecycle_record(index: usize, entry: &RuntimeLifecycleRecord) {
    println!("    RuntimeLifecycle #{index} @{}", entry.timestamp);
    match &entry.event {
        RuntimeLifecycleEvent::Instantiated {
            config_source,
            effective_config_ron,
            stack,
        } => {
            println!("      event: Instantiated");
            println!("      config_source: {config_source:?}");
            println!("      stack:");
            println!("        app_name: {}", stack.app_name);
            println!("        app_version: {}", stack.app_version);
            println!(
                "        git_commit: {}",
                stack.git_commit.as_deref().unwrap_or("n/a")
            );
            println!(
                "        git_dirty: {}",
                stack
                    .git_dirty
                    .map(|v| if v { "true" } else { "false" })
                    .unwrap_or("n/a")
            );
            println!(
                "        subsystem_id: {}",
                stack.subsystem_id.as_deref().unwrap_or("n/a")
            );
            println!("        subsystem_code: {}", stack.subsystem_code);
            println!("        instance_id: {}", stack.instance_id);
            println!("      effective_config_ron:");
            for line in effective_config_ron.lines() {
                println!("        {line}");
            }
        }
        RuntimeLifecycleEvent::MissionStarted { mission } => {
            println!("      event: MissionStarted");
            println!("      mission: {mission}");
        }
        RuntimeLifecycleEvent::MissionStopped { mission, reason } => {
            println!("      event: MissionStopped");
            println!("      mission: {mission}");
            println!("      reason: {reason}");
        }
        RuntimeLifecycleEvent::Panic {
            message,
            file,
            line,
            column,
        } => {
            println!("      event: Panic");
            println!("      message: {message}");
            println!("      file: {}", file.as_deref().unwrap_or("n/a"));
            println!(
                "      line: {}",
                line.map(|v| v.to_string())
                    .unwrap_or_else(|| "n/a".to_string())
            );
            println!(
                "      column: {}",
                column
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "n/a".to_string())
            );
        }
        RuntimeLifecycleEvent::ShutdownCompleted => {
            println!("      event: ShutdownCompleted");
        }
    }
}

pub(crate) fn check<P>(
    dl: &mut UnifiedLoggerRead,
    verbose: u8,
    dump_runtime_lifecycle: bool,
) -> Option<CuResult<()>>
where
    P: CopperListTuple,
{
    let header = dl.raw_main_header();

    if verbose > 0 {
        println!("Main header: \n{header}");
    }
    let mut overall_first_ts: OptionCuTime = OptionCuTime::none();
    let mut last_ts: OptionCuTime = OptionCuTime::none();
    let mut last_cl = 0;
    let mut keyframes = 0;
    let mut useful_size: usize = 0;
    let mut structured_log_size: usize = 0;
    let mut cls_size: usize = 0;
    let mut kfs_size: usize = 0;
    let mut runtime_lifecycle_size: usize = 0;
    let mut runtime_lifecycle_events: usize = 0;
    let mut sl_entries: usize = 0;

    let result = loop {
        // for _ in 0..4 {
        let section = dl.raw_read_section();
        match section {
            Ok((header, content)) => {
                useful_size += content.len();

                if verbose > 0 {
                    println!("Section: \n{header}");
                }

                match header.entry_type {
                    UnifiedLogType::StructuredLogLine => {
                        structured_log_size += content.len();
                        let mut reader: Cursor<Vec<u8>> = Cursor::new(content);
                        let iter = structlog_reader(&mut reader);
                        for entry in iter {
                            sl_entries += 1;
                            if entry.is_err() {
                                println!("Struct log #{sl_entries} is corrupted: {entry:?}");
                            }
                        }
                    }
                    UnifiedLogType::CopperList => {
                        cls_size += content.len();

                        let mut reader: Cursor<Vec<u8>> = Cursor::new(content);
                        let iter = copperlists_reader::<P>(&mut reader);
                        let mut first_cl = 0;
                        let mut first_ts: OptionCuTime = OptionCuTime::none();
                        for entry in iter {
                            last_cl = entry.id;
                            if first_ts.is_none() {
                                first_cl = entry.id;
                                first_ts = entry
                                    .cumsgs()
                                    .first()
                                    .expect("Empty copperlist")
                                    .metadata()
                                    .process_time()
                                    .start;
                                if overall_first_ts.is_none() {
                                    overall_first_ts = first_ts;
                                }
                            }
                            let last_msg = *entry.cumsgs().last().expect("Empty copperlist");
                            last_ts = last_msg.metadata().process_time().end;
                        }
                        if verbose > 0 {
                            println!(
                                "    CopperLists => OK (id range: [{first_cl}-{last_cl}] timerange: [{first_ts}-{last_ts}])"
                            );
                        }
                    }
                    UnifiedLogType::FrozenTasks => {
                        kfs_size += content.len();
                        let mut reader: Cursor<Vec<u8>> = Cursor::new(content);
                        let iter = keyframes_reader(&mut reader);
                        for entry in iter {
                            keyframes += 1;
                            if verbose > 0 {
                                println!(
                                    "    Keyframe CL/ts: {}/{} ",
                                    entry.culistid, entry.timestamp
                                );
                            }
                        }
                    }
                    UnifiedLogType::RuntimeLifecycle => {
                        runtime_lifecycle_size += content.len();
                        let mut reader: Cursor<Vec<u8>> = Cursor::new(content);
                        loop {
                            match decode_from_std_read::<RuntimeLifecycleRecord, _, _>(
                                &mut reader,
                                standard(),
                            ) {
                                Ok(entry) => {
                                    runtime_lifecycle_events += 1;
                                    if dump_runtime_lifecycle {
                                        print_runtime_lifecycle_record(
                                            runtime_lifecycle_events,
                                            &entry,
                                        );
                                    }
                                }
                                Err(DecodeError::UnexpectedEnd { .. }) => break,
                                Err(DecodeError::Io { inner, .. })
                                    if inner.kind() == std::io::ErrorKind::UnexpectedEof =>
                                {
                                    break;
                                }
                                Err(e) => {
                                    println!(
                                        "RuntimeLifecycle section entry #{} is corrupted: {e:?}",
                                        runtime_lifecycle_events + 1
                                    );
                                    break;
                                }
                            }
                        }
                    }
                    UnifiedLogType::LastEntry => {
                        if verbose > 0 {
                            println!("Last Entry / EOF.");
                            println!();
                        }
                        break Ok(());
                    }
                    UnifiedLogType::Empty => {
                        println!("Error: Found an empty / Uninitialized section");
                    }
                }
            }
            Err(e) => {
                println!("Failed to read section: {e}");
                break Err(e);
            }
        }
    };

    let total_time_nanos = if !overall_first_ts.is_none() && !last_ts.is_none() {
        (last_ts.unwrap() - overall_first_ts.unwrap()).as_nanos() as f64
    } else {
        0.0
    };
    let cl_rate = if last_cl != 0 && total_time_nanos > 0.0 {
        let cl_time_nanos = total_time_nanos / last_cl as f64;
        1_000_000_000f64 / cl_time_nanos
    } else {
        0.0
    };

    let kf_rate = if keyframes != 0 && total_time_nanos > 0.0 {
        let kf_time_nanos = total_time_nanos / keyframes as f64;
        1_000_000_000f64 / kf_time_nanos
    } else {
        0.0
    };

    if result.is_ok() {
        println!("The log checked out OK.");
    } else {
        println!("** The log is corrupted.");
    }

    let bytes_per_sec = if total_time_nanos > 0.0 {
        useful_size as f64 * 1e9 / total_time_nanos
    } else {
        0.0
    };
    let mib_per_sec = if total_time_nanos > 0.0 {
        bytes_per_sec / (1024.0 * 1024.0)
    } else {
        0.0
    };
    let l = &Locale::en;
    println!("        === Statistics ===");
    if !overall_first_ts.is_none() && !last_ts.is_none() {
        let total_time = last_ts.unwrap() - overall_first_ts.unwrap();
        println!("  Total time       -> {total_time}");
    } else {
        println!("  Total time       -> n/a (no copperlists)");
    }
    println!(
        "  Total used size  -> {} bytes",
        useful_size.to_formatted_string(l)
    );
    println!("  Logging rate     -> {mib_per_sec:.02} MiB/s (effective)");

    println!();
    println!("  # of CL          -> {}", last_cl.to_formatted_string(l));
    println!(
        "  CL rate          -> {}.{:02} Hz",
        (cl_rate.trunc() as u64).to_formatted_string(&Locale::en),
        (cl_rate.fract() * 100.0).round() as u64
    );
    println!(
        "  CL total size    -> {} bytes",
        cls_size.to_formatted_string(l)
    );
    println!();
    println!("  # of Keyframes   -> {}", keyframes.to_formatted_string(l));
    println!("  KF rate          -> {kf_rate:.2} Hz");
    println!(
        "  KF total size    -> {} bytes",
        kfs_size.to_formatted_string(l)
    );
    println!();
    println!(
        "  # of SL entries  -> {}",
        sl_entries.to_formatted_string(l)
    );
    println!(
        "  SL total size    -> {} bytes",
        structured_log_size.to_formatted_string(l)
    );
    println!();
    println!(
        "  # of Lifecycle events -> {}",
        runtime_lifecycle_events.to_formatted_string(l)
    );
    println!(
        "  Lifecycle total size  -> {} bytes",
        runtime_lifecycle_size.to_formatted_string(l)
    );

    None
}
