use crate::{copperlists_reader, keyframes_reader};
use cu29::prelude::UnifiedLoggerRead;
use cu29::prelude::*;
use cu29::{CopperListTuple, CuResult};
use std::io::Cursor;

pub(crate) fn check<P>(dl: &mut UnifiedLoggerRead, verbose: u8) -> Option<CuResult<()>>
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

    let total_time = last_ts.unwrap() - overall_first_ts.unwrap();
    let cl_rate = if last_cl != 0 {
        let cl_time = total_time / last_cl;
        1_000_000_000f64 / (cl_time.as_nanos() as f64)
    } else {
        0.0
    };

    let kf_rate = if keyframes != 0 {
        let kf_time = total_time / keyframes as u64;
        1_000_000_000f64 / (kf_time.as_nanos() as f64)
    } else {
        0.0
    };

    println!();
    println!("        === Statistics ===");
    println!("  Total time       -> {total_time}");
    println!("  Total used size  -> {useful_size} bytes");
    println!();
    println!("  # of CL          -> {last_cl}");
    println!("  CL rate          -> {cl_rate:.2} Hz");
    println!("  CL total size    -> {cls_size} bytes");
    println!();
    println!("  # of Keyframes   -> {keyframes}");
    println!("  KF rate          -> {kf_rate:.2} Hz");
    println!("  KF total size    -> {kfs_size} bytes");
    println!();
    // println!("  # of StructLogs  -> {keyframes}");
    println!("  SL total size    -> {structured_log_size} bytes");

    None
}
