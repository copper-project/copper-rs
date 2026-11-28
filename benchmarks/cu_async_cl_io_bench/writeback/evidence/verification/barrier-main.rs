fn main() -> CuResult<()> {
    use std::path::Path;
    use std::time::{Duration, Instant};

    let logger = Path::new("barrier.copper");
    let app = App::builder().with_log_path(logger,Some(8 << 20))?.build()?;
    let mut app = app.start()?;
    // Let startup finish. The one structured line then wakes the parked worker.
    std::thread::sleep(Duration::from_millis(250));
    std::fs::write("arm", b"").unwrap();
    let mut wake = CuLogEntry::new(1_000_000, CuLogLevel::Info);
    log(&mut wake)?;
    let wait = Instant::now();
    while !Path::new("held").exists() {
        assert!(wait.elapsed() < Duration::from_secs(4), "structured-only wake/deadline did not reach mmap flush");
        std::thread::sleep(Duration::from_millis(1));
    }

    const TICKS: u32 = 12;
    const LINES: u32 = 10_000;
    let during = Instant::now();
    for index in 0..LINES {
        let mut line = CuLogEntry::new(2_000_000 + index, CuLogLevel::Info);
        log(&mut line)?;
        if index < TICKS {
            app.run_one_iteration()?;
        }
    }
    let held_ms = during.elapsed().as_millis();
    std::fs::write("release", b"").unwrap();
    let mut stopped = app.stop()?.into_inner();
    let manager = &stopped.copper_runtime_mut().copperlists_manager;
    let cl_drops = manager.dropped_copperlists_total();
    let kf_drops = manager.dropped_keyframes_total();
    assert!(cl_drops > 0 && cl_drops < TICKS.into());
    assert_eq!(cl_drops, kf_drops);
    let error = stopped.log_shutdown_completed().unwrap_err().to_string();
    let (_, after) = error.split_once("buffer overflow: ").expect(&error);
    let text_drops: usize = after.split_whitespace().next().unwrap().parse().unwrap();
    assert!(text_drops > 0);
    stopped.log_shutdown_completed()?;
    drop(stopped);

    let mut log = UnifiedLoggerRead::new(logger).unwrap();
    assert!(log.raw_main_header().clean_close);
    let mut cl = Vec::new();
    let mut kf = Vec::new();
    let mut lines = Vec::new();
    let mut woke = 0;
    loop {
        let (header, bytes) = log.raw_read_section()?;
        if header.entry_type == UnifiedLogType::LastEntry { break; }
        let mut remaining = bytes.as_slice();
        while !remaining.is_empty() {
            let n = match header.entry_type {
                UnifiedLogType::CopperList => {
                    let (e,n) = bincode::decode_from_slice::<default::CuList,_>(remaining,bincode::config::standard()).unwrap();
                    cl.push(e.id);
                    n
                }
                UnifiedLogType::FrozenTasks => {
                    let (e,n) = bincode::decode_from_slice::<cu29::curuntime::KeyFrame,_>(remaining,bincode::config::standard()).unwrap();
                    kf.push(e.culistid);
                    n
                }
                UnifiedLogType::StructuredLogLine => {
                    let (e,n) = bincode::decode_from_slice::<CuLogEntry,_>(remaining,bincode::config::standard()).unwrap();
                    if (2_000_000..2_000_000+LINES).contains(&e.msg_index) { lines.push(e.msg_index-2_000_000); }
                    if e.msg_index == 1_000_000 { woke += 1; }
                    n
                }
                _ => break,
            };
            remaining = &remaining[n..];
        }
    }
    assert_eq!(woke,1);
    assert_eq!(cl.len() as u64 + cl_drops, u64::from(TICKS));
    assert!(cl.windows(2).all(|w| w[0] < w[1]));
    assert_eq!(kf,cl);
    assert_eq!(lines.len() + text_drops, LINES as usize);
    assert!(lines.iter().copied().eq(0..lines.len() as u32));
    println!("worker held for {held_ms} ms while producer completed {TICKS} ticks and {LINES} text submissions; recovered CL/keyframe IDs {cl:?}, dropped {cl_drops} pairs, recovered {} ordered text lines, reported {text_drops} text drops, clean close", lines.len());
    Ok(())
}
