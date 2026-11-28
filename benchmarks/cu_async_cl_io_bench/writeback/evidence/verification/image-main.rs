pub mod tasks;
use cu29::prelude::*;
use std::time::{Duration, Instant};

#[copper_runtime(config = "copperconfig.ron")]
struct ImageAlignerApp {}

fn main() -> CuResult<()> {
    // Keep the repository's source, aligner, sinks and their scheduling intact.
    // Only the input geometry, logging config, pacing and validation are adapted.
    let temp = tempfile::TempDir::new().unwrap();
    let path = temp.path().join("image-aligner.copper");
    let app = ImageAlignerApp::builder().with_log_path(&path,Some(64 << 20))?.build()?;
    let mut app = app.start()?;
    let n: usize = std::env::var("CU_FRAMES").unwrap().parse().unwrap();
    let mut samples = Vec::with_capacity(n);
    let start = Instant::now();
    for i in 0..n {
        let release = start + Duration::from_millis(i as u64 * 50);
        std::thread::sleep(release.saturating_duration_since(Instant::now()));
        let begin = Instant::now();
        app.run_one_iteration()?;
        samples.push((begin.duration_since(start).as_nanos(),begin.elapsed().as_nanos()));
    }
    let finish = Instant::now();
    let mut stopped = app.stop()?.into_inner();
    let drops = stopped.copper_runtime_mut().copperlists_manager.dropped_copperlists_total();
    stopped.log_shutdown_completed()?;
    drop(stopped);
    let drain = finish.elapsed();
    let mut reader = UnifiedLoggerRead::new(&path).unwrap();
    assert!(reader.raw_main_header().clean_close);
    let mut ids = Vec::new();
    let mut keyframes = Vec::new();
    let mut data_bytes = 0;
    loop {
        let (header, bytes) = reader.raw_read_section()?;
        if header.entry_type == UnifiedLogType::LastEntry { break; }
        let mut remaining = bytes.as_slice();
        if header.entry_type == UnifiedLogType::CopperList { data_bytes += bytes.len(); }
        while !remaining.is_empty() {
            let used = match header.entry_type {
                UnifiedLogType::CopperList => {
                    let (cl, used) = cu29::bincode::decode_from_slice::<default::CuList,_>(remaining,cu29::bincode::config::standard()).unwrap();
                    ids.push(cl.id);
                    used
                }
                UnifiedLogType::FrozenTasks => {
                    let (kf, used) = cu29::bincode::decode_from_slice::<cu29::curuntime::KeyFrame,_>(remaining,cu29::bincode::config::standard()).unwrap();
                    keyframes.push(kf.culistid);
                    used
                }
                _ => break,
            };
            remaining = &remaining[used..];
        }
    }
    assert!(ids.windows(2).all(|pair| pair[0] < pair[1]));
    assert_eq!(ids.len() as u64 + drops,n as u64);
    assert!(!ids.is_empty() && !keyframes.is_empty());
    assert!(keyframes.iter().all(|k| ids.binary_search(k).is_ok()));
    for (i,(begin,elapsed)) in samples.iter().enumerate() {
        println!("TICK {i} {begin} {elapsed}");
    }
    println!("RESULT frames={n} recovered={} drops={drops} keyframes={} cl_bytes={data_bytes} drain_us={}",ids.len(),keyframes.len(),drain.as_micros());
    Ok(())
}
