#![cfg(feature = "std")]
use cu29::prelude::*;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
static FAILURE: AtomicU8 = AtomicU8::new(0);
fn fail(operation: u8) -> CuResult<()> {
    if FAILURE.load(Ordering::Relaxed) == operation {
        Err(CuError::from("injected lifecycle failure"))
    } else {
        Ok(())
    }
}
#[derive(Reflect)]
struct Source;
impl Freezable for Source {}
impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        fail(1)?;
        Ok(Self)
    }
    fn start(&mut self, _: &CuContext) -> CuResult<()> {
        fail(2)
    }
    fn stop(&mut self, _: &CuContext) -> CuResult<()> {
        fail(3)
    }
    fn process(&mut self, _: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        assert_ne!(FAILURE.load(Ordering::Relaxed), 5, "injected process panic");
        fail(4)?;
        output.set_payload(42);
        Ok(())
    }
}
#[derive(Reflect)]
struct Sink;
impl Freezable for Sink {}
impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &CuContext, _: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
struct FatalMonitor;
impl CuMonitor for FatalMonitor {
    fn new(_: CuMonitoringMetadata, _: CuMonitoringRuntime) -> CuResult<Self> {
        Ok(Self)
    }
    fn process_copperlist(&self, _: &CuContext, _: CopperListView<'_>) -> CuResult<()> {
        Ok(())
    }
    fn process_error(&self, _: ComponentId, _: CuComponentState, _: &CuError) -> Decision {
        Decision::Shutdown
    }
}
#[copper_runtime(config = "tests/metadata_lifecycle.ron")]
struct App {}
fn logger(path: &std::path::Path) -> Arc<Mutex<UnifiedLoggerWrite>> {
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .preallocated_size(65536)
        .write(true)
        .create(true)
        .build()
        .unwrap()
    else {
        unreachable!()
    };
    Arc::new(Mutex::new(logger))
}
fn events(path: &std::path::Path) -> Vec<(SectionContext, RuntimeLifecycleEvent)> {
    let mut reader = UnifiedLoggerRead::new(path).unwrap();
    assert!(reader.application_metadata().unwrap().is_some());
    let mut events = Vec::new();
    loop {
        let (header, content) = reader.raw_read_section().unwrap();
        if header.entry_type == UnifiedLogType::LastEntry {
            break;
        }
        if header.entry_type == UnifiedLogType::RuntimeLifecycle {
            assert!(!header.is_open);
            let mut bytes = content.as_slice();
            while !bytes.is_empty() {
                let (record, used) = bincode::decode_from_slice::<RuntimeLifecycleRecord, _>(
                    bytes,
                    bincode::config::standard(),
                )
                .unwrap();
                events.push((header.context, record.event));
                bytes = &bytes[used..];
            }
        }
    }
    events
}
#[test]
fn successful_restart_and_failed_transitions_record_exact_lifecycle() {
    let root =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/lifecycle-tests");
    std::fs::create_dir_all(&root).unwrap();
    let dir = tempfile::TempDir::new_in(root).unwrap();
    for operation in 0..=4 {
        FAILURE.store(operation, Ordering::Relaxed);
        let path = dir.path().join(format!("operation{operation}.copper"));
        let logger = logger(&path);
        let app = App::builder()
            .with_logger::<memmap::MmapSectionStorage, UnifiedLoggerWrite>(logger.clone())
            .with_resources(|_| {
                assert!(
                    UnifiedLoggerRead::new(&path)
                        .unwrap()
                        .application_metadata()
                        .unwrap()
                        .is_some()
                );
                Ok(cu29::resource::ResourceManager::new(&[]))
            })
            .build();
        if operation == 1 {
            assert!(app.is_err());
        } else {
            let started = app.unwrap().start();
            if operation == 2 {
                let error = match started {
                    Ok(_) => panic!("start should fail"),
                    Err(error) => error,
                };
                FAILURE.store(0, Ordering::Relaxed);
                drop(error.app.stop().unwrap());
            } else {
                let mut running = started.unwrap();
                let iteration = running.run_one_iteration();
                assert_eq!(iteration.is_err(), operation == 4);
                let stopped = running.stop();
                if operation == 3 {
                    let error = match stopped {
                        Ok(_) => panic!("stop should fail"),
                        Err(error) => error,
                    };
                    FAILURE.store(0, Ordering::Relaxed);
                    drop(error.app.stop().unwrap());
                } else {
                    let stopped = stopped.unwrap();
                    if operation == 0 {
                        drop(stopped.start().unwrap().stop().unwrap());
                    } else {
                        drop(stopped);
                    }
                }
            }
        }
        drop(logger);
        let saved = events(&path);
        if operation == 1 {
            assert!(saved.is_empty());
            continue;
        }
        assert!(matches!(
            saved[0].1,
            RuntimeLifecycleEvent::Instantiated { .. }
        ));
        assert!(matches!(
            saved.last().unwrap().1,
            RuntimeLifecycleEvent::ShutdownCompleted
        ));
        assert!(saved.iter().all(|(context, _)| context.run_id == 1));
        let failures = saved
            .iter()
            .filter_map(|(_, event)| {
                if let RuntimeLifecycleEvent::LifecycleFailed { operation, .. } = event {
                    Some(*operation)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        assert_eq!(
            failures,
            match operation {
                0 => vec![],
                2 => vec![RuntimeLifecycleOperation::Start],
                3 => vec![RuntimeLifecycleOperation::Stop],
                4 => vec![RuntimeLifecycleOperation::Iteration],
                _ => unreachable!(),
            }
        );
        assert_eq!(
            saved
                .iter()
                .filter(|(_, event)| *event == RuntimeLifecycleEvent::MissionStarted)
                .count(),
            if operation == 0 {
                2
            } else if operation == 2 {
                0
            } else {
                1
            }
        );
    }
    let path = dir.path().join("panic.copper");
    let panic_logger = logger(&path);
    FAILURE.store(5, Ordering::Relaxed);
    assert!(
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut app = App::builder()
                .with_logger::<memmap::MmapSectionStorage, UnifiedLoggerWrite>(panic_logger.clone())
                .build()
                .unwrap()
                .start()
                .unwrap();
            app.run_one_iteration().unwrap();
        }))
        .is_err()
    );
    FAILURE.store(0, Ordering::Relaxed);
    drop(panic_logger);
    let saved = events(&path);
    assert!(
        saved
            .iter()
            .any(|(_, event)| matches!(event, RuntimeLifecycleEvent::Panic { .. }))
    );
    assert!(saved.iter().any(|(_, event)| *event
        == RuntimeLifecycleEvent::MissionStopped {
            reason: RuntimeStopReason::Panic
        }));
    assert!(matches!(
        saved.last().unwrap().1,
        RuntimeLifecycleEvent::ShutdownCompleted
    ));

    let path = dir.path().join("interleaved.copper");
    let shared = logger(&path);
    let build = || {
        App::builder()
            .with_logger::<memmap::MmapSectionStorage, UnifiedLoggerWrite>(shared.clone())
            .build()
            .unwrap()
            .start()
            .unwrap()
    };
    let mut first = build();
    let mut second = build();
    first.run_one_iteration().unwrap();
    second.run_one_iteration().unwrap();
    drop(first.stop().unwrap());
    second.run_one_iteration().unwrap();
    drop(second.stop().unwrap());
    drop(shared);
    let saved = events(&path);
    for run_id in [1, 2] {
        let events = saved
            .iter()
            .filter(|(context, _)| context.run_id == run_id)
            .collect::<Vec<_>>();
        assert!(matches!(
            events[0].1,
            RuntimeLifecycleEvent::Instantiated { .. }
        ));
        assert!(matches!(
            events.last().unwrap().1,
            RuntimeLifecycleEvent::ShutdownCompleted
        ));
    }
    FAILURE.store(0, Ordering::Relaxed);
}
