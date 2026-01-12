use cu29::bincode::config::standard;
use cu29::bincode::decode_from_std_read;
use cu29::bincode::error::DecodeError;
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::collections::HashMap;
use tempfile::tempdir;

mod tasks {
    use cu29::prelude::*;

    pub struct TestSrc {
        counter: u32,
    }

    impl Freezable for TestSrc {}

    impl CuSrcTask for TestSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self { counter: 0 })
        }

        fn process(&mut self, _clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
            debug!("culistid src {}", self.counter);
            output.set_payload(self.counter);
            self.counter += 1;
            Ok(())
        }
    }

    pub struct TestSink;

    impl Freezable for TestSink {}

    impl CuSinkTask for TestSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(val) = input.payload() {
                debug!("culistid sink {}", val);
            }
            Ok(())
        }
    }
}

#[copper_runtime(config = "tests/culistid_config.ron")]
struct App {}

#[test]
fn structured_logs_capture_culistid() {
    let dir = tempdir().expect("Failed to create temp dir");
    let path = dir.path().join("culistid_test.copper");

    {
        let ctx = basic_copper_setup(&path, None, false, None).expect("Failed to setup logger.");
        let mut app = App::new(ctx.clock.clone(), ctx.unified_logger.clone(), None)
            .expect("Failed to create application.");
        app.start_all_tasks().expect("Failed to start tasks.");
        app.run_one_iteration().expect("Failed to run iteration.");
        app.run_one_iteration().expect("Failed to run iteration.");
        app.stop_all_tasks().expect("Failed to stop tasks.");
        ctx.logger_runtime.flush();
    }

    let UnifiedLogger::Read(logger) = UnifiedLoggerBuilder::new()
        .file_base_name(&path)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };

    let mut reader = UnifiedLoggerIOReader::new(logger, UnifiedLogType::StructuredLogLine);
    let mut culist_counts: HashMap<u32, usize> = HashMap::new();
    loop {
        let entry = decode_from_std_read::<CuLogEntry, _, _>(&mut reader, standard());
        match entry {
            Ok(entry) => {
                if entry.msg_index == 0 {
                    continue;
                }
                if entry.culistid != CULISTID_UNKNOWN {
                    *culist_counts.entry(entry.culistid).or_insert(0) += 1;
                }
            }
            Err(DecodeError::UnexpectedEnd { .. }) => break,
            Err(DecodeError::Io { inner, .. })
                if inner.kind() == std::io::ErrorKind::UnexpectedEof =>
            {
                break;
            }
            Err(err) => panic!("Failed to decode log entry: {err:?}"),
        }
    }

    assert!(culist_counts.get(&0).copied().unwrap_or(0) > 0);
    assert!(culist_counts.get(&1).copied().unwrap_or(0) > 0);
}
