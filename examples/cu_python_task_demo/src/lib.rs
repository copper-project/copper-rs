use bincode::{Decode, Encode};
use cu_python_task::{PyTask, PyTaskMode};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::sync::{LazyLock, Mutex};

pub mod messages {
    use super::*;

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct LeftInput {
        pub value: i32,
    }

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct RightInput {
        pub label: String,
    }

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct SummaryOutput {
        pub doubled: i32,
        pub label: String,
        pub calls: u64,
    }

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct StateOutput {
        pub total: i32,
        pub last_label: String,
    }

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct ExampleState {
        pub calls: u64,
        pub total: i32,
        pub last_label: String,
    }
}

static SUMMARY_SNAPSHOTS: LazyLock<Mutex<Vec<messages::SummaryOutput>>> =
    LazyLock::new(|| Mutex::new(Vec::new()));
static STATE_SNAPSHOTS: LazyLock<Mutex<Vec<messages::StateOutput>>> =
    LazyLock::new(|| Mutex::new(Vec::new()));

pub fn reset_sinks() {
    SUMMARY_SNAPSHOTS.lock().unwrap().clear();
    STATE_SNAPSHOTS.lock().unwrap().clear();
}

pub fn summary_snapshots() -> Vec<messages::SummaryOutput> {
    SUMMARY_SNAPSHOTS.lock().unwrap().clone()
}

pub fn state_snapshots() -> Vec<messages::StateOutput> {
    STATE_SNAPSHOTS.lock().unwrap().clone()
}

pub mod tasks {
    use super::*;

    #[derive(Reflect)]
    pub struct LeftSource {
        next: i32,
    }

    impl Freezable for LeftSource {
        fn freeze<E: bincode::enc::Encoder>(
            &self,
            encoder: &mut E,
        ) -> Result<(), bincode::error::EncodeError> {
            Encode::encode(&self.next, encoder)
        }

        fn thaw<D: bincode::de::Decoder>(
            &mut self,
            decoder: &mut D,
        ) -> Result<(), bincode::error::DecodeError> {
            self.next = Decode::decode(decoder)?;
            Ok(())
        }
    }

    impl CuSrcTask for LeftSource {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(messages::LeftInput);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self { next: 1 })
        }

        fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            output.set_payload(messages::LeftInput { value: self.next });
            self.next += 1;
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct RightSource {
        next: usize,
    }

    impl Freezable for RightSource {
        fn freeze<E: bincode::enc::Encoder>(
            &self,
            encoder: &mut E,
        ) -> Result<(), bincode::error::EncodeError> {
            Encode::encode(&self.next, encoder)
        }

        fn thaw<D: bincode::de::Decoder>(
            &mut self,
            decoder: &mut D,
        ) -> Result<(), bincode::error::DecodeError> {
            self.next = Decode::decode(decoder)?;
            Ok(())
        }
    }

    impl CuSrcTask for RightSource {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(messages::RightInput);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self { next: 1 })
        }

        fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            output.set_payload(messages::RightInput {
                label: format!("tick-{}", self.next),
            });
            self.next += 1;
            Ok(())
        }
    }

    pub type ExamplePythonTask = PyTask<
        (messages::LeftInput, messages::RightInput),
        messages::ExampleState,
        (messages::SummaryOutput, messages::StateOutput),
    >;

    #[derive(Default, Reflect)]
    pub struct SummarySink;

    impl Freezable for SummarySink {}

    impl CuSinkTask for SummarySink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(messages::SummaryOutput);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(payload) = input.payload() {
                SUMMARY_SNAPSHOTS.lock().unwrap().push(payload.clone());
            }
            Ok(())
        }
    }

    #[derive(Default, Reflect)]
    pub struct StateSink;

    impl Freezable for StateSink {}

    impl CuSinkTask for StateSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(messages::StateOutput);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(payload) = input.payload() {
                STATE_SNAPSHOTS.lock().unwrap().push(payload.clone());
            }
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct PythonTaskDemoApp {}

pub fn script_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("python/task.py")
}

pub fn config_for_mode(mode: PyTaskMode) -> CuConfig {
    let mut config = CuConfig::deserialize_ron(include_str!("../copperconfig.ron"))
        .expect("static config should parse");
    let graph = config.get_graph_mut(None).expect("graph");
    let node_id = graph.get_node_id_by_name("py").expect("py task node");
    let node = graph.get_node_mut(node_id).expect("py task");
    node.set_param(
        "mode",
        match mode {
            PyTaskMode::Process => "process".to_string(),
            PyTaskMode::Embedded => "embedded".to_string(),
        },
    );
    node.set_param("script", script_path().display().to_string());
    config
}

pub fn run_demo(mode: PyTaskMode, iterations: usize) -> CuResult<()> {
    let temp_dir = tempfile::TempDir::new().map_err(|e| CuError::new_with_cause("temp dir", e))?;
    let log_path = temp_dir.path().join("python_task_demo.copper");
    let ctx = basic_copper_setup(&log_path, Some(32 * 1024 * 1024), false, None)?;
    let mut app = PythonTaskDemoApp::new(
        ctx.clock.clone(),
        ctx.unified_logger.clone(),
        Some(config_for_mode(mode)),
    )?;

    reset_sinks();
    app.start_all_tasks()?;
    for _ in 0..iterations {
        app.run_one_iteration()?;
    }
    app.stop_all_tasks()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::LazyLock;

    static TEST_MUTEX: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

    fn assert_expected_snapshots() {
        assert_eq!(
            summary_snapshots(),
            vec![
                messages::SummaryOutput {
                    doubled: 2,
                    label: "tick-1".to_string(),
                    calls: 1,
                },
                messages::SummaryOutput {
                    doubled: 4,
                    label: "tick-2".to_string(),
                    calls: 2,
                },
                messages::SummaryOutput {
                    doubled: 6,
                    label: "tick-3".to_string(),
                    calls: 3,
                },
            ]
        );
        assert_eq!(
            state_snapshots(),
            vec![
                messages::StateOutput {
                    total: 1,
                    last_label: "tick-1".to_string(),
                },
                messages::StateOutput {
                    total: 3,
                    last_label: "tick-2".to_string(),
                },
                messages::StateOutput {
                    total: 6,
                    last_label: "tick-3".to_string(),
                },
            ]
        );
    }

    #[test]
    fn process_mode_runs_python_task_end_to_end() {
        let _guard = TEST_MUTEX.lock().unwrap();
        run_demo(PyTaskMode::Process, 3).expect("process mode");
        assert_expected_snapshots();
    }

    #[cfg(not(target_os = "macos"))]
    #[test]
    fn embedded_mode_runs_python_task_end_to_end() {
        let _guard = TEST_MUTEX.lock().unwrap();
        run_demo(PyTaskMode::Embedded, 3).expect("embedded mode");
        assert_expected_snapshots();
    }

    #[cfg(not(target_os = "macos"))]
    #[test]
    fn both_modes_produce_the_same_outputs() {
        let _guard = TEST_MUTEX.lock().unwrap();

        run_demo(PyTaskMode::Process, 3).expect("process mode");
        let process_summary = summary_snapshots();
        let process_state = state_snapshots();

        run_demo(PyTaskMode::Embedded, 3).expect("embedded mode");
        assert_eq!(summary_snapshots(), process_summary);
        assert_eq!(state_snapshots(), process_state);
    }
}
