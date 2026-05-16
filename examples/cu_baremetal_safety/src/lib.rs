#[cfg(feature = "safety-ids")]
pub use cu29::serde;

#[cfg(feature = "safety-ids")]
pub mod harness {
    use cu29::config::{CuConfig, Node, TaskKind, resolve_task_kind_for_id};
    use cu29::curuntime::{CuExecutionLoop, CuExecutionUnit, CuTaskType, compute_runtime_plan};
    use cu29::prelude::*;
    use std::path::PathBuf;
    use std::sync::Mutex;
    use tempfile::TempDir;

    static CASE_LOCK: Mutex<()> = Mutex::new(());

    mod events {
        use std::sync::{LazyLock, Mutex};

        static EVENT_LOG: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));

        fn lock_log() -> std::sync::MutexGuard<'static, Vec<String>> {
            EVENT_LOG
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner())
        }

        pub fn record(event: impl Into<String>) {
            lock_log().push(event.into());
        }

        pub fn reset() {
            lock_log().clear();
        }

        pub fn snapshot() -> Vec<String> {
            lock_log().clone()
        }
    }

    pub mod resource_support {
        use cu29::prelude::*;
        use std::sync::Arc;

        #[derive(Debug)]
        pub struct OwnedCounter {
            next: i32,
        }

        impl OwnedCounter {
            pub fn new(start: i32) -> Self {
                Self { next: start }
            }

            pub fn peek(&self) -> i32 {
                self.next
            }

            pub fn next(&mut self) -> i32 {
                let current = self.next;
                self.next += 1;
                current
            }
        }

        pub struct BoardBundle;

        bundle_resources!(BoardBundle: Counter, Tag);

        impl ResourceBundle for BoardBundle {
            fn build(
                bundle: BundleContext<Self>,
                config: Option<&ComponentConfig>,
                manager: &mut ResourceManager,
            ) -> CuResult<()> {
                let tag = config
                    .and_then(|cfg| cfg.get::<String>("tag").ok().flatten())
                    .unwrap_or_else(|| format!("{}-tag", bundle.bundle_id()));
                let start = config
                    .and_then(|cfg| cfg.get::<i32>("start").ok().flatten())
                    .unwrap_or(0);

                manager.add_owned(bundle.key(BoardBundleId::Counter), OwnedCounter::new(start))?;
                manager.add_shared(bundle.key(BoardBundleId::Tag), Arc::new(tag))?;
                Ok(())
            }
        }
    }

    pub mod resource_tasks {
        use super::events;
        use super::resource_support::OwnedCounter;
        use cu29::prelude::*;
        use cu29::resources;

        #[derive(Default, Reflect)]
        pub struct TickSource;

        impl Freezable for TickSource {}

        impl CuSrcTask for TickSource {
            type Resources<'r> = ();
            type Output<'m> = CuMsg<i32>;

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                Ok(Self)
            }

            fn process<'o>(
                &mut self,
                _ctx: &CuContext,
                output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                output.set_payload(1);
                Ok(())
            }
        }

        mod probe_resources {
            use super::*;

            resources!({
                counter => Owned<OwnedCounter>,
                tag => Shared<String>,
            });
        }

        type ResourceProbeResources<'r> = probe_resources::Resources<'r>;

        #[derive(Reflect)]
        #[reflect(from_reflect = false)]
        pub struct ResourceProbeTask {
            #[reflect(ignore)]
            counter: OwnedCounter,
            tag: String,
        }

        impl Freezable for ResourceProbeTask {}

        impl CuTask for ResourceProbeTask {
            type Resources<'r> = ResourceProbeResources<'r>;
            type Input<'m> = CuMsg<i32>;
            type Output<'m> = CuMsg<i32>;

            fn new(
                _config: Option<&ComponentConfig>,
                resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                let probe_resources::Resources { counter, tag } = resources;
                let initial = counter.0.peek();
                let tag_value = tag.0.to_string();
                events::record(format!("task:new:{tag_value}:{initial}"));
                Ok(Self {
                    counter: counter.0,
                    tag: tag_value,
                })
            }

            fn process<'i, 'o>(
                &mut self,
                _ctx: &CuContext,
                _input: &Self::Input<'i>,
                output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                let value = self.counter.next();
                events::record(format!("task:process:{}:{value}", self.tag));
                output.set_payload(value);
                Ok(())
            }
        }

        #[derive(Default, Reflect)]
        pub struct IntSink;

        impl Freezable for IntSink {}

        impl CuSinkTask for IntSink {
            type Resources<'r> = ();
            type Input<'m> = CuMsg<i32>;

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                Ok(Self)
            }

            fn process<'i>(&mut self, _ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
                let value = input.payload().copied().unwrap_or_default();
                events::record(format!("sink:process:{value}"));
                Ok(())
            }
        }
    }

    pub mod resource_bridges {
        use super::events;
        use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
        use cu29::prelude::*;
        use cu29::resources;

        tx_channels! {
            probe_tx => i32
        }

        rx_channels! {
            probe_rx => i32
        }

        mod bridge_resources {
            use super::*;

            resources!({
                tag => Shared<String>,
            });
        }

        type ResourceBridgeResources<'r> = bridge_resources::Resources<'r>;

        #[derive(Reflect)]
        pub struct ResourceBridge {
            tag: String,
        }

        impl Freezable for ResourceBridge {}

        impl CuBridge for ResourceBridge {
            type Tx = TxChannels;
            type Rx = RxChannels;
            type Resources<'r> = ResourceBridgeResources<'r>;

            fn new(
                _config: Option<&ComponentConfig>,
                _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
                _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
                resources: Self::Resources<'_>,
            ) -> CuResult<Self>
            where
                Self: Sized,
            {
                let bridge_resources::Resources { tag } = resources;
                let tag_value = tag.0.to_string();
                events::record(format!("bridge:new:{tag_value}"));
                Ok(Self { tag: tag_value })
            }

            fn send<'a, Payload>(
                &mut self,
                _ctx: &CuContext,
                channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
                _msg: &CuMsg<Payload>,
            ) -> CuResult<()>
            where
                Payload: CuMsgPayload + 'a,
            {
                if matches!(channel.id(), TxId::ProbeTx) {
                    events::record(format!("bridge:send:{}", self.tag));
                }
                Ok(())
            }

            fn receive<'a, Payload>(
                &mut self,
                _ctx: &CuContext,
                channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
                msg: &mut CuMsg<Payload>,
            ) -> CuResult<()>
            where
                Payload: CuMsgPayload + 'a,
            {
                if matches!(channel.id(), RxId::ProbeRx) {
                    let payload: &mut CuMsg<i32> = msg.downcast_mut()?;
                    payload.set_payload(900 + self.tag.len() as i32);
                    events::record(format!("bridge:receive:{}", self.tag));
                }
                Ok(())
            }
        }
    }

    pub mod lifecycle_tasks {
        use super::events;
        use cu29::prelude::*;

        #[derive(Default, Reflect)]
        pub struct LifecycleSource;

        impl Freezable for LifecycleSource {}

        impl CuSrcTask for LifecycleSource {
            type Resources<'r> = ();
            type Output<'m> = CuMsg<u32>;

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                events::record("new:source");
                Ok(Self)
            }

            fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("start:source");
                Ok(())
            }

            fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("pre:source");
                Ok(())
            }

            fn process<'o>(
                &mut self,
                _ctx: &CuContext,
                output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                events::record("process:source");
                output.set_payload(1);
                Ok(())
            }

            fn postprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("post:source");
                Ok(())
            }

            fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("stop:source");
                Ok(())
            }
        }

        #[derive(Default, Reflect)]
        pub struct LifecycleTask;

        impl Freezable for LifecycleTask {}

        impl CuTask for LifecycleTask {
            type Resources<'r> = ();
            type Input<'m> = CuMsg<u32>;
            type Output<'m> = CuMsg<u32>;

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                events::record("new:task");
                Ok(Self)
            }

            fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("start:task");
                Ok(())
            }

            fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("pre:task");
                Ok(())
            }

            fn process<'i, 'o>(
                &mut self,
                _ctx: &CuContext,
                input: &Self::Input<'i>,
                output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                events::record("process:task");
                let next = input.payload().copied().unwrap_or_default() + 1;
                output.set_payload(next);
                Ok(())
            }

            fn postprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("post:task");
                Ok(())
            }

            fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("stop:task");
                Ok(())
            }
        }

        #[derive(Default, Reflect)]
        pub struct LifecycleSink;

        impl Freezable for LifecycleSink {}

        impl CuSinkTask for LifecycleSink {
            type Resources<'r> = ();
            type Input<'m> = CuMsg<u32>;

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                events::record("new:sink");
                Ok(Self)
            }

            fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("start:sink");
                Ok(())
            }

            fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("pre:sink");
                Ok(())
            }

            fn process<'i>(&mut self, _ctx: &CuContext, _input: &Self::Input<'i>) -> CuResult<()> {
                events::record("process:sink");
                Ok(())
            }

            fn postprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("post:sink");
                Ok(())
            }

            fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
                events::record("stop:sink");
                Ok(())
            }
        }
    }

    mod resource_app {
        use super::*;
        use std::path::Path;

        #[copper_runtime(config = "copperconfig_resources.ron")]
        struct App {}

        pub(super) fn capture_build_and_run_events(
            logger_path: &Path,
        ) -> CuResult<(Vec<String>, Vec<String>)> {
            super::events::reset();

            let mut app = App::builder().with_log_path(logger_path, None)?.build()?;
            let build_events = super::events::snapshot();

            app.start_all_tasks()?;
            app.run_one_iteration()?;
            app.stop_all_tasks()?;

            let runtime_events = super::slice_from(&super::events::snapshot(), build_events.len());
            Ok((build_events, runtime_events))
        }
    }

    mod lifecycle_app {
        use super::*;
        use std::path::Path;

        #[copper_runtime(config = "copperconfig_lifecycle.ron")]
        struct App {}

        pub(super) fn capture_lifecycle_events(
            logger_path: &Path,
        ) -> CuResult<(Vec<String>, Vec<String>, Vec<String>, Vec<String>)> {
            super::events::reset();

            let mut app = App::builder().with_log_path(logger_path, None)?.build()?;
            let build_events = super::events::snapshot();

            app.start_all_tasks()?;
            let after_start = super::events::snapshot();

            app.run_one_iteration()?;
            let after_run = super::events::snapshot();

            app.stop_all_tasks()?;
            let after_stop = super::events::snapshot();

            Ok((
                build_events.clone(),
                super::slice_from(&after_start, build_events.len()),
                super::slice_from(&after_run, after_start.len()),
                super::slice_from(&after_stop, after_run.len()),
            ))
        }
    }

    fn case_guard() -> std::sync::MutexGuard<'static, ()> {
        CASE_LOCK
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    fn fresh_log_path(case: &str) -> (TempDir, PathBuf) {
        let dir = tempfile::tempdir().expect("failed to create tempdir");
        let path = dir.path().join(format!("{case}.copper"));
        (dir, path)
    }

    fn step_for(runtime: &CuExecutionLoop, node_id: u32) -> &cu29::curuntime::CuExecutionStep {
        runtime
            .steps
            .iter()
            .find_map(|unit| match unit {
                CuExecutionUnit::Step(step) if step.node_id == node_id => Some(step.as_ref()),
                _ => None,
            })
            .expect("runtime step missing")
    }

    fn phases_for_task<'a>(events: &'a [String], task: &str) -> Vec<&'a str> {
        events
            .iter()
            .filter_map(|event| {
                let (phase, name) = event.split_once(':')?;
                (name == task).then_some(phase)
            })
            .collect()
    }

    fn tasks_for_phase(events: &[String], phase: &str) -> Vec<String> {
        events
            .iter()
            .filter_map(|event| {
                let (event_phase, name) = event.split_once(':')?;
                (event_phase == phase).then_some(name.to_string())
            })
            .collect()
    }

    fn count_event(events: &[String], expected: &str) -> usize {
        events
            .iter()
            .filter(|event| event.as_str() == expected)
            .count()
    }

    fn slice_from(events: &[String], start: usize) -> Vec<String> {
        events[start..].to_vec()
    }

    #[cfg_attr(test, test)]
    #[safety_case("CGC-TEST-001")]
    fn compiler_task_kind_validation_and_inference() {
        let _guard = case_guard();

        let invalid_source = CuConfig::deserialize_ron(
            r#"(
                tasks: [(id: "upstream", type: "a"), (id: "bad", type: "b", kind: source)],
                cnx: [(src: "upstream", dst: "bad", msg: "msg::A")],
            )"#,
        )
        .unwrap();
        let invalid_source_graph = invalid_source.get_graph(None).unwrap();
        let invalid_source_id = invalid_source_graph.get_node_id_by_name("bad").unwrap();
        let invalid_source_err =
            resolve_task_kind_for_id(invalid_source_graph, invalid_source_id).unwrap_err();

        safety_check!(
            "CGC-TEST-001-C1",
            "CGC-REQ-001",
            invalid_source_err
                .to_string()
                .contains("declared as kind 'source' but has incoming connections"),
        );

        let invalid_regular = CuConfig::deserialize_ron(
            r#"(
                tasks: [(id: "bad", type: "b", kind: task)],
                cnx: [(src: "bad", dst: "__nc__", msg: "msg::A")],
            )"#,
        )
        .unwrap();
        let invalid_regular_graph = invalid_regular.get_graph(None).unwrap();
        let invalid_regular_id = invalid_regular_graph.get_node_id_by_name("bad").unwrap();
        let invalid_regular_err =
            resolve_task_kind_for_id(invalid_regular_graph, invalid_regular_id).unwrap_err();

        safety_check!(
            "CGC-TEST-001-C2",
            "CGC-REQ-001",
            invalid_regular_err
                .to_string()
                .contains("declared as kind 'task' but has no incoming connections"),
        );

        let invalid_sink = CuConfig::deserialize_ron(
            r#"(
                tasks: [(id: "src", type: "a"), (id: "bad", type: "b", kind: sink)],
                cnx: [
                    (src: "src", dst: "bad", msg: "msg::A"),
                    (src: "bad", dst: "__nc__", msg: "msg::B"),
                ],
            )"#,
        )
        .unwrap();
        let invalid_sink_graph = invalid_sink.get_graph(None).unwrap();
        let invalid_sink_id = invalid_sink_graph.get_node_id_by_name("bad").unwrap();
        let invalid_sink_err =
            resolve_task_kind_for_id(invalid_sink_graph, invalid_sink_id).unwrap_err();

        safety_check!(
            "CGC-TEST-001-C3",
            "CGC-REQ-001",
            invalid_sink_err
                .to_string()
                .contains("declared as kind 'sink' but has outgoing or NC outputs"),
        );

        let ambiguous = CuConfig::deserialize_ron(
            r#"(
                tasks: [(id: "bad", type: "b")],
                cnx: [],
            )"#,
        )
        .unwrap();
        let ambiguous_graph = ambiguous.get_graph(None).unwrap();
        let ambiguous_id = ambiguous_graph.get_node_id_by_name("bad").unwrap();
        let ambiguous_err = resolve_task_kind_for_id(ambiguous_graph, ambiguous_id).unwrap_err();

        safety_check!(
            "CGC-TEST-001-C4",
            "CGC-REQ-001",
            ambiguous_err
                .to_string()
                .contains("cannot infer whether it is a source, task, or sink"),
        );

        let inferred = CuConfig::deserialize_ron(
            r#"(
                tasks: [
                    (id: "src", type: "a"),
                    (id: "mid", type: "b"),
                    (id: "sink", type: "c"),
                ],
                cnx: [
                    (src: "src", dst: "mid", msg: "msg::A"),
                    (src: "mid", dst: "sink", msg: "msg::B"),
                ],
            )"#,
        )
        .unwrap();
        let inferred_graph = inferred.get_graph(None).unwrap();

        let inferred_roles = (
            resolve_task_kind_for_id(
                inferred_graph,
                inferred_graph.get_node_id_by_name("src").unwrap(),
            )
            .unwrap(),
            resolve_task_kind_for_id(
                inferred_graph,
                inferred_graph.get_node_id_by_name("mid").unwrap(),
            )
            .unwrap(),
            resolve_task_kind_for_id(
                inferred_graph,
                inferred_graph.get_node_id_by_name("sink").unwrap(),
            )
            .unwrap(),
        );

        safety_check_eq!(
            "CGC-TEST-001-C5",
            "CGC-REQ-001",
            inferred_roles,
            (TaskKind::Source, TaskKind::Regular, TaskKind::Sink),
        );
    }

    #[cfg_attr(test, test)]
    #[safety_case("CGC-TEST-002")]
    fn compiler_connection_order_is_preserved_in_runtime_lowering() {
        let _guard = case_guard();

        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src1_id = graph.add_node(Node::new("a", "Source1")).unwrap();
        let src2_id = graph.add_node(Node::new("b", "Source2")).unwrap();
        let sink_id = graph.add_node(Node::new("c", "Sink")).unwrap();
        graph.connect(src2_id, sink_id, "src2_type").unwrap();
        graph.connect(src1_id, sink_id, "src1_type").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let sink_step = step_for(&runtime, sink_id);
        let sink_inputs: Vec<String> = sink_step
            .input_msg_indices_types
            .iter()
            .map(|msg| msg.msg_type.clone())
            .collect();

        safety_check_eq!(
            "CGC-TEST-002-C1",
            "CGC-REQ-002",
            sink_inputs,
            vec!["src2_type".to_string(), "src1_type".to_string()],
        );

        let mut case1 = CuConfig::default();
        let graph = case1.get_graph_mut(None).unwrap();
        let cam0_id = graph
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask"))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask"))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new("broadcast", "tasks::MergingSinkTask"))
            .unwrap();
        graph.connect(cam0_id, broadcast_id, "i32").unwrap();
        graph.connect(cam0_id, inf0_id, "i32").unwrap();
        graph.connect(inf0_id, broadcast_id, "f32").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let broadcast_step = step_for(&runtime, broadcast_id);
        let case1_inputs: Vec<String> = broadcast_step
            .input_msg_indices_types
            .iter()
            .map(|msg| msg.msg_type.clone())
            .collect();

        safety_check_eq!(
            "CGC-TEST-002-C2",
            "CGC-REQ-002",
            case1_inputs,
            vec!["i32".to_string(), "f32".to_string()],
        );

        let mut case2 = CuConfig::default();
        let graph = case2.get_graph_mut(None).unwrap();
        let cam0_id = graph
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask"))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask"))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new("broadcast", "tasks::MergingSinkTask"))
            .unwrap();
        graph.connect(cam0_id, inf0_id, "i32").unwrap();
        graph.connect(cam0_id, broadcast_id, "i32").unwrap();
        graph.connect(inf0_id, broadcast_id, "f32").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let broadcast_step = step_for(&runtime, broadcast_id);
        let case2_inputs: Vec<String> = broadcast_step
            .input_msg_indices_types
            .iter()
            .map(|msg| msg.msg_type.clone())
            .collect();

        safety_check_eq!(
            "CGC-TEST-002-C3",
            "CGC-REQ-002",
            case2_inputs,
            vec!["i32".to_string(), "f32".to_string()],
        );
    }

    #[cfg_attr(test, test)]
    #[safety_case("CGC-TEST-003")]
    fn compiler_nc_outputs_and_fanout_lowering_are_stable() {
        let _guard = case_guard();

        let nc_config = CuConfig::deserialize_ron(
            r#"(
                tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
                cnx: [
                    (src: "src", dst: "sink", msg: "msg::A"),
                    (src: "src", dst: "__nc__", msg: "msg::B"),
                ],
            )"#,
        )
        .unwrap();
        let nc_graph = nc_config.get_graph(None).unwrap();
        let src_id = nc_graph.get_node_id_by_name("src").unwrap();
        let src_node = nc_graph.get_node(src_id).unwrap();

        safety_check!(
            "CGC-TEST-003-C1",
            "CGC-REQ-003",
            nc_graph.edge_count() == 1 && src_node.nc_outputs() == ["msg::B".to_string()],
        );

        let roundtrip = CuConfig::deserialize_ron(&nc_config.serialize_ron().unwrap()).unwrap();
        let roundtrip_graph = roundtrip.get_graph(None).unwrap();
        let roundtrip_src = roundtrip_graph
            .get_node(roundtrip_graph.get_node_id_by_name("src").unwrap())
            .unwrap();

        safety_check!(
            "CGC-TEST-003-C2",
            "CGC-REQ-003",
            roundtrip_graph.edge_count() == 1
                && roundtrip_src.nc_outputs() == ["msg::B".to_string()],
        );

        let mut fanout = CuConfig::default();
        let graph = fanout.get_graph_mut(None).unwrap();
        let src_id = graph.add_node(Node::new("src", "Source")).unwrap();
        let dst_a = graph.add_node(Node::new("dst_a", "SinkA")).unwrap();
        let dst_b = graph.add_node(Node::new("dst_b", "SinkB")).unwrap();
        graph.connect(src_id, dst_a, "i32").unwrap();
        graph.connect(src_id, dst_b, "i32").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let src_step = step_for(&runtime, src_id);

        safety_check_eq!(
            "CGC-TEST-003-C3",
            "CGC-REQ-003",
            src_step.output_msg_pack.as_ref().unwrap().msg_types.clone(),
            vec!["i32".to_string()],
        );

        let ordered = CuConfig::deserialize_ron(
            r#"(
                tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
                cnx: [
                    (src: "src", dst: "__nc__", msg: "msg::A"),
                    (src: "src", dst: "sink", msg: "msg::B"),
                ],
            )"#,
        )
        .unwrap();
        let ordered_graph = ordered.get_graph(None).unwrap();
        let ordered_src = ordered_graph.get_node_id_by_name("src").unwrap();
        let ordered_sink = ordered_graph.get_node_id_by_name("sink").unwrap();
        let runtime = compute_runtime_plan(ordered_graph).unwrap();
        let src_step = step_for(&runtime, ordered_src);
        let sink_step = step_for(&runtime, ordered_sink);

        safety_check_eq!(
            "CGC-TEST-003-C4",
            "CGC-REQ-003",
            (
                src_step.output_msg_pack.as_ref().unwrap().msg_types.clone(),
                sink_step.input_msg_indices_types[0].src_port,
            ),
            (vec!["msg::A".to_string(), "msg::B".to_string()], 1),
        );

        let inferred = CuConfig::deserialize_ron(
            r#"(
                tasks: [
                    (id: "src", type: "a"),
                    (id: "regular", type: "b"),
                ],
                cnx: [
                    (src: "src", dst: "regular", msg: "msg::A"),
                    (src: "regular", dst: "__nc__", msg: "msg::B"),
                ],
            )"#,
        )
        .unwrap();
        let inferred_graph = inferred.get_graph(None).unwrap();
        let regular_id = inferred_graph.get_node_id_by_name("regular").unwrap();
        let runtime = compute_runtime_plan(inferred_graph).unwrap();
        let regular_step = step_for(&runtime, regular_id);

        safety_check_eq!(
            "CGC-TEST-003-C5",
            "CGC-REQ-003",
            (
                regular_step.task_type,
                regular_step
                    .output_msg_pack
                    .as_ref()
                    .unwrap()
                    .msg_types
                    .clone(),
            ),
            (CuTaskType::Regular, vec!["msg::B".to_string()]),
        );
    }

    #[cfg_attr(test, test)]
    #[safety_case("CGC-TEST-004")]
    fn compiler_resource_bindings_roundtrip_and_lower_into_runtime() {
        let _guard = case_guard();

        let config =
            CuConfig::deserialize_ron(include_str!("../copperconfig_resources.ron")).unwrap();
        let graph = config.get_graph(None).unwrap();
        let probe_node = graph
            .get_node(graph.get_node_id_by_name("probe").unwrap())
            .expect("probe node missing");
        let tap_node = graph
            .get_node(graph.get_node_id_by_name("tap").unwrap())
            .expect("tap node missing");

        safety_check!(
            "CGC-TEST-004-C1",
            "CGC-REQ-004",
            probe_node
                .get_resources()
                .and_then(|resources| resources.get("counter"))
                .map(String::as_str)
                == Some("board.counter")
                && probe_node
                    .get_resources()
                    .and_then(|resources| resources.get("tag"))
                    .map(String::as_str)
                    == Some("board.tag"),
        );

        let roundtrip = CuConfig::deserialize_ron(&config.serialize_ron().unwrap()).unwrap();
        let roundtrip_graph = roundtrip.get_graph(None).unwrap();
        let roundtrip_tap = roundtrip_graph
            .get_node(roundtrip_graph.get_node_id_by_name("tap").unwrap())
            .expect("tap node missing after roundtrip");

        safety_check!(
            "CGC-TEST-004-C2",
            "CGC-REQ-004",
            tap_node
                .get_resources()
                .and_then(|resources| resources.get("tag"))
                .map(String::as_str)
                == Some("board.tag")
                && roundtrip_tap
                    .get_resources()
                    .and_then(|resources| resources.get("tag"))
                    .map(String::as_str)
                    == Some("board.tag"),
        );

        let (_dir, logger_path) = fresh_log_path("resource_bindings");
        let (build_events, runtime_events) =
            resource_app::capture_build_and_run_events(&logger_path)
                .expect("resource harness scenario failed");

        safety_check!(
            "CGC-TEST-004-C3",
            "CGC-REQ-004",
            build_events.len() == 2
                && build_events
                    .iter()
                    .any(|event| event == "task:new:alpha-board:7")
                && build_events
                    .iter()
                    .any(|event| event == "bridge:new:alpha-board"),
        );

        safety_check!(
            "CGC-TEST-004-C4",
            "CGC-REQ-004",
            runtime_events.len() == 4
                && runtime_events
                    .iter()
                    .any(|event| event == "task:process:alpha-board:7")
                && runtime_events
                    .iter()
                    .any(|event| event == "bridge:send:alpha-board")
                && runtime_events
                    .iter()
                    .any(|event| event == "bridge:receive:alpha-board")
                && runtime_events
                    .iter()
                    .any(|event| event == "sink:process:911"),
        );
    }

    #[cfg_attr(test, test)]
    #[safety_case("TLC-TEST-001")]
    fn baremetal_foreground_task_lifecycle_is_stable() {
        let _guard = case_guard();

        let (_dir, logger_path) = fresh_log_path("lifecycle");
        let (build_events, start_events, run_events, stop_events) =
            lifecycle_app::capture_lifecycle_events(&logger_path)
                .expect("lifecycle harness scenario failed");

        safety_check!(
            "TLC-TEST-001-C1",
            "TLC-REQ-001",
            count_event(&build_events, "new:source") == 1
                && count_event(&build_events, "new:task") == 1
                && count_event(&build_events, "new:sink") == 1
                && count_event(&start_events, "start:source") == 1
                && count_event(&start_events, "start:task") == 1
                && count_event(&start_events, "start:sink") == 1
                && tasks_for_phase(&start_events, "pre").is_empty()
                && tasks_for_phase(&start_events, "process").is_empty()
                && tasks_for_phase(&start_events, "post").is_empty(),
        );

        safety_check_eq!(
            "TLC-TEST-001-C2",
            "TLC-REQ-002",
            tasks_for_phase(&run_events, "process"),
            vec!["source".to_string(), "task".to_string(), "sink".to_string(),],
        );

        safety_check_eq!(
            "TLC-TEST-001-C3",
            "TLC-REQ-003",
            (
                phases_for_task(&run_events, "source"),
                phases_for_task(&run_events, "task"),
                phases_for_task(&run_events, "sink"),
            ),
            (
                vec!["pre", "process", "post"],
                vec!["pre", "process", "post"],
                vec!["pre", "process", "post"],
            ),
        );

        safety_check!(
            "TLC-TEST-001-C4",
            "TLC-REQ-004",
            count_event(&stop_events, "stop:source") == 1
                && count_event(&stop_events, "stop:task") == 1
                && count_event(&stop_events, "stop:sink") == 1
                && tasks_for_phase(&stop_events, "pre").is_empty()
                && tasks_for_phase(&stop_events, "process").is_empty()
                && tasks_for_phase(&stop_events, "post").is_empty(),
        );

        safety_check_eq!(
            "TLC-TEST-001-C5",
            "TLC-REQ-005",
            tasks_for_phase(&run_events, "process"),
            vec!["source".to_string(), "task".to_string(), "sink".to_string(),],
        );
    }

    pub fn link_safety_ids() {
        let _ = compiler_task_kind_validation_and_inference as fn();
        let _ = compiler_connection_order_is_preserved_in_runtime_lowering as fn();
        let _ = compiler_nc_outputs_and_fanout_lowering_are_stable as fn();
        let _ = compiler_resource_bindings_roundtrip_and_lower_into_runtime as fn();
        let _ = baremetal_foreground_task_lifecycle_is_stable as fn();
    }
}

#[cfg(feature = "safety-ids")]
pub use harness::link_safety_ids;
