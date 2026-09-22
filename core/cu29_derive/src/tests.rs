use super::*;
use std::fs;
use std::path::{Path, PathBuf};

fn unique_test_dir(name: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock before unix epoch")
        .as_nanos();
    std::env::temp_dir().join(format!("cu29_derive_{name}_{nanos}"))
}

fn write_file(path: &Path, content: &str) {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).expect("create parent dirs");
    }
    fs::write(path, content).expect("write file");
}

#[test]
fn disabled_keyframe_capture_emits_no_freeze_calls() {
    let task = quote! { tasks.0 };
    assert!(keyframe_freeze_task_tokens(false, &task).is_empty());
    assert!(keyframe_freeze_bridge_tokens(false).is_empty());
    assert!(
        keyframe_freeze_task_tokens(true, &task)
            .to_string()
            .contains("freeze_task")
    );
    assert!(
        keyframe_freeze_bridge_tokens(true)
            .to_string()
            .contains("freeze_any")
    );
}

// See tests/compile_file directory for more information
#[test]
fn test_compile_fail() {
    use rustc_version::{Channel, version_meta};
    use std::{env, fs, path::Path};

    let log_index_dir = env::temp_dir()
        .join("cu29_derive_trybuild_log_index")
        .join("a")
        .join("b")
        .join("c");
    fs::create_dir_all(&log_index_dir).unwrap();
    unsafe {
        env::set_var("LOG_INDEX_DIR", &log_index_dir);
    }

    let dir = Path::new("tests/compile_fail");
    for entry in fs::read_dir(dir).unwrap() {
        let entry = entry.unwrap();
        if !entry.file_type().unwrap().is_dir() {
            continue;
        }
        for file in fs::read_dir(entry.path()).unwrap() {
            let file = file.unwrap();
            let p = file.path();
            if p.extension().and_then(|x| x.to_str()) != Some("rs") {
                continue;
            }

            let base = p.with_extension("stderr"); // the file trybuild reads
            let src = match version_meta().unwrap().channel {
                Channel::Beta => Path::new(&format!("{}.beta", base.display())).to_path_buf(),
                _ => Path::new(&format!("{}.stable", base.display())).to_path_buf(),
            };

            if src.exists() {
                fs::copy(src, &base).unwrap();
            }
        }
    }

    // One TestCases keeps fail+pass in the same cargo profile so workspace
    // deps compile once; the umbrella collapses pass tests into one bin.
    let umbrella = build_compile_pass_umbrella();
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/compile_fail/*/*.rs");
    t.pass(&umbrella);
}

fn build_compile_pass_umbrella() -> std::path::PathBuf {
    use std::fmt::Write as _;
    let pass_dir = std::path::Path::new("tests/compile_pass");
    let mut entries: Vec<std::path::PathBuf> = Vec::new();
    for sub in std::fs::read_dir(pass_dir).expect("read tests/compile_pass") {
        let sub = sub.expect("read tests/compile_pass entry");
        if !sub
            .file_type()
            .expect("stat tests/compile_pass entry")
            .is_dir()
        {
            continue;
        }
        for file in std::fs::read_dir(sub.path()).expect("read compile_pass subdir") {
            let p = file.expect("read compile_pass subdir entry").path();
            if p.extension().and_then(|x| x.to_str()) == Some("rs") {
                entries.push(p);
            }
        }
    }
    entries.sort();

    let mut src = String::from("#![allow(dead_code, unused_imports, non_snake_case)]\n");
    for p in &entries {
        let abs = std::fs::canonicalize(p)
            .unwrap_or_else(|e| panic!("canonicalize {}: {e}", p.display()));
        let subdir = abs
            .parent()
            .and_then(|d| d.file_name())
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let stem = abs
            .file_stem()
            .expect("compile_pass file has stem")
            .to_string_lossy()
            .into_owned();
        let mod_name = format!("{subdir}_{stem}").replace('-', "_");
        writeln!(
            src,
            "#[path = {:?}] mod compile_pass_{};",
            abs.to_string_lossy(),
            mod_name
        )
        .expect("write to String");
    }
    src.push_str("fn main() {}\n");

    // Keep the umbrella outside `tests/` so cargo doesn't pick it up as an
    // integration test.
    let target_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::PathBuf::from("../../target"));
    let umbrella_dir = target_dir.join("generated");
    std::fs::create_dir_all(&umbrella_dir)
        .unwrap_or_else(|e| panic!("create {}: {e}", umbrella_dir.display()));
    let umbrella = umbrella_dir.join("compile_pass_umbrella.rs");
    if std::fs::read_to_string(&umbrella).ok().as_deref() != Some(src.as_str()) {
        std::fs::write(&umbrella, &src)
            .unwrap_or_else(|e| panic!("write {}: {e}", umbrella.display()));
    }
    std::fs::canonicalize(&umbrella)
        .unwrap_or_else(|e| panic!("canonicalize {}: {e}", umbrella.display()))
}

#[test]
fn runtime_plan_keeps_nc_order_for_non_first_connected_output() {
    use super::*;
    use cu29::config::CuConfig;
    use cu29::curuntime::{CuExecutionUnit, compute_runtime_plan};

    let config: CuConfig =
        read_config("tests/config/multi_output_source_non_first_connected_valid.ron")
            .expect("failed to read test config");
    let graph = config.get_graph(None).expect("missing graph");
    let src_id = graph.get_node_id_by_name("src").expect("missing src node");

    let runtime = compute_runtime_plan(graph).expect("runtime plan failed");
    let src_step = runtime
        .steps
        .iter()
        .find_map(|step| match step {
            CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
            _ => None,
        })
        .expect("missing source step");

    assert_eq!(
        src_step.output_msg_pack.as_ref().unwrap().msg_types,
        vec!["i32", "bool"]
    );
}

#[test]
fn matching_task_ids_are_flattened_per_output_message() {
    use super::*;
    use cu29::config::CuConfig;

    let config: CuConfig =
        read_config("tests/config/multi_output_source_non_first_connected_valid.ron")
            .expect("failed to read test config");
    let graph = config.get_graph(None).expect("missing graph");
    let channel_usage = collect_bridge_channel_usage(graph);
    let mut bridge_specs = build_bridge_specs(&config, graph, &channel_usage);
    let (runtime_plan, exec_entities, plan_to_original) =
        build_execution_plan(&config, graph, "default", &mut bridge_specs)
            .expect("runtime plan failed");
    let output_packs = extract_output_packs(&runtime_plan);
    let task_names = collect_task_names(graph);
    let (_, node_output_positions) = collect_culist_metadata(
        &runtime_plan,
        &exec_entities,
        &mut bridge_specs,
        &plan_to_original,
    );

    // Rebuild per-slot origin ids like `gen_culist_support` does.
    let mut slot_origin_ids: Vec<Option<String>> = vec![None; output_packs.len()];
    for (node_id, task_id, _) in task_names {
        let output_position = node_output_positions
            .get(&node_id)
            .unwrap_or_else(|| panic!("Task {task_id} (node id: {node_id}) not found"));
        slot_origin_ids[*output_position] = Some(task_id);
    }

    let flattened_ids = flatten_slot_origin_ids(&output_packs, &slot_origin_ids);

    // src emits two messages (i32 + bool), both map to src.
    // sink contributes its own output slot (CuMsg<()>), mapped to sink.
    assert_eq!(
        flattened_ids,
        vec!["src".to_string(), "src".to_string(), "sink".to_string()]
    );
}

#[test]
fn bridge_resources_are_collected() {
    use super::*;
    use cu29::config::{CuGraph, Flavor, Node};
    use std::collections::HashMap;
    use syn::parse_str;

    let mut graph = CuGraph::default();
    let mut node = Node::new_with_flavor("radio", "bridge::Dummy", Flavor::Bridge);
    let mut res = HashMap::new();
    res.insert("serial".to_string(), "fc.serial0".to_string());
    node.set_resources(Some(res));
    graph.add_node(node).expect("bridge node");

    let task_specs = CuTaskSpecSet::from_graph(&graph).expect("task specs");
    let bridge_spec = BridgeSpec {
        id: "radio".to_string(),
        type_path: parse_str("bridge::Dummy").unwrap(),
        run_in_sim: true,
        config_index: 0,
        tuple_index: 0,
        monitor_index: None,
        rx_channels: Vec::new(),
        tx_channels: Vec::new(),
    };

    let mut config = cu29::config::CuConfig::default();
    config.resources.push(ResourceBundleConfig {
        resources: None,
        id: "fc".to_string(),
        provider: "board::Bundle".to_string(),
        config: None,
        missions: None,
    });
    let bundle_specs = build_bundle_specs(&config, "default").expect("bundle specs");
    let specs = collect_resource_specs(&graph, &task_specs, &[bridge_spec], &bundle_specs)
        .expect("collect specs");
    assert_eq!(specs.len(), 1);
    assert!(matches!(specs[0].owner, ResourceOwner::Bridge(0)));
    assert_eq!(specs[0].binding_name, "serial");
    assert_eq!(specs[0].bundle_index, 0);
    assert_eq!(specs[0].resource_name, "serial0");
}

#[test]
fn copper_runtime_args_parse_subsystem_mode() {
    use super::*;
    use quote::quote;

    let args = CopperRuntimeArgs::parse_tokens(quote!(
        config = "multi_copper.ron",
        subsystem = "ping",
        sim_mode,
        ignore_resources
    ))
    .expect("parse runtime args");

    assert_eq!(args.config_path, "multi_copper.ron");
    assert_eq!(args.subsystem_id.as_deref(), Some("ping"));
    assert!(args.sim_mode);
    assert!(args.ignore_resources);
}

#[test]
fn resolve_runtime_config_from_multi_config_selects_local_subsystem() {
    use super::*;

    let root = unique_test_dir("multi_runtime_resolve");
    let alpha_config = root.join("alpha.ron");
    let beta_base_config = root.join("beta_base.ron");
    let beta_config = root.join("beta.ron");
    let network_config = root.join("multi.ron");

    write_file(
        &alpha_config,
        r#"
(
    tasks: [
        (id: "src", type: "AlphaSource", run_in_sim: true),
        (id: "sink", type: "AlphaSink", run_in_sim: true),
    ],
    cnx: [
        (src: "src", dst: "sink", msg: "u32"),
    ],
)
"#,
    );
    write_file(
        &beta_base_config,
        r#"
(
    tasks: [
        (id: "src", type: "BetaSource", run_in_sim: true),
    ],
)
"#,
    );
    write_file(
        &beta_config,
        r#"
(
    includes: [
        (path: "beta_base.ron", params: {}),
    ],
    tasks: [
        (id: "sink", type: "BetaSink", run_in_sim: true),
    ],
    cnx: [
        (src: "src", dst: "sink", msg: "u64"),
    ],
)
"#,
    );
    write_file(
        &network_config,
        r#"
(
    subsystems: [
        (id: "beta", config: "beta.ron"),
        (id: "alpha", config: "alpha.ron"),
    ],
    interconnects: [],
)
"#,
    );

    let args = CopperRuntimeArgs {
        config_path: "multi.ron".to_string(),
        subsystem_id: Some("beta".to_string()),
        sim_mode: false,
        ignore_resources: false,
    };

    let resolved =
        resolve_runtime_config_with_root(&args, &root).expect("resolve multi runtime config");

    assert_eq!(resolved.subsystem_id.as_deref(), Some("beta"));
    assert_eq!(resolved.subsystem_code, 1);
    let graph = resolved
        .local_config
        .get_graph(None)
        .expect("resolved local config graph");
    assert!(graph.get_node_id_by_name("src").is_some());
    assert!(resolved.bundled_local_config_content.contains("BetaSource"));
    assert!(
        !resolved
            .bundled_local_config_content
            .contains("beta_base.ron")
    );

    let bundled = CuConfig::deserialize_ron(&resolved.bundled_local_config_content)
        .expect("bundled subsystem config must not need include path resolution");
    let bundled_graph = bundled.get_graph(None).expect("bundled graph");
    assert!(bundled_graph.get_node_id_by_name("src").is_some());
    assert!(bundled_graph.get_node_id_by_name("sink").is_some());
    assert_eq!(bundled_graph.edge_count(), 1);
}

#[test]
fn resolve_runtime_config_bundles_resolved_single_config() {
    use super::*;

    let root = unique_test_dir("single_runtime_resolve");
    let base_config = root.join("base.ron");
    let app_config = root.join("app.ron");

    write_file(
        &base_config,
        r#"
(
    tasks: [
        (id: "src", type: "IncludedSource", run_in_sim: true),
    ],
)
"#,
    );
    write_file(
        &app_config,
        r#"
(
    includes: [
        (path: "base.ron", params: {}),
    ],
    tasks: [
        (id: "sink", type: "LocalSink", run_in_sim: true),
    ],
    cnx: [
        (src: "src", dst: "sink", msg: "u32"),
    ],
)
"#,
    );

    let args = CopperRuntimeArgs {
        config_path: "app.ron".to_string(),
        subsystem_id: None,
        sim_mode: false,
        ignore_resources: false,
    };

    let resolved =
        resolve_runtime_config_with_root(&args, &root).expect("resolve single runtime config");

    assert!(
        resolved
            .bundled_local_config_content
            .contains("IncludedSource")
    );
    assert!(!resolved.bundled_local_config_content.contains("base.ron"));
    let bundled = CuConfig::deserialize_ron(&resolved.bundled_local_config_content)
        .expect("bundled config must not need include path resolution");
    let graph = bundled.get_graph(None).expect("bundled graph");
    assert!(graph.get_node_id_by_name("src").is_some());
    assert!(graph.get_node_id_by_name("sink").is_some());
    assert_eq!(graph.edge_count(), 1);
}

#[test]
fn resolve_runtime_config_uses_forwarded_features_for_codegen_and_reload() {
    use super::*;

    let root = unique_test_dir("feature_runtime_resolve");
    let camera_config = root.join("camera.ron");
    let app_config = root.join("app.ron");

    write_file(
        &camera_config,
        r#"
(
    tasks: [
        (id: "camera", type: "target_camera::Camera"),
        (id: "sink", type: "tasks::CameraSink"),
    ],
    cnx: [
        (src: "camera", dst: "sink", msg: "target_camera::Frame"),
    ],
)
"#,
    );
    write_file(
        &app_config,
        r#"
(
    tasks: [],
    cnx: [],
    includes: [
        (path: "camera.ron", when: Feature("camera")),
    ],
)
"#,
    );

    let args = CopperRuntimeArgs {
        config_path: "app.ron".to_string(),
        subsystem_id: None,
        sim_mode: false,
        ignore_resources: false,
    };

    let without_camera = resolve_runtime_config_with_root_and_features(&args, &root, &[]).unwrap();
    assert!(without_camera.active_features.is_empty());
    assert!(
        !without_camera
            .bundled_local_config_content
            .contains("target_camera")
    );

    let with_camera =
        resolve_runtime_config_with_root_and_features(&args, &root, &["camera"]).unwrap();
    assert_eq!(with_camera.active_features, ["camera"]);
    assert!(
        with_camera
            .bundled_local_config_content
            .contains("target_camera::Camera")
    );
    assert!(
        with_camera
            .bundled_local_config_content
            .contains("target_camera::Frame")
    );
    assert!(!with_camera.bundled_local_config_content.contains("Feature"));
}

#[test]
fn resolve_multi_runtime_config_uses_forwarded_features() {
    use super::*;

    let root = unique_test_dir("feature_multi_runtime_resolve");
    write_file(
        &root.join("camera.ron"),
        r#"
(
    tasks: [
        (id: "camera", type: "target_camera::Camera"),
        (id: "sink", type: "tasks::CameraSink"),
    ],
    cnx: [
        (src: "camera", dst: "sink", msg: "target_camera::Frame"),
    ],
)
"#,
    );
    write_file(
        &root.join("robot.ron"),
        r#"
(
    tasks: [],
    cnx: [],
    includes: [
        (path: "camera.ron", when: Feature("camera")),
    ],
)
"#,
    );
    write_file(
        &root.join("multi.ron"),
        r#"
(
    subsystems: [
        (id: "robot", config: "robot.ron"),
    ],
    interconnects: [],
)
"#,
    );

    let args = CopperRuntimeArgs {
        config_path: "multi.ron".to_string(),
        subsystem_id: Some("robot".to_string()),
        sim_mode: false,
        ignore_resources: false,
    };

    let without_camera = resolve_runtime_config_with_root_and_features(&args, &root, &[]).unwrap();
    assert_eq!(
        without_camera
            .local_config
            .get_graph(None)
            .unwrap()
            .node_count(),
        0
    );

    let with_camera =
        resolve_runtime_config_with_root_and_features(&args, &root, &["camera"]).unwrap();
    assert_eq!(
        with_camera
            .local_config
            .get_graph(None)
            .unwrap()
            .node_count(),
        2
    );
    assert!(
        with_camera
            .bundled_local_config_content
            .contains("target_camera::Frame")
    );
}

#[test]
fn resolve_runtime_config_preserves_mission_task_order_in_bundle() {
    use super::*;

    let root = unique_test_dir("mission_runtime_resolve_order");
    let base_config = root.join("base.ron");
    let app_config = root.join("app.ron");

    write_file(
        &base_config,
        r#"
(
    tasks: [
        (id: "c", type: "TaskC", missions: ["one", "two"]),
    ],
)
"#,
    );
    write_file(
        &app_config,
        r#"
(
    includes: [
        (path: "base.ron", params: {}),
    ],
    missions: [
        (id: "one"),
        (id: "two"),
    ],
    tasks: [
        (id: "a", type: "TaskA", missions: ["two"]),
        (id: "b", type: "TaskB", missions: ["one"]),
    ],
)
"#,
    );

    let args = CopperRuntimeArgs {
        config_path: "app.ron".to_string(),
        subsystem_id: None,
        sim_mode: false,
        ignore_resources: false,
    };
    let resolved = resolve_runtime_config_with_root(&args, &root)
        .expect("resolve mission config with includes");
    let bundled = CuConfig::deserialize_ron(&resolved.bundled_local_config_content)
        .expect("bundled mission config");

    let task_order = |config: &CuConfig, mission: &str| {
        config
            .get_graph(Some(mission))
            .expect("mission graph")
            .get_all_nodes()
            .into_iter()
            .filter(|(_, node)| node.get_flavor() == Flavor::Task)
            .map(|(_, node)| node.get_id())
            .collect::<Vec<_>>()
    };

    assert_eq!(task_order(&resolved.local_config, "one"), vec!["b", "c"]);
    assert_eq!(task_order(&resolved.local_config, "two"), vec!["a", "c"]);
    assert_eq!(
        task_order(&bundled, "one"),
        task_order(&resolved.local_config, "one")
    );
    assert_eq!(
        task_order(&bundled, "two"),
        task_order(&resolved.local_config, "two")
    );
    assert!(!resolved.bundled_local_config_content.contains("base.ron"));
}

#[test]
fn resolve_runtime_config_rejects_missing_subsystem() {
    use super::*;

    let root = unique_test_dir("multi_runtime_missing_subsystem");
    let alpha_config = root.join("alpha.ron");
    let network_config = root.join("multi.ron");

    write_file(
        &alpha_config,
        r#"
(
    tasks: [
        (id: "src", type: "AlphaSource", run_in_sim: true),
        (id: "sink", type: "AlphaSink", run_in_sim: true),
    ],
    cnx: [
        (src: "src", dst: "sink", msg: "u32"),
    ],
)
"#,
    );
    write_file(
        &network_config,
        r#"
(
    subsystems: [
        (id: "alpha", config: "alpha.ron"),
    ],
    interconnects: [],
)
"#,
    );

    let args = CopperRuntimeArgs {
        config_path: "multi.ron".to_string(),
        subsystem_id: Some("missing".to_string()),
        sim_mode: false,
        ignore_resources: false,
    };

    let err = resolve_runtime_config_with_root(&args, &root).expect_err("missing subsystem");
    assert!(err.to_string().contains("Subsystem 'missing'"));
}

#[test]
fn synthesized_single_output_type_name_parses_for_task_traits() {
    use super::*;

    let src_ty: Type = parse_quote!(SingleSource);
    let regular_ty: Type = parse_quote!(RegularTask);

    let src_name = synthesized_single_output_msg_name(&src_ty, CuTaskType::Source, false, false);
    let regular_name =
        synthesized_single_output_msg_name(&regular_ty, CuTaskType::Regular, false, false);
    let anytime_name =
        synthesized_single_output_msg_name(&regular_ty, CuTaskType::Regular, true, false);
    let stateless_name =
        synthesized_single_output_msg_name(&regular_ty, CuTaskType::Regular, false, true);

    parse_str::<Type>(src_name.as_str()).expect("source payload type should parse");
    parse_str::<Type>(regular_name.as_str()).expect("regular payload type should parse");
    parse_str::<Type>(anytime_name.as_str()).expect("anytime payload type should parse");
    parse_str::<Type>(stateless_name.as_str()).expect("stateless payload type should parse");
}
