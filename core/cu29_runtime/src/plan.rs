use clap::Parser;
use cu29_runtime::config::{
    CuConfig, CuGraph, OnError, RT_POOL, SchedulingPolicy, ThreadPoolConfig,
    read_configuration_with_features, read_multi_configuration_with_features,
};
use cu29_runtime::curuntime::{CuExecutionStep, CuExecutionUnit, CuStepPhase, CuTaskType};
use cu29_runtime::planner::{
    AssembledPlan, DEFAULT_COPPERLIST_COUNT, PlanEntity, PlanEntityKind, assemble_runtime_plan,
    assemble_runtime_plan_resolved, mission_graphs, step_key,
};
use cu29_traits::{CuError, CuResult};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const LABEL_WIDTH: f64 = 108.0;
const MARGIN: f64 = 28.0;
const SECTION_GAP: f64 = 44.0;
const SOURCE_COLOR: &str = "#ddefc7";
const TASK_COLOR: &str = "#fde7c2";
const SINK_COLOR: &str = "#cce0ff";
const BRIDGE_COLOR: &str = "#f7d7e4";
const BACKGROUND_GATEWAY_COLOR: &str = "#eeeafe";
const WAVE_COLUMNS: usize = 8;
const WAVE_CELL_WIDTH: f64 = 124.0;
const WAVE_CELL_HEIGHT: f64 = 70.0;
const WAVE_CELL_GAP: f64 = 12.0;
const MAX_VISIBLE_COPPERLISTS: usize = 6;
const PARALLEL_WIDTH: f64 =
    LABEL_WIDTH + WAVE_COLUMNS as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP) - WAVE_CELL_GAP;

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "Render Copper's generated per-CopperList process schedule"
)]
struct Args {
    /// Copper RON configuration file.
    config: PathBuf,
    /// Render only this mission; omit to stack every mission.
    #[arg(long)]
    mission: Option<String>,
    /// Comma-separated Cargo features used by conditional config fragments.
    #[arg(long, value_delimiter = ',')]
    features: Vec<String>,
    /// List mission ids and exit.
    #[arg(long)]
    list_missions: bool,
    /// Open the generated SVG in the default viewer.
    #[arg(long)]
    open: bool,
    /// Output SVG path.
    #[arg(long, default_value = "plan.svg")]
    output: PathBuf,
    /// Log-derived statistics produced by an application logreader.
    #[arg(long)]
    logstats: Option<PathBuf>,
}

fn main() {
    if let Err(error) = run(Args::parse()) {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

fn run(args: Args) -> CuResult<()> {
    let feature_refs = args.features.iter().map(String::as_str).collect::<Vec<_>>();
    let config = load_single_config(&args.config, &feature_refs)?;
    if args.list_missions {
        for (mission, _) in mission_graphs(&config) {
            println!("{mission}");
        }
        return Ok(());
    }

    let logstats = args
        .logstats
        .as_deref()
        .map(load_observed_logstats)
        .transpose()?;
    let selected_mission = args
        .mission
        .as_deref()
        .or_else(|| logstats.as_ref().and_then(|stats| stats.mission.as_deref()));
    let sections = selected_graphs(&config, selected_mission)?;
    if let Some(logstats) = &logstats {
        validate_observed_logstats(logstats, &config, selected_mission);
    }
    let svg = render_document(&config, &sections, logstats.as_ref())?;
    fs::write(&args.output, svg).map_err(|error| {
        CuError::new_with_cause(
            &format!("Failed to write plan SVG '{}'.", args.output.display()),
            error,
        )
    })?;
    if args.open {
        open_svg(&args.output).map_err(|error| {
            CuError::new_with_cause(
                &format!("Failed to open plan SVG '{}'.", args.output.display()),
                error,
            )
        })?;
    }
    Ok(())
}

fn load_single_config(path: &Path, features: &[&str]) -> CuResult<CuConfig> {
    let filename = path.to_str().ok_or_else(|| {
        CuError::from(format!(
            "Config path '{}' is not valid UTF-8.",
            path.display()
        ))
    })?;
    if read_multi_configuration_with_features(filename, features).is_ok() {
        return Err(CuError::from(format!(
            "Multi-Copper scheduling plans are not supported yet: '{}'. Render each subsystem's copperconfig.ron separately.",
            path.display()
        )));
    }
    read_configuration_with_features(filename, features).map_err(|error| {
        CuError::from(format!(
            "Failed to read Copper config '{}': {error}",
            path.display()
        ))
    })
}

fn selected_graphs<'a>(
    config: &'a CuConfig,
    requested: Option<&str>,
) -> CuResult<Vec<(String, &'a CuGraph)>> {
    let all = mission_graphs(config);
    let Some(requested) = requested else {
        return Ok(all);
    };
    all.into_iter()
        .find(|(mission, _)| mission == requested)
        .map(|section| vec![section])
        .ok_or_else(|| {
            let available = mission_graphs(config)
                .into_iter()
                .map(|(mission, _)| mission)
                .collect::<Vec<_>>()
                .join(", ");
            CuError::from(format!(
                "Mission '{requested}' not found. Available missions: {available}"
            ))
        })
}

fn render_document(
    config: &CuConfig,
    sections: &[(String, &CuGraph)],
    logstats: Option<&ObservedLogStats>,
) -> CuResult<String> {
    let mut rendered = Vec::new();
    let mut total_height = MARGIN;
    for (mission, graph) in sections {
        let plan = match config.planner_resolved(mission) {
            Some(step_keys) => assemble_runtime_plan_resolved(config, graph, step_keys),
            None => assemble_runtime_plan(config, graph),
        }
        .map_err(|error| {
            CuError::from(format!(
                "Could not compute scheduling plan for mission '{mission}': {error}"
            ))
        })?;
        let observed = logstats
            .filter(|stats| mission_key(stats.mission.as_deref()) == mission_key(Some(mission)))
            .and_then(|stats| stats.schedule.as_ref());
        let section = render_mission(config, mission, &plan, observed)?;
        total_height += section.height + SECTION_GAP;
        rendered.push(section);
    }
    let width = MARGIN * 2.0 + PARALLEL_WIDTH;
    let mut svg = String::new();
    writeln!(
        svg,
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{total_height}" viewBox="0 0 {width} {total_height}">"#
    )
    .unwrap();
    svg.push_str(
        r##"<defs>
<marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
  <path d="M 0 0 L 10 5 L 0 10 z" fill="#667085"/>
</marker>
</defs>
<style>
text { font-family: 'Noto Sans', sans-serif; fill: #18212b; }
.mono { font-family: 'Noto Sans Mono', monospace; }
.title { font-size: 21px; font-weight: 700; }
.subtitle { font-size: 14px; font-weight: 700; }
.meta { font-size: 11px; fill: #4b5563; }
.ordinal { font-size: 10px; font-weight: 700; fill: #667085; }
.card-title { font-size: 12px; font-weight: 700; }
.card-line { font-size: 9px; }
.lane { font-size: 10px; font-weight: 700; fill: #475467; }
.grid { stroke: #d7dce2; stroke-width: 1; stroke-dasharray: 3 3; }
.flow { fill: none; stroke: #667085; stroke-width: 2; marker-end: url(#arrow); }
.queue { fill: #f2f4f7; stroke: #98a2b3; stroke-width: 1; }
.system { fill: #f8fafc; stroke: #667085; stroke-width: 1.5; }
.blocking { font-size: 10px; fill: #344054; }
.gateway { fill: #eeeafe; stroke: #7f56d9; stroke-width: 2; stroke-dasharray: 4 2; }
.pool { fill: #f8fafc; stroke: #667085; stroke-width: 1.5; }
.job { fill: #eef4ff; stroke: #528bcd; stroke-width: 1; }
.job-flow { fill: none; stroke: #528bcd; stroke-width: 3; stroke-dasharray: 7 4; marker-end: url(#arrow); }
.bg-trigger { fill: none; stroke: #7f56d9; stroke-width: 2; stroke-dasharray: 4 3; opacity: 0.55; marker-end: url(#arrow); }
.observed-segment { cursor: help; }
</style>
<rect width="100%" height="100%" fill="#ffffff"/>
"##,
    );
    let mut y = MARGIN;
    for section in rendered {
        writeln!(svg, r#"<g transform="translate({MARGIN},{y})">"#).unwrap();
        svg.push_str(&section.svg);
        svg.push_str("</g>\n");
        y += section.height + SECTION_GAP;
    }
    svg.push_str("</svg>\n");
    Ok(svg)
}

struct RenderedSection {
    svg: String,
    height: f64,
}

#[derive(Debug, Deserialize)]
struct ObservedLogStats {
    schema_version: u32,
    config_signature: String,
    mission: Option<String>,
    #[serde(default)]
    schedule: Option<ObservedSchedule>,
}

#[derive(Debug, Deserialize)]
struct ObservedSchedule {
    #[serde(default)]
    stages: Vec<ObservedStage>,
    #[serde(default)]
    traces: Vec<ObservedTrace>,
    #[serde(default)]
    residual_before: ObservedDurationStats,
    #[serde(default)]
    resource_overlaps: Vec<ObservedResourceOverlap>,
}

#[derive(Debug, Deserialize)]
struct ObservedStage {
    origin: String,
    samples: u64,
    durations: ObservedDurationStats,
}

#[derive(Debug, Default, Deserialize)]
struct ObservedDurationStats {
    p50_ns: Option<u64>,
    p95_ns: Option<u64>,
    max_ns: Option<u64>,
}

#[derive(Debug, Deserialize)]
struct ObservedTrace {
    kind: String,
    culist_id: u64,
    #[serde(default)]
    wall_span_ns: u64,
    #[serde(default)]
    excluded_intervals: u32,
    residual_before_ns: Option<u64>,
    intervals: Vec<ObservedInterval>,
}

#[derive(Debug, Deserialize)]
struct ObservedInterval {
    origin: String,
    start_ns: u64,
    end_ns: u64,
}

#[derive(Debug, Deserialize)]
struct ObservedResourceOverlap {
    resource: String,
    left: String,
    right: String,
    occurrences: u64,
}

fn load_observed_logstats(path: &Path) -> CuResult<ObservedLogStats> {
    let contents = fs::read_to_string(path).map_err(|error| {
        CuError::new_with_cause(
            &format!("Failed to read logstats '{}'.", path.display()),
            error,
        )
    })?;
    let stats: ObservedLogStats = serde_json::from_str(&contents).map_err(|error| {
        CuError::new_with_cause(
            &format!("Failed to parse logstats '{}'.", path.display()),
            error,
        )
    })?;
    if stats.schema_version < 2 {
        eprintln!(
            "Warning: logstats schema {} has no observed schedule data; regenerate it with the current logreader.",
            stats.schema_version
        );
    } else if stats.schema_version != 2 {
        eprintln!(
            "Warning: logstats schema {} is newer than the supported schema 2.",
            stats.schema_version
        );
    }
    Ok(stats)
}

fn validate_observed_logstats(
    stats: &ObservedLogStats,
    config: &CuConfig,
    requested_mission: Option<&str>,
) {
    if requested_mission.is_some()
        && mission_key(requested_mission) != mission_key(stats.mission.as_deref())
    {
        eprintln!(
            "Warning: logstats mission '{}' does not match requested mission '{}'.",
            stats.mission.as_deref().unwrap_or("default"),
            requested_mission.unwrap_or("default")
        );
    }
    match build_logstats_signature(config, stats.mission.as_deref()) {
        Ok(signature) if signature != stats.config_signature => eprintln!(
            "Warning: logstats signature mismatch (expected {}, got {}).",
            signature, stats.config_signature
        ),
        Err(error) => eprintln!("Warning: unable to validate logstats signature: {error}"),
        _ => {}
    }
}

fn build_logstats_signature(config: &CuConfig, mission: Option<&str>) -> CuResult<String> {
    let graph = config.get_graph(mission)?;
    let mut parts = vec![format!("mission={}", mission.unwrap_or("default"))];
    let mut nodes = graph.get_all_nodes();
    nodes.sort_by_key(|(_, node)| node.get_id());
    for (_, node) in nodes {
        parts.push(format!(
            "node|{}|{}|{}",
            node.get_id(),
            node.get_type(),
            match node.get_flavor() {
                cu29_runtime::config::Flavor::Task => "task",
                cu29_runtime::config::Flavor::Bridge => "bridge",
            }
        ));
    }
    let mut edges = graph
        .edges()
        .map(|connection| {
            format!(
                "edge|{}|{}|{}",
                format_endpoint_for_signature(&connection.src, connection.src_channel.as_deref()),
                format_endpoint_for_signature(&connection.dst, connection.dst_channel.as_deref()),
                connection.msg
            )
        })
        .collect::<Vec<_>>();
    edges.sort();
    parts.extend(edges);
    Ok(format!(
        "fnv1a64:{:016x}",
        fnv1a64(parts.join("\n").as_bytes())
    ))
}

fn format_endpoint_for_signature(node: &str, channel: Option<&str>) -> String {
    channel.map_or_else(|| node.to_string(), |channel| format!("{node}/{channel}"))
}

fn fnv1a64(data: &[u8]) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in data {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

fn mission_key(mission: Option<&str>) -> &str {
    match mission {
        Some(value) if value != "default" => value,
        _ => "default",
    }
}

fn render_mission(
    config: &CuConfig,
    mission: &str,
    plan: &AssembledPlan,
    observed: Option<&ObservedSchedule>,
) -> CuResult<RenderedSection> {
    let steps = plan
        .execution
        .steps
        .iter()
        .map(|unit| match unit {
            CuExecutionUnit::Step(step) => Ok(step.as_ref()),
            CuExecutionUnit::Loop(_) => Err(CuError::from(
                "Nested execution loops are not supported by the plan visualizer.",
            )),
        })
        .collect::<CuResult<Vec<_>>>()?;
    let stages = steps
        .iter()
        .copied()
        .filter(|step| step.phase != CuStepPhase::AnytimeRefine)
        .collect::<Vec<_>>();
    let message_slots = steps
        .iter()
        .filter_map(|step| step.output_msg_pack.as_ref())
        .map(|pack| pack.culist_index as usize + 1)
        .max()
        .unwrap_or(0);
    let in_flight_limit = config
        .logging
        .as_ref()
        .and_then(|logging| logging.copperlist_count)
        .unwrap_or(DEFAULT_COPPERLIST_COUNT);
    let refine_totals = refine_totals(&steps);

    let runtime = config.runtime.as_ref();
    let rate = runtime
        .and_then(|runtime| runtime.rate_target_hz)
        .map(|rate| format!("{rate} Hz"))
        .unwrap_or_else(|| "best effort".to_string());
    let pools = runtime
        .map(|runtime| runtime.thread_pools.as_slice())
        .unwrap_or_default();

    let mut svg = String::new();
    writeln!(
        svg,
        r#"<text class="title" x="0" y="22">Mission: {}</text>"#,
        xml(mission)
    )
    .unwrap();
    writeln!(
        svg,
        r#"<text class="meta" x="0" y="43">{} serial steps · {} parallel stages · {} message slots per CopperList · {} CopperLists max in flight · rate target: {}</text>"#,
        steps.len(),
        stages.len(),
        message_slots,
        in_flight_limit,
        xml(&rate)
    )
    .unwrap();

    let mut y = 61.0;
    if pools.is_empty() {
        svg.push_str(r#"<text class="meta" x="0" y="61">Pools: none configured</text>"#);
        y += 18.0;
    } else {
        for pool in pools {
            writeln!(
                svg,
                r#"<text class="meta mono" x="0" y="{y}">pool {}</text>"#,
                xml(&format_pool(pool))
            )
            .unwrap();
            y += 16.0;
        }
    }

    y += 14.0;
    writeln!(
        svg,
        r#"<text class="subtitle" x="0" y="{y}">Serial projection · main CopperList executor · 1 worker</text>"#
    )
    .unwrap();
    y += 17.0;
    let serial = render_serial(config, mission, &steps, &plan.entities, &refine_totals, y);
    svg.push_str(&serial.svg);
    y += serial.height + 26.0;
    writeln!(
        svg,
        r#"<text class="subtitle" x="0" y="{y}">Parallel projection · generated stage workers + background pools</text>"#
    )
    .unwrap();
    y += 17.0;
    let parallel = render_parallel(
        config,
        &stages,
        &plan.entities,
        &refine_totals,
        in_flight_limit,
        y,
    );
    svg.push_str(&parallel.svg);
    y += parallel.height;

    if let Some(observed) = observed {
        y += 28.0;
        let section = render_observed(config, &stages, &plan.entities, observed, y);
        svg.push_str(&section.svg);
        y += section.height;
    }

    Ok(RenderedSection {
        svg,
        height: y + 10.0,
    })
}

fn refine_totals(steps: &[&CuExecutionStep]) -> HashMap<u32, u32> {
    let mut totals = HashMap::new();
    for step in steps {
        if step.phase == CuStepPhase::AnytimeRefine {
            *totals.entry(step.node_id).or_insert(0) += 1;
        }
    }
    totals
}

fn render_serial(
    config: &CuConfig,
    mission: &str,
    steps: &[&CuExecutionStep],
    entities: &[PlanEntity],
    refine_totals: &HashMap<u32, u32>,
    top: f64,
) -> RenderedSection {
    let mut svg = String::new();
    let background_pools = background_pool_views(config, steps);
    let cycles = if background_pools.is_empty() { 1 } else { 2 };
    let column_count = (steps.len() * cycles).max(1);
    let mut y = top;

    for block_start in (0..column_count).step_by(WAVE_COLUMNS) {
        let block_end = (block_start + WAVE_COLUMNS).min(column_count);
        let row_y = y + 19.0;
        writeln!(
            svg,
            r#"<text class="lane mono" x="0" y="{}">MAIN w1</text>"#,
            row_y + WAVE_CELL_HEIGHT / 2.0 + 4.0
        )
        .unwrap();

        for column in block_start..block_end {
            let local_column = column - block_start;
            let x = LABEL_WIDTH + local_column as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
            let cycle = column / steps.len().max(1);
            let step_index = column % steps.len().max(1);
            let Some(step) = steps.get(step_index).copied() else {
                continue;
            };
            let entity = &entities[step.node_id as usize];
            let is_background = step.node.is_background();
            let (_, entity_fill) = entity_style(entity, step);
            let fill = if is_background {
                BACKGROUND_GATEWAY_COLOR
            } else {
                entity_fill
            };
            let cl_label = if cycle == 0 {
                "CL n".to_string()
            } else {
                format!("CL n+{cycle}")
            };
            let refine_ordinal = if step.phase == CuStepPhase::AnytimeRefine {
                Some(
                    steps[..=step_index]
                        .iter()
                        .filter(|candidate| {
                            candidate.node_id == step.node_id
                                && candidate.phase == CuStepPhase::AnytimeRefine
                        })
                        .count() as u32,
                )
            } else {
                None
            };
            let phase = match step.phase {
                CuStepPhase::Whole if is_background => "POLL / DISPATCH?".to_string(),
                CuStepPhase::Whole => "whole".to_string(),
                CuStepPhase::AnytimeBase => "base".to_string(),
                CuStepPhase::AnytimeRefine => format!(
                    "refine {}/{}",
                    refine_ordinal.unwrap_or_default(),
                    refine_totals.get(&step.node_id).copied().unwrap_or(0)
                ),
            };
            let resources = entity_resources(config, entity, step);
            let footer = if is_background {
                format!("→ pool {}", step.node.background_pool())
            } else if resources.is_empty() {
                "res —".to_string()
            } else {
                format!("res {}", resources.join(", "))
            };
            let key = step_key(mission, entity, step.phase, refine_ordinal);
            let tooltip = format!(
                "{} · main executor worker 1\nStep {}: {}\nphase: {}\nresources: {}\nstable key: {}",
                cl_label,
                step_index + 1,
                entity.label,
                phase,
                if resources.is_empty() {
                    "—".to_string()
                } else {
                    resources.join(", ")
                },
                key
            );
            writeln!(
                svg,
                r##"<text class="ordinal mono" x="{x}" y="{}">{} · S{:02}</text><g data-step-key="{}" data-serial-column="{}"{}><title>{}</title><rect x="{x}" y="{row_y}" width="{WAVE_CELL_WIDTH}" height="{WAVE_CELL_HEIGHT}" rx="6" fill="{fill}" stroke="{}"{}/><text class="ordinal mono" x="{}" y="{}">MAIN w1 · {}</text><text class="card-title" x="{}" y="{}">{}</text><text class="card-line" x="{}" y="{}">{}</text><text class="card-line mono" x="{}" y="{}">{}</text></g>"##,
                row_y - 7.0,
                xml(&cl_label),
                step_index + 1,
                xml_attr(&key),
                column + 1,
                if is_background { r#" data-background-gateway="true""# } else { "" },
                xml(&tooltip),
                if is_background { "#7f56d9" } else { "#98a2b3" },
                if is_background { r#" stroke-width="2" stroke-dasharray="4 2""# } else { "" },
                x + 7.0,
                row_y + 13.0,
                if is_background { "GATEWAY" } else { "SERIAL" },
                x + 7.0,
                row_y + 29.0,
                xml(&truncate(&entity.label, 16)),
                x + 7.0,
                row_y + 45.0,
                xml(&truncate(&phase, 18)),
                x + 7.0,
                row_y + 62.0,
                xml(&truncate(&footer, 18)),
            )
            .unwrap();

            if column + 1 < block_end {
                writeln!(
                    svg,
                    r#"<line class="flow" x1="{}" y1="{}" x2="{}" y2="{}"/>"#,
                    x + WAVE_CELL_WIDTH,
                    row_y + WAVE_CELL_HEIGHT / 2.0,
                    x + WAVE_CELL_WIDTH + WAVE_CELL_GAP,
                    row_y + WAVE_CELL_HEIGHT / 2.0,
                )
                .unwrap();
            }
        }

        let pools_top = row_y + WAVE_CELL_HEIGHT + WAVE_CELL_GAP;
        let mut triggers = Vec::new();
        for column in block_start..block_end {
            let step_index = column % steps.len().max(1);
            let Some(step) = steps.get(step_index).copied() else {
                continue;
            };
            if !step.node.is_background() {
                continue;
            }
            let cycle = column / steps.len().max(1);
            triggers.push(BackgroundTrigger {
                column,
                stage_index: step_index,
                step,
                entity: &entities[step.node_id as usize],
                source_y: row_y + WAVE_CELL_HEIGHT,
                cl_label: if cycle == 0 {
                    "CL n".to_string()
                } else {
                    format!("CL n+{cycle}")
                },
            });
        }
        let background = render_background_lanes(
            steps,
            entities,
            &background_pools,
            &triggers,
            block_start,
            block_end,
            pools_top,
        );
        svg.push_str(&background.svg);
        y = pools_top + background.height + 26.0;
    }

    RenderedSection {
        svg,
        height: y - top,
    }
}

fn render_parallel(
    config: &CuConfig,
    stages: &[&CuExecutionStep],
    entities: &[PlanEntity],
    refine_totals: &HashMap<u32, u32>,
    in_flight_limit: usize,
    top: f64,
) -> RenderedSection {
    let rt_pool = config
        .runtime
        .as_ref()
        .and_then(|runtime| runtime.thread_pools.iter().find(|pool| pool.id == RT_POOL));
    render_staggered_wavefront(
        config,
        stages,
        entities,
        refine_totals,
        in_flight_limit,
        rt_pool,
        top,
    )
}

struct ObservedStageView {
    label: String,
    resources: Vec<String>,
}

fn render_observed(
    config: &CuConfig,
    stages: &[&CuExecutionStep],
    entities: &[PlanEntity],
    observed: &ObservedSchedule,
    top: f64,
) -> RenderedSection {
    let mut svg = String::new();
    let stage_views = stages
        .iter()
        .map(|step| {
            let entity = &entities[step.node_id as usize];
            let origin = observed_origin(entity);
            let resources = entity_resource_bindings(config, entity, step)
                .into_iter()
                .map(|(_, target)| target)
                .collect();
            (
                origin,
                ObservedStageView {
                    label: entity.label.clone(),
                    resources,
                },
            )
        })
        .collect::<HashMap<_, _>>();
    let aggregates = observed
        .stages
        .iter()
        .map(|stage| (stage.origin.as_str(), stage))
        .collect::<HashMap<_, _>>();

    writeln!(
        svg,
        r#"<text class="subtitle" x="0" y="{}">Observed execution · proportional recorded process intervals</text>"#,
        top + 14.0
    )
    .unwrap();
    writeln!(
        svg,
        r#"<text class="meta" x="0" y="{}">Recorded task durations are packed back-to-back; hover any segment for its task and timing details.</text>"#,
        top + 33.0
    )
    .unwrap();

    let mut y = top + 55.0;
    if observed.traces.is_empty() {
        writeln!(
            svg,
            r##"<rect x="0" y="{y}" width="{PARALLEL_WIDTH}" height="44" rx="8" fill="#fff7e6" stroke="#f0a04b"/><text class="blocking" x="12" y="{}">No complete process intervals were found in the CopperList log.</text>"##,
            y + 27.0
        )
        .unwrap();
        y += 44.0;
    }

    for trace in &observed.traces {
        let timeline_width = PARALLEL_WIDTH - LABEL_WIDTH;
        let collisions = observed_trace_collisions(trace, &stage_views);
        let active_ns: u64 = trace
            .intervals
            .iter()
            .map(|interval| interval.end_ns.saturating_sub(interval.start_ns))
            .sum();
        let trace_start = trace
            .intervals
            .iter()
            .map(|interval| interval.start_ns)
            .min()
            .unwrap_or(0);
        let trace_end = trace
            .intervals
            .iter()
            .map(|interval| interval.end_ns)
            .max()
            .unwrap_or(trace_start);
        let wall_span_ns = if trace.wall_span_ns == 0 {
            trace_end.saturating_sub(trace_start)
        } else {
            trace.wall_span_ns
        };
        let excluded_note = if trace.excluded_intervals == 0 {
            String::new()
        } else {
            format!(
                " · {} carried-forward slot(s) excluded",
                trace.excluded_intervals
            )
        };

        writeln!(
            svg,
            r#"<text class="card-title" x="0" y="{y}">{} · CL #{} · {} active · {} wall{}</text>"#,
            xml(&trace.kind.to_uppercase()),
            trace.culist_id,
            xml(&format_duration_ns(active_ns as f64)),
            xml(&format_duration_ns(wall_span_ns as f64)),
            xml(&excluded_note),
        )
        .unwrap();
        y += 17.0;
        for tick in 0..=4 {
            let x = LABEL_WIDTH + timeline_width * tick as f64 / 4.0;
            let elapsed = active_ns as f64 * tick as f64 / 4.0;
            let (label_x, anchor) = if tick == 4 {
                (x - 3.0, "end")
            } else {
                (x + 3.0, "start")
            };
            writeln!(
                svg,
                r#"<line class="grid" x1="{x}" y1="{y}" x2="{x}" y2="{}"/><text class="ordinal mono" text-anchor="{anchor}" x="{label_x}" y="{}">{}</text>"#,
                y + 48.0,
                y + 10.0,
                xml(&format_duration_ns(elapsed)),
            )
            .unwrap();
        }
        y += 15.0;

        let minimum_width = 5.0;
        let minimum_total = minimum_width * trace.intervals.len() as f64;
        let proportional_width = (timeline_width - minimum_total).max(0.0);
        let mut x = LABEL_WIDTH;
        writeln!(
            svg,
            r#"<text class="lane mono" x="0" y="{}">process</text>"#,
            y + 21.0
        )
        .unwrap();
        for interval in &trace.intervals {
            let view = stage_views.get(&interval.origin);
            let label = view
                .map(|view| view.label.as_str())
                .unwrap_or(interval.origin.as_str());
            let fill = observed_interval_fill(&interval.origin);
            let duration_ns = interval.end_ns.saturating_sub(interval.start_ns);
            let width = if active_ns == 0 {
                timeline_width / trace.intervals.len().max(1) as f64
            } else {
                minimum_width + proportional_width * duration_ns as f64 / active_ns as f64
            };
            let aggregate = aggregates.get(interval.origin.as_str());
            let collision_resources = collisions.get(&interval.origin);
            let tooltip = format!(
                "{}\norigin: {} · CL #{}\nrecorded process: {}\noriginal start: +{}\nest. p50: {} · est. p95: {} · max: {} · samples: {}\nresources: {}{}",
                label,
                interval.origin,
                trace.culist_id,
                format_duration_ns(duration_ns as f64),
                format_duration_ns(interval.start_ns.saturating_sub(trace_start) as f64),
                aggregate
                    .and_then(|stage| stage.durations.p50_ns)
                    .map_or_else(
                        || "n/a".to_string(),
                        |value| format_duration_ns(value as f64)
                    ),
                aggregate
                    .and_then(|stage| stage.durations.p95_ns)
                    .map_or_else(
                        || "n/a".to_string(),
                        |value| format_duration_ns(value as f64)
                    ),
                aggregate
                    .and_then(|stage| stage.durations.max_ns)
                    .map_or_else(
                        || "n/a".to_string(),
                        |value| format_duration_ns(value as f64)
                    ),
                aggregate.map_or(0, |stage| stage.samples),
                view.filter(|view| !view.resources.is_empty())
                    .map_or_else(|| "—".to_string(), |view| view.resources.join(", ")),
                collision_resources.map_or_else(String::new, |resources| format!(
                    "\nobserved overlap risk: {}",
                    resources.join(", ")
                )),
            );
            let inside_label = if width >= 68.0 {
                format!(
                    r#"<text class="ordinal" x="{}" y="{}">{}</text>"#,
                    x + 5.0,
                    y + 19.0,
                    xml(&truncate(label, 10))
                )
            } else {
                String::new()
            };
            writeln!(
                svg,
                r##"<g class="observed-segment" data-observed-origin="{}"><title>{}</title><rect x="{x}" y="{y}" width="{width}" height="30" rx="3" fill="{fill}" stroke="{}" stroke-width="{}"/>{inside_label}</g>"##,
                xml_attr(&interval.origin),
                xml(&tooltip),
                if collision_resources.is_some() { "#d92d20" } else { "#475467" },
                if collision_resources.is_some() { 2 } else { 1 },
            )
            .unwrap();
            x += width;
        }
        y += 39.0;
        if let Some(residual) = trace.residual_before_ns {
            writeln!(
                svg,
                r##"<text class="lane mono" x="0" y="{}">between CLs</text><rect x="{LABEL_WIDTH}" y="{y}" width="120" height="13" rx="3" fill="#d7dce2"/><text class="meta mono" x="{}" y="{}">{} before this CL · unclassified, not on the process scale</text>"##,
                y + 11.0,
                LABEL_WIDTH + 127.0,
                y + 11.0,
                xml(&format_duration_ns(residual as f64)),
            )
            .unwrap();
            y += 24.0;
        }
        y += 18.0;
    }

    writeln!(
        svg,
        r#"<text class="subtitle" x="0" y="{y}">Observed declared-resource overlap across the full log</text>"#
    )
    .unwrap();
    y += 14.0;
    if observed.resource_overlaps.is_empty() {
        writeln!(
            svg,
            r##"<rect x="0" y="{y}" width="{PARALLEL_WIDTH}" height="38" rx="8" fill="#ecfdf3" stroke="#75c58f"/><text class="blocking" x="12" y="{}">No simultaneous recorded process intervals shared a declared resource target.</text>"##,
            y + 24.0
        )
        .unwrap();
        y += 38.0;
    } else {
        let panel_height = 34.0 + observed.resource_overlaps.len() as f64 * 18.0;
        writeln!(
            svg,
            r##"<rect x="0" y="{y}" width="{PARALLEL_WIDTH}" height="{panel_height}" rx="8" fill="#fff1f0" stroke="#d92d20"/><text class="blocking" x="12" y="{}">Overlap is a contention risk signal, not proof that either task waited.</text>"##,
            y + 20.0
        )
        .unwrap();
        for (index, overlap) in observed.resource_overlaps.iter().enumerate() {
            writeln!(
                svg,
                r#"<text class="card-line mono" x="12" y="{}">⚠ {}: {} ↔ {} · {} overlap(s)</text>"#,
                y + 40.0 + index as f64 * 18.0,
                xml(&truncate(&overlap.resource, 28)),
                xml(&truncate(&overlap.left, 28)),
                xml(&truncate(&overlap.right, 28)),
                overlap.occurrences,
            )
            .unwrap();
        }
        y += panel_height;
    }

    y += 18.0;
    let residual_p50 = observed.residual_before.p50_ns.map_or_else(
        || "n/a".to_string(),
        |value| format_duration_ns(value as f64),
    );
    let residual_p95 = observed.residual_before.p95_ns.map_or_else(
        || "n/a".to_string(),
        |value| format_duration_ns(value as f64),
    );
    writeln!(
        svg,
        r##"<rect x="0" y="{y}" width="{PARALLEL_WIDTH}" height="58" rx="8" fill="#f8fafc" stroke="#98a2b3"/><text class="blocking" x="12" y="{}">Recorded process intervals exclude queueing, keyframes, logging overhead, and serialization.</text><text class="meta" x="12" y="{}">Residual gaps may include serialization, rate limiting, scheduling, and I/O; est. p50 {}, est. p95 {}. Anytime bars span base through final refine.</text>"##,
        y + 22.0,
        y + 43.0,
        xml(&residual_p50),
        xml(&residual_p95),
    )
    .unwrap();
    y += 58.0;

    RenderedSection {
        svg,
        height: y - top,
    }
}

fn observed_origin(entity: &PlanEntity) -> String {
    match entity.kind {
        PlanEntityKind::Task { .. } => entity.label.clone(),
        PlanEntityKind::BridgeRx { .. } | PlanEntityKind::BridgeTx { .. } => {
            format!("bridge::{}", entity.label)
        }
    }
}

fn observed_interval_fill(origin: &str) -> &'static str {
    const COLORS: [&str; 12] = [
        "#bde3ff", "#ffd6a5", "#cdeac0", "#e2cfea", "#ffcad4", "#b8f2e6", "#f1e3a4", "#cddafd",
        "#d8f3dc", "#f7c6c7", "#c9e4de", "#dec9e9",
    ];
    COLORS[(fnv1a64(origin.as_bytes()) as usize) % COLORS.len()]
}

fn observed_trace_collisions(
    trace: &ObservedTrace,
    stages: &HashMap<String, ObservedStageView>,
) -> HashMap<String, Vec<String>> {
    let mut collisions = HashMap::<String, Vec<String>>::new();
    for (index, left) in trace.intervals.iter().enumerate() {
        for right in &trace.intervals[..index] {
            if left.start_ns >= right.end_ns || right.start_ns >= left.end_ns {
                continue;
            }
            let (Some(left_stage), Some(right_stage)) =
                (stages.get(&left.origin), stages.get(&right.origin))
            else {
                continue;
            };
            for resource in &left_stage.resources {
                if right_stage.resources.contains(resource) {
                    for origin in [&left.origin, &right.origin] {
                        let targets = collisions.entry(origin.clone()).or_default();
                        if !targets.contains(resource) {
                            targets.push(resource.clone());
                        }
                    }
                }
            }
        }
    }
    collisions
}

fn render_staggered_wavefront(
    config: &CuConfig,
    stages: &[&CuExecutionStep],
    entities: &[PlanEntity],
    refine_totals: &HashMap<u32, u32>,
    in_flight_limit: usize,
    rt_pool: Option<&ThreadPoolConfig>,
    top: f64,
) -> RenderedSection {
    let mut svg = String::new();
    let background_pools = background_pool_views(config, stages);
    let active_lanes = in_flight_limit.min(stages.len()).max(1);
    let visible_lanes = active_lanes.min(MAX_VISIBLE_COPPERLISTS);
    let formation_count = stages
        .len()
        .saturating_add(visible_lanes)
        .saturating_sub(1)
        .max(1);

    writeln!(
        svg,
        r#"<text class="subtitle" x="0" y="{}">Staggered concurrency formations · cards in one column can run together</text>"#,
        top + 14.0
    )
    .unwrap();
    writeln!(
        svg,
        r#"<text class="meta" x="0" y="{}">Columns can run together. Purple cells are RT poll/dispatch gateways; arrows feed the aligned background-pool worker lanes.</text>"#,
        top + 33.0
    )
    .unwrap();
    let depth_note = if active_lanes > visible_lanes {
        format!(
            "Configured in-flight depth: {in_flight_limit}. Showing CL n through CL n+{} ({} of {active_lanes} active pipeline positions); the overlap audit still covers every stage.",
            visible_lanes - 1,
            visible_lanes,
        )
    } else if in_flight_limit < MAX_VISIBLE_COPPERLISTS {
        format!(
            "Configured in-flight depth: {in_flight_limit}. CL n+{in_flight_limit} cannot be admitted until an older CopperList is committed and recycled."
        )
    } else {
        format!(
            "Configured in-flight depth: {in_flight_limit}. The matrix shows every concurrently active worker lane allowed by that bound."
        )
    };
    writeln!(
        svg,
        r#"<text class="meta mono" x="0" y="{}">{}</text>"#,
        top + 51.0,
        xml(&depth_note)
    )
    .unwrap();

    let mut y = top + 76.0;
    for block_start in (0..formation_count).step_by(WAVE_COLUMNS) {
        let block_end = (block_start + WAVE_COLUMNS).min(formation_count);
        for formation in block_start..block_end {
            let column = formation - block_start;
            let x = LABEL_WIDTH + column as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
            let collisions =
                formation_collisions(config, stages, entities, visible_lanes, formation);
            writeln!(
                svg,
                r#"<text class="ordinal mono" x="{x}" y="{y}"{}>FORMATION {:02}{}</text>"#,
                if collisions.is_empty() {
                    ""
                } else {
                    r##" fill="#b42318""##
                },
                formation + 1,
                if collisions.is_empty() { "" } else { " ⚠" }
            )
            .unwrap();
        }

        let rows_top = y + 11.0;
        for lane in 0..visible_lanes {
            let row_y = rows_top + lane as f64 * (WAVE_CELL_HEIGHT + WAVE_CELL_GAP);
            writeln!(
                svg,
                r#"<text class="lane mono" x="0" y="{}">CL n{}</text>"#,
                row_y + WAVE_CELL_HEIGHT / 2.0 + 4.0,
                if lane == 0 {
                    String::new()
                } else {
                    format!("+{lane}")
                }
            )
            .unwrap();

            // Connect consecutive occupied cells before drawing the cards so
            // each CopperList's diagonal progression remains visually explicit.
            for formation in (block_start + 1)..block_end {
                let Some(previous_stage) = (formation - 1).checked_sub(lane) else {
                    continue;
                };
                let Some(current_stage) = formation.checked_sub(lane) else {
                    continue;
                };
                if previous_stage >= stages.len() || current_stage >= stages.len() {
                    continue;
                }
                let column = formation - block_start;
                let previous_x =
                    LABEL_WIDTH + (column - 1) as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
                let current_x = LABEL_WIDTH + column as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
                writeln!(
                    svg,
                    r#"<line class="flow" x1="{}" y1="{}" x2="{current_x}" y2="{}"/>"#,
                    previous_x + WAVE_CELL_WIDTH,
                    row_y + WAVE_CELL_HEIGHT / 2.0,
                    row_y + WAVE_CELL_HEIGHT / 2.0,
                )
                .unwrap();
            }

            for formation in block_start..block_end {
                let column = formation - block_start;
                let x = LABEL_WIDTH + column as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
                let Some(stage_index) = formation.checked_sub(lane) else {
                    writeln!(
                        svg,
                        r##"<rect x="{x}" y="{row_y}" width="{WAVE_CELL_WIDTH}" height="{WAVE_CELL_HEIGHT}" rx="6" fill="#fafafa" stroke="#e4e7ec" stroke-dasharray="3 3"/>"##
                    )
                    .unwrap();
                    continue;
                };
                let Some(step) = stages.get(stage_index) else {
                    writeln!(
                        svg,
                        r##"<rect x="{x}" y="{row_y}" width="{WAVE_CELL_WIDTH}" height="{WAVE_CELL_HEIGHT}" rx="6" fill="#fafafa" stroke="#e4e7ec" stroke-dasharray="3 3"/>"##
                    )
                    .unwrap();
                    continue;
                };
                let entity = &entities[step.node_id as usize];
                let (_, entity_fill) = entity_style(entity, step);
                let is_background = step.node.is_background();
                let fill = if is_background {
                    BACKGROUND_GATEWAY_COLOR
                } else {
                    entity_fill
                };
                let collisions =
                    formation_collisions(config, stages, entities, visible_lanes, formation);
                let collision_targets = collisions.get(&lane).cloned().unwrap_or_default();
                let stroke = if collision_targets.is_empty() {
                    "#98a2b3"
                } else {
                    "#d92d20"
                };
                let stroke_width = if collision_targets.is_empty() {
                    1.0
                } else {
                    3.0
                };
                let placement = worker_placement(rt_pool, stage_index);
                let resources = entity_resource_bindings(config, entity, step)
                    .into_iter()
                    .map(|(_, target)| target)
                    .collect::<Vec<_>>();
                let phase = if step.phase == CuStepPhase::AnytimeBase {
                    format!(
                        "base + {} refine",
                        refine_totals.get(&step.node_id).copied().unwrap_or(0)
                    )
                } else if is_background {
                    "poll / maybe dispatch".to_string()
                } else {
                    "whole".to_string()
                };
                let background_detail = if is_background {
                    format!(
                        "\nRT GATEWAY ONLY: the task work does not execute in this cell\nidle/ready: emit buffered output + FIFO-enqueue current input\nbusy/not ready: emit empty output; do not enqueue another job\nbackground pool: {}",
                        step.node.background_pool()
                    )
                } else {
                    String::new()
                };
                let tooltip = format!(
                    "Formation {} · CL n+{}\nStage {}: {}\nphase: {}\nworker placement: {}\nconfigured resource targets: {}{}{}",
                    formation + 1,
                    lane,
                    stage_index + 1,
                    entity.label,
                    phase,
                    placement,
                    if resources.is_empty() {
                        "—".to_string()
                    } else {
                        resources.join(", ")
                    },
                    background_detail,
                    if collision_targets.is_empty() {
                        String::new()
                    } else {
                        format!(
                            "\nPOTENTIAL CONTENTION in this formation: {}",
                            collision_targets.join(", ")
                        )
                    }
                );
                let placement_label = if is_background {
                    "RT GATEWAY".to_string()
                } else {
                    truncate(&placement, 13)
                };
                let footer = if is_background {
                    format!("pool {}", step.node.background_pool())
                } else if collision_targets.is_empty() {
                    if resources.is_empty() {
                        "res —".to_string()
                    } else {
                        format!("res {}", resources.join(", "))
                    }
                } else {
                    format!("⚠ {}", collision_targets.join(", "))
                };
                writeln!(
                    svg,
                    r##"<g data-formation="{}" data-cl-offset="{lane}" data-stage="{}"{}><title>{}</title><rect x="{x}" y="{row_y}" width="{WAVE_CELL_WIDTH}" height="{WAVE_CELL_HEIGHT}" rx="6" fill="{fill}" stroke="{stroke}" stroke-width="{stroke_width}"{}/><text class="ordinal mono" x="{}" y="{}">S{:02} · {}</text><text class="card-title" x="{}" y="{}">{}</text><text class="card-line" x="{}" y="{}">{}</text><text class="card-line mono" x="{}" y="{}">{}</text></g>"##,
                    formation + 1,
                    stage_index + 1,
                    if is_background { r#" data-background-gateway="true""# } else { "" },
                    xml(&tooltip),
                    if is_background { r#" stroke-dasharray="4 2""# } else { "" },
                    x + 7.0,
                    row_y + 13.0,
                    stage_index + 1,
                    xml(&placement_label),
                    x + 7.0,
                    row_y + 29.0,
                    xml(&truncate(&entity.label, 16)),
                    x + 7.0,
                    row_y + 45.0,
                    xml(&truncate(&phase, 18)),
                    x + 7.0,
                    row_y + 62.0,
                    xml(&truncate(&footer, 18)),
                )
                .unwrap();
            }
        }
        let pools_top = rows_top + visible_lanes as f64 * (WAVE_CELL_HEIGHT + WAVE_CELL_GAP);
        let mut triggers = Vec::new();
        for lane in 0..visible_lanes {
            for formation in block_start..block_end {
                let Some(stage_index) = formation.checked_sub(lane) else {
                    continue;
                };
                let Some(step) = stages.get(stage_index).copied() else {
                    continue;
                };
                if !step.node.is_background() {
                    continue;
                }
                triggers.push(BackgroundTrigger {
                    column: formation,
                    stage_index,
                    step,
                    entity: &entities[step.node_id as usize],
                    source_y: rows_top
                        + lane as f64 * (WAVE_CELL_HEIGHT + WAVE_CELL_GAP)
                        + WAVE_CELL_HEIGHT,
                    cl_label: if lane == 0 {
                        "CL n".to_string()
                    } else {
                        format!("CL n+{lane}")
                    },
                });
            }
        }
        let background = render_background_lanes(
            stages,
            entities,
            &background_pools,
            &triggers,
            block_start,
            block_end,
            pools_top,
        );
        svg.push_str(&background.svg);
        y = pools_top + background.height + 28.0;
    }

    let overlap_groups = resource_overlap_groups(config, stages, entities, in_flight_limit);
    let has_background = stages.iter().any(|step| step.node.is_background());
    writeln!(
        svg,
        r#"<text class="subtitle" x="0" y="{y}">Potential configured-resource overlap across skewed formations and background jobs</text>"#
    )
    .unwrap();
    y += 13.0;
    if in_flight_limit < 2 && !has_background {
        writeln!(
            svg,
            r##"<rect x="0" y="{y}" width="{PARALLEL_WIDTH}" height="50" rx="8" fill="#ecfdf3" stroke="#75c58f"/><text class="blocking" x="12" y="{}">Configured depth is 1, so foreground stages cannot overlap across CopperLists.</text><text class="meta" x="12" y="{}">Repeated bindings still execute serially inside that single CopperList.</text>"##,
            y + 21.0,
            y + 39.0,
        )
        .unwrap();
        y += 50.0;
    } else if overlap_groups.is_empty() {
        writeln!(
            svg,
            r##"<rect x="0" y="{y}" width="{PARALLEL_WIDTH}" height="50" rx="8" fill="#ecfdf3" stroke="#75c58f"/><text class="blocking" x="12" y="{}">No resource target is bound by more than one potentially overlapping stage or background job.</text><text class="meta" x="12" y="{}">This cannot detect GPU/device use that tasks do not declare as a Copper resource binding.</text>"##,
            y + 21.0,
            y + 39.0,
        )
        .unwrap();
        y += 50.0;
    } else {
        let panel_height = 42.0 + overlap_groups.len() as f64 * 20.0;
        writeln!(
            svg,
            r##"<rect x="0" y="{y}" width="{PARALLEL_WIDTH}" height="{panel_height}" rx="8" fill="#fff1f0" stroke="#d92d20"/><text class="blocking" x="12" y="{}">These executions can overlap across CopperLists or while a background job remains active; bindings are not scheduler reservations.</text>"##,
            y + 20.0,
        )
        .unwrap();
        for (index, (target, stage_labels)) in overlap_groups.iter().enumerate() {
            writeln!(
                svg,
                r#"<text class="card-line mono" x="12" y="{}">⚠ {}: {}</text>"#,
                y + 42.0 + index as f64 * 20.0,
                xml(&truncate(target, 36)),
                xml(&truncate(&stage_labels.join(" ↔ "), 118)),
            )
            .unwrap();
        }
        y += panel_height;
    }

    RenderedSection {
        svg,
        height: y - top,
    }
}

struct BackgroundPoolView<'a> {
    id: String,
    config: Option<&'a ThreadPoolConfig>,
    stage_indices: Vec<usize>,
}

struct BackgroundTrigger<'a> {
    column: usize,
    stage_index: usize,
    step: &'a CuExecutionStep,
    entity: &'a PlanEntity,
    source_y: f64,
    cl_label: String,
}

fn background_pool_views<'a>(
    config: &'a CuConfig,
    stages: &[&CuExecutionStep],
) -> Vec<BackgroundPoolView<'a>> {
    let mut assignments = BTreeMap::<String, Vec<usize>>::new();
    for (stage_index, step) in stages.iter().enumerate() {
        if step.node.is_background() {
            assignments
                .entry(step.node.background_pool().to_string())
                .or_default()
                .push(stage_index);
        }
    }
    let configured = config
        .runtime
        .as_ref()
        .map(|runtime| runtime.thread_pools.as_slice())
        .unwrap_or_default();
    assignments
        .into_iter()
        .map(|(id, stage_indices)| BackgroundPoolView {
            config: configured.iter().find(|pool| pool.id == id),
            id,
            stage_indices,
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn render_background_lanes(
    stages: &[&CuExecutionStep],
    entities: &[PlanEntity],
    pools: &[BackgroundPoolView<'_>],
    triggers: &[BackgroundTrigger<'_>],
    block_start: usize,
    block_end: usize,
    top: f64,
) -> RenderedSection {
    if pools.is_empty() {
        return RenderedSection {
            svg: String::new(),
            height: 0.0,
        };
    }

    let block_width =
        (block_end - block_start) as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP) - WAVE_CELL_GAP;
    let mut pool_tops = BTreeMap::<String, f64>::new();
    let mut y = top + 29.0;
    for pool in pools {
        pool_tops.insert(pool.id.clone(), y);
        let threads = pool.config.map(|pool| pool.threads).unwrap_or(1);
        let queue_rows = usize::from(pool.stage_indices.len() > threads);
        y += 25.0 + (threads + queue_rows) as f64 * 39.0 + 12.0;
    }

    let mut svg = String::new();
    writeln!(
        svg,
        r#"<text class="ordinal mono" x="0" y="{}">BACKGROUND POOL WORKER THREADS · DASHED BARS MAY CONTINUE</text>"#,
        top + 17.0
    )
    .unwrap();

    for pool in pools {
        let pool_y = pool_tops[&pool.id];
        let threads = pool.config.map(|pool| pool.threads).unwrap_or(1);
        let active_slots = threads.min(pool.stage_indices.len());
        let task_labels = pool
            .stage_indices
            .iter()
            .map(|stage_index| {
                entities[stages[*stage_index].node_id as usize]
                    .label
                    .as_str()
            })
            .collect::<Vec<_>>();
        let metadata = pool
            .config
            .map(format_pool)
            .unwrap_or_else(|| format!("{}: pool metadata unavailable", pool.id));
        writeln!(
            svg,
            r#"<g data-background-pool="{}"><title>{}</title><text class="card-title" x="0" y="{}">POOL {}</text><text class="meta mono" x="{}" y="{}">{}</text>"#,
            xml_attr(&pool.id),
            xml(&metadata),
            pool_y + 13.0,
            xml(&pool.id),
            LABEL_WIDTH + 22.0,
            pool_y + 13.0,
            xml(&truncate(&metadata, 105)),
        )
        .unwrap();

        let earliest_dispatch = pool
            .stage_indices
            .iter()
            .copied()
            .min()
            .unwrap_or(usize::MAX);
        for worker in 0..threads {
            let lane_y = pool_y + 20.0 + worker as f64 * 39.0;
            writeln!(
                svg,
                r#"<text class="lane mono" x="0" y="{}">{} w{}</text><rect class="pool" x="{LABEL_WIDTH}" y="{lane_y}" width="{block_width}" height="31" rx="5"/>"#,
                lane_y + 20.0,
                xml(&truncate(&pool.id, 11)),
                worker + 1,
            )
            .unwrap();

            if worker < active_slots && earliest_dispatch < block_end {
                let start_column = earliest_dispatch.saturating_sub(block_start);
                let bar_column = start_column.min(block_end - block_start - 1);
                let bar_x = LABEL_WIDTH + bar_column as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
                let bar_width = LABEL_WIDTH + block_width - bar_x;
                let carry_in = earliest_dispatch < block_start;
                let label = if task_labels.len() == 1 {
                    format!("{} job · running ?", task_labels[0])
                } else {
                    format!("{} jobs · running ?", task_labels.join(" / "))
                };
                writeln!(
                    svg,
                    r##"<rect class="job" x="{bar_x}" y="{}" width="{bar_width}" height="25" rx="4" stroke-dasharray="6 3"/><text class="card-line mono" x="{}" y="{}">{}{}</text><line class="job-flow" x1="{}" y1="{}" x2="{}" y2="{}"/>"##,
                    lane_y + 3.0,
                    bar_x + 7.0,
                    lane_y + 19.0,
                    if carry_in { "← " } else { "" },
                    xml(&truncate(&label, 42)),
                    (bar_x + 190.0).min(LABEL_WIDTH + block_width - 40.0),
                    lane_y + 16.0,
                    LABEL_WIDTH + block_width - 10.0,
                    lane_y + 16.0,
                )
                .unwrap();
            }
        }

        if pool.stage_indices.len() > threads {
            let queue_y = pool_y + 20.0 + threads as f64 * 39.0;
            writeln!(
                svg,
                r##"<text class="lane mono" x="0" y="{}" fill="#b42318">FIFO queue</text><rect x="{LABEL_WIDTH}" y="{queue_y}" width="{block_width}" height="31" rx="5" fill="#fff1f0" stroke="#d92d20"/><text class="card-line mono" x="{}" y="{}">⚠ {} task jobs share {} workers · excess jobs can wait here</text>"##,
                queue_y + 20.0,
                LABEL_WIDTH + 7.0,
                queue_y + 20.0,
                pool.stage_indices.len(),
                threads,
            )
            .unwrap();
        }
        svg.push_str("</g>\n");
    }

    for (trigger_ordinal, trigger) in triggers.iter().enumerate() {
        let pool_id = trigger.step.node.background_pool();
        let Some(pool_y) = pool_tops.get(pool_id) else {
            continue;
        };
        let column = trigger.column - block_start;
        let cell_x = LABEL_WIDTH + column as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
        let source_x = cell_x + WAVE_CELL_WIDTH - 8.0;
        let target_x = (cell_x + WAVE_CELL_WIDTH + 3.0 + (trigger_ordinal % 3) as f64 * 2.0)
            .min(LABEL_WIDTH + block_width - 4.0);
        let source_y = trigger.source_y;
        let target_y = pool_y + 36.0;
        let tooltip = format!(
            "{} {} gateway → pool {}\nDispatch occurs only if this background task is idle/ready\nIf already busy: empty output, no second job",
            trigger.cl_label, trigger.entity.label, pool_id,
        );
        writeln!(
            svg,
            r##"<g data-background-trigger="{}"><title>{}</title><path class="bg-trigger" d="M {source_x} {source_y} L {target_x} {} L {target_x} {target_y}"/><circle cx="{target_x}" cy="{target_y}" r="3" fill="#7f56d9"/></g>"##,
            trigger.stage_index + 1,
            xml(&tooltip),
            source_y + 6.0,
        )
        .unwrap();
    }

    RenderedSection {
        svg,
        height: y - top,
    }
}

fn formation_collisions(
    config: &CuConfig,
    stages: &[&CuExecutionStep],
    entities: &[PlanEntity],
    visible_lanes: usize,
    formation: usize,
) -> HashMap<usize, Vec<String>> {
    let mut target_lanes = BTreeMap::<String, BTreeSet<usize>>::new();
    for lane in 0..visible_lanes {
        let Some(stage_index) = formation.checked_sub(lane) else {
            continue;
        };
        let Some(step) = stages.get(stage_index) else {
            continue;
        };
        // A background stage is only a state-check/dispatch gateway. Its
        // declared resources belong to the asynchronous job, whose lifetime
        // is independent of this nominal RT formation and is audited below.
        if step.node.is_background() {
            continue;
        }
        let entity = &entities[step.node_id as usize];
        for (_, target) in entity_resource_bindings(config, entity, step) {
            target_lanes.entry(target).or_default().insert(lane);
        }
    }

    let mut collisions = HashMap::<usize, Vec<String>>::new();
    for (target, lanes) in target_lanes {
        if lanes.len() < 2 {
            continue;
        }
        for lane in lanes {
            collisions.entry(lane).or_default().push(target.clone());
        }
    }
    collisions
}

fn resource_overlap_groups(
    config: &CuConfig,
    stages: &[&CuExecutionStep],
    entities: &[PlanEntity],
    in_flight_limit: usize,
) -> Vec<(String, Vec<String>)> {
    let mut target_stages = BTreeMap::<String, BTreeSet<usize>>::new();
    for (stage_index, step) in stages.iter().enumerate() {
        let entity = &entities[step.node_id as usize];
        for (_, target) in entity_resource_bindings(config, entity, step) {
            target_stages.entry(target).or_default().insert(stage_index);
        }
    }

    target_stages
        .into_iter()
        .filter(|(_, stage_indices)| {
            stage_indices.len() > 1
                && (in_flight_limit >= 2
                    || stage_indices
                        .iter()
                        .any(|stage_index| stages[*stage_index].node.is_background()))
        })
        .map(|(target, stage_indices)| {
            let labels = stage_indices
                .into_iter()
                .map(|stage_index| {
                    let step = stages[stage_index];
                    let entity = &entities[step.node_id as usize];
                    format!(
                        "S{:02} {}{}",
                        stage_index + 1,
                        entity.label,
                        if step.node.is_background() {
                            " (background job)"
                        } else {
                            ""
                        }
                    )
                })
                .collect();
            (target, labels)
        })
        .collect()
}

fn worker_placement(rt_pool: Option<&ThreadPoolConfig>, stage_index: usize) -> String {
    let Some(pool) = rt_pool else {
        return "OS scheduled".to_string();
    };
    let affinity = pool.affinity.as_deref().unwrap_or_default();
    if affinity.is_empty() {
        "OS scheduled".to_string()
    } else {
        format!("CPU {}", affinity[stage_index % affinity.len()])
    }
}

fn entity_style(entity: &PlanEntity, step: &CuExecutionStep) -> (&'static str, &'static str) {
    match entity.kind {
        PlanEntityKind::BridgeRx { .. } | PlanEntityKind::BridgeTx { .. } => {
            ("bridge", BRIDGE_COLOR)
        }
        PlanEntityKind::Task { .. } => match step.task_type {
            CuTaskType::Source => ("source", SOURCE_COLOR),
            CuTaskType::Regular => ("task", TASK_COLOR),
            CuTaskType::Sink => ("sink", SINK_COLOR),
        },
    }
}

fn entity_resources(config: &CuConfig, entity: &PlanEntity, step: &CuExecutionStep) -> Vec<String> {
    entity_resource_bindings(config, entity, step)
        .into_iter()
        .map(|(binding, resource)| format!("{binding}→{resource}"))
        .collect()
}

fn entity_resource_bindings(
    config: &CuConfig,
    entity: &PlanEntity,
    step: &CuExecutionStep,
) -> Vec<(String, String)> {
    let resources = match entity.kind {
        PlanEntityKind::Task { .. } => step.node.get_resources(),
        PlanEntityKind::BridgeRx {
            bridge_config_index,
            ..
        }
        | PlanEntityKind::BridgeTx {
            bridge_config_index,
            ..
        } => config.bridges[bridge_config_index].resources.as_ref(),
    };
    let mut bindings = resources
        .into_iter()
        .flat_map(|resources| resources.iter())
        .map(|(binding, resource)| (binding.clone(), resource.clone()))
        .collect::<Vec<_>>();
    bindings.sort();
    bindings
}

fn format_pool(pool: &ThreadPoolConfig) -> String {
    let affinity = pool
        .affinity
        .as_ref()
        .map(|cores| {
            format!(
                "[{}]",
                cores
                    .iter()
                    .map(usize::to_string)
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })
        .unwrap_or_else(|| "OS scheduled".to_string());
    format!(
        "{}: threads={}, affinity={}, policy={}, on_error={}",
        pool.id,
        pool.threads,
        affinity,
        format_policy(pool.policy),
        match pool.on_error {
            OnError::Warn => "Warn",
            OnError::Strict => "Strict",
        }
    )
}

fn format_policy(policy: SchedulingPolicy) -> String {
    match policy {
        SchedulingPolicy::Fair => "Fair".to_string(),
        SchedulingPolicy::Nice(value) => format!("Nice({value})"),
        SchedulingPolicy::Fifo { priority } => format!("Fifo(priority: {priority})"),
        SchedulingPolicy::RoundRobin { priority } => {
            format!("RoundRobin(priority: {priority})")
        }
    }
}

fn truncate(value: &str, max_chars: usize) -> String {
    let mut chars = value.chars();
    let prefix = chars.by_ref().take(max_chars).collect::<String>();
    if chars.next().is_some() {
        format!("{prefix}…")
    } else {
        prefix
    }
}

fn format_duration_ns(nanos: f64) -> String {
    if nanos >= 1_000_000_000.0 {
        format!("{:.3} s", nanos / 1_000_000_000.0)
    } else if nanos >= 1_000_000.0 {
        format!("{:.3} ms", nanos / 1_000_000.0)
    } else if nanos >= 1_000.0 {
        format!("{:.3} us", nanos / 1_000.0)
    } else {
        format!("{nanos:.0} ns")
    }
}

fn xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

fn xml_attr(value: &str) -> String {
    xml(value).replace('"', "&quot;").replace('\'', "&apos;")
}

fn open_svg(path: &Path) -> std::io::Result<()> {
    if cfg!(target_os = "windows") {
        Command::new("cmd")
            .args(["/C", "start", ""])
            .arg(path)
            .status()?;
        return Ok(());
    }
    let program = if cfg!(target_os = "macos") {
        "open"
    } else {
        "xdg-open"
    };
    Command::new(program).arg(path).status()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_runtime::config::CuConfig;

    fn config(ron: &str) -> CuConfig {
        CuConfig::deserialize_ron(ron).expect("valid test config")
    }

    #[test]
    fn deterministic_svg_contains_schedule_details_and_stable_keys() {
        let config = config(
            r#"(
                runtime: (
                    rate_target_hz: 100,
                    thread_pools: [
                        (id: "rt", threads: 2, affinity: [2, 4], policy: Fifo(priority: 80), on_error: Strict),
                    ],
                ),
                resources: [(id: "board", provider: "demo::Board")],
                tasks: [
                    (id: "src", type: "demo::Src", kind: source),
                    (id: "work", type: "demo::Work", kind: task, resources: { "bus": "board.spi" }),
                    (id: "sink", type: "demo::Sink", kind: sink),
                ],
                cnx: [
                    (src: "src", dst: "work", msg: "demo::A"),
                    (src: "work", dst: "sink", msg: "demo::B"),
                ],
            )"#,
        );
        let sections = selected_graphs(&config, None).unwrap();
        let first = render_document(&config, &sections, None).unwrap();
        let second = render_document(&config, &sections, None).unwrap();
        assert_eq!(first, second);
        assert!(first.contains("Serial projection"));
        assert!(first.contains("Parallel projection"));
        assert!(first.contains("CPU 2"));
        assert!(first.contains("CPU 4"));
        assert!(first.contains("bus→board.spi"));
        assert!(first.contains("mission:default|task:work|phase:whole"));
        assert!(first.contains("threads=2"));
    }

    #[test]
    fn anytime_is_woven_in_serial_and_collapsed_in_parallel() {
        let config = config(
            r#"(
                tasks: [
                    (id: "src", type: "demo::Src", kind: source),
                    (id: "any", type: "demo::Any", kind: task, anytime: (max_refines: 3)),
                    (id: "other", type: "demo::Other", kind: source),
                    (id: "sink", type: "demo::Sink", kind: sink),
                ],
                cnx: [
                    (src: "src", dst: "any", msg: "demo::A"),
                    (src: "any", dst: "sink", msg: "demo::B"),
                    (src: "other", dst: "__nc__", msg: "demo::OtherMsg"),
                ],
            )"#,
        );
        let svg = render_document(&config, &selected_graphs(&config, None).unwrap(), None).unwrap();
        assert!(svg.contains("refine 1/3"));
        assert!(svg.contains("refine 3/3"));
        assert!(svg.contains("base + 3 refine"));
        assert!(svg.contains("phase:refine:3"));
    }

    #[test]
    fn missions_are_sorted_and_selectable() {
        let config = config(
            r#"(
                missions: [(id: "zeta"), (id: "alpha")],
                tasks: [(id: "src", type: "demo::Src", kind: source)],
                cnx: [(src: "src", dst: "__nc__", msg: "demo::A")],
            )"#,
        );
        let all = selected_graphs(&config, None).unwrap();
        assert_eq!(all[0].0, "alpha");
        assert_eq!(all[1].0, "zeta");
        let selected = selected_graphs(&config, Some("zeta")).unwrap();
        assert_eq!(selected.len(), 1);
        assert!(selected_graphs(&config, Some("missing")).is_err());
    }

    #[test]
    fn cli_defaults_to_plan_svg_and_parses_features() {
        let args = Args::try_parse_from([
            "cu29-plan",
            "copperconfig.ron",
            "--features",
            "camera,mock",
            "--logstats",
            "stats.json",
        ])
        .unwrap();
        assert_eq!(args.output, PathBuf::from("plan.svg"));
        assert_eq!(args.features, ["camera", "mock"]);
        assert_eq!(args.logstats, Some(PathBuf::from("stats.json")));
    }

    #[test]
    fn background_and_unpinned_workers_are_explicit() {
        let config = config(
            r#"(
                runtime: (
                    thread_pools: [
                        (id: "vision", threads: 1, policy: Nice(10), on_error: Warn),
                    ],
                ),
                tasks: [
                    (id: "src", type: "demo::Src", kind: source),
                    (id: "vision", type: "demo::Vision", kind: task, background: (pool: "vision")),
                    (id: "sink", type: "demo::Sink", kind: sink),
                ],
                cnx: [
                    (src: "src", dst: "vision", msg: "demo::A"),
                    (src: "vision", dst: "sink", msg: "demo::B"),
                ],
            )"#,
        );
        let svg = render_document(&config, &selected_graphs(&config, None).unwrap(), None).unwrap();
        assert!(svg.contains("POLL / DISPATCH?"));
        assert!(svg.contains("BACKGROUND POOL WORKER THREADS"));
        assert!(svg.contains("RT GATEWAY ONLY"));
        assert!(svg.contains("POOL vision"));
        assert!(svg.contains("vision job · running ?"));
        assert!(svg.contains(r#"data-background-gateway="true""#));
        assert!(svg.contains(r#"data-background-pool="vision""#));
        assert!(svg.contains(r#"data-background-trigger="2""#));
        assert!(svg.contains("OS scheduled"));
        assert!(svg.contains("policy=Nice(10)"));
        assert!(svg.contains("Stage 2: vision"));
        assert!(svg.contains("2 CopperLists max in flight"));
        assert!(svg.contains("Configured in-flight depth: 2"));
        assert!(svg.contains("CL n+1"));
        assert!(!svg.contains("Queue and blocking detail"));
    }

    #[test]
    fn shared_background_pool_and_persistent_resource_contention_are_explicit() {
        let config = config(
            r#"(
                runtime: (
                    thread_pools: [
                        (id: "vision", threads: 1, affinity: [3], policy: Fair, on_error: Warn),
                    ],
                ),
                resources: [(id: "gpu0", provider: "demo::Gpu")],
                tasks: [
                    (id: "src", type: "demo::Src", kind: source),
                    (id: "detect", type: "demo::Detect", kind: task, background: (pool: "vision"), resources: { "gpu": "gpu0" }),
                    (id: "refine", type: "demo::Refine", kind: task, background: (pool: "vision"), resources: { "gpu": "gpu0" }),
                    (id: "sink", type: "demo::Sink", kind: sink),
                ],
                cnx: [
                    (src: "src", dst: "detect", msg: "demo::A"),
                    (src: "detect", dst: "refine", msg: "demo::B"),
                    (src: "refine", dst: "sink", msg: "demo::C"),
                ],
            )"#,
        );
        let svg = render_document(&config, &selected_graphs(&config, None).unwrap(), None).unwrap();
        assert!(svg.contains("FIFO queue"));
        assert!(svg.contains("2 task jobs share 1 workers"));
        assert!(svg.contains("excess jobs can wait here"));
        assert!(svg.contains("S02 detect (background job) ↔ S03 refine (background job)"));
        assert!(!svg.contains("POTENTIAL CONTENTION in this formation: gpu0"));
    }

    #[test]
    fn staggered_wavefront_uses_the_configured_six_copperlist_depth() {
        let config = config(
            r#"(
                logging: (copperlist_count: 6),
                tasks: [
                    (id: "s0", type: "demo::S0", kind: source),
                    (id: "s1", type: "demo::S1", kind: source),
                    (id: "s2", type: "demo::S2", kind: source),
                    (id: "s3", type: "demo::S3", kind: source),
                    (id: "s4", type: "demo::S4", kind: source),
                    (id: "s5", type: "demo::S5", kind: source),
                ],
                cnx: [
                    (src: "s0", dst: "__nc__", msg: "demo::M0"),
                    (src: "s1", dst: "__nc__", msg: "demo::M1"),
                    (src: "s2", dst: "__nc__", msg: "demo::M2"),
                    (src: "s3", dst: "__nc__", msg: "demo::M3"),
                    (src: "s4", dst: "__nc__", msg: "demo::M4"),
                    (src: "s5", dst: "__nc__", msg: "demo::M5"),
                ],
            )"#,
        );
        let svg = render_document(&config, &selected_graphs(&config, None).unwrap(), None).unwrap();
        assert!(svg.contains("Configured in-flight depth: 6"));
        assert!(svg.contains("CL n+5"));
        assert!(svg.contains(r#"data-cl-offset="5""#));
    }

    #[test]
    fn large_in_flight_depth_is_a_six_copperlist_window() {
        let config = config(
            r#"(
                logging: (copperlist_count: 32),
                tasks: [
                    (id: "s0", type: "demo::S0", kind: source),
                    (id: "s1", type: "demo::S1", kind: source),
                    (id: "s2", type: "demo::S2", kind: source),
                    (id: "s3", type: "demo::S3", kind: source),
                    (id: "s4", type: "demo::S4", kind: source),
                    (id: "s5", type: "demo::S5", kind: source),
                    (id: "s6", type: "demo::S6", kind: source),
                ],
                cnx: [
                    (src: "s0", dst: "__nc__", msg: "demo::M0"),
                    (src: "s1", dst: "__nc__", msg: "demo::M1"),
                    (src: "s2", dst: "__nc__", msg: "demo::M2"),
                    (src: "s3", dst: "__nc__", msg: "demo::M3"),
                    (src: "s4", dst: "__nc__", msg: "demo::M4"),
                    (src: "s5", dst: "__nc__", msg: "demo::M5"),
                    (src: "s6", dst: "__nc__", msg: "demo::M6"),
                ],
            )"#,
        );
        let svg = render_document(&config, &selected_graphs(&config, None).unwrap(), None).unwrap();
        assert!(svg.contains("Showing CL n through CL n+5 (6 of 7 active pipeline positions)"));
        assert!(svg.contains(r#"data-cl-offset="5""#));
        assert!(!svg.contains(r#"data-cl-offset="6""#));
    }

    #[test]
    fn staggered_wavefront_highlights_declared_resource_contention() {
        let config = config(
            r#"(
                resources: [(id: "gpu0", provider: "demo::Gpu")],
                tasks: [
                    (id: "src", type: "demo::Src", kind: source),
                    (id: "detect", type: "demo::Detect", kind: task, resources: { "gpu": "gpu0" }),
                    (id: "refine", type: "demo::Refine", kind: task, resources: { "accelerator": "gpu0" }),
                    (id: "sink", type: "demo::Sink", kind: sink),
                ],
                cnx: [
                    (src: "src", dst: "detect", msg: "demo::A"),
                    (src: "detect", dst: "refine", msg: "demo::B"),
                    (src: "refine", dst: "sink", msg: "demo::C"),
                ],
            )"#,
        );
        let svg = render_document(&config, &selected_graphs(&config, None).unwrap(), None).unwrap();
        assert!(svg.contains("Staggered concurrency formations"));
        assert!(svg.contains("POTENTIAL CONTENTION in this formation: gpu0"));
        assert!(svg.contains("⚠ gpu0: S02 detect ↔ S03 refine"));
        assert!(svg.contains(r##"stroke="#d92d20" stroke-width="3""##));
    }

    #[test]
    fn observed_schedule_is_proportional_and_labels_inference_limits() {
        let config = config(
            r#"(
                resources: [(id: "gpu0", provider: "demo::Gpu")],
                tasks: [
                    (id: "left", type: "demo::Left", kind: source, resources: { "gpu": "gpu0" }),
                    (id: "right", type: "demo::Right", kind: source, resources: { "gpu": "gpu0" }),
                ],
                cnx: [
                    (src: "left", dst: "__nc__", msg: "demo::A"),
                    (src: "right", dst: "__nc__", msg: "demo::B"),
                ],
            )"#,
        );
        let duration = ObservedDurationStats {
            p50_ns: Some(100),
            p95_ns: Some(200),
            max_ns: Some(250),
        };
        let stats = ObservedLogStats {
            schema_version: 2,
            config_signature: build_logstats_signature(&config, None).unwrap(),
            mission: None,
            schedule: Some(ObservedSchedule {
                stages: vec![ObservedStage {
                    origin: "left".to_string(),
                    samples: 4,
                    durations: duration,
                }],
                traces: vec![ObservedTrace {
                    kind: "typical".to_string(),
                    culist_id: 7,
                    wall_span_ns: 300,
                    excluded_intervals: 1,
                    residual_before_ns: Some(50),
                    intervals: vec![
                        ObservedInterval {
                            origin: "left".to_string(),
                            start_ns: 1_000,
                            end_ns: 1_200,
                        },
                        ObservedInterval {
                            origin: "right".to_string(),
                            start_ns: 1_100,
                            end_ns: 1_300,
                        },
                    ],
                }],
                residual_before: ObservedDurationStats {
                    p50_ns: Some(50),
                    p95_ns: Some(75),
                    max_ns: Some(90),
                },
                resource_overlaps: vec![ObservedResourceOverlap {
                    resource: "gpu0".to_string(),
                    left: "left".to_string(),
                    right: "right".to_string(),
                    occurrences: 3,
                }],
            }),
        };
        let svg = render_document(
            &config,
            &selected_graphs(&config, None).unwrap(),
            Some(&stats),
        )
        .unwrap();
        assert!(svg.contains("Observed execution"));
        assert!(svg.contains("packed back-to-back"));
        assert!(svg.contains("CL #7"));
        assert!(svg.contains("1 carried-forward slot(s) excluded"));
        assert!(svg.contains("max: 250 ns"));
        assert!(svg.contains("observed overlap risk: gpu0"));
        assert!(svg.contains("3 overlap(s)"));
        assert!(svg.contains("Residual gaps may include serialization"));
        assert!(svg.contains(r#"data-observed-origin="left""#));
    }

    #[test]
    fn run_writes_an_explicit_output_and_missing_config_errors() {
        let temp = tempfile::tempdir().unwrap();
        let config_path = temp.path().join("copperconfig.ron");
        let output_path = temp.path().join("custom.svg");
        fs::write(
            &config_path,
            r#"(
                tasks: [(id: "src", type: "demo::Src", kind: source)],
                cnx: [],
            )"#,
        )
        .unwrap();
        run(Args {
            config: config_path,
            mission: None,
            features: Vec::new(),
            list_missions: false,
            open: false,
            output: output_path.clone(),
            logstats: None,
        })
        .unwrap();
        assert!(output_path.is_file());
        assert!(
            fs::read_to_string(output_path)
                .unwrap()
                .contains("Mission: default")
        );

        let missing = run(Args {
            config: temp.path().join("missing.ron"),
            mission: None,
            features: Vec::new(),
            list_missions: false,
            open: false,
            output: temp.path().join("missing.svg"),
            logstats: None,
        });
        assert!(missing.is_err());
    }

    #[test]
    fn logstats_mission_becomes_the_default_plan_selection() {
        let temp = tempfile::tempdir().unwrap();
        let config_path = temp.path().join("copperconfig.ron");
        let logstats_path = temp.path().join("logstats.json");
        let output_path = temp.path().join("plan.svg");
        fs::write(
            &config_path,
            r#"(
                missions: [(id: "default"), (id: "flow")],
                tasks: [(id: "src", type: "demo::Src", kind: source)],
                cnx: [],
            )"#,
        )
        .unwrap();
        fs::write(
            &logstats_path,
            r#"{
                "schema_version": 2,
                "config_signature": "test",
                "mission": "default",
                "schedule": null
            }"#,
        )
        .unwrap();

        run(Args {
            config: config_path,
            mission: None,
            features: Vec::new(),
            list_missions: false,
            open: false,
            output: output_path.clone(),
            logstats: Some(logstats_path),
        })
        .unwrap();

        let svg = fs::read_to_string(output_path).unwrap();
        assert!(svg.contains("Mission: default"));
        assert!(!svg.contains("Mission: flow"));
    }

    #[test]
    fn multi_copper_input_is_rejected_with_clear_guidance() {
        let temp = tempfile::tempdir().unwrap();
        fs::write(
            temp.path().join("alpha.ron"),
            r#"(
                tasks: [(id: "src", type: "demo::Src", kind: source)],
                cnx: [],
            )"#,
        )
        .unwrap();
        let multi_path = temp.path().join("multi_copper.ron");
        fs::write(
            &multi_path,
            r#"(
                subsystems: [(id: "alpha", config: "alpha.ron")],
                interconnects: [],
            )"#,
        )
        .unwrap();
        let error = load_single_config(&multi_path, &[]).unwrap_err();
        assert!(error.to_string().contains("not supported yet"));
    }
}
