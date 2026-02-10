mod config;
use clap::Parser;
use config::{ConfigGraphs, PortLookup, build_render_topology, read_configuration};
pub use cu29_traits::*;
use hashbrown::HashMap;
use hashbrown::hash_map::Entry;
use layout::adt::dag::NodeHandle;
use layout::core::base::Orientation;
use layout::core::color::Color;
use layout::core::format::{RenderBackend, Visible};
use layout::core::geometry::{Point, get_size_for_str, pad_shape_scalar};
use layout::core::style::{LineStyleKind, StyleAttr};
use layout::std_shapes::shapes::{Arrow, Element, LineEndKind, RecordDef, ShapeKind};
use layout::topo::layout::VisualGraph;
use serde::Deserialize;
use std::cmp::Ordering;
use std::collections::{BTreeSet, HashSet};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use svg::Document;
use svg::node::Node;
use svg::node::Text as SvgTextNode;
use svg::node::element::path::Data;
use svg::node::element::{
    Circle, Definitions, Element as SvgElement, Group, Image, Line, Marker, Path as SvgPath,
    Polygon, Rectangle, Text, TextPath, Title,
};
use tempfile::Builder;

// Typography and text formatting.
const FONT_FAMILY: &str = "'Noto Sans', sans-serif";
const MONO_FONT_FAMILY: &str = "'Noto Sans Mono'";
const FONT_SIZE: usize = 12;
const TYPE_FONT_SIZE: usize = FONT_SIZE * 7 / 10;
const PORT_HEADER_FONT_SIZE: usize = FONT_SIZE * 4 / 6;
const PORT_VALUE_FONT_SIZE: usize = FONT_SIZE * 4 / 6;
const CONFIG_FONT_SIZE: usize = PORT_VALUE_FONT_SIZE - 1;
const EDGE_FONT_SIZE: usize = 7;
const TYPE_WRAP_WIDTH: usize = 24;
const CONFIG_WRAP_WIDTH: usize = 32;
const MODULE_TRUNC_MARKER: &str = "…";
const MODULE_SEPARATOR: &str = "⠶";
const PLACEHOLDER_TEXT: &str = "\u{2014}";
const COPPER_LOGO_SVG: &str = include_str!("../assets/cu29.svg");
const LOGSTATS_SCHEMA_VERSION: u32 = 1;

// Color palette and fills.
const BORDER_COLOR: &str = "#999999";
const BACKGROUND_COLOR: &str = "#ffffff";
const HEADER_BG: &str = "#f4f4f4";
const DIM_GRAY: &str = "dimgray";
const LIGHT_GRAY: &str = "lightgray";
const CLUSTER_COLOR: &str = "#bbbbbb";
const BRIDGE_HEADER_BG: &str = "#f7d7e4";
const SOURCE_HEADER_BG: &str = "#ddefc7";
const SINK_HEADER_BG: &str = "#cce0ff";
const TASK_HEADER_BG: &str = "#fde7c2";
const RESOURCE_TITLE_BG: &str = "#eef1f6";
const RESOURCE_EXCLUSIVE_BG: &str = "#e3f4e7";
const RESOURCE_SHARED_BG: &str = "#fff0d9";
const RESOURCE_UNUSED_BG: &str = "#f1f1f1";
const RESOURCE_UNUSED_TEXT: &str = "#8d8d8d";
const PERF_TITLE_BG: &str = "#eaf2ff";
const COPPER_LINK_COLOR: &str = "#0000E0";
const EDGE_COLOR_PALETTE: [&str; 10] = [
    "#1F77B4", "#FF7F0E", "#2CA02C", "#D62728", "#9467BD", "#8C564B", "#E377C2", "#7F7F7F",
    "#BCBD22", "#17BECF",
];
const EDGE_COLOR_ORDER: [usize; 10] = [0, 2, 1, 9, 7, 8, 3, 5, 6, 4];

// Layout spacing and sizing.
const GRAPH_MARGIN: f64 = 20.0;
const CLUSTER_MARGIN: f64 = 20.0;
const SECTION_SPACING: f64 = 60.0;
const RESOURCE_TABLE_MARGIN: f64 = 18.0;
const RESOURCE_TABLE_GAP: f64 = 12.0;
const BOX_SHAPE_PADDING: f64 = 10.0;
const CELL_PADDING: f64 = 6.0;
const CELL_LINE_SPACING: f64 = 2.0;
const VALUE_BORDER_WIDTH: f64 = 0.6;
const OUTER_BORDER_WIDTH: f64 = 1.3;
const LAYOUT_SCALE_X: f64 = 1.8;
const LAYOUT_SCALE_Y: f64 = 1.2;

// Edge routing and label placement.
const EDGE_LABEL_FIT_RATIO: f64 = 0.8;
const EDGE_LABEL_OFFSET: f64 = 8.0;
const EDGE_LABEL_LIGHTEN: f64 = 0.35;
const EDGE_LABEL_HALO_WIDTH: f64 = 3.0;
const EDGE_HITBOX_STROKE_WIDTH: usize = 12;
const EDGE_HITBOX_OPACITY: f64 = 0.01;
const EDGE_HOVER_POINT_RADIUS: f64 = 2.4;
const EDGE_HOVER_POINT_STROKE_WIDTH: f64 = 1.0;
const EDGE_TOOLTIP_CSS: &str = r#"
.edge-hover .edge-tooltip {
  opacity: 0;
  pointer-events: none;
  transition: opacity 120ms ease-out;
}
.edge-hover:hover .edge-tooltip {
  opacity: 1;
}
.edge-hover .edge-hover-point {
  opacity: 0.65;
  pointer-events: none;
  transition: opacity 120ms ease-out;
}
.edge-hover:hover .edge-hover-point {
  opacity: 0.95;
}
"#;
const DETOUR_LABEL_CLEARANCE: f64 = 6.0;
const BACK_EDGE_STACK_SPACING: f64 = 16.0;
const BACK_EDGE_NODE_GAP: f64 = 12.0;
const BACK_EDGE_DUP_SPACING: f64 = 6.0;
const BACK_EDGE_SPAN_EPS: f64 = 4.0;
const INTERMEDIATE_X_EPS: f64 = 6.0;
const EDGE_STUB_LEN: f64 = 32.0;
const EDGE_STUB_MIN: f64 = 18.0;
const EDGE_PORT_HANDLE: f64 = 12.0;
const TOOLTIP_FONT_SIZE: usize = 9;
const TOOLTIP_PADDING: f64 = 6.0;
const TOOLTIP_LINE_GAP: f64 = 2.0;
const TOOLTIP_RADIUS: f64 = 3.0;
const TOOLTIP_OFFSET_X: f64 = 12.0;
const TOOLTIP_OFFSET_Y: f64 = 12.0;
const TOOLTIP_BORDER_WIDTH: f64 = 1.0;
const TOOLTIP_BG: &str = "#fff7d1";
const TOOLTIP_BORDER: &str = "#d9c37f";
const TOOLTIP_TEXT: &str = "#111111";
const PORT_DOT_RADIUS: f64 = 2.6;
const PORT_LINE_GAP: f64 = 2.8;
const LEGEND_TITLE_SIZE: usize = 11;
const LEGEND_FONT_SIZE: usize = 10;
const LEGEND_SWATCH_SIZE: f64 = 10.0;
const LEGEND_PADDING: f64 = 8.0;
const LEGEND_CORNER_RADIUS: f64 = 6.0;
const LEGEND_ROW_GAP: f64 = 6.0;
const LEGEND_LINK_GAP: f64 = 3.0;
const LEGEND_WITH_LOGO_GAP: f64 = 4.0;
const LEGEND_VERSION_GAP: f64 = 0.0;
const LEGEND_SECTION_GAP: f64 = 8.0;
const LEGEND_BOTTOM_PADDING: f64 = 6.0;
const LEGEND_LOGO_SIZE: f64 = 16.0;
const LEGEND_TEXT_WIDTH_FACTOR: f64 = 0.52;
const COPPER_GITHUB_URL: &str = "https://github.com/copper-project/copper-rs";
const LEGEND_ITEMS: [(&str, &str); 4] = [
    ("Source", SOURCE_HEADER_BG),
    ("Task", TASK_HEADER_BG),
    ("Sink", SINK_HEADER_BG),
    ("Bridge", BRIDGE_HEADER_BG),
];
const RESOURCE_LEGEND_TITLE: &str = "Resources";
const RESOURCE_LEGEND_ITEMS: [(&str, &str); 3] = [
    ("Exclusive", RESOURCE_EXCLUSIVE_BG),
    ("Shared", RESOURCE_SHARED_BG),
    ("Unused", RESOURCE_UNUSED_BG),
];
const LINUX_RESOURCE_SLOT_NAMES: [&str; 15] = [
    "serial_acm0",
    "serial_acm1",
    "serial_acm2",
    "serial_usb0",
    "serial_usb1",
    "serial_usb2",
    "i2c0",
    "i2c1",
    "i2c2",
    "gpio_out0",
    "gpio_out1",
    "gpio_out2",
    "gpio_in0",
    "gpio_in1",
    "gpio_in2",
];

#[derive(Parser)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Config file name
    #[clap(value_parser)]
    config: PathBuf,
    /// Log statistics JSON file to enrich the DAG
    #[clap(long)]
    logstats: Option<PathBuf>,
    /// Mission id to render (omit to render every mission)
    #[clap(long)]
    mission: Option<String>,
    /// List missions contained in the configuration and exit
    #[clap(long, action)]
    list_missions: bool,
    /// Open the SVG in the default system viewer
    #[clap(long)]
    open: bool,
}

/// Render the configuration file to an SVG and optionally opens it with inkscape.
/// CLI entrypoint that parses args, renders SVG, and optionally opens it.
fn main() -> std::io::Result<()> {
    // Parse command line arguments
    let args = Args::parse();

    let config = read_configuration(args.config.to_str().unwrap())
        .expect("Failed to read configuration file");

    if args.list_missions {
        print_mission_list(&config);
        return Ok(());
    }

    let mission = match validate_mission_arg(&config, args.mission.as_deref()) {
        Ok(mission) => mission,
        Err(err) => {
            eprintln!("{err}");
            std::process::exit(1);
        }
    };

    let logstats = match args.logstats.as_deref() {
        Some(path) => match load_logstats(path, &config, args.mission.as_deref()) {
            Ok(stats) => Some(stats),
            Err(err) => {
                eprintln!("{err}");
                std::process::exit(1);
            }
        },
        None => None,
    };

    let graph_svg = match render_config_svg(&config, mission.as_deref(), logstats.as_ref()) {
        Ok(svg) => svg,
        Err(err) => {
            eprintln!("{err}");
            std::process::exit(1);
        }
    };

    if args.open {
        // Create a temporary file to store the SVG
        let mut temp_file = Builder::new().suffix(".svg").tempfile()?;
        temp_file.write_all(graph_svg.as_slice())?;
        let temp_path = temp_file
            .into_temp_path()
            .keep()
            .map_err(std::io::Error::other)?;

        open_svg(&temp_path)?;
    } else {
        // Write the SVG content to a file
        let mut svg_file = std::fs::File::create("output.svg")?;
        svg_file.write_all(graph_svg.as_slice())?;
    }
    Ok(())
}

/// Hide platform-specific open commands behind a single helper.
fn open_svg(path: &std::path::Path) -> std::io::Result<()> {
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

/// Run the full render pipeline and return SVG bytes for the CLI.
fn render_config_svg(
    config: &config::CuConfig,
    mission_id: Option<&str>,
    logstats: Option<&LogStatsIndex>,
) -> CuResult<Vec<u8>> {
    let sections = build_sections(config, mission_id)?;
    let resource_catalog = collect_resource_catalog(config)?;
    let mut layouts = Vec::new();
    let mut logstats_applied = false;
    for section in sections {
        let section_logstats =
            logstats.filter(|stats| stats.applies_to(section.mission_id.as_deref()));
        if section_logstats.is_some() {
            logstats_applied = true;
        }
        layouts.push(build_section_layout(
            config,
            &section,
            &resource_catalog,
            section_logstats,
        )?);
    }
    if logstats.is_some() && !logstats_applied {
        eprintln!("Warning: logstats did not match any rendered mission");
    }

    Ok(render_sections_to_svg(&layouts).into_bytes())
}

fn load_logstats(
    path: &Path,
    config: &config::CuConfig,
    expected_mission: Option<&str>,
) -> CuResult<LogStatsIndex> {
    let contents = fs::read_to_string(path)
        .map_err(|e| CuError::new_with_cause("Failed to read logstats file", e))?;
    let logstats: LogStats = serde_json::from_str(&contents)
        .map_err(|e| CuError::new_with_cause("Failed to parse logstats JSON", e))?;

    if logstats.schema_version != LOGSTATS_SCHEMA_VERSION {
        eprintln!(
            "Warning: logstats schema version {} does not match renderer {}",
            logstats.schema_version, LOGSTATS_SCHEMA_VERSION
        );
    }

    if let Ok(signature) = build_graph_signature(config, logstats.mission.as_deref()) {
        if signature != logstats.config_signature {
            eprintln!(
                "Warning: logstats signature mismatch (expected {}, got {})",
                signature, logstats.config_signature
            );
        }
    } else {
        eprintln!("Warning: unable to validate logstats signature");
    }

    if expected_mission.is_some()
        && mission_key(expected_mission) != mission_key(logstats.mission.as_deref())
    {
        eprintln!(
            "Warning: logstats mission '{}' does not match requested mission '{}'",
            logstats.mission.as_deref().unwrap_or("default"),
            expected_mission.unwrap_or("default")
        );
    }

    let edge_map = logstats
        .edges
        .into_iter()
        .map(|edge| (EdgeStatsKey::from_edge(&edge), edge))
        .collect();

    Ok(LogStatsIndex {
        mission: logstats.mission,
        edges: edge_map,
        perf: logstats.perf,
    })
}

/// Normalize mission selection into a list of sections to render.
fn build_sections<'a>(
    config: &'a config::CuConfig,
    mission_id: Option<&str>,
) -> CuResult<Vec<SectionRef<'a>>> {
    let sections = match (&config.graphs, mission_id) {
        (ConfigGraphs::Simple(graph), _) => vec![SectionRef {
            label: Some("Default".to_string()),
            mission_id: None,
            graph,
        }],
        (ConfigGraphs::Missions(graphs), Some(id)) => {
            let graph = graphs
                .get(id)
                .ok_or_else(|| CuError::from(format!("Mission {id} not found")))?;
            vec![SectionRef {
                label: Some(id.to_string()),
                mission_id: Some(id.to_string()),
                graph,
            }]
        }
        (ConfigGraphs::Missions(graphs), None) => {
            let mut missions: Vec<_> = graphs.iter().collect();
            missions.sort_by(|a, b| a.0.cmp(b.0));
            missions
                .into_iter()
                .map(|(label, graph)| SectionRef {
                    label: Some(label.clone()),
                    mission_id: Some(label.clone()),
                    graph,
                })
                .collect()
        }
    };

    Ok(sections)
}

/// Convert a config graph into positioned nodes, edges, and port anchors.
fn build_section_layout(
    config: &config::CuConfig,
    section: &SectionRef<'_>,
    resource_catalog: &HashMap<String, BTreeSet<String>>,
    logstats: Option<&LogStatsIndex>,
) -> CuResult<SectionLayout> {
    let mut topology = build_render_topology(section.graph, &config.bridges);
    topology.sort_connections();

    let graph_orientation = Orientation::LeftToRight;
    let node_orientation = graph_orientation.flip();
    let mut graph = VisualGraph::new(graph_orientation);
    let mut node_handles = HashMap::new();
    let mut port_lookups = HashMap::new();
    let mut nodes = Vec::new();

    for node in &topology.nodes {
        let node_idx = section
            .graph
            .get_node_id_by_name(node.id.as_str())
            .ok_or_else(|| CuError::from(format!("Node '{}' missing from graph", node.id)))?;
        let node_weight = section
            .graph
            .get_node(node_idx)
            .ok_or_else(|| CuError::from(format!("Node '{}' missing weight", node.id)))?;

        let is_src = section
            .graph
            .get_dst_edges(node_idx)
            .unwrap_or_default()
            .is_empty();
        let is_sink = section
            .graph
            .get_src_edges(node_idx)
            .unwrap_or_default()
            .is_empty();

        let header_fill = match node.flavor {
            config::Flavor::Bridge => BRIDGE_HEADER_BG,
            config::Flavor::Task if is_src => SOURCE_HEADER_BG,
            config::Flavor::Task if is_sink => SINK_HEADER_BG,
            _ => TASK_HEADER_BG,
        };

        let (table, port_lookup) = build_node_table(node, node_weight, header_fill);
        let record = table_to_record(&table);
        let shape = ShapeKind::Record(record);
        let look = StyleAttr::new(
            Color::fast(BORDER_COLOR),
            1,
            Some(Color::fast("white")),
            0,
            FONT_SIZE,
        );
        let size = record_size(&table, node_orientation);
        let element = Element::create(shape, look, node_orientation, size);
        let handle = graph.add_node(element);

        node_handles.insert(node.id.clone(), handle);
        port_lookups.insert(node.id.clone(), port_lookup);
        nodes.push(NodeRender { handle, table });
    }

    let mut edges = Vec::new();
    let mut edge_groups: HashMap<EdgeGroupKey, usize> = HashMap::new();
    let mut next_color_slot = 0usize;
    let edge_look = StyleAttr::new(Color::fast("black"), 1, None, 0, EDGE_FONT_SIZE);
    for cnx in &topology.connections {
        let src_handle = node_handles
            .get(&cnx.src)
            .ok_or_else(|| CuError::from(format!("Unknown node '{}'", cnx.src)))?;
        let dst_handle = node_handles
            .get(&cnx.dst)
            .ok_or_else(|| CuError::from(format!("Unknown node '{}'", cnx.dst)))?;
        let src_port = port_lookups
            .get(&cnx.src)
            .and_then(|lookup| lookup.resolve_output(cnx.src_port.as_deref()))
            .map(|port| port.to_string());
        let dst_port = port_lookups
            .get(&cnx.dst)
            .and_then(|lookup| lookup.resolve_input(cnx.dst_port.as_deref()))
            .map(|port| port.to_string());

        let arrow = Arrow::new(
            LineEndKind::None,
            LineEndKind::Arrow,
            LineStyleKind::Normal,
            "",
            &edge_look,
            &src_port,
            &dst_port,
        );
        graph.add_edge(arrow.clone(), *src_handle, *dst_handle);
        let edge_stats = logstats.and_then(|stats| stats.edge_stats_for(cnx));
        let group_key = EdgeGroupKey {
            src: *src_handle,
            src_port: src_port.clone(),
            msg: cnx.msg.clone(),
        };
        let (color_idx, show_label) = match edge_groups.entry(group_key) {
            Entry::Occupied(entry) => (*entry.get(), false),
            Entry::Vacant(entry) => {
                let color_idx = edge_cycle_color_index(&mut next_color_slot);
                entry.insert(color_idx);
                (color_idx, true)
            }
        };
        edges.push(RenderEdge {
            src: *src_handle,
            dst: *dst_handle,
            arrow,
            label: if show_label {
                cnx.msg.clone()
            } else {
                String::new()
            },
            color_idx,
            src_port,
            dst_port,
            stats: edge_stats,
        });
    }

    let mut null_backend = NullBackend;
    graph.do_it(false, false, false, &mut null_backend);
    scale_layout_positions(&mut graph);

    let node_bounds = collect_node_bounds(&nodes, &graph);
    reorder_auto_input_rows(&mut nodes, &topology, &node_handles, &node_bounds, &graph);

    let mut min = Point::new(f64::INFINITY, f64::INFINITY);
    let mut max = Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for node in &nodes {
        let pos = graph.element(node.handle).position();
        let (top_left, bottom_right) = pos.bbox(false);
        min.x = min.x.min(top_left.x);
        min.y = min.y.min(top_left.y);
        max.x = max.x.max(bottom_right.x);
        max.y = max.y.max(bottom_right.y);
    }
    if !min.x.is_finite() || !min.y.is_finite() {
        min = Point::new(0.0, 0.0);
        max = Point::new(0.0, 0.0);
    }

    let mut port_anchors = HashMap::new();
    for node in &nodes {
        let element = graph.element(node.handle);
        let anchors = collect_port_anchors(node, element);
        port_anchors.insert(node.handle, anchors);
    }

    let resource_tables = build_resource_tables(config, section, resource_catalog)?;
    let perf_table = logstats.map(|stats| build_perf_table(&stats.perf));

    Ok(SectionLayout {
        label: section.label.clone(),
        graph,
        nodes,
        edges,
        bounds: (min, max),
        port_anchors,
        resource_tables,
        perf_table,
    })
}

fn collect_resource_catalog(
    config: &config::CuConfig,
) -> CuResult<HashMap<String, BTreeSet<String>>> {
    let bundle_ids: HashSet<String> = config
        .resources
        .iter()
        .map(|bundle| bundle.id.clone())
        .collect();
    let mut catalog: HashMap<String, BTreeSet<String>> = HashMap::new();

    let mut collect_graph = |graph: &config::CuGraph| -> CuResult<()> {
        for (_, node) in graph.get_all_nodes() {
            let Some(resources) = node.get_resources() else {
                continue;
            };
            for path in resources.values() {
                let (bundle_id, resource_name) = parse_resource_path(path)?;
                if !bundle_ids.contains(&bundle_id) {
                    return Err(CuError::from(format!(
                        "Resource '{}' references unknown bundle '{}'",
                        path, bundle_id
                    )));
                }
                catalog.entry(bundle_id).or_default().insert(resource_name);
            }
        }
        Ok(())
    };

    match &config.graphs {
        ConfigGraphs::Simple(graph) => collect_graph(graph)?,
        ConfigGraphs::Missions(graphs) => {
            for graph in graphs.values() {
                collect_graph(graph)?;
            }
        }
    }

    for bundle in &config.resources {
        let Some(resource_names) = provider_resource_slots(bundle.provider.as_str()) else {
            continue;
        };
        let bundle_resources = catalog.entry(bundle.id.clone()).or_default();
        for resource_name in resource_names {
            bundle_resources.insert((*resource_name).to_string());
        }
    }

    Ok(catalog)
}

fn build_resource_tables(
    config: &config::CuConfig,
    section: &SectionRef<'_>,
    resource_catalog: &HashMap<String, BTreeSet<String>>,
) -> CuResult<Vec<ResourceTable>> {
    let owners_by_bundle = collect_graph_resource_owners(section.graph)?;
    let mission_id = section.mission_id.as_deref();
    let mut tables = Vec::new();

    for bundle in &config.resources {
        if !bundle_applies(&bundle.missions, mission_id) {
            continue;
        }
        let resources = resource_catalog
            .get(&bundle.id)
            .map(|set| set.iter().cloned().collect::<Vec<_>>())
            .unwrap_or_default();
        let table = build_resource_table(bundle, &resources, owners_by_bundle.get(&bundle.id));
        let size = record_size(&table, Orientation::TopToBottom);
        tables.push(ResourceTable { table, size });
    }

    Ok(tables)
}

fn build_resource_table(
    bundle: &config::ResourceBundleConfig,
    resources: &[String],
    owners_by_resource: Option<&HashMap<String, Vec<ResourceOwner>>>,
) -> TableNode {
    let mut rows = Vec::new();
    let provider_label = wrap_type_label(
        &strip_type_params(bundle.provider.as_str()),
        TYPE_WRAP_WIDTH,
    );
    let header_lines = vec![
        CellLine::new(format!("Bundle: {}", bundle.id), "black", true, FONT_SIZE),
        CellLine::code(provider_label, DIM_GRAY, false, TYPE_FONT_SIZE),
    ];
    rows.push(TableNode::Cell(
        TableCell::new(header_lines)
            .with_background(RESOURCE_TITLE_BG)
            .with_align(TextAlign::Center),
    ));

    let mut resource_column = Vec::new();
    let mut users_column = Vec::new();
    resource_column.push(TableNode::Cell(
        TableCell::single_line_sized("Resource", "black", false, PORT_HEADER_FONT_SIZE)
            .with_background(HEADER_BG)
            .with_align(TextAlign::Left),
    ));
    users_column.push(TableNode::Cell(
        TableCell::single_line_sized("Used by", "black", false, PORT_HEADER_FONT_SIZE)
            .with_background(HEADER_BG)
            .with_align(TextAlign::Left),
    ));

    if resources.is_empty() {
        let resource_cell =
            TableCell::single_line_sized(PLACEHOLDER_TEXT, LIGHT_GRAY, false, PORT_VALUE_FONT_SIZE)
                .with_background(RESOURCE_UNUSED_BG)
                .with_border_width(VALUE_BORDER_WIDTH)
                .with_align(TextAlign::Left);
        let owners_cell = TableCell::single_line_sized(
            "unused",
            RESOURCE_UNUSED_TEXT,
            false,
            PORT_VALUE_FONT_SIZE,
        )
        .with_border_width(VALUE_BORDER_WIDTH)
        .with_align(TextAlign::Left);
        resource_column.push(TableNode::Cell(resource_cell));
        users_column.push(TableNode::Cell(owners_cell));
    } else {
        for resource in resources {
            let owners = owners_by_resource
                .and_then(|map| map.get(resource))
                .cloned()
                .unwrap_or_default();
            let usage = resource_usage(&owners);
            let resource_label = format!("{}.{}", bundle.id, resource);
            let resource_cell = TableCell::new(vec![CellLine::code(
                resource_label,
                "black",
                false,
                PORT_VALUE_FONT_SIZE,
            )])
            .with_background(resource_usage_color(usage))
            .with_border_width(VALUE_BORDER_WIDTH)
            .with_align(TextAlign::Left);
            let owners_cell = TableCell::new(format_resource_owners(&owners, usage))
                .with_border_width(VALUE_BORDER_WIDTH)
                .with_align(TextAlign::Left);
            resource_column.push(TableNode::Cell(resource_cell));
            users_column.push(TableNode::Cell(owners_cell));
        }
    }

    rows.push(TableNode::Array(vec![
        TableNode::Array(resource_column),
        TableNode::Array(users_column),
    ]));

    TableNode::Array(rows)
}

fn build_perf_table(perf: &PerfStats) -> ResourceTable {
    let header_lines = vec![CellLine::new("Log Performance", "black", true, FONT_SIZE)];
    let mut rows = Vec::new();
    rows.push(TableNode::Cell(
        TableCell::new(header_lines)
            .with_background(PERF_TITLE_BG)
            .with_align(TextAlign::Center),
    ));

    let mut metric_column = Vec::new();
    let mut value_column = Vec::new();
    metric_column.push(TableNode::Cell(
        TableCell::single_line_sized("Metric", "black", false, PORT_HEADER_FONT_SIZE)
            .with_background(HEADER_BG)
            .with_align(TextAlign::Left),
    ));
    value_column.push(TableNode::Cell(
        TableCell::single_line_sized("Value", "black", false, PORT_HEADER_FONT_SIZE)
            .with_background(HEADER_BG)
            .with_align(TextAlign::Left),
    ));

    let sample_text = format!("{}/{}", perf.valid_time_samples, perf.samples);
    let metrics = [
        ("Samples (valid/total)", sample_text),
        (
            "End-to-end mean",
            format_duration_ns_f64(perf.end_to_end.mean_ns),
        ),
        (
            "End-to-end min",
            format_duration_ns_u64(perf.end_to_end.min_ns),
        ),
        (
            "End-to-end max",
            format_duration_ns_u64(perf.end_to_end.max_ns),
        ),
        (
            "End-to-end sigma",
            format_duration_ns_f64(perf.end_to_end.stddev_ns),
        ),
        ("Jitter mean", format_duration_ns_f64(perf.jitter.mean_ns)),
        (
            "Jitter sigma",
            format_duration_ns_f64(perf.jitter.stddev_ns),
        ),
    ];

    for (label, value) in metrics {
        metric_column.push(TableNode::Cell(
            TableCell::single_line_sized(label, "black", false, PORT_VALUE_FONT_SIZE)
                .with_border_width(VALUE_BORDER_WIDTH)
                .with_align(TextAlign::Left),
        ));
        value_column.push(TableNode::Cell(
            TableCell::single_line_sized(&value, "black", false, PORT_VALUE_FONT_SIZE)
                .with_border_width(VALUE_BORDER_WIDTH)
                .with_align(TextAlign::Left),
        ));
    }

    rows.push(TableNode::Array(vec![
        TableNode::Array(metric_column),
        TableNode::Array(value_column),
    ]));

    let table = TableNode::Array(rows);
    let size = record_size(&table, Orientation::TopToBottom);
    ResourceTable { table, size }
}

fn collect_graph_resource_owners(
    graph: &config::CuGraph,
) -> CuResult<HashMap<String, HashMap<String, Vec<ResourceOwner>>>> {
    let mut owners: HashMap<String, HashMap<String, Vec<ResourceOwner>>> = HashMap::new();
    for (_, node) in graph.get_all_nodes() {
        let Some(resources) = node.get_resources() else {
            continue;
        };
        let owner = ResourceOwner {
            name: node.get_id(),
            flavor: node.get_flavor(),
        };
        for path in resources.values() {
            let (bundle_id, resource_name) = parse_resource_path(path)?;
            owners
                .entry(bundle_id)
                .or_default()
                .entry(resource_name)
                .or_default()
                .push(owner.clone());
        }
    }

    for bundle in owners.values_mut() {
        for list in bundle.values_mut() {
            dedup_owners(list);
        }
    }

    Ok(owners)
}

fn dedup_owners(owners: &mut Vec<ResourceOwner>) {
    owners.sort_by(|a, b| {
        flavor_rank(a.flavor)
            .cmp(&flavor_rank(b.flavor))
            .then_with(|| a.name.cmp(&b.name))
    });
    owners.dedup_by(|a, b| a.flavor == b.flavor && a.name == b.name);
}

fn flavor_rank(flavor: config::Flavor) -> u8 {
    match flavor {
        config::Flavor::Task => 0,
        config::Flavor::Bridge => 1,
    }
}

fn resource_usage(owners: &[ResourceOwner]) -> ResourceUsage {
    match owners.len() {
        0 => ResourceUsage::Unused,
        1 => ResourceUsage::Exclusive,
        _ => ResourceUsage::Shared,
    }
}

fn resource_usage_color(usage: ResourceUsage) -> &'static str {
    match usage {
        ResourceUsage::Exclusive => RESOURCE_EXCLUSIVE_BG,
        ResourceUsage::Shared => RESOURCE_SHARED_BG,
        ResourceUsage::Unused => RESOURCE_UNUSED_BG,
    }
}

fn format_resource_owners(owners: &[ResourceOwner], usage: ResourceUsage) -> Vec<CellLine> {
    if owners.is_empty() && matches!(usage, ResourceUsage::Unused) {
        return vec![CellLine::new(
            "unused",
            RESOURCE_UNUSED_TEXT,
            false,
            PORT_VALUE_FONT_SIZE,
        )];
    }

    owners
        .iter()
        .map(|owner| {
            let (label, color) = match owner.flavor {
                config::Flavor::Task => (format!("task: {}", owner.name), "black"),
                config::Flavor::Bridge => (format!("bridge: {}", owner.name), DIM_GRAY),
            };
            CellLine::code(label, color, false, PORT_VALUE_FONT_SIZE)
        })
        .collect()
}

fn bundle_applies(missions: &Option<Vec<String>>, mission_id: Option<&str>) -> bool {
    match mission_id {
        None => true,
        Some(id) => missions
            .as_ref()
            .map(|list| list.iter().any(|m| m == id))
            .unwrap_or(true),
    }
}

fn parse_resource_path(path: &str) -> CuResult<(String, String)> {
    let (bundle_id, name) = path.split_once('.').ok_or_else(|| {
        CuError::from(format!(
            "Resource '{path}' is missing a bundle prefix (expected bundle.resource)"
        ))
    })?;

    if bundle_id.is_empty() || name.is_empty() {
        return Err(CuError::from(format!(
            "Resource '{path}' must use the 'bundle.resource' format"
        )));
    }

    Ok((bundle_id.to_string(), name.to_string()))
}

/// Build the record table for a node and capture port ids for routing.
fn build_node_table(
    node: &config::RenderNode,
    node_weight: &config::Node,
    header_fill: &str,
) -> (TableNode, PortLookup) {
    let mut rows = Vec::new();

    let header_lines = vec![
        CellLine::new(node.id.clone(), "black", true, FONT_SIZE),
        CellLine::code(
            wrap_type_label(&strip_type_params(&node.type_name), TYPE_WRAP_WIDTH),
            DIM_GRAY,
            false,
            TYPE_FONT_SIZE,
        ),
    ];
    rows.push(TableNode::Cell(
        TableCell::new(header_lines)
            .with_background(header_fill)
            .with_align(TextAlign::Center),
    ));

    let mut port_lookup = PortLookup::default();
    let max_ports = node.inputs.len().max(node.outputs.len());
    let inputs = build_port_column(
        "Inputs",
        &node.inputs,
        "in",
        &mut port_lookup.inputs,
        &mut port_lookup.default_input,
        max_ports,
        TextAlign::Left,
    );
    let outputs = build_port_column(
        "Outputs",
        &node.outputs,
        "out",
        &mut port_lookup.outputs,
        &mut port_lookup.default_output,
        max_ports,
        TextAlign::Right,
    );
    rows.push(TableNode::Array(vec![inputs, outputs]));

    if let Some(config) = node_weight.get_instance_config() {
        let config_rows = build_config_rows(config);
        if !config_rows.is_empty() {
            rows.extend(config_rows);
        }
    }

    (TableNode::Array(rows), port_lookup)
}

/// Keep input/output rows aligned and generate stable port identifiers.
fn build_port_column(
    title: &str,
    names: &[String],
    prefix: &str,
    lookup: &mut HashMap<String, String>,
    default_port: &mut Option<String>,
    target_len: usize,
    align: TextAlign,
) -> TableNode {
    let mut rows = Vec::new();
    rows.push(TableNode::Cell(
        TableCell::single_line_sized(title, "black", false, PORT_HEADER_FONT_SIZE)
            .with_background(HEADER_BG)
            .with_align(align),
    ));

    let desired_rows = target_len.max(1);
    for idx in 0..desired_rows {
        if let Some(name) = names.get(idx) {
            let port_id = format!("{prefix}_{idx}");
            lookup.insert(name.clone(), port_id.clone());
            if default_port.is_none() {
                *default_port = Some(port_id.clone());
            }
            rows.push(TableNode::Cell(
                TableCell::single_line_sized(name, "black", false, PORT_VALUE_FONT_SIZE)
                    .with_port(port_id)
                    .with_border_width(VALUE_BORDER_WIDTH)
                    .with_align(align),
            ));
        } else {
            rows.push(TableNode::Cell(
                TableCell::single_line_sized(
                    PLACEHOLDER_TEXT,
                    LIGHT_GRAY,
                    false,
                    PORT_VALUE_FONT_SIZE,
                )
                .with_border_width(VALUE_BORDER_WIDTH)
                .with_align(align),
            ));
        }
    }

    TableNode::Array(rows)
}

/// Render config entries in a stable order for readability and diffs.
fn build_config_rows(config: &config::ComponentConfig) -> Vec<TableNode> {
    if config.0.is_empty() {
        return Vec::new();
    }

    let mut entries: Vec<_> = config.0.iter().collect();
    entries.sort_by(|a, b| a.0.cmp(b.0));

    let header = TableNode::Cell(
        TableCell::single_line_sized("Config", "black", false, PORT_HEADER_FONT_SIZE)
            .with_background(HEADER_BG),
    );

    let mut key_lines = Vec::new();
    let mut value_lines = Vec::new();
    for (key, value) in entries {
        let value_str = wrap_text(&format!("{value}"), CONFIG_WRAP_WIDTH);
        let value_parts: Vec<_> = value_str.split('\n').collect();
        for (idx, part) in value_parts.iter().enumerate() {
            let key_text = if idx == 0 { key.as_str() } else { "" };
            key_lines.push(CellLine::code(key_text, DIM_GRAY, true, CONFIG_FONT_SIZE));
            value_lines.push(CellLine::code(*part, DIM_GRAY, false, CONFIG_FONT_SIZE));
        }
    }

    let keys_cell = TableCell::new(key_lines).with_border_width(VALUE_BORDER_WIDTH);
    let values_cell = TableCell::new(value_lines).with_border_width(VALUE_BORDER_WIDTH);
    let body = TableNode::Array(vec![
        TableNode::Cell(keys_cell),
        TableNode::Cell(values_cell),
    ]);

    vec![header, body]
}

/// Adapt our table tree into the layout-rs record format.
fn table_to_record(node: &TableNode) -> RecordDef {
    match node {
        TableNode::Cell(cell) => RecordDef::Text(cell.label(), cell.port.clone()),
        TableNode::Array(children) => {
            RecordDef::Array(children.iter().map(table_to_record).collect())
        }
    }
}

/// Estimate record size before layout so edges and clusters can be sized.
fn record_size(node: &TableNode, dir: Orientation) -> Point {
    match node {
        TableNode::Cell(cell) => pad_shape_scalar(cell_text_size(cell), BOX_SHAPE_PADDING),
        TableNode::Array(children) => {
            if children.is_empty() {
                return Point::new(1.0, 1.0);
            }
            let mut x: f64 = 0.0;
            let mut y: f64 = 0.0;
            for child in children {
                let sz = record_size(child, dir.flip());
                if dir.is_left_right() {
                    x += sz.x;
                    y = y.max(sz.y);
                } else {
                    x = x.max(sz.x);
                    y += sz.y;
                }
            }
            Point::new(x, y)
        }
    }
}

/// Walk table cells to compute positions and collect port anchors.
fn visit_table(
    node: &TableNode,
    dir: Orientation,
    loc: Point,
    size: Point,
    visitor: &mut dyn TableVisitor,
) {
    match node {
        TableNode::Cell(cell) => {
            visitor.handle_cell(cell, loc, size);
        }
        TableNode::Array(children) => {
            if children.is_empty() {
                return;
            }

            let mut sizes = Vec::new();
            let mut sum = Point::new(0.0, 0.0);

            for child in children {
                let child_size = record_size(child, dir.flip());
                sizes.push(child_size);
                if dir.is_left_right() {
                    sum.x += child_size.x;
                    sum.y = sum.y.max(child_size.y);
                } else {
                    sum.x = sum.x.max(child_size.x);
                    sum.y += child_size.y;
                }
            }

            for child_size in &mut sizes {
                if dir.is_left_right() {
                    if sum.x > 0.0 {
                        *child_size = Point::new(size.x * child_size.x / sum.x, size.y);
                    } else {
                        *child_size = Point::new(1.0, size.y);
                    }
                } else if sum.y > 0.0 {
                    *child_size = Point::new(size.x, size.y * child_size.y / sum.y);
                } else {
                    *child_size = Point::new(size.x, 1.0);
                }
            }

            if dir.is_left_right() {
                let mut start_x = loc.x - size.x / 2.0;
                for (idx, child) in children.iter().enumerate() {
                    let child_loc = Point::new(start_x + sizes[idx].x / 2.0, loc.y);
                    visit_table(child, dir.flip(), child_loc, sizes[idx], visitor);
                    start_x += sizes[idx].x;
                }
            } else {
                let mut start_y = loc.y - size.y / 2.0;
                for (idx, child) in children.iter().enumerate() {
                    let child_loc = Point::new(loc.x, start_y + sizes[idx].y / 2.0);
                    visit_table(child, dir.flip(), child_loc, sizes[idx], visitor);
                    start_y += sizes[idx].y;
                }
            }
        }
    }
}

fn reorder_auto_input_rows(
    nodes: &mut [NodeRender],
    topology: &config::RenderTopology,
    node_handles: &HashMap<String, NodeHandle>,
    node_bounds: &[NodeBounds],
    graph: &VisualGraph,
) {
    let mut inputs_by_id = HashMap::new();
    for node in &topology.nodes {
        inputs_by_id.insert(node.id.clone(), node.inputs.clone());
    }

    let mut order_info_by_dst: HashMap<String, HashMap<String, (usize, f64)>> = HashMap::new();
    for cnx in &topology.connections {
        let Some(dst_port) = cnx.dst_port.as_ref() else {
            continue;
        };
        let (Some(src_handle), Some(dst_handle)) =
            (node_handles.get(&cnx.src), node_handles.get(&cnx.dst))
        else {
            continue;
        };
        let src_pos = graph.element(*src_handle).position().center();
        let dst_pos = graph.element(*dst_handle).position().center();
        let span_min_x = src_pos.x.min(dst_pos.x);
        let span_max_x = src_pos.x.max(dst_pos.x);
        let is_self = src_handle == dst_handle;
        let has_intermediate = !is_self
            && span_has_intermediate(
                node_bounds,
                span_min_x,
                span_max_x,
                *src_handle,
                *dst_handle,
            );
        let is_reverse = src_pos.x > dst_pos.x;
        let is_detour = !is_self && (is_reverse || has_intermediate);
        let detour_above = is_detour && !is_reverse;
        let group_rank = if detour_above { 0 } else { 1 };
        order_info_by_dst
            .entry(cnx.dst.clone())
            .or_default()
            .insert(dst_port.clone(), (group_rank, src_pos.y));
    }

    let mut handle_to_id = HashMap::new();
    for (id, handle) in node_handles {
        handle_to_id.insert(*handle, id.clone());
    }

    for node in nodes {
        let Some(node_id) = handle_to_id.get(&node.handle) else {
            continue;
        };
        let Some(inputs) = inputs_by_id.get(node_id) else {
            continue;
        };
        if inputs.len() <= 1 {
            continue;
        }
        let Some(order_info) = order_info_by_dst.get(node_id) else {
            continue;
        };
        if order_info.len() < 2 {
            continue;
        }

        let mut indexed: Vec<_> = inputs
            .iter()
            .enumerate()
            .map(|(idx, label)| {
                let (group_rank, src_y) = order_info.get(label).copied().unwrap_or((2, 0.0));
                (group_rank, src_y, idx, label.clone())
            })
            .collect();
        indexed.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
                .then_with(|| a.2.cmp(&b.2))
        });

        let mut order = HashMap::new();
        for (pos, (_, _, _, label)) in indexed.into_iter().enumerate() {
            order.insert(label, pos);
        }
        reorder_input_rows(&mut node.table, &order);
    }
}

fn reorder_input_rows(table: &mut TableNode, order: &HashMap<String, usize>) {
    let TableNode::Array(rows) = table else {
        return;
    };
    if rows.len() < 2 {
        return;
    }
    let TableNode::Array(columns) = &mut rows[1] else {
        return;
    };
    if columns.is_empty() {
        return;
    }
    let TableNode::Array(input_rows) = &mut columns[0] else {
        return;
    };
    if input_rows.len() <= 2 {
        return;
    }

    let header = input_rows[0].clone();
    let mut inputs = Vec::new();
    let mut placeholders = Vec::new();
    for row in input_rows.iter().skip(1) {
        match row {
            TableNode::Cell(cell) if cell.port.is_some() => {
                let label = cell.label();
                let key = *order.get(&label).unwrap_or(&usize::MAX);
                inputs.push((key, row.clone()));
            }
            _ => placeholders.push(row.clone()),
        }
    }
    if inputs.len() <= 1 {
        return;
    }
    inputs.sort_by(|a, b| a.0.cmp(&b.0));
    let mut new_rows = Vec::with_capacity(input_rows.len());
    new_rows.push(header);
    for (_, row) in inputs {
        new_rows.push(row);
    }
    for row in placeholders {
        new_rows.push(row);
    }
    *input_rows = new_rows;
}

/// Render each section and merge them into a single SVG canvas.
fn render_sections_to_svg(sections: &[SectionLayout]) -> String {
    let mut svg = SvgWriter::new();
    let mut cursor_y = GRAPH_MARGIN;
    let mut last_section_bottom = GRAPH_MARGIN;
    let mut last_section_right = GRAPH_MARGIN;

    for section in sections {
        let cluster_margin = if section.label.is_some() {
            CLUSTER_MARGIN
        } else {
            0.0
        };
        let (min, max) = section.bounds;
        let label_padding = if section.label.is_some() {
            FONT_SIZE as f64
        } else {
            0.0
        };
        let node_bounds = collect_node_bounds(&section.nodes, &section.graph);
        let mut expanded_bounds = (min, max);
        let mut edge_paths: Vec<Vec<BezierSegment>> = Vec::with_capacity(section.edges.len());
        let mut edge_points: Vec<(Point, Point)> = Vec::with_capacity(section.edges.len());
        let mut edge_is_self: Vec<bool> = Vec::with_capacity(section.edges.len());
        let mut edge_is_detour: Vec<bool> = Vec::with_capacity(section.edges.len());
        let mut detour_above = vec![false; section.edges.len()];
        let mut detour_base_y = vec![0.0; section.edges.len()];
        let mut back_plans_above: Vec<BackEdgePlan> = Vec::new();
        let mut back_plans_below: Vec<BackEdgePlan> = Vec::new();

        for (idx, edge) in section.edges.iter().enumerate() {
            let src_point = resolve_anchor(section, edge.src, edge.src_port.as_ref());
            let dst_point = resolve_anchor(section, edge.dst, edge.dst_port.as_ref());
            let span_min_x = src_point.x.min(dst_point.x);
            let span_max_x = src_point.x.max(dst_point.x);
            let is_self = edge.src == edge.dst;
            let has_intermediate = !is_self
                && span_has_intermediate(&node_bounds, span_min_x, span_max_x, edge.src, edge.dst);
            let is_reverse = src_point.x > dst_point.x;
            let is_detour = !is_self && (is_reverse || has_intermediate);
            edge_points.push((src_point, dst_point));
            edge_is_self.push(is_self);
            edge_is_detour.push(is_detour);

            if is_detour {
                let span = (src_point.x - dst_point.x).abs();
                let above = !is_reverse;
                let base_y = if above {
                    min_top_for_span(&node_bounds, span_min_x, span_max_x) - BACK_EDGE_NODE_GAP
                } else {
                    max_bottom_for_span(&node_bounds, span_min_x, span_max_x) + BACK_EDGE_NODE_GAP
                };
                detour_above[idx] = above;
                detour_base_y[idx] = base_y;
                let plan = BackEdgePlan {
                    idx,
                    span,
                    order_y: dst_point.y,
                };
                if above {
                    back_plans_above.push(plan);
                } else {
                    back_plans_below.push(plan);
                }
            }
        }

        let mut back_offsets = vec![0.0; section.edges.len()];
        assign_back_edge_offsets(&back_plans_below, &mut back_offsets);
        assign_back_edge_offsets(&back_plans_above, &mut back_offsets);
        let mut detour_lane_y = vec![0.0; section.edges.len()];
        for idx in 0..section.edges.len() {
            if edge_is_detour[idx] {
                detour_lane_y[idx] = if detour_above[idx] {
                    detour_base_y[idx] - back_offsets[idx]
                } else {
                    detour_base_y[idx] + back_offsets[idx]
                };
            }
        }
        let detour_slots =
            build_detour_label_slots(&edge_points, &edge_is_detour, &detour_above, &detour_lane_y);

        for (idx, edge) in section.edges.iter().enumerate() {
            let (src_point, dst_point) = edge_points[idx];
            let (fallback_start_dir, fallback_end_dir) = fallback_port_dirs(src_point, dst_point);
            let start_dir = port_dir(edge.src_port.as_ref()).unwrap_or(fallback_start_dir);
            let end_dir = port_dir_incoming(edge.dst_port.as_ref()).unwrap_or(fallback_end_dir);
            let path = if edge.src == edge.dst {
                let pos = section.graph.element(edge.src).position();
                let bbox = pos.bbox(false);
                build_loop_path(src_point, dst_point, bbox, start_dir, end_dir)
            } else if edge_is_detour[idx] {
                build_back_edge_path(src_point, dst_point, detour_lane_y[idx], start_dir, end_dir)
            } else {
                build_edge_path(src_point, dst_point, start_dir, end_dir)
            };

            for segment in &path {
                expand_bounds(&mut expanded_bounds, segment.start);
                expand_bounds(&mut expanded_bounds, segment.c1);
                expand_bounds(&mut expanded_bounds, segment.c2);
                expand_bounds(&mut expanded_bounds, segment.end);
            }

            edge_paths.push(path);
        }

        let mut info_table_positions: Vec<(Point, &ResourceTable)> = Vec::new();
        if section.perf_table.is_some() || !section.resource_tables.is_empty() {
            let content_left = expanded_bounds.0.x;
            let content_bottom = expanded_bounds.1.y;
            let mut max_table_width: f64 = 0.0;
            let mut cursor_table_y = content_bottom + RESOURCE_TABLE_MARGIN;
            for table in section
                .perf_table
                .iter()
                .chain(section.resource_tables.iter())
            {
                let top_left = Point::new(content_left, cursor_table_y);
                info_table_positions.push((top_left, table));
                cursor_table_y += table.size.y + RESOURCE_TABLE_GAP;
                max_table_width = max_table_width.max(table.size.x);
            }
            let tables_bottom = cursor_table_y - RESOURCE_TABLE_GAP;
            expanded_bounds.1.y = expanded_bounds.1.y.max(tables_bottom);
            expanded_bounds.1.x = expanded_bounds.1.x.max(content_left + max_table_width);
        }

        let section_min = Point::new(
            expanded_bounds.0.x - cluster_margin,
            expanded_bounds.0.y - cluster_margin,
        );
        let section_max = Point::new(
            expanded_bounds.1.x + cluster_margin,
            expanded_bounds.1.y + cluster_margin + label_padding,
        );
        let offset = Point::new(GRAPH_MARGIN - section_min.x, cursor_y - section_min.y);
        let content_offset = offset.add(Point::new(0.0, label_padding));
        let cluster_top_left = section_min.add(offset);
        let cluster_bottom_right = section_max.add(offset);
        last_section_bottom = last_section_bottom.max(cluster_bottom_right.y);
        last_section_right = last_section_right.max(cluster_bottom_right.x);
        let label_bounds_min = Point::new(
            cluster_top_left.x + 4.0,
            cluster_top_left.y + label_padding + 4.0,
        );
        let label_bounds_max =
            Point::new(cluster_bottom_right.x - 4.0, cluster_bottom_right.y - 4.0);

        if let Some(label) = &section.label {
            draw_cluster(&mut svg, section_min, section_max, label, offset);
        }

        let mut blocked_boxes: Vec<(Point, Point)> = node_bounds
            .iter()
            .map(|b| {
                (
                    Point::new(b.left, b.top)
                        .add(content_offset)
                        .sub(Point::new(4.0, 4.0)),
                    Point::new(b.right, b.bottom)
                        .add(content_offset)
                        .add(Point::new(4.0, 4.0)),
                )
            })
            .collect();
        if let Some(label) = &section.label {
            let label_text = format!("Mission: {label}");
            let label_size = get_size_for_str(&label_text, FONT_SIZE);
            let label_pos = Point::new(
                section_min.x + offset.x + CELL_PADDING,
                section_min.y + offset.y + FONT_SIZE as f64,
            );
            blocked_boxes.push((
                Point::new(label_pos.x, label_pos.y - label_size.y / 2.0).sub(Point::new(2.0, 2.0)),
                Point::new(label_pos.x + label_size.x, label_pos.y + label_size.y / 2.0)
                    .add(Point::new(2.0, 2.0)),
            ));
        }

        for (top_left, table) in &info_table_positions {
            let top_left = top_left.add(content_offset);
            let bottom_right = Point::new(top_left.x + table.size.x, top_left.y + table.size.y);
            blocked_boxes.push((
                top_left.sub(Point::new(4.0, 4.0)),
                bottom_right.add(Point::new(4.0, 4.0)),
            ));
        }

        let straight_slots =
            build_straight_label_slots(&edge_points, &edge_is_detour, &edge_is_self);

        for ((idx, edge), path) in section.edges.iter().enumerate().zip(edge_paths.iter()) {
            let path = path
                .iter()
                .map(|seg| BezierSegment {
                    start: seg.start.add(content_offset),
                    c1: seg.c1.add(content_offset),
                    c2: seg.c2.add(content_offset),
                    end: seg.end.add(content_offset),
                })
                .collect::<Vec<_>>();
            let dashed = matches!(
                edge.arrow.line_style,
                LineStyleKind::Dashed | LineStyleKind::Dotted
            );
            let start = matches!(edge.arrow.start, LineEndKind::Arrow);
            let end = matches!(edge.arrow.end, LineEndKind::Arrow);
            let line_color = EDGE_COLOR_PALETTE[edge.color_idx];
            let label = if edge.label.is_empty() {
                None
            } else {
                let (text, font_size) = if edge_is_self[idx] {
                    fit_edge_label(&edge.label, &path, EDGE_FONT_SIZE)
                } else if let Some(slot) = straight_slots.get(&idx) {
                    let mut max_width = slot.width;
                    if slot.group_count <= 1 {
                        let path_width = approximate_path_length(&path);
                        max_width = max_width.max(path_width);
                    }
                    fit_label_to_width(&edge.label, max_width, EDGE_FONT_SIZE)
                } else if let Some(slot) = detour_slots.get(&idx) {
                    let mut max_width = slot.width;
                    if slot.group_count <= 1 {
                        if let Some((_, _, lane_len)) = find_horizontal_lane_span(&path) {
                            max_width = max_width.max(lane_len);
                        } else if slot.group_width > 0.0 {
                            max_width = max_width.max(slot.group_width * 0.9);
                        }
                    }
                    fit_label_to_width(&edge.label, max_width, EDGE_FONT_SIZE)
                } else if edge_is_detour[idx] {
                    let (lane_left, lane_right) =
                        detour_lane_bounds_from_points(edge_points[idx].0, edge_points[idx].1);
                    fit_label_to_width(
                        &edge.label,
                        (lane_right - lane_left).max(1.0),
                        EDGE_FONT_SIZE,
                    )
                } else {
                    fit_edge_label(&edge.label, &path, EDGE_FONT_SIZE)
                };
                let label_color = lighten_hex(line_color, EDGE_LABEL_LIGHTEN);
                let mut label =
                    ArrowLabel::new(text, &label_color, font_size, true, FontFamily::Mono);
                let label_pos = if edge_is_self[idx] {
                    let node_center = section
                        .graph
                        .element(edge.src)
                        .position()
                        .center()
                        .add(content_offset);
                    if let Some((center_x, lane_y, _)) = find_horizontal_lane_span(&path) {
                        let above = lane_y < node_center.y;
                        place_detour_label(
                            &label.text,
                            label.font_size,
                            center_x,
                            lane_y,
                            above,
                            &blocked_boxes,
                        )
                    } else {
                        place_self_loop_label(
                            &label.text,
                            label.font_size,
                            &path,
                            node_center,
                            &blocked_boxes,
                        )
                    }
                } else if edge_is_detour[idx] {
                    let mut label_pos = None;
                    if let Some(slot) = detour_slots.get(&idx) {
                        label_pos = Some(place_detour_label(
                            &label.text,
                            label.font_size,
                            slot.center_x + content_offset.x,
                            slot.lane_y + content_offset.y,
                            slot.above,
                            &blocked_boxes,
                        ));
                    }
                    if label_pos.is_none() {
                        label_pos = Some(place_detour_label(
                            &label.text,
                            label.font_size,
                            (edge_points[idx].0.x + edge_points[idx].1.x) / 2.0 + content_offset.x,
                            detour_lane_y[idx] + content_offset.y,
                            detour_above[idx],
                            &blocked_boxes,
                        ));
                    }
                    if let Some(pos) = label_pos {
                        pos
                    } else {
                        let dir = direction_unit(edge_points[idx].0, edge_points[idx].1);
                        place_label_with_offset(
                            &label.text,
                            label.font_size,
                            edge_points[idx].0.add(content_offset),
                            dir,
                            EDGE_LABEL_OFFSET,
                            &blocked_boxes,
                        )
                    }
                } else if let Some(slot) = straight_slots.get(&idx) {
                    let mut normal = slot.normal;
                    if normal.y > 0.0 {
                        normal = Point::new(-normal.x, -normal.y);
                    }
                    place_label_with_offset(
                        &label.text,
                        label.font_size,
                        slot.center.add(content_offset),
                        normal,
                        slot.stack_offset,
                        &blocked_boxes,
                    )
                } else {
                    place_edge_label(&label.text, label.font_size, &path, &blocked_boxes)
                };
                let clamped = clamp_label_position(
                    label_pos,
                    &label.text,
                    label.font_size,
                    label_bounds_min,
                    label_bounds_max,
                );
                label = label.with_position(clamped);
                Some(label)
            };

            let edge_look = colored_edge_style(&edge.arrow.look, line_color);
            let tooltip = edge.stats.as_ref().map(format_edge_tooltip);
            svg.draw_arrow(
                &path,
                dashed,
                (start, end),
                &edge_look,
                label.as_ref(),
                tooltip.as_deref(),
            );
        }

        for node in &section.nodes {
            let element = section.graph.element(node.handle);
            draw_node_table(&mut svg, node, element, content_offset);
        }

        for (top_left, table) in &info_table_positions {
            draw_resource_table(&mut svg, table, top_left.add(content_offset));
        }

        cursor_y += (section_max.y - section_min.y) + SECTION_SPACING;
    }

    let legend_top = last_section_bottom + GRAPH_MARGIN;
    let _legend_height = draw_legend(&mut svg, legend_top, last_section_right);

    svg.finalize()
}

/// Draw table cells manually since the layout engine only positions shapes.
fn draw_node_table(svg: &mut SvgWriter, node: &NodeRender, element: &Element, offset: Point) {
    let pos = element.position();
    let center = pos.center().add(offset);
    let size = pos.size(false);
    let top_left = Point::new(center.x - size.x / 2.0, center.y - size.y / 2.0);

    svg.draw_rect(top_left, size, None, 0.0, Some("white"), 0.0);

    let mut renderer = TableRenderer {
        svg,
        node_left_x: top_left.x,
        node_right_x: top_left.x + size.x,
    };
    visit_table(
        &node.table,
        element.orientation,
        center,
        size,
        &mut renderer,
    );
    svg.draw_rect(
        top_left,
        size,
        Some(BORDER_COLOR),
        OUTER_BORDER_WIDTH,
        None,
        0.0,
    );
}

fn draw_resource_table(svg: &mut SvgWriter, table: &ResourceTable, top_left: Point) {
    let size = table.size;
    let center = Point::new(top_left.x + size.x / 2.0, top_left.y + size.y / 2.0);
    svg.draw_rect(top_left, size, None, 0.0, Some("white"), 0.0);

    let mut renderer = TableRenderer {
        svg,
        node_left_x: top_left.x,
        node_right_x: top_left.x + size.x,
    };
    visit_table(
        &table.table,
        Orientation::TopToBottom,
        center,
        size,
        &mut renderer,
    );
    svg.draw_rect(
        top_left,
        size,
        Some(BORDER_COLOR),
        OUTER_BORDER_WIDTH,
        None,
        0.0,
    );
}

/// Visually group mission sections with a labeled bounding box.
fn draw_cluster(svg: &mut SvgWriter, min: Point, max: Point, label: &str, offset: Point) {
    let top_left = min.add(offset);
    let size = max.sub(min);
    svg.draw_rect(top_left, size, Some(CLUSTER_COLOR), 1.0, None, 10.0);

    let label_text = format!("Mission: {label}");
    let label_pos = Point::new(top_left.x + CELL_PADDING, top_left.y + FONT_SIZE as f64);
    svg.draw_text(
        label_pos,
        &label_text,
        FONT_SIZE,
        DIM_GRAY,
        true,
        "start",
        FontFamily::Sans,
    );
}

/// Render a legend cartridge for task colors and the copper-rs credit line.
fn draw_legend(svg: &mut SvgWriter, top_y: f64, content_right: f64) -> f64 {
    let metrics = measure_legend();
    let legend_x = (content_right - metrics.width).max(GRAPH_MARGIN);
    let top_left = Point::new(legend_x, top_y);

    svg.draw_rect(
        top_left,
        Point::new(metrics.width, metrics.height),
        Some(BORDER_COLOR),
        0.6,
        Some("white"),
        LEGEND_CORNER_RADIUS,
    );

    let title_pos = Point::new(
        top_left.x + LEGEND_PADDING,
        top_left.y + LEGEND_PADDING + LEGEND_TITLE_SIZE as f64 / 2.0,
    );
    svg.draw_text(
        title_pos,
        "Legend",
        LEGEND_TITLE_SIZE,
        DIM_GRAY,
        true,
        "start",
        FontFamily::Sans,
    );

    let mut cursor_y = top_left.y + LEGEND_PADDING + LEGEND_TITLE_SIZE as f64 + LEGEND_ROW_GAP;
    let item_height = LEGEND_SWATCH_SIZE.max(LEGEND_FONT_SIZE as f64);
    for (label, color) in LEGEND_ITEMS {
        let center_y = cursor_y + item_height / 2.0;
        let swatch_top = center_y - LEGEND_SWATCH_SIZE / 2.0;
        let swatch_left = top_left.x + LEGEND_PADDING;
        svg.draw_rect(
            Point::new(swatch_left, swatch_top),
            Point::new(LEGEND_SWATCH_SIZE, LEGEND_SWATCH_SIZE),
            Some(BORDER_COLOR),
            0.6,
            Some(color),
            2.0,
        );
        let text_x = swatch_left + LEGEND_SWATCH_SIZE + 4.0;
        svg.draw_text(
            Point::new(text_x, center_y),
            label,
            LEGEND_FONT_SIZE,
            "black",
            false,
            "start",
            FontFamily::Sans,
        );
        cursor_y += item_height + LEGEND_ROW_GAP;
    }

    if !RESOURCE_LEGEND_ITEMS.is_empty() {
        cursor_y += LEGEND_SECTION_GAP;
        let title_y = cursor_y + LEGEND_FONT_SIZE as f64 / 2.0;
        svg.draw_text(
            Point::new(top_left.x + LEGEND_PADDING, title_y),
            RESOURCE_LEGEND_TITLE,
            LEGEND_FONT_SIZE,
            DIM_GRAY,
            true,
            "start",
            FontFamily::Sans,
        );
        cursor_y += LEGEND_FONT_SIZE as f64 + LEGEND_ROW_GAP;

        for (label, color) in RESOURCE_LEGEND_ITEMS {
            let center_y = cursor_y + item_height / 2.0;
            let swatch_top = center_y - LEGEND_SWATCH_SIZE / 2.0;
            let swatch_left = top_left.x + LEGEND_PADDING;
            svg.draw_rect(
                Point::new(swatch_left, swatch_top),
                Point::new(LEGEND_SWATCH_SIZE, LEGEND_SWATCH_SIZE),
                Some(BORDER_COLOR),
                0.6,
                Some(color),
                2.0,
            );
            let text_x = swatch_left + LEGEND_SWATCH_SIZE + 4.0;
            svg.draw_text(
                Point::new(text_x, center_y),
                label,
                LEGEND_FONT_SIZE,
                "black",
                false,
                "start",
                FontFamily::Sans,
            );
            cursor_y += item_height + LEGEND_ROW_GAP;
        }
    }

    cursor_y += LEGEND_SECTION_GAP;
    let divider_y = cursor_y - LEGEND_ROW_GAP / 2.0;
    svg.draw_line(
        Point::new(top_left.x + LEGEND_PADDING, divider_y),
        Point::new(top_left.x + metrics.width - LEGEND_PADDING, divider_y),
        "#e0e0e0",
        0.5,
    );

    let credit_height = draw_created_with(
        svg,
        Point::new(top_left.x + LEGEND_PADDING, cursor_y),
        top_left.x + metrics.width - LEGEND_PADDING,
    );
    cursor_y += credit_height;

    cursor_y - top_left.y + LEGEND_BOTTOM_PADDING
}

fn draw_created_with(svg: &mut SvgWriter, top_left: Point, right_edge: f64) -> f64 {
    let left_text = "Created with";
    let link_text = "Copper-rs";
    let version_text = format!("v{}", env!("CARGO_PKG_VERSION"));
    let left_width = legend_text_width(left_text, LEGEND_FONT_SIZE);
    let link_width = legend_text_width(link_text, LEGEND_FONT_SIZE);
    let version_width = legend_text_width(version_text.as_str(), LEGEND_FONT_SIZE);
    let height = LEGEND_LOGO_SIZE.max(LEGEND_FONT_SIZE as f64);
    let center_y = top_left.y + height / 2.0;
    let version_text_x = right_edge;
    let link_text_x = version_text_x - version_width - LEGEND_VERSION_GAP;
    let link_start_x = link_text_x - link_width;
    let logo_left = link_start_x - LEGEND_LINK_GAP - LEGEND_LOGO_SIZE;
    let logo_top = center_y - LEGEND_LOGO_SIZE / 2.0;
    let left_text_anchor = logo_left - LEGEND_WITH_LOGO_GAP;

    svg.draw_text(
        Point::new(left_text_anchor, center_y),
        left_text,
        LEGEND_FONT_SIZE,
        DIM_GRAY,
        false,
        "end",
        FontFamily::Sans,
    );

    let logo_uri = svg_data_uri(COPPER_LOGO_SVG);
    let image = Image::new()
        .set("x", logo_left)
        .set("y", logo_top)
        .set("width", LEGEND_LOGO_SIZE)
        .set("height", LEGEND_LOGO_SIZE)
        .set("href", logo_uri.clone())
        .set("xlink:href", logo_uri);
    let mut text_node = build_text_node(
        Point::new(link_text_x, center_y),
        link_text,
        LEGEND_FONT_SIZE,
        COPPER_LINK_COLOR,
        false,
        "end",
        FontFamily::Sans,
    );
    text_node.assign("text-decoration", "underline");
    text_node.assign("text-underline-offset", "1");
    text_node.assign("text-decoration-thickness", "0.6");

    let mut link = SvgElement::new("a");
    link.assign("href", COPPER_GITHUB_URL);
    link.assign("target", "_blank");
    link.assign("rel", "noopener noreferrer");
    link.append(image);
    link.append(text_node);
    svg.append_node(link);

    svg.draw_text(
        Point::new(version_text_x, center_y),
        version_text.as_str(),
        LEGEND_FONT_SIZE,
        DIM_GRAY,
        false,
        "end",
        FontFamily::Sans,
    );

    let left_text_start = left_text_anchor - left_width;
    let total_width = right_edge - left_text_start;
    svg.grow_window(
        Point::new(left_text_start, top_left.y),
        Point::new(total_width, height),
    );

    height
}

struct LegendMetrics {
    width: f64,
    height: f64,
}

fn measure_legend() -> LegendMetrics {
    let title_width = get_size_for_str("Legend", LEGEND_TITLE_SIZE).x;
    let mut max_line_width = title_width;

    for (label, _) in LEGEND_ITEMS {
        let label_width = get_size_for_str(label, LEGEND_FONT_SIZE).x;
        let line_width = LEGEND_SWATCH_SIZE + 4.0 + label_width;
        max_line_width = max_line_width.max(line_width);
    }

    if !RESOURCE_LEGEND_ITEMS.is_empty() {
        let section_width = get_size_for_str(RESOURCE_LEGEND_TITLE, LEGEND_FONT_SIZE).x;
        max_line_width = max_line_width.max(section_width);
        for (label, _) in RESOURCE_LEGEND_ITEMS {
            let label_width = get_size_for_str(label, LEGEND_FONT_SIZE).x;
            let line_width = LEGEND_SWATCH_SIZE + 4.0 + label_width;
            max_line_width = max_line_width.max(line_width);
        }
    }

    let credit_left = "Created with";
    let credit_link = "Copper-rs";
    let credit_version = format!("v{}", env!("CARGO_PKG_VERSION"));
    let credit_width = legend_text_width(credit_left, LEGEND_FONT_SIZE)
        + LEGEND_WITH_LOGO_GAP
        + LEGEND_LOGO_SIZE
        + LEGEND_LINK_GAP
        + legend_text_width(credit_link, LEGEND_FONT_SIZE)
        + LEGEND_VERSION_GAP
        + legend_text_width(credit_version.as_str(), LEGEND_FONT_SIZE);
    max_line_width = max_line_width.max(credit_width);

    let item_height = LEGEND_SWATCH_SIZE.max(LEGEND_FONT_SIZE as f64);
    let items_count = LEGEND_ITEMS.len() as f64;
    let items_height = if items_count > 0.0 {
        items_count * item_height + (items_count - 1.0) * LEGEND_ROW_GAP
    } else {
        0.0
    };
    let resource_count = RESOURCE_LEGEND_ITEMS.len() as f64;
    let resource_height = if resource_count > 0.0 {
        resource_count * item_height + (resource_count - 1.0) * LEGEND_ROW_GAP
    } else {
        0.0
    };
    let resource_section_height = if resource_count > 0.0 {
        LEGEND_SECTION_GAP + LEGEND_FONT_SIZE as f64 + LEGEND_ROW_GAP + resource_height
    } else {
        0.0
    };
    let credit_height = LEGEND_LOGO_SIZE.max(LEGEND_FONT_SIZE as f64);
    let height = LEGEND_PADDING
        + LEGEND_BOTTOM_PADDING
        + LEGEND_TITLE_SIZE as f64
        + LEGEND_ROW_GAP
        + items_height
        + LEGEND_ROW_GAP
        + resource_section_height
        + LEGEND_SECTION_GAP
        + credit_height;

    LegendMetrics {
        width: LEGEND_PADDING * 2.0 + max_line_width,
        height,
    }
}

/// Fail fast on invalid mission ids and provide a readable list.
fn validate_mission_arg(
    config: &config::CuConfig,
    requested: Option<&str>,
) -> CuResult<Option<String>> {
    match (&config.graphs, requested) {
        (ConfigGraphs::Simple(_), None) => Ok(None),
        (ConfigGraphs::Simple(_), Some("default")) => Ok(None),
        (ConfigGraphs::Simple(_), Some(id)) => Err(CuError::from(format!(
            "Config is not mission-based; remove --mission (received '{id}')"
        ))),
        (ConfigGraphs::Missions(graphs), Some(id)) => {
            if graphs.contains_key(id) {
                Ok(Some(id.to_string()))
            } else {
                Err(CuError::from(format!(
                    "Mission '{id}' not found. Available missions: {}",
                    format_mission_list(graphs)
                )))
            }
        }
        (ConfigGraphs::Missions(_), None) => Ok(None),
    }
}

/// Support a CLI mode that prints mission names and exits.
fn print_mission_list(config: &config::CuConfig) {
    match &config.graphs {
        ConfigGraphs::Simple(_) => println!("default"),
        ConfigGraphs::Missions(graphs) => {
            let mut missions: Vec<_> = graphs.keys().cloned().collect();
            missions.sort();
            for mission in missions {
                println!("{mission}");
            }
        }
    }
}

/// Keep mission lists stable for consistent error messages.
fn format_mission_list(graphs: &HashMap<String, config::CuGraph>) -> String {
    let mut missions: Vec<_> = graphs.keys().cloned().collect();
    missions.sort();
    missions.join(", ")
}

struct SectionRef<'a> {
    label: Option<String>,
    mission_id: Option<String>,
    graph: &'a config::CuGraph,
}

struct SectionLayout {
    label: Option<String>,
    graph: VisualGraph,
    nodes: Vec<NodeRender>,
    edges: Vec<RenderEdge>,
    bounds: (Point, Point),
    port_anchors: HashMap<NodeHandle, HashMap<String, Point>>,
    resource_tables: Vec<ResourceTable>,
    perf_table: Option<ResourceTable>,
}

struct NodeRender {
    handle: NodeHandle,
    table: TableNode,
}

struct ResourceTable {
    table: TableNode,
    size: Point,
}

#[derive(Clone)]
struct ResourceOwner {
    name: String,
    flavor: config::Flavor,
}

#[derive(Clone, Copy)]
enum ResourceUsage {
    Exclusive,
    Shared,
    Unused,
}

#[derive(Clone, Deserialize)]
struct LogStats {
    schema_version: u32,
    config_signature: String,
    mission: Option<String>,
    edges: Vec<EdgeLogStats>,
    perf: PerfStats,
}

#[derive(Clone, Deserialize)]
struct EdgeLogStats {
    src: String,
    src_channel: Option<String>,
    dst: String,
    dst_channel: Option<String>,
    msg: String,
    samples: u64,
    none_samples: u64,
    valid_time_samples: u64,
    total_raw_bytes: u64,
    avg_raw_bytes: Option<f64>,
    rate_hz: Option<f64>,
    throughput_bytes_per_sec: Option<f64>,
}

#[derive(Clone, Deserialize)]
struct PerfStats {
    samples: u64,
    valid_time_samples: u64,
    end_to_end: DurationStats,
    jitter: DurationStats,
}

#[derive(Clone, Deserialize)]
struct DurationStats {
    min_ns: Option<u64>,
    max_ns: Option<u64>,
    mean_ns: Option<f64>,
    stddev_ns: Option<f64>,
}

struct LogStatsIndex {
    mission: Option<String>,
    edges: HashMap<EdgeStatsKey, EdgeLogStats>,
    perf: PerfStats,
}

impl LogStatsIndex {
    fn applies_to(&self, mission_id: Option<&str>) -> bool {
        mission_key(self.mission.as_deref()) == mission_key(mission_id)
    }

    fn edge_stats_for(&self, cnx: &config::RenderConnection) -> Option<EdgeLogStats> {
        let key = EdgeStatsKey::from_connection(cnx);
        self.edge_stats_for_key(&key)
    }

    fn edge_stats_for_key(&self, key: &EdgeStatsKey) -> Option<EdgeLogStats> {
        self.edges
            .get(key)
            .or_else(|| {
                if key.src_channel.is_some() || key.dst_channel.is_some() {
                    self.edges.get(&key.without_channels())
                } else {
                    None
                }
            })
            .cloned()
    }
}

struct RenderEdge {
    src: NodeHandle,
    dst: NodeHandle,
    arrow: Arrow,
    label: String,
    color_idx: usize,
    src_port: Option<String>,
    dst_port: Option<String>,
    stats: Option<EdgeLogStats>,
}

#[derive(Clone, Hash, PartialEq, Eq)]
struct EdgeGroupKey {
    src: NodeHandle,
    src_port: Option<String>,
    msg: String,
}

#[derive(Clone, Hash, PartialEq, Eq)]
struct EdgeStatsKey {
    src: String,
    src_channel: Option<String>,
    dst: String,
    dst_channel: Option<String>,
    msg: String,
}

impl EdgeStatsKey {
    fn from_edge(edge: &EdgeLogStats) -> Self {
        Self {
            src: edge.src.clone(),
            src_channel: edge.src_channel.clone(),
            dst: edge.dst.clone(),
            dst_channel: edge.dst_channel.clone(),
            msg: edge.msg.clone(),
        }
    }

    fn from_connection(cnx: &config::RenderConnection) -> Self {
        Self {
            src: cnx.src.clone(),
            src_channel: cnx.src_channel.clone(),
            dst: cnx.dst.clone(),
            dst_channel: cnx.dst_channel.clone(),
            msg: cnx.msg.clone(),
        }
    }

    fn without_channels(&self) -> Self {
        Self {
            src: self.src.clone(),
            src_channel: None,
            dst: self.dst.clone(),
            dst_channel: None,
            msg: self.msg.clone(),
        }
    }
}

#[derive(Clone)]
enum TableNode {
    Cell(TableCell),
    Array(Vec<TableNode>),
}

#[derive(Clone)]
struct TableCell {
    lines: Vec<CellLine>,
    port: Option<String>,
    background: Option<String>,
    border_width: f64,
    align: TextAlign,
}

impl TableCell {
    fn new(lines: Vec<CellLine>) -> Self {
        Self {
            lines,
            port: None,
            background: None,
            border_width: 1.0,
            align: TextAlign::Left,
        }
    }

    fn single_line_sized(
        text: impl Into<String>,
        color: &str,
        bold: bool,
        font_size: usize,
    ) -> Self {
        Self::new(vec![CellLine::new(text, color, bold, font_size)])
    }

    fn with_port(mut self, port: String) -> Self {
        self.port = Some(port);
        self
    }

    fn with_background(mut self, color: &str) -> Self {
        self.background = Some(color.to_string());
        self
    }

    fn with_border_width(mut self, width: f64) -> Self {
        self.border_width = width;
        self
    }

    fn with_align(mut self, align: TextAlign) -> Self {
        self.align = align;
        self
    }

    fn label(&self) -> String {
        self.lines
            .iter()
            .map(|line| line.text.as_str())
            .collect::<Vec<_>>()
            .join("\n")
    }
}

#[derive(Clone, Copy)]
enum TextAlign {
    Left,
    Center,
    Right,
}

#[derive(Clone)]
struct CellLine {
    text: String,
    color: String,
    bold: bool,
    font_size: usize,
    font_family: FontFamily,
}

impl CellLine {
    fn new(text: impl Into<String>, color: &str, bold: bool, font_size: usize) -> Self {
        Self {
            text: text.into(),
            color: color.to_string(),
            bold,
            font_size,
            font_family: FontFamily::Sans,
        }
    }

    fn code(text: impl Into<String>, color: &str, bold: bool, font_size: usize) -> Self {
        let mut line = Self::new(text, color, bold, font_size);
        line.font_family = FontFamily::Mono;
        line
    }
}

#[derive(Clone, Copy)]
enum FontFamily {
    Sans,
    Mono,
}

impl FontFamily {
    fn as_css(self) -> &'static str {
        match self {
            FontFamily::Sans => FONT_FAMILY,
            FontFamily::Mono => MONO_FONT_FAMILY,
        }
    }
}

trait TableVisitor {
    fn handle_cell(&mut self, cell: &TableCell, loc: Point, size: Point);
}

struct TableRenderer<'a> {
    svg: &'a mut SvgWriter,
    node_left_x: f64,
    node_right_x: f64,
}

impl TableVisitor for TableRenderer<'_> {
    fn handle_cell(&mut self, cell: &TableCell, loc: Point, size: Point) {
        let top_left = Point::new(loc.x - size.x / 2.0, loc.y - size.y / 2.0);

        if let Some(bg) = &cell.background {
            self.svg.draw_rect(top_left, size, None, 0.0, Some(bg), 0.0);
        }
        self.svg.draw_rect(
            top_left,
            size,
            Some(BORDER_COLOR),
            cell.border_width,
            None,
            0.0,
        );

        if let Some(port) = &cell.port {
            let is_output = port.starts_with("out_");
            let dot_x = if is_output {
                self.node_right_x
            } else {
                self.node_left_x
            };
            self.svg
                .draw_circle_overlay(Point::new(dot_x, loc.y), PORT_DOT_RADIUS, BORDER_COLOR);
        }

        if cell.lines.is_empty() {
            return;
        }

        let total_height = cell_text_height(cell);
        let mut current_y = loc.y - total_height / 2.0;
        let (text_x, anchor) = match cell.align {
            TextAlign::Left => (loc.x - size.x / 2.0 + CELL_PADDING, "start"),
            TextAlign::Center => (loc.x, "middle"),
            TextAlign::Right => (loc.x + size.x / 2.0 - CELL_PADDING, "end"),
        };

        for (idx, line) in cell.lines.iter().enumerate() {
            let line_height = line.font_size as f64;
            let y = current_y + line_height / 2.0;
            self.svg.draw_text(
                Point::new(text_x, y),
                &line.text,
                line.font_size,
                &line.color,
                line.bold,
                anchor,
                line.font_family,
            );
            current_y += line_height;
            if idx + 1 < cell.lines.len() {
                current_y += CELL_LINE_SPACING;
            }
        }
    }
}

struct ArrowLabel {
    text: String,
    color: String,
    font_size: usize,
    bold: bool,
    font_family: FontFamily,
    position: Option<Point>,
}

struct StraightLabelSlot {
    center: Point,
    width: f64,
    normal: Point,
    stack_offset: f64,
    group_count: usize,
}

struct DetourLabelSlot {
    center_x: f64,
    width: f64,
    lane_y: f64,
    above: bool,
    group_count: usize,
    group_width: f64,
}

struct BezierSegment {
    start: Point,
    c1: Point,
    c2: Point,
    end: Point,
}

impl ArrowLabel {
    fn new(
        text: String,
        color: &str,
        font_size: usize,
        bold: bool,
        font_family: FontFamily,
    ) -> Self {
        Self {
            text,
            color: color.to_string(),
            font_size,
            bold,
            font_family,
            position: None,
        }
    }

    fn with_position(mut self, position: Point) -> Self {
        self.position = Some(position);
        self
    }
}

struct NullBackend;

impl RenderBackend for NullBackend {
    fn draw_rect(
        &mut self,
        _xy: Point,
        _size: Point,
        _look: &StyleAttr,
        _properties: Option<String>,
        _clip: Option<layout::core::format::ClipHandle>,
    ) {
    }

    fn draw_line(
        &mut self,
        _start: Point,
        _stop: Point,
        _look: &StyleAttr,
        _properties: Option<String>,
    ) {
    }

    fn draw_circle(
        &mut self,
        _xy: Point,
        _size: Point,
        _look: &StyleAttr,
        _properties: Option<String>,
    ) {
    }

    fn draw_text(&mut self, _xy: Point, _text: &str, _look: &StyleAttr) {}

    fn draw_arrow(
        &mut self,
        _path: &[(Point, Point)],
        _dashed: bool,
        _head: (bool, bool),
        _look: &StyleAttr,
        _properties: Option<String>,
        _text: &str,
    ) {
    }

    fn create_clip(&mut self, _xy: Point, _size: Point, _rounded_px: usize) -> usize {
        0
    }
}

struct SvgWriter {
    content: Group,
    overlay: Group,
    defs: Definitions,
    view_size: Point,
    counter: usize,
}

impl SvgWriter {
    fn new() -> Self {
        let mut defs = Definitions::new();
        let start_marker = Marker::new()
            .set("id", "startarrow")
            .set("markerWidth", 10)
            .set("markerHeight", 7)
            .set("refX", 2)
            .set("refY", 3.5)
            .set("orient", "auto")
            .add(
                Polygon::new()
                    .set("points", "10 0, 10 7, 0 3.5")
                    .set("fill", "context-stroke"),
            );
        let end_marker = Marker::new()
            .set("id", "endarrow")
            .set("markerWidth", 10)
            .set("markerHeight", 7)
            .set("refX", 8)
            .set("refY", 3.5)
            .set("orient", "auto")
            .add(
                Polygon::new()
                    .set("points", "0 0, 10 3.5, 0 7")
                    .set("fill", "context-stroke"),
            );
        defs.append(start_marker);
        defs.append(end_marker);
        let mut style = SvgElement::new("style");
        style.assign("type", "text/css");
        style.append(SvgTextNode::new(EDGE_TOOLTIP_CSS));
        defs.append(style);

        Self {
            content: Group::new(),
            overlay: Group::new(),
            defs,
            view_size: Point::new(0.0, 0.0),
            counter: 0,
        }
    }

    fn grow_window(&mut self, point: Point, size: Point) {
        self.view_size.x = self.view_size.x.max(point.x + size.x);
        self.view_size.y = self.view_size.y.max(point.y + size.y);
    }

    fn draw_rect(
        &mut self,
        top_left: Point,
        size: Point,
        stroke: Option<&str>,
        stroke_width: f64,
        fill: Option<&str>,
        rounded: f64,
    ) {
        self.grow_window(top_left, size);

        let stroke_color = stroke.unwrap_or("none");
        let fill_color = fill.unwrap_or("none");
        let width = if stroke.is_some() { stroke_width } else { 0.0 };
        let mut rect = Rectangle::new()
            .set("x", top_left.x)
            .set("y", top_left.y)
            .set("width", size.x)
            .set("height", size.y)
            .set("fill", fill_color)
            .set("stroke", stroke_color)
            .set("stroke-width", width);
        if rounded > 0.0 {
            rect = rect.set("rx", rounded).set("ry", rounded);
        }
        self.content.append(rect);
    }

    fn draw_circle_overlay(&mut self, center: Point, radius: f64, fill: &str) {
        let circle = Circle::new()
            .set("cx", center.x)
            .set("cy", center.y)
            .set("r", radius)
            .set("fill", fill);
        self.overlay.append(circle);

        let top_left = Point::new(center.x - radius, center.y - radius);
        let size = Point::new(radius * 2.0, radius * 2.0);
        self.grow_window(top_left, size);
    }

    fn draw_line(&mut self, start: Point, end: Point, color: &str, width: f64) {
        let line = Line::new()
            .set("x1", start.x)
            .set("y1", start.y)
            .set("x2", end.x)
            .set("y2", end.y)
            .set("stroke", color)
            .set("stroke-width", width);
        self.content.append(line);

        let top_left = Point::new(start.x.min(end.x), start.y.min(end.y));
        let size = Point::new((start.x - end.x).abs(), (start.y - end.y).abs());
        self.grow_window(top_left, size);
    }

    fn append_node<T>(&mut self, node: T)
    where
        T: Into<Box<dyn Node>>,
    {
        self.content.append(node);
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_text(
        &mut self,
        pos: Point,
        text: &str,
        font_size: usize,
        color: &str,
        bold: bool,
        anchor: &str,
        family: FontFamily,
    ) {
        if text.is_empty() {
            return;
        }

        let weight = if bold { "bold" } else { "normal" };
        let node = Text::new(text)
            .set("x", pos.x)
            .set("y", pos.y)
            .set("text-anchor", anchor)
            .set("dominant-baseline", "middle")
            .set("font-family", family.as_css())
            .set("font-size", format!("{font_size}px"))
            .set("fill", color)
            .set("font-weight", weight);
        self.content.append(node);

        let size = get_size_for_str(text, font_size);
        let top_left = Point::new(pos.x, pos.y - size.y / 2.0);
        self.grow_window(top_left, size);
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_text_overlay(
        &mut self,
        pos: Point,
        text: &str,
        font_size: usize,
        color: &str,
        bold: bool,
        anchor: &str,
        family: FontFamily,
    ) {
        if text.is_empty() {
            return;
        }

        let weight = if bold { "bold" } else { "normal" };
        let node = Text::new(text)
            .set("x", pos.x)
            .set("y", pos.y)
            .set("text-anchor", anchor)
            .set("dominant-baseline", "middle")
            .set("font-family", family.as_css())
            .set("font-size", format!("{font_size}px"))
            .set("fill", color)
            .set("font-weight", weight)
            .set("stroke", "white")
            .set("stroke-width", EDGE_LABEL_HALO_WIDTH)
            .set("paint-order", "stroke")
            .set("stroke-linejoin", "round");
        self.overlay.append(node);

        let size = get_size_for_str(text, font_size);
        let top_left = Point::new(pos.x, pos.y - size.y / 2.0);
        self.grow_window(top_left, size);
    }

    fn draw_arrow(
        &mut self,
        path: &[BezierSegment],
        dashed: bool,
        head: (bool, bool),
        look: &StyleAttr,
        label: Option<&ArrowLabel>,
        tooltip: Option<&str>,
    ) {
        if path.is_empty() {
            return;
        }

        for segment in path {
            self.grow_window(segment.start, Point::new(0.0, 0.0));
            self.grow_window(segment.c1, Point::new(0.0, 0.0));
            self.grow_window(segment.c2, Point::new(0.0, 0.0));
            self.grow_window(segment.end, Point::new(0.0, 0.0));
        }

        let stroke_color = look.line_color.to_web_color();
        let stroke_color = normalize_web_color(&stroke_color);

        let path_data = build_path_data(path);
        let path_id = format!("arrow{}", self.counter);
        let mut path_el = SvgPath::new()
            .set("id", path_id.clone())
            .set("d", path_data)
            .set("stroke", stroke_color.clone())
            .set("stroke-width", look.line_width)
            .set("fill", "none");
        if dashed {
            path_el = path_el.set("stroke-dasharray", "5,5");
        }
        if head.0 {
            path_el = path_el.set("marker-start", "url(#startarrow)");
        }
        if head.1 {
            path_el = path_el.set("marker-end", "url(#endarrow)");
        }
        self.content.append(path_el);

        if let Some(label) = label {
            if label.text.is_empty() {
                self.counter += 1;
                return;
            }
            if let Some(pos) = label.position {
                self.draw_text_overlay(
                    pos,
                    &label.text,
                    label.font_size,
                    &label.color,
                    label.bold,
                    "middle",
                    label.font_family,
                );
            } else {
                let label_path_id = format!("{}_label", path_id);
                let start = path[0].start;
                let end = path[path.len() - 1].end;
                let label_path_data = build_explicit_path_data(path, start.x > end.x);
                let label_path_el = SvgPath::new()
                    .set("id", label_path_id.clone())
                    .set("d", label_path_data)
                    .set("fill", "none")
                    .set("stroke", "none");
                self.overlay.append(label_path_el);

                let weight = if label.bold { "bold" } else { "normal" };
                let text_path = TextPath::new(label.text.as_str())
                    .set("href", format!("#{label_path_id}"))
                    .set("startOffset", "50%")
                    .set("text-anchor", "middle")
                    .set("dy", EDGE_LABEL_OFFSET)
                    .set("font-family", label.font_family.as_css())
                    .set("font-size", format!("{}px", label.font_size))
                    .set("fill", label.color.clone())
                    .set("font-weight", weight)
                    .set("stroke", "white")
                    .set("stroke-width", EDGE_LABEL_HALO_WIDTH)
                    .set("paint-order", "stroke")
                    .set("stroke-linejoin", "round");
                let mut text_node = SvgElement::new("text");
                text_node.append(text_path);
                self.overlay.append(text_node);
            }
        }

        if let Some(tooltip) = tooltip {
            let (hover_group, tooltip_top_left, tooltip_size) =
                build_edge_hover_overlay(path, tooltip, &stroke_color, look.line_width);
            self.grow_window(tooltip_top_left, tooltip_size);
            self.overlay.append(hover_group);
        }

        self.counter += 1;
    }

    fn finalize(self) -> String {
        let width = if self.view_size.x < 1.0 {
            1.0
        } else {
            self.view_size.x + GRAPH_MARGIN
        };
        let height = if self.view_size.y < 1.0 {
            1.0
        } else {
            self.view_size.y + GRAPH_MARGIN
        };

        let background = Rectangle::new()
            .set("x", 0)
            .set("y", 0)
            .set("width", width)
            .set("height", height)
            .set("fill", BACKGROUND_COLOR);

        Document::new()
            .set("width", width)
            .set("height", height)
            .set("viewBox", (0, 0, width, height))
            .set("xmlns", "http://www.w3.org/2000/svg")
            .set("xmlns:xlink", "http://www.w3.org/1999/xlink")
            .add(self.defs)
            .add(background)
            .add(self.content)
            .add(self.overlay)
            .to_string()
    }
}

fn build_text_node(
    pos: Point,
    text: &str,
    font_size: usize,
    color: &str,
    bold: bool,
    anchor: &str,
    family: FontFamily,
) -> Text {
    let weight = if bold { "bold" } else { "normal" };
    Text::new(text)
        .set("x", pos.x)
        .set("y", pos.y)
        .set("text-anchor", anchor)
        .set("dominant-baseline", "middle")
        .set("font-family", family.as_css())
        .set("font-size", format!("{font_size}px"))
        .set("fill", color)
        .set("font-weight", weight)
}

fn svg_data_uri(svg: &str) -> String {
    format!(
        "data:image/svg+xml;base64,{}",
        base64_encode(svg.as_bytes())
    )
}

fn base64_encode(input: &[u8]) -> String {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(input.len().div_ceil(3) * 4);
    let mut i = 0;
    while i < input.len() {
        let b0 = input[i];
        let b1 = if i + 1 < input.len() { input[i + 1] } else { 0 };
        let b2 = if i + 2 < input.len() { input[i + 2] } else { 0 };
        let triple = ((b0 as u32) << 16) | ((b1 as u32) << 8) | (b2 as u32);
        let idx0 = ((triple >> 18) & 0x3F) as usize;
        let idx1 = ((triple >> 12) & 0x3F) as usize;
        let idx2 = ((triple >> 6) & 0x3F) as usize;
        let idx3 = (triple & 0x3F) as usize;
        out.push(TABLE[idx0] as char);
        out.push(TABLE[idx1] as char);
        if i + 1 < input.len() {
            out.push(TABLE[idx2] as char);
        } else {
            out.push('=');
        }
        if i + 2 < input.len() {
            out.push(TABLE[idx3] as char);
        } else {
            out.push('=');
        }
        i += 3;
    }
    out
}

fn legend_text_width(text: &str, font_size: usize) -> f64 {
    text.chars().count() as f64 * font_size as f64 * LEGEND_TEXT_WIDTH_FACTOR
}

fn normalize_web_color(color: &str) -> String {
    if color.len() == 9 && color.starts_with('#') {
        return format!("#{}", &color[1..7]);
    }
    color.to_string()
}

fn colored_edge_style(base: &StyleAttr, color: &str) -> StyleAttr {
    StyleAttr::new(Color::fast(color), base.line_width, None, 0, EDGE_FONT_SIZE)
}

fn edge_cycle_color_index(slot: &mut usize) -> usize {
    let idx = EDGE_COLOR_ORDER[*slot % EDGE_COLOR_ORDER.len()];
    *slot += 1;
    idx
}

fn lighten_hex(color: &str, amount: f64) -> String {
    let Some(hex) = color.strip_prefix('#') else {
        return color.to_string();
    };
    if hex.len() != 6 {
        return color.to_string();
    }
    let parse = |idx| u8::from_str_radix(&hex[idx..idx + 2], 16).ok();
    let (Some(r), Some(g), Some(b)) = (parse(0), parse(2), parse(4)) else {
        return color.to_string();
    };
    let blend = |c| ((c as f64) + (255.0 - c as f64) * amount).round() as u8;
    format!("#{:02X}{:02X}{:02X}", blend(r), blend(g), blend(b))
}

fn wrap_text(text: &str, max_width: usize) -> String {
    if text.len() <= max_width {
        return text.to_string();
    }
    let mut out = String::new();
    let mut line_len = 0;
    for word in text.split_whitespace() {
        let next_len = if line_len == 0 {
            word.len()
        } else {
            line_len + 1 + word.len()
        };
        if next_len > max_width && line_len > 0 {
            out.push('\n');
            out.push_str(word);
            line_len = word.len();
        } else {
            if line_len > 0 {
                out.push(' ');
            }
            out.push_str(word);
            line_len = next_len;
        }
    }
    out
}

fn wrap_type_label(label: &str, max_width: usize) -> String {
    if label.len() <= max_width {
        return label.to_string();
    }

    let tokens = split_type_tokens(label);
    let mut lines: Vec<String> = Vec::new();
    let mut current = String::new();
    let mut current_len = 0usize;

    for token in tokens {
        if token.is_empty() {
            continue;
        }

        let chunks = split_long_token(&token, max_width);
        for chunk in chunks {
            if current_len + chunk.len() > max_width && !current.is_empty() {
                lines.push(current);
                current = String::new();
                current_len = 0;
            }

            current.push_str(&chunk);
            current_len += chunk.len();

            if chunk == "," || chunk == "<" || chunk == ">" {
                lines.push(current);
                current = String::new();
                current_len = 0;
            }
        }
    }

    if !current.is_empty() {
        lines.push(current);
    }

    if lines.is_empty() {
        return label.to_string();
    }
    lines.join("\n")
}

fn split_long_token(token: &str, max_width: usize) -> Vec<String> {
    if token.len() <= max_width || token == "::" {
        return vec![token.to_string()];
    }

    let mut out = Vec::new();
    let mut start = 0;
    let chars: Vec<char> = token.chars().collect();
    while start < chars.len() {
        let end = (start + max_width).min(chars.len());
        out.push(chars[start..end].iter().collect());
        start = end;
    }
    out
}

fn cell_text_size(cell: &TableCell) -> Point {
    let mut max_width: f64 = 0.0;
    if cell.lines.is_empty() {
        return Point::new(1.0, 1.0);
    }
    for line in &cell.lines {
        let size = get_size_for_str(&line.text, line.font_size);
        max_width = max_width.max(size.x);
    }
    Point::new(max_width, cell_text_height(cell).max(1.0))
}

fn cell_text_height(cell: &TableCell) -> f64 {
    if cell.lines.is_empty() {
        return 1.0;
    }
    let base: f64 = cell.lines.iter().map(|line| line.font_size as f64).sum();
    let spacing = CELL_LINE_SPACING * (cell.lines.len().saturating_sub(1) as f64);
    base + spacing
}

fn collect_port_anchors(node: &NodeRender, element: &Element) -> HashMap<String, Point> {
    let pos = element.position();
    let center = pos.center();
    let size = pos.size(false);
    let left_x = center.x - size.x / 2.0;
    let right_x = center.x + size.x / 2.0;

    let mut anchors = HashMap::new();
    let mut collector = PortAnchorCollector {
        anchors: &mut anchors,
        node_left_x: left_x,
        node_right_x: right_x,
    };
    visit_table(
        &node.table,
        element.orientation,
        center,
        size,
        &mut collector,
    );
    anchors
}

struct PortAnchorCollector<'a> {
    anchors: &'a mut HashMap<String, Point>,
    node_left_x: f64,
    node_right_x: f64,
}

impl TableVisitor for PortAnchorCollector<'_> {
    fn handle_cell(&mut self, cell: &TableCell, loc: Point, _size: Point) {
        let Some(port) = &cell.port else {
            return;
        };

        let is_output = port.starts_with("out_");
        let port_offset = PORT_LINE_GAP + PORT_DOT_RADIUS;
        let x = if is_output {
            self.node_right_x + port_offset
        } else {
            self.node_left_x - port_offset
        };
        self.anchors.insert(port.clone(), Point::new(x, loc.y));
    }
}

fn resolve_anchor(section: &SectionLayout, node: NodeHandle, port: Option<&String>) -> Point {
    if let Some(port) = port
        && let Some(anchors) = section.port_anchors.get(&node)
        && let Some(point) = anchors.get(port)
    {
        return *point;
    }

    section.graph.element(node).position().center()
}

#[derive(Clone, Copy)]
struct BackEdgePlan {
    idx: usize,
    span: f64,
    order_y: f64,
}

struct NodeBounds {
    handle: NodeHandle,
    left: f64,
    right: f64,
    top: f64,
    bottom: f64,
    center_x: f64,
}

fn collect_node_bounds(nodes: &[NodeRender], graph: &VisualGraph) -> Vec<NodeBounds> {
    let mut bounds = Vec::with_capacity(nodes.len());
    for node in nodes {
        let pos = graph.element(node.handle).position();
        let (top_left, bottom_right) = pos.bbox(false);
        bounds.push(NodeBounds {
            handle: node.handle,
            left: top_left.x,
            right: bottom_right.x,
            top: top_left.y,
            bottom: bottom_right.y,
            center_x: (top_left.x + bottom_right.x) / 2.0,
        });
    }
    bounds
}

fn max_bottom_for_span(bounds: &[NodeBounds], min_x: f64, max_x: f64) -> f64 {
    bounds
        .iter()
        .filter(|b| b.right >= min_x && b.left <= max_x)
        .map(|b| b.bottom)
        .fold(f64::NEG_INFINITY, f64::max)
}

fn min_top_for_span(bounds: &[NodeBounds], min_x: f64, max_x: f64) -> f64 {
    bounds
        .iter()
        .filter(|b| b.right >= min_x && b.left <= max_x)
        .map(|b| b.top)
        .fold(f64::INFINITY, f64::min)
}

fn span_has_intermediate(
    bounds: &[NodeBounds],
    min_x: f64,
    max_x: f64,
    src: NodeHandle,
    dst: NodeHandle,
) -> bool {
    bounds.iter().any(|b| {
        b.handle != src
            && b.handle != dst
            && b.center_x > min_x + INTERMEDIATE_X_EPS
            && b.center_x < max_x - INTERMEDIATE_X_EPS
    })
}

fn assign_back_edge_offsets(plans: &[BackEdgePlan], offsets: &mut [f64]) {
    let mut plans = plans.to_vec();
    plans.sort_by(|a, b| {
        a.span
            .partial_cmp(&b.span)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.order_y.partial_cmp(&b.order_y).unwrap_or(Ordering::Equal))
    });

    let mut layer = 0usize;
    let mut last_span: Option<f64> = None;
    let mut layer_counts: HashMap<usize, usize> = HashMap::new();

    for plan in plans {
        if let Some(prev_span) = last_span
            && (plan.span - prev_span).abs() > BACK_EDGE_SPAN_EPS
        {
            layer += 1;
        }
        last_span = Some(plan.span);

        let dup = layer_counts.entry(layer).or_insert(0);
        offsets[plan.idx] =
            layer as f64 * BACK_EDGE_STACK_SPACING + *dup as f64 * BACK_EDGE_DUP_SPACING;
        *dup += 1;
    }
}

fn expand_bounds(bounds: &mut (Point, Point), point: Point) {
    bounds.0.x = bounds.0.x.min(point.x);
    bounds.0.y = bounds.0.y.min(point.y);
    bounds.1.x = bounds.1.x.max(point.x);
    bounds.1.y = bounds.1.y.max(point.y);
}

fn port_dir(port: Option<&String>) -> Option<f64> {
    port.and_then(|name| {
        if name.starts_with("out_") {
            Some(1.0)
        } else if name.starts_with("in_") {
            Some(-1.0)
        } else {
            None
        }
    })
}

fn port_dir_incoming(port: Option<&String>) -> Option<f64> {
    port_dir(port).map(|dir| -dir)
}

fn fallback_port_dirs(start: Point, end: Point) -> (f64, f64) {
    let dir = if end.x >= start.x { 1.0 } else { -1.0 };
    (dir, dir)
}

fn lerp_point(a: Point, b: Point, t: f64) -> Point {
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

fn straight_segment(start: Point, end: Point) -> BezierSegment {
    BezierSegment {
        start,
        c1: lerp_point(start, end, 1.0 / 3.0),
        c2: lerp_point(start, end, 2.0 / 3.0),
        end,
    }
}

fn edge_stub_len(start: Point, end: Point) -> f64 {
    let dx = (end.x - start.x).abs();
    if dx <= 0.0 {
        return 0.0;
    }
    let max_stub = dx * 0.45;
    let mut stub = EDGE_STUB_LEN.min(max_stub);
    let min_stub = EDGE_STUB_MIN.min(max_stub);
    if stub < min_stub {
        stub = min_stub;
    }
    stub
}

fn edge_port_handle(start: Point, end: Point) -> f64 {
    let dx = (end.x - start.x).abs();
    let mut handle = EDGE_PORT_HANDLE.min(dx * 0.2);
    if handle < 6.0 {
        handle = 6.0;
    }
    handle
}

fn build_edge_path(start: Point, end: Point, start_dir: f64, end_dir: f64) -> Vec<BezierSegment> {
    let dir = if end.x >= start.x { 1.0 } else { -1.0 };
    let stub = edge_stub_len(start, end);
    if stub <= 1.0 {
        let dx = (end.x - start.x).abs().max(40.0);
        let ctrl1 = Point::new(start.x + dir * dx * 0.5, start.y);
        let ctrl2 = Point::new(end.x - dir * dx * 0.5, end.y);
        return vec![BezierSegment {
            start,
            c1: ctrl1,
            c2: ctrl2,
            end,
        }];
    }

    let start_stub = Point::new(start.x + start_dir * stub, start.y);
    let end_stub = Point::new(end.x - end_dir * stub, end.y);
    let inner_dir = if end_stub.x >= start_stub.x {
        1.0
    } else {
        -1.0
    };
    let curve_dx = ((end_stub.x - start_stub.x).abs() * 0.35).max(10.0);

    let seg1 = straight_segment(start, start_stub);
    let seg2 = BezierSegment {
        start: start_stub,
        c1: Point::new(start_stub.x + inner_dir * curve_dx, start_stub.y),
        c2: Point::new(end_stub.x - inner_dir * curve_dx, end_stub.y),
        end: end_stub,
    };
    let seg3 = straight_segment(end_stub, end);
    vec![seg1, seg2, seg3]
}

fn build_back_edge_path(
    start: Point,
    end: Point,
    lane_y: f64,
    start_dir: f64,
    end_dir: f64,
) -> Vec<BezierSegment> {
    build_lane_path(start, end, lane_y, start_dir, end_dir)
}

fn build_loop_path(
    start: Point,
    end: Point,
    bbox: (Point, Point),
    start_dir: f64,
    end_dir: f64,
) -> Vec<BezierSegment> {
    let height = bbox.1.y - bbox.0.y;
    let loop_dy = height * 0.8 + 30.0;

    let center_y = (bbox.0.y + bbox.1.y) / 2.0;

    let dir_y = if (start.y + end.y) / 2.0 < center_y {
        -1.0
    } else {
        1.0
    };
    let lane_y = center_y + dir_y * loop_dy;
    build_back_edge_path(start, end, lane_y, start_dir, end_dir)
}

fn build_lane_path(
    start: Point,
    end: Point,
    lane_y: f64,
    start_dir: f64,
    end_dir: f64,
) -> Vec<BezierSegment> {
    let base_stub = edge_port_handle(start, end);
    let dy_start = (lane_y - start.y).abs();
    let dy_end = (lane_y - end.y).abs();
    let max_stub = (end.x - start.x).abs().max(40.0) * 0.45;
    let start_stub = (base_stub + dy_start * 0.6).min(max_stub.max(base_stub));
    let end_stub = (base_stub + dy_end * 0.6).min(max_stub.max(base_stub));
    let mut start_corner = Point::new(start.x, lane_y);
    let mut end_corner = Point::new(end.x, lane_y);
    let lane_dir = if (end_corner.x - start_corner.x).abs() < 1.0 {
        if end_dir.abs() > 0.0 {
            end_dir
        } else {
            start_dir
        }
    } else if end_corner.x >= start_corner.x {
        1.0
    } else {
        -1.0
    };
    let span = (end_corner.x - start_corner.x).abs();
    if start.x < end.x && span > 1.0 {
        let min_span = 60.0;
        let mut shrink = (span * 0.2).min(80.0);
        let max_shrink = ((span - min_span).max(0.0)) / 2.0;
        if shrink > max_shrink {
            shrink = max_shrink;
        }
        start_corner.x += lane_dir * shrink;
        end_corner.x -= lane_dir * shrink;
    }
    let entry_dir = -lane_dir;
    let handle_scale = if start.x < end.x { 0.6 } else { 1.0 };
    let entry_handle = (start_stub * handle_scale).max(6.0);
    let exit_handle = (end_stub * handle_scale).max(6.0);
    let seg1 = BezierSegment {
        start,
        c1: Point::new(start.x + start_dir * entry_handle, start.y),
        c2: Point::new(start_corner.x + entry_dir * entry_handle, lane_y),
        end: start_corner,
    };
    let seg2 = straight_segment(start_corner, end_corner);
    let seg3 = BezierSegment {
        start: end_corner,
        c1: Point::new(end_corner.x + lane_dir * exit_handle, lane_y),
        c2: Point::new(end.x - end_dir * exit_handle, end.y),
        end,
    };
    vec![seg1, seg2, seg3]
}

fn build_path_data(path: &[BezierSegment]) -> Data {
    if path.is_empty() {
        return Data::new();
    }

    let first = &path[0];
    let mut data = Data::new()
        .move_to((first.start.x, first.start.y))
        .cubic_curve_to((
            first.c1.x,
            first.c1.y,
            first.c2.x,
            first.c2.y,
            first.end.x,
            first.end.y,
        ));
    for segment in path.iter().skip(1) {
        data = data.cubic_curve_to((
            segment.c1.x,
            segment.c1.y,
            segment.c2.x,
            segment.c2.y,
            segment.end.x,
            segment.end.y,
        ));
    }
    data
}

fn build_explicit_path_data(path: &[BezierSegment], reverse: bool) -> Data {
    if path.is_empty() {
        return Data::new();
    }

    if !reverse {
        return build_path_data(path);
    }

    let mut iter = path.iter().rev();
    let Some(first) = iter.next() else {
        return Data::new();
    };
    let mut data = Data::new()
        .move_to((first.end.x, first.end.y))
        .cubic_curve_to((
            first.c2.x,
            first.c2.y,
            first.c1.x,
            first.c1.y,
            first.start.x,
            first.start.y,
        ));
    for segment in iter {
        data = data.cubic_curve_to((
            segment.c2.x,
            segment.c2.y,
            segment.c1.x,
            segment.c1.y,
            segment.start.x,
            segment.start.y,
        ));
    }
    data
}

fn place_edge_label(
    text: &str,
    font_size: usize,
    path: &[BezierSegment],
    blocked: &[(Point, Point)],
) -> Point {
    let (mid, dir) = path_label_anchor(path);
    let mut normal = Point::new(-dir.y, dir.x);
    if normal.x == 0.0 && normal.y == 0.0 {
        normal = Point::new(0.0, -1.0);
    }
    if normal.y > 0.0 {
        normal = Point::new(-normal.x, -normal.y);
    }
    place_label_with_normal(text, font_size, mid, normal, blocked)
}

fn place_self_loop_label(
    text: &str,
    font_size: usize,
    path: &[BezierSegment],
    node_center: Point,
    blocked: &[(Point, Point)],
) -> Point {
    if path.is_empty() {
        return node_center;
    }
    let mut best = &path[0];
    let mut best_len = 0.0;
    for seg in path {
        let len = segment_length(seg);
        if len > best_len {
            best_len = len;
            best = seg;
        }
    }
    let mid = segment_point(best, 0.5);
    let mut normal = Point::new(mid.x - node_center.x, mid.y - node_center.y);
    let norm = (normal.x * normal.x + normal.y * normal.y).sqrt();
    if norm > 0.0 {
        normal = Point::new(normal.x / norm, normal.y / norm);
    } else {
        normal = Point::new(0.0, 1.0);
    }
    place_label_with_offset(text, font_size, mid, normal, 0.0, blocked)
}

fn find_horizontal_lane_span(path: &[BezierSegment]) -> Option<(f64, f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    let mut best_dx = 0.0;
    let tol = 0.5;

    for seg in path {
        let dy = (seg.end.y - seg.start.y).abs();
        if dy > tol {
            continue;
        }
        if (seg.c1.y - seg.start.y).abs() > tol || (seg.c2.y - seg.start.y).abs() > tol {
            continue;
        }
        let dx = (seg.end.x - seg.start.x).abs();
        if dx <= best_dx {
            continue;
        }
        best_dx = dx;
        best = Some((
            (seg.start.x + seg.end.x) / 2.0,
            (seg.start.y + seg.end.y) / 2.0,
        ));
    }

    best.map(|(x, y)| (x, y, best_dx))
}

fn place_detour_label(
    text: &str,
    font_size: usize,
    center_x: f64,
    lane_y: f64,
    above: bool,
    blocked: &[(Point, Point)],
) -> Point {
    let mid = Point::new(center_x, lane_y);
    let normal = if above {
        Point::new(0.0, -1.0)
    } else {
        Point::new(0.0, 1.0)
    };
    let extra = (font_size as f64 * 0.6).max(DETOUR_LABEL_CLEARANCE);
    place_label_with_offset(text, font_size, mid, normal, extra, blocked)
}

fn place_label_with_normal(
    text: &str,
    font_size: usize,
    mid: Point,
    normal: Point,
    blocked: &[(Point, Point)],
) -> Point {
    place_label_with_offset(text, font_size, mid, normal, 0.0, blocked)
}

fn place_label_with_offset(
    text: &str,
    font_size: usize,
    mid: Point,
    normal: Point,
    offset: f64,
    blocked: &[(Point, Point)],
) -> Point {
    let size = get_size_for_str(text, font_size);
    let mut normal = normal;
    if normal.x == 0.0 && normal.y == 0.0 {
        normal = Point::new(0.0, -1.0);
    }
    let base_offset = EDGE_LABEL_OFFSET + offset;
    let step = font_size as f64 + 6.0;
    let mut last = Point::new(
        mid.x + normal.x * base_offset,
        mid.y + normal.y * base_offset,
    );
    for attempt in 0..6 {
        let offset = base_offset + attempt as f64 * step;
        let pos = Point::new(mid.x + normal.x * offset, mid.y + normal.y * offset);
        let bbox = label_bbox(pos, size, 2.0);
        if !blocked.iter().any(|b| rects_overlap(*b, bbox)) {
            return pos;
        }
        last = pos;
    }
    last
}

fn label_bbox(center: Point, size: Point, pad: f64) -> (Point, Point) {
    let half_w = size.x / 2.0 + pad;
    let half_h = size.y / 2.0 + pad;
    (
        Point::new(center.x - half_w, center.y - half_h),
        Point::new(center.x + half_w, center.y + half_h),
    )
}

fn rects_overlap(a: (Point, Point), b: (Point, Point)) -> bool {
    a.1.x >= b.0.x && b.1.x >= a.0.x && a.1.y >= b.0.y && b.1.y >= a.0.y
}

fn clamp_label_position(pos: Point, text: &str, font_size: usize, min: Point, max: Point) -> Point {
    let size = get_size_for_str(text, font_size);
    let half_w = size.x / 2.0 + 2.0;
    let half_h = size.y / 2.0 + 2.0;
    let min_x = min.x + half_w;
    let max_x = max.x - half_w;
    let min_y = min.y + half_h;
    let max_y = max.y - half_h;

    Point::new(pos.x.clamp(min_x, max_x), pos.y.clamp(min_y, max_y))
}

fn segment_length(seg: &BezierSegment) -> f64 {
    seg.start.distance_to(seg.c1) + seg.c1.distance_to(seg.c2) + seg.c2.distance_to(seg.end)
}

fn segment_point(seg: &BezierSegment, t: f64) -> Point {
    let u = 1.0 - t;
    let tt = t * t;
    let uu = u * u;
    let uuu = uu * u;
    let ttt = tt * t;

    let mut p = Point::new(0.0, 0.0);
    p.x = uuu * seg.start.x + 3.0 * uu * t * seg.c1.x + 3.0 * u * tt * seg.c2.x + ttt * seg.end.x;
    p.y = uuu * seg.start.y + 3.0 * uu * t * seg.c1.y + 3.0 * u * tt * seg.c2.y + ttt * seg.end.y;
    p
}

fn detour_lane_bounds_from_points(start: Point, end: Point) -> (f64, f64) {
    let dx_total = (start.x - end.x).abs().max(40.0);
    let max_dx = (dx_total / 2.0 - 10.0).max(20.0);
    let curve_dx = (dx_total * 0.25).min(max_dx);
    let left = start.x.min(end.x) + curve_dx;
    let right = start.x.max(end.x) - curve_dx;
    (left, right)
}

fn build_straight_label_slots(
    edge_points: &[(Point, Point)],
    edge_is_detour: &[bool],
    edge_is_self: &[bool],
) -> HashMap<usize, StraightLabelSlot> {
    type StraightGroupKey = (i64, i64); // (start_x_bucket, start_y_bucket)
    type StraightEdgeEntry = (usize, Point, Point); // (edge_idx, start, end)

    let mut groups: HashMap<StraightGroupKey, Vec<StraightEdgeEntry>> = HashMap::new();
    for (idx, (start, end)) in edge_points.iter().enumerate() {
        if edge_is_detour[idx] || edge_is_self[idx] {
            continue;
        }
        let key = (
            (start.x / 10.0).round() as i64,
            (start.y / 10.0).round() as i64,
        );
        groups.entry(key).or_default().push((idx, *start, *end));
    }

    let mut slots = HashMap::new();
    for (_key, mut edges) in groups {
        edges.sort_by(|a, b| {
            a.1.y
                .partial_cmp(&b.1.y)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.2.y.partial_cmp(&b.2.y).unwrap_or(Ordering::Equal))
        });

        let group_count = edges.len();
        for (slot_idx, (edge_idx, start, end)) in edges.into_iter().enumerate() {
            let center_x = (start.x + end.x) / 2.0;
            let center_y = (start.y + end.y) / 2.0;
            let span = (end.x - start.x).abs().max(1.0);
            let width = span * EDGE_LABEL_FIT_RATIO;
            let normal = if end.x >= start.x {
                Point::new(0.0, -1.0)
            } else {
                Point::new(0.0, 1.0)
            };
            slots.insert(
                edge_idx,
                StraightLabelSlot {
                    center: Point::new(center_x, center_y),
                    width,
                    normal,
                    stack_offset: slot_idx as f64 * (EDGE_FONT_SIZE as f64 + 4.0),
                    group_count,
                },
            );
        }
    }

    slots
}

fn build_detour_label_slots(
    edge_points: &[(Point, Point)],
    edge_is_detour: &[bool],
    detour_above: &[bool],
    detour_lane_y: &[f64],
) -> HashMap<usize, DetourLabelSlot> {
    type DetourLaneKey = (i64, i64, bool); // (left_bucket, right_bucket, above)
    type DetourEdgeEntry = (usize, f64, f64, f64); // (edge_idx, lane_left, lane_right, start_x)

    let mut groups: HashMap<DetourLaneKey, Vec<DetourEdgeEntry>> = HashMap::new();
    for (idx, (start, end)) in edge_points.iter().enumerate() {
        if !edge_is_detour[idx] {
            continue;
        }
        let (left, right) = detour_lane_bounds_from_points(*start, *end);
        let key = (
            (left / 10.0).round() as i64,
            (right / 10.0).round() as i64,
            detour_above[idx],
        );
        groups
            .entry(key)
            .or_default()
            .push((idx, left, right, start.x));
    }

    let mut slots = HashMap::new();
    for (_key, mut edges) in groups {
        edges.sort_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(Ordering::Equal));
        let mut left = f64::INFINITY;
        let mut right = f64::NEG_INFINITY;
        for (_, lane_left, lane_right, _) in &edges {
            left = left.min(*lane_left);
            right = right.max(*lane_right);
        }
        let width = (right - left).max(1.0);
        let count = edges.len();
        let slot_width = width / count as f64;

        for (slot_idx, (edge_idx, _, _, _)) in edges.into_iter().enumerate() {
            let center_x = left + (slot_idx as f64 + 0.5) * slot_width;
            slots.insert(
                edge_idx,
                DetourLabelSlot {
                    center_x,
                    width: slot_width * 0.9,
                    lane_y: detour_lane_y[edge_idx],
                    above: detour_above[edge_idx],
                    group_count: count,
                    group_width: width,
                },
            );
        }
    }

    slots
}

fn fit_label_to_width(label: &str, max_width: f64, base_size: usize) -> (String, usize) {
    if max_width <= 0.0 {
        return (String::new(), base_size);
    }
    let mut candidate = shorten_module_path(label, max_width, base_size);
    let width = get_size_for_str(&candidate, base_size).x;
    if width <= max_width {
        return (candidate, base_size);
    }

    let mut max_chars = (max_width / base_size as f64).floor() as usize;
    if max_chars == 0 {
        max_chars = 1;
    }
    candidate = truncate_label_left(&candidate, max_chars);
    (candidate, base_size)
}

fn path_label_anchor(path: &[BezierSegment]) -> (Point, Point) {
    if path.is_empty() {
        return (Point::new(0.0, 0.0), Point::new(1.0, 0.0));
    }

    let mut best_score = 0.0;
    let mut best_mid = path[0].start;
    let mut best_dir = Point::new(1.0, 0.0);
    for seg in path {
        let a = seg.start;
        let b = seg.end;
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let len = (dx * dx + dy * dy).sqrt();
        if len <= 0.0 {
            continue;
        }
        let horiz_bonus = if dx.abs() >= dy.abs() { 50.0 } else { 0.0 };
        let score = len + horiz_bonus;
        if score > best_score {
            best_score = score;
            let t = 0.5;
            best_mid = Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            best_dir = Point::new(dx / len, dy / len);
        }
    }

    (best_mid, best_dir)
}

fn fit_edge_label(label: &str, path: &[BezierSegment], base_size: usize) -> (String, usize) {
    if label.is_empty() || path.is_empty() {
        return (label.to_string(), base_size);
    }
    let approx_len = approximate_path_length(path);
    let available = approx_len * EDGE_LABEL_FIT_RATIO;
    if available <= 0.0 {
        return (label.to_string(), base_size);
    }

    fit_label_to_width(label, available, base_size)
}

fn direction_unit(start: Point, end: Point) -> Point {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let len = (dx * dx + dy * dy).sqrt();
    if len <= 0.0 {
        Point::new(1.0, 0.0)
    } else {
        Point::new(dx / len, dy / len)
    }
}

fn approximate_path_length(path: &[BezierSegment]) -> f64 {
    let mut length = 0.0;
    for seg in path {
        length += seg.start.distance_to(seg.c1);
        length += seg.c1.distance_to(seg.c2);
        length += seg.c2.distance_to(seg.end);
    }
    length
}

fn split_type_tokens(label: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut buf = String::new();
    let chars: Vec<char> = label.chars().collect();
    let mut idx = 0;
    while idx < chars.len() {
        let ch = chars[idx];
        if ch == ':' && idx + 1 < chars.len() && chars[idx + 1] == ':' {
            if !buf.is_empty() {
                tokens.push(buf.clone());
                buf.clear();
            }
            tokens.push("::".to_string());
            idx += 2;
            continue;
        }

        if ch == '<' || ch == '>' || ch == ',' {
            if !buf.is_empty() {
                tokens.push(buf.clone());
                buf.clear();
            }
            tokens.push(ch.to_string());
            idx += 1;
            continue;
        }

        if ch.is_whitespace() {
            if !buf.is_empty() {
                tokens.push(buf.clone());
                buf.clear();
            }
            idx += 1;
            continue;
        }

        buf.push(ch);
        idx += 1;
    }

    if !buf.is_empty() {
        tokens.push(buf);
    }

    tokens
}

fn shorten_module_path(label: &str, max_width: f64, font_size: usize) -> String {
    let segments: Vec<&str> = label.split("::").collect();
    if segments.len() <= 1 {
        return label.to_string();
    }

    for keep in (1..=segments.len()).rev() {
        let slice = &segments[segments.len() - keep..];
        let mut candidate = slice.join(MODULE_SEPARATOR);
        if keep < segments.len() {
            candidate = format!("{MODULE_TRUNC_MARKER}{MODULE_SEPARATOR}{candidate}");
        }
        if get_size_for_str(&candidate, font_size).x <= max_width {
            return candidate;
        }
    }

    format!(
        "{MODULE_TRUNC_MARKER}{MODULE_SEPARATOR}{}",
        segments.last().unwrap_or(&label)
    )
}

fn truncate_label_left(label: &str, max_chars: usize) -> String {
    if max_chars == 0 {
        return String::new();
    }
    let count = label.chars().count();
    if count <= max_chars {
        return label.to_string();
    }
    let keep = max_chars.saturating_sub(1);
    let tail: String = label
        .chars()
        .rev()
        .take(keep)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();
    format!("{MODULE_TRUNC_MARKER}{tail}")
}

fn strip_type_params(label: &str) -> String {
    let mut depth = 0usize;
    let mut out = String::new();
    for ch in label.chars() {
        match ch {
            '<' => {
                depth += 1;
            }
            '>' => {
                depth = depth.saturating_sub(1);
            }
            _ => {
                if depth == 0 {
                    out.push(ch);
                }
            }
        }
    }
    out
}

fn provider_resource_slots(provider: &str) -> Option<&'static [&'static str]> {
    let provider = strip_type_params(provider);
    let compact: String = provider.chars().filter(|ch| !ch.is_whitespace()).collect();
    let segments: Vec<&str> = compact
        .split("::")
        .filter(|segment| !segment.is_empty())
        .collect();
    let is_linux_bundle = segments.last().copied() == Some("LinuxResources")
        && segments
            .iter()
            .any(|segment| *segment == "cu_linux_resources");
    if is_linux_bundle {
        Some(&LINUX_RESOURCE_SLOT_NAMES)
    } else {
        None
    }
}

fn scale_layout_positions(graph: &mut VisualGraph) {
    for handle in graph.iter_nodes() {
        let center = graph.element(handle).position().center();
        let scaled = Point::new(center.x * LAYOUT_SCALE_X, center.y * LAYOUT_SCALE_Y);
        graph.element_mut(handle).position_mut().move_to(scaled);
    }
}

fn build_edge_hover_overlay(
    path: &[BezierSegment],
    tooltip: &str,
    stroke_color: &str,
    line_width: usize,
) -> (Group, Point, Point) {
    let hitbox_width = line_width.max(EDGE_HITBOX_STROKE_WIDTH);
    let mut hover_group = Group::new().set("class", "edge-hover");
    let mut hitbox_el = SvgPath::new()
        .set("d", build_path_data(path))
        .set("stroke", stroke_color)
        .set("stroke-opacity", EDGE_HITBOX_OPACITY)
        .set("stroke-width", hitbox_width)
        .set("fill", "none")
        .set("pointer-events", "stroke")
        .set("cursor", "help");
    hitbox_el.append(Title::new(tooltip));
    hover_group.append(hitbox_el);

    let anchor = tooltip_anchor_for_path(path);
    let hover_point = Circle::new()
        .set("class", "edge-hover-point")
        .set("cx", anchor.x)
        .set("cy", anchor.y)
        .set("r", EDGE_HOVER_POINT_RADIUS)
        .set("fill", BACKGROUND_COLOR)
        .set("stroke", stroke_color)
        .set("stroke-width", EDGE_HOVER_POINT_STROKE_WIDTH);
    hover_group.append(hover_point);

    let (tooltip_group, tooltip_top_left, tooltip_size) = build_edge_tooltip_group(path, tooltip);
    hover_group.append(tooltip_group);

    (hover_group, tooltip_top_left, tooltip_size)
}

fn build_edge_tooltip_group(path: &[BezierSegment], tooltip: &str) -> (Group, Point, Point) {
    let lines: Vec<&str> = if tooltip.is_empty() {
        vec![""]
    } else {
        tooltip.lines().collect()
    };
    let line_height = tooltip_line_height();
    let mut max_width: f64 = 0.0;
    for line in &lines {
        let size = get_size_for_str(line, TOOLTIP_FONT_SIZE);
        max_width = max_width.max(size.x);
    }
    let content_height = line_height * (lines.len() as f64);
    let box_width = max_width + TOOLTIP_PADDING * 2.0;
    let box_height = content_height + TOOLTIP_PADDING * 2.0;
    let anchor = tooltip_anchor_for_path(path);
    let top_left = Point::new(
        anchor.x + TOOLTIP_OFFSET_X,
        anchor.y - TOOLTIP_OFFSET_Y - box_height,
    );

    let mut group = Group::new()
        .set("class", "edge-tooltip")
        .set("pointer-events", "none");
    let rect = Rectangle::new()
        .set("x", top_left.x)
        .set("y", top_left.y)
        .set("width", box_width)
        .set("height", box_height)
        .set("rx", TOOLTIP_RADIUS)
        .set("ry", TOOLTIP_RADIUS)
        .set("fill", TOOLTIP_BG)
        .set("stroke", TOOLTIP_BORDER)
        .set("stroke-width", TOOLTIP_BORDER_WIDTH);
    group.append(rect);

    let text_x = top_left.x + TOOLTIP_PADDING;
    let mut text_y = top_left.y + TOOLTIP_PADDING;
    for line in lines {
        let text = Text::new(line)
            .set("x", text_x)
            .set("y", text_y)
            .set("dominant-baseline", "hanging")
            .set("font-family", MONO_FONT_FAMILY)
            .set("font-size", format!("{TOOLTIP_FONT_SIZE}px"))
            .set("fill", TOOLTIP_TEXT);
        group.append(text);
        text_y += line_height;
    }

    (group, top_left, Point::new(box_width, box_height))
}

fn tooltip_line_height() -> f64 {
    TOOLTIP_FONT_SIZE as f64 + TOOLTIP_LINE_GAP
}

fn tooltip_anchor_for_path(path: &[BezierSegment]) -> Point {
    let mut min = Point::new(f64::INFINITY, f64::INFINITY);
    let mut max = Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for segment in path {
        for point in [segment.start, segment.c1, segment.c2, segment.end] {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
        }
    }
    if !min.x.is_finite() || !min.y.is_finite() {
        return Point::new(0.0, 0.0);
    }
    Point::new((min.x + max.x) / 2.0, (min.y + max.y) / 2.0)
}

fn format_edge_tooltip(stats: &EdgeLogStats) -> String {
    [
        format!("Message: {}", stats.msg),
        format!(
            "Message size (avg): {}",
            format_bytes_opt(stats.avg_raw_bytes)
        ),
        format!(
            "Rate: {}",
            format_rate_bytes_per_sec(stats.throughput_bytes_per_sec)
        ),
        format!(
            "None: {}",
            format_none_ratio(stats.none_samples, stats.samples)
        ),
        format!("Message rate: {}", format_rate_hz(stats.rate_hz)),
        format!(
            "Total bytes: {}",
            format_bytes(stats.total_raw_bytes as f64)
        ),
        format!(
            "Time samples: {}/{}",
            stats.valid_time_samples, stats.samples
        ),
    ]
    .join("\n")
}

fn format_none_ratio(none_samples: u64, samples: u64) -> String {
    if samples == 0 {
        return "n/a".to_string();
    }
    let ratio = (none_samples as f64) / (samples as f64) * 100.0;
    format!("{ratio:.1}% ({none_samples}/{samples})")
}

fn format_rate_hz(rate: Option<f64>) -> String {
    rate.map_or_else(|| "n/a".to_string(), |value| format!("{value:.2} Hz"))
}

fn format_rate_bytes_per_sec(value: Option<f64>) -> String {
    value.map_or_else(|| "n/a".to_string(), format_rate_units)
}

fn format_rate_units(bytes: f64) -> String {
    const UNITS: [&str; 4] = ["B/s", "KB/s", "MB/s", "GB/s"];
    let mut value = bytes;
    let mut unit_idx = 0;
    while value >= 1000.0 && unit_idx < UNITS.len() - 1 {
        value /= 1000.0;
        unit_idx += 1;
    }
    let formatted = if unit_idx == 0 {
        format!("{value:.0}")
    } else if value < 10.0 {
        format!("{value:.2}")
    } else if value < 100.0 {
        format!("{value:.1}")
    } else {
        format!("{value:.0}")
    };
    format!("{formatted} {}", UNITS[unit_idx])
}

fn format_bytes_opt(value: Option<f64>) -> String {
    value.map_or_else(|| "n/a".to_string(), format_bytes)
}

fn format_bytes(bytes: f64) -> String {
    const UNITS: [&str; 4] = ["B", "KiB", "MiB", "GiB"];
    let mut value = bytes;
    let mut unit_idx = 0;
    while value >= 1024.0 && unit_idx < UNITS.len() - 1 {
        value /= 1024.0;
        unit_idx += 1;
    }
    if unit_idx == 0 {
        format!("{value:.0} {}", UNITS[unit_idx])
    } else {
        format!("{value:.2} {}", UNITS[unit_idx])
    }
}

fn format_duration_ns_f64(value: Option<f64>) -> String {
    value.map_or_else(|| "n/a".to_string(), format_duration_ns)
}

fn format_duration_ns_u64(value: Option<u64>) -> String {
    value.map_or_else(
        || "n/a".to_string(),
        |nanos| format_duration_ns(nanos as f64),
    )
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

fn mission_key(mission: Option<&str>) -> &str {
    match mission {
        Some(value) if value != "default" => value,
        _ => "default",
    }
}

fn build_graph_signature(config: &config::CuConfig, mission: Option<&str>) -> CuResult<String> {
    let graph = config.get_graph(mission)?;
    let mut parts = Vec::new();
    parts.push(format!("mission={}", mission.unwrap_or("default")));

    let mut nodes: Vec<_> = graph.get_all_nodes();
    nodes.sort_by(|a, b| a.1.get_id().cmp(&b.1.get_id()));
    for (_, node) in nodes {
        parts.push(format!(
            "node|{}|{}|{}",
            node.get_id(),
            node.get_type(),
            flavor_label(node.get_flavor())
        ));
    }

    let mut edges: Vec<String> = graph
        .edges()
        .map(|cnx| {
            format!(
                "edge|{}|{}|{}",
                format_endpoint(cnx.src.as_str(), cnx.src_channel.as_deref()),
                format_endpoint(cnx.dst.as_str(), cnx.dst_channel.as_deref()),
                cnx.msg
            )
        })
        .collect();
    edges.sort();
    parts.extend(edges);

    let joined = parts.join("\n");
    Ok(format!("fnv1a64:{:016x}", fnv1a64(joined.as_bytes())))
}

fn format_endpoint(node: &str, channel: Option<&str>) -> String {
    match channel {
        Some(ch) => format!("{node}/{ch}"),
        None => node.to_string(),
    }
}

fn flavor_label(flavor: config::Flavor) -> &'static str {
    match flavor {
        config::Flavor::Task => "task",
        config::Flavor::Bridge => "bridge",
    }
}

fn fnv1a64(data: &[u8]) -> u64 {
    const OFFSET_BASIS: u64 = 0xcbf29ce484222325;
    const PRIME: u64 = 0x100000001b3;
    let mut hash = OFFSET_BASIS;
    for byte in data {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tooltip_formats_missing_values() {
        let stats = EdgeLogStats {
            src: "a".to_string(),
            src_channel: None,
            dst: "b".to_string(),
            dst_channel: None,
            msg: "Msg".to_string(),
            samples: 0,
            none_samples: 0,
            valid_time_samples: 0,
            total_raw_bytes: 0,
            avg_raw_bytes: None,
            rate_hz: None,
            throughput_bytes_per_sec: None,
        };
        let tooltip = format_edge_tooltip(&stats);
        assert!(tooltip.contains("Message size (avg): n/a"));
        assert!(tooltip.contains("Rate: n/a"));
        assert!(tooltip.contains("None: n/a"));
    }

    #[test]
    fn provider_slot_matching_handles_type_params() {
        assert_eq!(
            provider_resource_slots("cu_linux_resources::LinuxResources"),
            Some(&LINUX_RESOURCE_SLOT_NAMES[..])
        );
        assert_eq!(
            provider_resource_slots(" crate::x::cu_linux_resources::LinuxResources < Foo<Bar> > "),
            Some(&LINUX_RESOURCE_SLOT_NAMES[..])
        );
        assert!(provider_resource_slots("board::MicoAirH743").is_none());
    }

    #[test]
    fn linux_bundle_catalog_includes_known_slots_without_bindings() {
        let config = config::CuConfig {
            monitor: None,
            logging: None,
            runtime: None,
            resources: vec![config::ResourceBundleConfig {
                id: "linux".to_string(),
                provider: "cu_linux_resources::LinuxResources".to_string(),
                config: None,
                missions: None,
            }],
            bridges: Vec::new(),
            graphs: ConfigGraphs::Simple(config::CuGraph::default()),
        };

        let catalog = collect_resource_catalog(&config).expect("catalog should build");
        let linux_slots = catalog
            .get("linux")
            .expect("linux bundle should expose slot catalog");
        assert_eq!(linux_slots.len(), LINUX_RESOURCE_SLOT_NAMES.len());
        for slot in LINUX_RESOURCE_SLOT_NAMES {
            assert!(linux_slots.contains(slot), "missing slot {slot}");
        }
    }
}
