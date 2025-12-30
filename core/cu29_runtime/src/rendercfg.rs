mod config;
use clap::Parser;
use config::{build_render_topology, read_configuration, ConfigGraphs};
pub use cu29_traits::*;
use layout::adt::dag::NodeHandle;
use layout::core::base::Orientation;
use layout::core::color::Color;
use layout::core::format::{RenderBackend, Visible};
use layout::core::geometry::{get_size_for_str, pad_shape_scalar, Point};
use layout::core::style::{LineStyleKind, StyleAttr};
use layout::std_shapes::shapes::{Arrow, Element, LineEndKind, RecordDef, ShapeKind};
use layout::topo::layout::VisualGraph;
use std::collections::HashMap;
use std::cmp::Ordering;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;
use tempfile::Builder;

const FONT_FAMILY: &str = "'Noto Sans', sans-serif";
const MONO_FONT_FAMILY: &str = "'Noto Sans Mono'";
const FONT_SIZE: usize = 12;
const TYPE_FONT_SIZE: usize = FONT_SIZE * 7 / 10;
const PORT_HEADER_FONT_SIZE: usize = FONT_SIZE * 4 / 6;
const PORT_VALUE_FONT_SIZE: usize = FONT_SIZE * 4 / 6;
const CONFIG_FONT_SIZE: usize = PORT_VALUE_FONT_SIZE - 1;
const EDGE_FONT_SIZE: usize = 9;
const EDGE_LABEL_FIT_RATIO: f64 = 0.8;
const EDGE_LABEL_OFFSET: f64 = 8.0;
const EDGE_LABEL_LIGHTEN: f64 = 0.35;
const MODULE_TRUNC_MARKER: &str = "…";
const MODULE_SEPARATOR: &str = "⠶";
const BACK_EDGE_STACK_SPACING: f64 = 16.0;
const BACK_EDGE_NODE_GAP: f64 = 12.0;
const BACK_EDGE_DUP_SPACING: f64 = 6.0;
const BACK_EDGE_SPAN_EPS: f64 = 4.0;
const DETOUR_LABEL_CLEARANCE: f64 = 6.0;
const EDGE_STUB_LEN: f64 = 32.0;
const EDGE_STUB_MIN: f64 = 18.0;
const INTERMEDIATE_X_EPS: f64 = 6.0;
const BORDER_COLOR: &str = "#999999";
const HEADER_BG: &str = "#f4f4f4";
const VALUE_BORDER_WIDTH: f64 = 0.6;
const OUTER_BORDER_WIDTH: f64 = 1.3;
const EDGE_PORT_HANDLE: f64 = 12.0;
const ARROW_HEAD_MARGIN: f64 = 6.0;
const DIM_GRAY: &str = "dimgray";
const LIGHT_GRAY: &str = "lightgray";
const CLUSTER_COLOR: &str = "#bbbbbb";
const CLUSTER_MARGIN: f64 = 20.0;
const GRAPH_MARGIN: f64 = 20.0;
const SECTION_SPACING: f64 = 60.0;
const BOX_SHAPE_PADDING: f64 = 10.0;
const CELL_PADDING: f64 = 6.0;
const CELL_LINE_SPACING: f64 = 2.0;
const PLACEHOLDER_TEXT: &str = "\u{2014}";
const TYPE_WRAP_WIDTH: usize = 24;
const CONFIG_WRAP_WIDTH: usize = 32;
const LAYOUT_SCALE_X: f64 = 1.4;
const LAYOUT_SCALE_Y: f64 = 1.1;

const EDGE_COLOR_PALETTE: [&str; 10] = [
    "#1F77B4",
    "#FF7F0E",
    "#2CA02C",
    "#D62728",
    "#9467BD",
    "#8C564B",
    "#E377C2",
    "#7F7F7F",
    "#BCBD22",
    "#17BECF",
];

#[derive(Parser)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Config file name
    #[clap(value_parser)]
    config: PathBuf,
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

    let graph_svg = match render_config_svg(&config, mission.as_deref()) {
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
            .map_err(|err| std::io::Error::new(std::io::ErrorKind::Other, err))?;

        open_svg(&temp_path)?;
    } else {
        // Write the SVG content to a file
        let mut svg_file = std::fs::File::create("output.svg")?;
        svg_file.write_all(graph_svg.as_slice())?;
    }
    Ok(())
}

fn open_svg(path: &std::path::Path) -> std::io::Result<()> {
    if cfg!(target_os = "windows") {
        Command::new("cmd")
            .args(["/C", "start", ""])
            .arg(path)
            .status()?;
        return Ok(());
    }

    let program = if cfg!(target_os = "macos") { "open" } else { "xdg-open" };
    Command::new(program).arg(path).status()?;
    Ok(())
}

fn render_config_svg(config: &config::CuConfig, mission_id: Option<&str>) -> CuResult<Vec<u8>> {
    let sections = build_sections(config, mission_id)?;
    let mut layouts = Vec::new();
    for section in sections {
        layouts.push(build_section_layout(config, &section)?);
    }

    Ok(render_sections_to_svg(&layouts).into_bytes())
}

fn build_sections<'a>(
    config: &'a config::CuConfig,
    mission_id: Option<&str>,
) -> CuResult<Vec<SectionRef<'a>>> {
    let sections = match (&config.graphs, mission_id) {
        (ConfigGraphs::Simple(graph), _) => vec![SectionRef { label: None, graph }],
        (ConfigGraphs::Missions(graphs), Some(id)) => {
            let graph = graphs
                .get(id)
                .ok_or_else(|| CuError::from(format!("Mission {id} not found")))?;
            vec![SectionRef {
                label: Some(id.to_string()),
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
                    graph,
                })
                .collect()
        }
    };

    Ok(sections)
}

fn build_section_layout(
    config: &config::CuConfig,
    section: &SectionRef<'_>,
) -> CuResult<SectionLayout> {
    let mut topology = build_render_topology(section.graph, &config.bridges);
    topology.connections.sort_by(|a, b| {
        a.src
            .cmp(&b.src)
            .then(a.dst.cmp(&b.dst))
            .then(a.msg.cmp(&b.msg))
    });

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
            config::Flavor::Bridge => "#f7d7e4",
            config::Flavor::Task if is_src => "#ddefc7",
            config::Flavor::Task if is_sink => "#cce0ff",
            _ => "#fde7c2",
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
    let mut label_seen: HashMap<(NodeHandle, NodeHandle, String), bool> = HashMap::new();
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
        let label_key = (*src_handle, *dst_handle, cnx.msg.clone());
        let show_label = !label_seen.contains_key(&label_key);
        label_seen.insert(label_key, true);
        edges.push(RenderEdge {
            src: *src_handle,
            dst: *dst_handle,
            arrow,
            label: if show_label {
                cnx.msg.clone()
            } else {
                String::new()
            },
            src_port,
            dst_port,
        });
    }

    let mut null_backend = NullBackend;
    graph.do_it(false, false, false, &mut null_backend);
    scale_layout_positions(&mut graph);

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

    Ok(SectionLayout {
        label: section.label.clone(),
        graph,
        nodes,
        edges,
        bounds: (min, max),
        port_anchors,
    })
}

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
        TableCell::new(header_lines).with_background(header_fill),
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
    );
    let outputs = build_port_column(
        "Outputs",
        &node.outputs,
        "out",
        &mut port_lookup.outputs,
        &mut port_lookup.default_output,
        max_ports,
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

fn build_port_column(
    title: &str,
    names: &[String],
    prefix: &str,
    lookup: &mut HashMap<String, String>,
    default_port: &mut Option<String>,
    target_len: usize,
) -> TableNode {
    let mut rows = Vec::new();
    rows.push(TableNode::Cell(
        TableCell::single_line_sized(title, "black", false, PORT_HEADER_FONT_SIZE)
            .with_background(HEADER_BG),
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
                    .with_border_width(VALUE_BORDER_WIDTH),
            ));
        } else {
            rows.push(TableNode::Cell(
                TableCell::single_line_sized(
                    PLACEHOLDER_TEXT,
                    LIGHT_GRAY,
                    false,
                    PORT_VALUE_FONT_SIZE,
                )
                .with_border_width(VALUE_BORDER_WIDTH),
            ));
        }
    }

    TableNode::Array(rows)
}

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
            key_lines.push(CellLine::code(
                key_text,
                DIM_GRAY,
                true,
                CONFIG_FONT_SIZE,
            ));
            value_lines.push(CellLine::code(
                *part,
                DIM_GRAY,
                false,
                CONFIG_FONT_SIZE,
            ));
        }
    }

    let keys_cell = TableCell::new(key_lines).with_border_width(VALUE_BORDER_WIDTH);
    let values_cell = TableCell::new(value_lines).with_border_width(VALUE_BORDER_WIDTH);
    let body = TableNode::Array(vec![TableNode::Cell(keys_cell), TableNode::Cell(values_cell)]);

    vec![header, body]
}

fn table_to_record(node: &TableNode) -> RecordDef {
    match node {
        TableNode::Cell(cell) => RecordDef::Text(cell.label(), cell.port.clone()),
        TableNode::Array(children) => {
            RecordDef::Array(children.iter().map(table_to_record).collect())
        }
    }
}

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

fn render_sections_to_svg(sections: &[SectionLayout]) -> String {
    let mut svg = SvgWriter::new();
    let mut cursor_y = GRAPH_MARGIN;

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
        let node_bounds = collect_node_bounds(section);
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
                && span_has_intermediate(
                    &node_bounds,
                    span_min_x,
                    span_max_x,
                    edge.src,
                    edge.dst,
                );
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
                let plan = BackEdgePlan { idx, span };
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
        let detour_slots = build_detour_label_slots(
            &edge_points,
            &edge_is_detour,
            &detour_above,
            &detour_lane_y,
        );

        for (idx, edge) in section.edges.iter().enumerate() {
            let (src_point, dst_point) = edge_points[idx];
            let (fallback_start_dir, fallback_end_dir) =
                fallback_port_dirs(src_point, dst_point);
            let start_dir =
                port_dir(edge.src_port.as_ref()).unwrap_or(fallback_start_dir);
            let end_dir = port_dir_incoming(edge.dst_port.as_ref()).unwrap_or(fallback_end_dir);
            let path = if edge.src == edge.dst {
                let pos = section.graph.element(edge.src).position();
                let bbox = pos.bbox(false);
                build_loop_path(src_point, dst_point, bbox, start_dir, end_dir)
            } else if edge_is_detour[idx] {
                let lane_span = detour_slots.get(&idx).map(|slot| {
                    let mid_x = (src_point.x + dst_point.x) / 2.0;
                    let half = slot.width / 2.0;
                    (
                        mid_x - half,
                        mid_x + half,
                    )
                });
                build_back_edge_path(
                    src_point,
                    dst_point,
                    detour_lane_y[idx],
                    start_dir,
                    end_dir,
                    lane_span,
                )
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
        let label_bounds_min = Point::new(
            cluster_top_left.x + 4.0,
            cluster_top_left.y + label_padding + 4.0,
        );
        let label_bounds_max = Point::new(
            cluster_bottom_right.x - 4.0,
            cluster_bottom_right.y - 4.0,
        );

        if let Some(label) = &section.label {
            draw_cluster(&mut svg, section_min, section_max, label, offset);
        }

        let mut blocked_boxes: Vec<(Point, Point)> = node_bounds
            .iter()
            .map(|b| {
                (
                    Point::new(b.left, b.top).add(content_offset).sub(Point::new(4.0, 4.0)),
                    Point::new(b.right, b.bottom).add(content_offset).add(Point::new(4.0, 4.0)),
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
                Point::new(label_pos.x, label_pos.y - label_size.y / 2.0)
                    .sub(Point::new(2.0, 2.0)),
                Point::new(label_pos.x + label_size.x, label_pos.y + label_size.y / 2.0)
                    .add(Point::new(2.0, 2.0)),
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
            let color_idx = edge_color_index(edge, idx);
            let line_color = EDGE_COLOR_PALETTE[color_idx];
            let label = if edge.label.is_empty() {
                None
            } else {
                let (text, font_size) = if edge_is_self[idx] {
                    fit_edge_label(&edge.label, &path, EDGE_FONT_SIZE)
                } else if let Some(slot) = straight_slots.get(&idx) {
                    fit_label_to_width(&edge.label, slot.width, EDGE_FONT_SIZE)
                } else if let Some(slot) = detour_slots.get(&idx) {
                    fit_label_to_width(&edge.label, slot.width, EDGE_FONT_SIZE)
                } else if edge_is_detour[idx] {
                    let (lane_left, lane_right) = detour_lane_bounds_from_points(
                        edge_points[idx].0,
                        edge_points[idx].1,
                    );
                    fit_label_to_width(
                        &edge.label,
                        (lane_right - lane_left).max(1.0),
                        EDGE_FONT_SIZE,
                    )
                } else {
                    fit_edge_label(&edge.label, &path, EDGE_FONT_SIZE)
                };
                let label_color = lighten_hex(line_color, EDGE_LABEL_LIGHTEN);
                let mut label = ArrowLabel::new(
                    text,
                    &label_color,
                    font_size,
                    true,
                    FontFamily::Mono,
                );
            let label_pos = if edge_is_self[idx] {
                    let node_center = section
                        .graph
                        .element(edge.src)
                        .position()
                        .center()
                        .add(content_offset);
                    place_self_loop_label(
                        &label.text,
                        label.font_size,
                        &path,
                        node_center,
                        &blocked_boxes,
                    )
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
            svg.draw_arrow(
                &path,
                dashed,
                (start, end),
                &edge_look,
                label.as_ref(),
            );
        }

        for node in &section.nodes {
            let element = section.graph.element(node.handle);
            draw_node_table(&mut svg, node, element, content_offset);
        }

        cursor_y += (section_max.y - section_min.y) + SECTION_SPACING;
    }

    svg.finalize()
}

fn draw_node_table(svg: &mut SvgWriter, node: &NodeRender, element: &Element, offset: Point) {
    let pos = element.position();
    let center = pos.center().add(offset);
    let size = pos.size(false);
    let top_left = Point::new(center.x - size.x / 2.0, center.y - size.y / 2.0);

    svg.draw_rect(top_left, size, None, 0.0, Some("white"), 0.0);

    let mut renderer = TableRenderer { svg };
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

fn format_mission_list(graphs: &hashbrown::HashMap<String, config::CuGraph>) -> String {
    let mut missions: Vec<_> = graphs.keys().cloned().collect();
    missions.sort();
    missions.join(", ")
}

struct SectionRef<'a> {
    label: Option<String>,
    graph: &'a config::CuGraph,
}

struct SectionLayout {
    label: Option<String>,
    graph: VisualGraph,
    nodes: Vec<NodeRender>,
    edges: Vec<RenderEdge>,
    bounds: (Point, Point),
    port_anchors: HashMap<NodeHandle, HashMap<String, Point>>,
}

struct NodeRender {
    handle: NodeHandle,
    table: TableNode,
}

struct RenderEdge {
    src: NodeHandle,
    dst: NodeHandle,
    arrow: Arrow,
    label: String,
    src_port: Option<String>,
    dst_port: Option<String>,
}

#[derive(Default)]
struct PortLookup {
    inputs: HashMap<String, String>,
    outputs: HashMap<String, String>,
    default_input: Option<String>,
    default_output: Option<String>,
}

impl PortLookup {
    fn resolve_input(&self, name: Option<&str>) -> Option<&str> {
        if let Some(name) = name
            && let Some(port) = self.inputs.get(name)
        {
            return Some(port.as_str());
        }
        self.default_input.as_deref()
    }

    fn resolve_output(&self, name: Option<&str>) -> Option<&str> {
        if let Some(name) = name
            && let Some(port) = self.outputs.get(name)
        {
            return Some(port.as_str());
        }
        self.default_output.as_deref()
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
}

impl TableCell {
    fn new(lines: Vec<CellLine>) -> Self {
        Self {
            lines,
            port: None,
            background: None,
            border_width: 1.0,
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

    fn label(&self) -> String {
        self.lines
            .iter()
            .map(|line| line.text.as_str())
            .collect::<Vec<_>>()
            .join("\n")
    }
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

        if cell.lines.is_empty() {
            return;
        }

        let total_height = cell_text_height(cell);
        let mut current_y = loc.y - total_height / 2.0;
        let text_x = loc.x - size.x / 2.0 + CELL_PADDING;

        for (idx, line) in cell.lines.iter().enumerate() {
            let line_height = line.font_size as f64;
            let y = current_y + line_height / 2.0;
            self.svg.draw_text(
                Point::new(text_x, y),
                &line.text,
                line.font_size,
                &line.color,
                line.bold,
                "start",
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
}

struct DetourLabelSlot {
    center_x: f64,
    width: f64,
    lane_y: f64,
    above: bool,
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
    content: String,
    view_size: Point,
    counter: usize,
}

impl SvgWriter {
    fn new() -> Self {
        Self {
            content: String::new(),
            view_size: Point::new(0.0, 0.0),
            counter: 0,
        }
    }

    fn grow_window(&mut self, point: Point, size: Point) {
        self.view_size.x = self.view_size.x.max(point.x + size.x + 5.0);
        self.view_size.y = self.view_size.y.max(point.y + size.y + 5.0);
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
        let line = format!(
            "<rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" fill=\"{}\" stroke=\"{}\" stroke-width=\"{}\" rx=\"{}\" ry=\"{}\" />\n",
            top_left.x,
            top_left.y,
            size.x,
            size.y,
            fill_color,
            stroke_color,
            width,
            rounded,
            rounded
        );
        self.content.push_str(&line);
    }

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

        let escaped = escape_xml(text);
        let weight = if bold { "bold" } else { "normal" };
        let line = format!(
            "<text x=\"{}\" y=\"{}\" text-anchor=\"{}\" dominant-baseline=\"middle\" style=\"font-family:{}; font-size:{}px; fill:{}; font-weight:{};\">{}</text>\n",
            pos.x,
            pos.y,
            anchor,
            family.as_css(),
            font_size,
            color,
            weight,
            escaped
        );
        self.content.push_str(&line);

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

        let dash = if dashed {
            "stroke-dasharray=\"5,5\""
        } else {
            ""
        };
        let start = if head.0 {
            "marker-start=\"url(#startarrow)\""
        } else {
            ""
        };
        let end = if head.1 {
            "marker-end=\"url(#endarrow)\""
        } else {
            ""
        };
        let stroke_color = look.line_color.to_web_color();
        let stroke_color = normalize_web_color(&stroke_color);

        let path_builder = build_path_string(path);

        let path_id = format!("arrow{}", self.counter);
        let line = format!(
            "<path id=\"{}\" d=\"{}\" stroke=\"{}\" stroke-width=\"{}\" {} {} {} fill=\"none\" />\n",
            path_id,
            path_builder.trim(),
            stroke_color,
            look.line_width,
            dash,
            start,
            end
        );
        self.content.push_str(&line);

        if let Some(label) = label {
            if label.text.is_empty() {
                self.counter += 1;
                return;
            }
            let escaped = escape_xml(&label.text);
            let weight = if label.bold { "bold" } else { "normal" };
            if let Some(pos) = label.position {
                let line = format!(
                    "<text x=\"{}\" y=\"{}\" text-anchor=\"middle\" dominant-baseline=\"middle\" style=\"font-family:{}; font-size:{}px; fill:{}; font-weight:{};\">{}</text>\n",
                    pos.x,
                    pos.y,
                    label.font_family.as_css(),
                    label.font_size,
                    label.color,
                    weight,
                    escaped
                );
                self.content.push_str(&line);
            } else {
                let label_path_id = format!("{}_label", path_id);
                let start = path[0].start;
                let end = path[path.len() - 1].end;
                let label_path_data = build_explicit_path_string(path, start.x > end.x);
                let label_path_el = format!(
                    "<path id=\"{}\" d=\"{}\" fill=\"none\" stroke=\"none\" />\n",
                    label_path_id,
                    label_path_data
                );
                self.content.push_str(&label_path_el);

                let line = format!(
                    "<text><textPath href=\"#{}\" startOffset=\"50%\" text-anchor=\"middle\" dy=\"{}\" style=\"font-family:{}; font-size:{}px; fill:{}; font-weight:{};\">{}</textPath></text>\n",
                    label_path_id,
                    EDGE_LABEL_OFFSET,
                    label.font_family.as_css(),
                    label.font_size,
                    label.color,
                    weight,
                    escaped
                );
                self.content.push_str(&line);
            }
        }

        self.counter += 1;
    }

    fn finalize(self) -> String {
        let width = if self.view_size.x < 1.0 {
            1.0
        } else {
            self.view_size.x
        };
        let height = if self.view_size.y < 1.0 {
            1.0
        } else {
            self.view_size.y
        };
        let header = format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n<svg width=\"{}\" height=\"{}\" viewBox=\"0 0 {} {}\" xmlns=\"http://www.w3.org/2000/svg\">\n",
            width, height, width, height
        );
        let defs = r#"<defs>
<marker id="startarrow" markerWidth="10" markerHeight="7" refX="0" refY="3.5" orient="auto">
<polygon points="10 0, 10 7, 0 3.5" fill="context-stroke" />
</marker>
<marker id="endarrow" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
<polygon points="0 0, 10 3.5, 0 7" fill="context-stroke" />
</marker>
</defs>\n"#;
        let footer = "</svg>";
        format!("{}{}{}{}", header, defs, self.content, footer)
    }
}

fn escape_xml(input: &str) -> String {
    let mut res = String::new();
    for ch in input.chars() {
        match ch {
            '&' => res.push_str("&amp;"),
            '<' => res.push_str("&lt;"),
            '>' => res.push_str("&gt;"),
            '"' => res.push_str("&quot;"),
            '\'' => res.push_str("&apos;"),
            _ => res.push(ch),
        }
    }
    res
}

fn normalize_web_color(color: &str) -> String {
    if color.len() == 9 && color.starts_with('#') {
        return format!("#{}", &color[1..7]);
    }
    color.to_string()
}

fn colored_edge_style(base: &StyleAttr, color: &str) -> StyleAttr {
    StyleAttr::new(
        Color::fast(color),
        base.line_width,
        None,
        0,
        EDGE_FONT_SIZE,
    )
}

fn edge_color_index(edge: &RenderEdge, idx: usize) -> usize {
    let mut hash = idx as u64;
    if !edge.label.is_empty() {
        for byte in edge.label.as_bytes() {
            hash = hash.wrapping_mul(131).wrapping_add(*byte as u64);
        }
    }
    if let Some(port) = &edge.src_port {
        for byte in port.as_bytes() {
            hash = hash.wrapping_mul(131).wrapping_add(*byte as u64);
        }
    }
    if let Some(port) = &edge.dst_port {
        for byte in port.as_bytes() {
            hash = hash.wrapping_mul(131).wrapping_add(*byte as u64);
        }
    }
    (hash as usize) % EDGE_COLOR_PALETTE.len()
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
    format!(
        "#{:02X}{:02X}{:02X}",
        blend(r),
        blend(g),
        blend(b)
    )
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

    let mut anchors = HashMap::new();
    let mut collector = PortAnchorCollector {
        anchors: &mut anchors,
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
}

impl TableVisitor for PortAnchorCollector<'_> {
    fn handle_cell(&mut self, cell: &TableCell, loc: Point, size: Point) {
        let Some(port) = &cell.port else {
            return;
        };

        let is_output = port.starts_with("out_");
        let x = if is_output {
            loc.x + size.x / 2.0 + ARROW_HEAD_MARGIN
        } else {
            loc.x - size.x / 2.0 - ARROW_HEAD_MARGIN
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
}

struct NodeBounds {
    handle: NodeHandle,
    left: f64,
    right: f64,
    top: f64,
    bottom: f64,
    center_x: f64,
}

fn collect_node_bounds(section: &SectionLayout) -> Vec<NodeBounds> {
    let mut bounds = Vec::with_capacity(section.nodes.len());
    for node in &section.nodes {
        let pos = section.graph.element(node.handle).position();
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
    plans.sort_by(|a, b| a.span.partial_cmp(&b.span).unwrap_or(Ordering::Equal));

    let mut layer = 0usize;
    let mut last_span: Option<f64> = None;
    let mut layer_counts: HashMap<usize, usize> = HashMap::new();

    for plan in plans {
        if let Some(prev_span) = last_span {
            if (plan.span - prev_span).abs() > BACK_EDGE_SPAN_EPS {
                layer += 1;
            }
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
    let inner_dir = if end_stub.x >= start_stub.x { 1.0 } else { -1.0 };
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
    lane_span: Option<(f64, f64)>,
) -> Vec<BezierSegment> {
    build_lane_path(start, end, lane_y, start_dir, end_dir, lane_span)
}

fn build_loop_path(
    start: Point,
    end: Point,
    bbox: (Point, Point),
    start_dir: f64,
    end_dir: f64,
) -> Vec<BezierSegment> {
    let width = bbox.1.x - bbox.0.x;
    let height = bbox.1.y - bbox.0.y;
    let loop_dy = height * 0.8 + 30.0;

    let center_x = (bbox.0.x + bbox.1.x) / 2.0;
    let center_y = (bbox.0.y + bbox.1.y) / 2.0;

    let dir_y = if (start.y + end.y) / 2.0 < center_y {
        -1.0
    } else {
        1.0
    };
    let _dir_x_start = if start_dir.abs() > 0.0 {
        start_dir
    } else if start.x >= center_x {
        1.0
    } else {
        -1.0
    };
    let _dir_x_end = if end_dir.abs() > 0.0 {
        end_dir
    } else if end.x >= center_x {
        1.0
    } else {
        -1.0
    };

    let loop_pad = width * 0.4 + 40.0;
    let left = start.x.min(end.x) - loop_pad;
    let right = start.x.max(end.x) + loop_pad;
    let top_y = center_y + dir_y * loop_dy;

    let (c1, c2) = if start.x <= end.x {
        (Point::new(left, top_y), Point::new(right, top_y))
    } else {
        (Point::new(right, top_y), Point::new(left, top_y))
    };

    vec![BezierSegment {
        start,
        c1: Point::new(c1.x, c1.y),
        c2: Point::new(c2.x, c2.y),
        end,
    }]
}

fn build_lane_path(
    start: Point,
    end: Point,
    lane_y: f64,
    start_dir: f64,
    end_dir: f64,
    lane_span: Option<(f64, f64)>,
) -> Vec<BezierSegment> {
    let _ = lane_span;
    let base_stub = edge_port_handle(start, end);
    let dy_start = (lane_y - start.y).abs();
    let dy_end = (lane_y - end.y).abs();
    let max_stub = (end.x - start.x).abs().max(40.0) * 0.45;
    let start_stub = (base_stub + dy_start * 0.6).min(max_stub.max(base_stub));
    let end_stub = (base_stub + dy_end * 0.6).min(max_stub.max(base_stub));
    let mut start_corner = Point::new(start.x, lane_y);
    let mut end_corner = Point::new(end.x, lane_y);
    let lane_dir = if (end_corner.x - start_corner.x).abs() < 1.0 {
        if end_dir.abs() > 0.0 { end_dir } else { start_dir }
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

fn build_path_string(path: &[BezierSegment]) -> String {
    if path.is_empty() {
        return String::new();
    }

    let mut path_builder = String::new();
    let first = &path[0];
    path_builder.push_str(&format!(
        "M {} {} C {} {}, {} {}, {} {} ",
        first.start.x,
        first.start.y,
        first.c1.x,
        first.c1.y,
        first.c2.x,
        first.c2.y,
        first.end.x,
        first.end.y
    ));
    for segment in path.iter().skip(1) {
        path_builder.push_str(&format!(
            "C {} {}, {} {}, {} {} ",
            segment.c1.x,
            segment.c1.y,
            segment.c2.x,
            segment.c2.y,
            segment.end.x,
            segment.end.y
        ));
    }
    path_builder.trim().to_string()
}

fn build_explicit_path_string(path: &[BezierSegment], reverse: bool) -> String {
    if path.is_empty() {
        return String::new();
    }

    let mut path_builder = String::new();
    let reversed;
    let segments: &[BezierSegment] = if reverse {
        reversed = path
            .iter()
            .rev()
            .map(|seg| BezierSegment {
                start: seg.end,
                c1: seg.c2,
                c2: seg.c1,
                end: seg.start,
            })
            .collect::<Vec<_>>();
        &reversed
    } else {
        path
    };
    let first = &segments[0];
    path_builder.push_str(&format!(
        "M {} {} C {} {}, {} {}, {} {} ",
        first.start.x,
        first.start.y,
        first.c1.x,
        first.c1.y,
        first.c2.x,
        first.c2.y,
        first.end.x,
        first.end.y
    ));
    for segment in segments.iter().skip(1) {
        path_builder.push_str(&format!(
            "C {} {}, {} {}, {} {} ",
            segment.c1.x,
            segment.c1.y,
            segment.c2.x,
            segment.c2.y,
            segment.end.x,
            segment.end.y
        ));
    }
    path_builder.trim().to_string()
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
    let mut last = Point::new(mid.x + normal.x * base_offset, mid.y + normal.y * base_offset);
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

fn clamp_label_position(
    pos: Point,
    text: &str,
    font_size: usize,
    min: Point,
    max: Point,
) -> Point {
    let size = get_size_for_str(text, font_size);
    let half_w = size.x / 2.0 + 2.0;
    let half_h = size.y / 2.0 + 2.0;
    let min_x = min.x + half_w;
    let max_x = max.x - half_w;
    let min_y = min.y + half_h;
    let max_y = max.y - half_h;

    Point::new(
        pos.x.clamp(min_x, max_x),
        pos.y.clamp(min_y, max_y),
    )
}

fn segment_length(seg: &BezierSegment) -> f64 {
    seg.start.distance_to(seg.c1)
        + seg.c1.distance_to(seg.c2)
        + seg.c2.distance_to(seg.end)
}

fn segment_point(seg: &BezierSegment, t: f64) -> Point {
    let u = 1.0 - t;
    let tt = t * t;
    let uu = u * u;
    let uuu = uu * u;
    let ttt = tt * t;

    let mut p = Point::new(0.0, 0.0);
    p.x = uuu * seg.start.x
        + 3.0 * uu * t * seg.c1.x
        + 3.0 * u * tt * seg.c2.x
        + ttt * seg.end.x;
    p.y = uuu * seg.start.y
        + 3.0 * uu * t * seg.c1.y
        + 3.0 * u * tt * seg.c2.y
        + ttt * seg.end.y;
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
    let mut groups: HashMap<(i64, i64), Vec<(usize, Point, Point)>> = HashMap::new();
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
    let mut groups: HashMap<(i64, i64, bool), Vec<(usize, f64, f64, f64)>> = HashMap::new();
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
        let count = edges.len() as f64;
        let slot_width = width / count;

        for (slot_idx, (edge_idx, _, _, _)) in edges.into_iter().enumerate() {
            let center_x = left + (slot_idx as f64 + 0.5) * slot_width;
            slots.insert(
                edge_idx,
                DetourLabelSlot {
                    center_x,
                    width: slot_width * 0.9,
                    lane_y: detour_lane_y[edge_idx],
                    above: detour_above[edge_idx],
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
                if depth > 0 {
                    depth -= 1;
                }
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

fn scale_layout_positions(graph: &mut VisualGraph) {
    for handle in graph.iter_nodes() {
        let center = graph.element(handle).position().center();
        let scaled = Point::new(center.x * LAYOUT_SCALE_X, center.y * LAYOUT_SCALE_Y);
        graph.element_mut(handle).position_mut().move_to(scaled);
    }
}
