//! Shared monitor rendering and interaction regressions.

use super::*;
#[cfg(feature = "dag")]
use cu29::monitoring::MonitorConnection;
use cu29::monitoring::{
    ComponentType, CopperListInfo, CopperListIoStats, MonitorComponentMetadata, MonitorNode,
    MonitorTopology,
};
use ratatui::Terminal;
use ratatui::backend::TestBackend;

#[derive(Debug)]
struct StreamFixture(cu29::monitoring::LogStreamStats);
impl cu29::monitoring::LogStreamStatsSource for StreamFixture {
    fn snapshot(&self) -> cu29::monitoring::LogStreamStats {
        self.0
    }
}

fn stream_ui(stats: cu29::monitoring::LogStreamStats) -> MonitorUi {
    use cu29::monitoring::LogStreamMonitor;
    let mut model = test_monitor_model();
    model.streams = Some(std::sync::Arc::from(vec![LogStreamMonitor::new(
        "ground",
        1_000_000,
        4,
        StreamFixture(stats),
    )]));
    let mut ui = MonitorUi::new(model, MonitorUiOptions::default());
    ui.set_active_screen(MonitorScreen::CopperList);
    ui
}

fn stream_text(ui: &mut MonitorUi, width: u16, height: u16) -> String {
    let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
    terminal.draw(|f| ui.draw_content(f, f.area())).unwrap();
    terminal
        .backend()
        .buffer()
        .content
        .chunks(usize::from(width))
        .map(|row| row.iter().map(|cell| cell.symbol()).collect::<String>())
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn bandwidth_shows_one_way_stats_and_hides_receiver_fields() {
    let mut ui = stream_ui(cu29::monitoring::LogStreamStats {
        packets_sent: 12345,
        queue_drops: 7,
        ..Default::default()
    });
    let text = stream_text(&mut ui, 132, 40);
    assert!(text.contains("Telemetry / TX: ground"));
    assert!(text.contains("One-way"));
    assert!(text.contains("Disabled"));
    assert!(text.contains("12345"));
    assert!(text.contains("Queue drops"));
    assert!(!text.contains("RX BW"));
}

#[test]
fn bandwidth_panels_share_width_columns_and_full_height() {
    let mut ui = stream_ui(cu29::monitoring::LogStreamStats::default());
    for width in [126, 180] {
        let text = stream_text(&mut ui, width, 40);
        let lines: Vec<_> = text.lines().collect();
        let panel_width = usize::from(width / 3);
        let bottom: Vec<_> = lines[39].chars().collect();
        for panel in 0..3 {
            let start = panel * panel_width;
            assert_eq!(bottom[start], '╰');
            assert_eq!(bottom[start + panel_width - 1], '╯');
        }
        let header: Vec<_> = lines[1].chars().collect();
        let headers: Vec<_> = header
            .windows(5)
            .enumerate()
            .filter_map(|(i, cells)| (cells == ['V', 'a', 'l', 'u', 'e']).then_some(i))
            .collect();
        assert_eq!(headers.len(), 3);
        assert_eq!(headers[1] - headers[0], panel_width);
        assert_eq!(headers[2] - headers[1], panel_width);
    }
}

#[test]
fn bandwidth_distinguishes_waiting_active_stale_and_failed_feedback() {
    use cu29::monitoring::{LogStreamFeedbackState, LogStreamFeedbackStats, LogStreamStats};
    for (state, label, failed) in [
        (LogStreamFeedbackState::Waiting, "Waiting", false),
        (LogStreamFeedbackState::Active, "Active", false),
        (LogStreamFeedbackState::Stale, "Stale", false),
        (LogStreamFeedbackState::Active, "Failed", true),
    ] {
        let mut ui = stream_ui(LogStreamStats {
            feedback: Some(LogStreamFeedbackStats {
                state,
                failed,
                reports: 4,
                finalized_symbols: 100,
                rates_available: true,
                source_metrics_available: true,
                loss_basis_points: 1250,
                latest_copperlist: Some(987654321),
                effective_repair_every_source_symbols: 2,
                ..Default::default()
            }),
            ..Default::default()
        });
        let text = stream_text(&mut ui, 132, 45);
        assert!(text.contains("Two-way"));
        assert!(text.contains(label));
        assert!(text.contains("RX BW"));
        assert!(text.contains("FEC effective interval"));
        assert_eq!(
            text.contains("987654321"),
            state == LogStreamFeedbackState::Active && !failed
        );
        assert_eq!(
            text.contains("12.50%"),
            state == LogStreamFeedbackState::Active && !failed
        );
    }
}

#[test]
fn bandwidth_scrolls_to_multiple_destinations_on_narrow_terminals() {
    use cu29::monitoring::{LogStreamMonitor, LogStreamStats};
    let mut model = test_monitor_model();
    model.streams = Some(std::sync::Arc::from(vec![
        LogStreamMonitor::new(
            "first",
            1_000_000,
            4,
            StreamFixture(LogStreamStats::default()),
        ),
        LogStreamMonitor::new(
            "second",
            1_000_000,
            4,
            StreamFixture(LogStreamStats::default()),
        ),
    ]));
    let mut ui = MonitorUi::new(model, MonitorUiOptions::default());
    ui.set_active_screen(MonitorScreen::CopperList);
    ui.scroll(ScrollDirection::Right, 84);
    ui.scroll(ScrollDirection::Down, 21);
    let text = stream_text(&mut ui, 42, 12);
    assert!(text.contains("Telemetry / TX: second"), "{text}");
    // Tiny terminal sizes and resizing clamp rather than overflow.
    stream_text(&mut ui, 1, 1);
    ui.scroll(ScrollDirection::Left, 200);
    ui.scroll(ScrollDirection::Up, 200);
    assert!(stream_text(&mut ui, 132, 48).contains("Telemetry / TX: first"));
}

#[test]
fn normalize_text_colors_replaces_reset_fg_and_bg() {
    let mut text = Text::from(Line::from(vec![Span::styled(
        "pfetch",
        Style::default().fg(Color::Reset).bg(Color::Reset),
    )]));

    palette::normalize_text_colors(&mut text, palette::FOREGROUND, palette::BACKGROUND);

    let span = &text.lines[0].spans[0];
    assert_eq!(span.style.fg, Some(palette::FOREGROUND));
    assert_eq!(span.style.bg, Some(palette::BACKGROUND));
}

#[test]
#[cfg(not(feature = "dag"))]
fn monitor_ui_starts_on_first_available_view() {
    let ui = MonitorUi::new(test_monitor_model(), MonitorUiOptions::default());
    #[cfg(feature = "neighbors")]
    assert_eq!(ui.active_screen(), MonitorScreen::Neighbors);
    #[cfg(not(feature = "neighbors"))]
    assert_eq!(ui.active_screen(), MonitorScreen::Latency);
}

#[test]
#[cfg(feature = "dag")]
fn monitor_ui_starts_on_dag_tab() {
    let ui = MonitorUi::new(test_monitor_model(), MonitorUiOptions::default());

    assert_eq!(ui.active_screen(), MonitorScreen::Dag);
}

#[test]
fn copperlist_screen_highlights_dropped_counts() {
    let model = test_monitor_model();
    model.observe_copperlist_io(CopperListIoStats {
        dropped_copperlists_total: 7,
        dropped_keyframes_total: 3,
        ..CopperListIoStats::default()
    });
    let mut ui = MonitorUi::new(model, MonitorUiOptions::default());
    ui.set_active_screen(MonitorScreen::CopperList);
    let mut terminal = Terminal::new(TestBackend::new(90, 18)).unwrap();

    terminal.draw(|frame| ui.draw(frame)).unwrap();

    let buffer = terminal.backend().buffer();
    let dropped_row = buffer
        .content
        .chunks(buffer.area.width as usize)
        .find(|cells| {
            cells
                .iter()
                .map(|cell| cell.symbol())
                .collect::<String>()
                .contains("Dropped CopperLists")
        })
        .expect("Dropped CopperLists row");
    let value = dropped_row
        .iter()
        .find(|cell| cell.symbol() == "7")
        .expect("dropped count value");
    assert_eq!(value.fg, palette::LIGHT_RED);
    assert!(value.modifier.contains(Modifier::BOLD));

    let dropped_keyframe_row = buffer
        .content
        .chunks(buffer.area.width as usize)
        .find(|cells| {
            cells
                .iter()
                .map(|cell| cell.symbol())
                .collect::<String>()
                .contains("Dropped keyframes")
        })
        .expect("Dropped keyframes row");
    let value = dropped_keyframe_row
        .iter()
        .find(|cell| cell.symbol() == "3")
        .expect("dropped keyframe count value");
    assert_eq!(value.fg, palette::LIGHT_RED);
    assert!(value.modifier.contains(Modifier::BOLD));
}

#[test]
#[cfg(feature = "dag")]
fn initial_graph_scroll_offset_targets_center_right() {
    let area = Rect::new(0, 0, 80, 20);
    let content_size = Size::new(240, 90);
    let graph_bounds = Size::new(200, 70);

    let offset = initial_graph_scroll_offset(area, content_size, graph_bounds);

    assert_eq!(offset, Position::new(85, 25));
}

#[test]
#[cfg(feature = "dag")]
fn first_graph_build_seeds_a_non_zero_horizontal_offset_for_wide_dags() {
    let mut state = NodesScrollableWidgetState::new(wide_test_monitor_model());

    let content_size = state.ensure_graph_cache(Rect::new(0, 0, 80, 20));
    let offset = state.nodes_scrollable_state.offset();

    assert!(content_size.width > 80);
    assert!(offset.x > 0);
}

#[test]
#[cfg(feature = "dag")]
fn resizing_wide_dag_reuses_cached_graph_layout_and_clamps_scroll() {
    let mut state = NodesScrollableWidgetState::new(wide_test_monitor_model());
    let initial_area = Rect::new(0, 0, 80, 20);
    let resized_area = Rect::new(0, 0, 120, 24);

    let initial_content_size = state.ensure_graph_cache(initial_area);
    let initial_key = state.graph_cache.key;

    state
        .nodes_scrollable_state
        .set_offset(Position::new(u16::MAX, u16::MAX));
    let resized_content_size = state.ensure_graph_cache(resized_area);
    let offset = state.nodes_scrollable_state.offset();
    let max_x = resized_content_size
        .width
        .saturating_sub(resized_area.width.saturating_sub(1));
    let max_y = resized_content_size
        .height
        .saturating_sub(resized_area.height.saturating_sub(1));

    assert_eq!(resized_content_size, initial_content_size);
    assert_eq!(state.graph_cache.key, initial_key);
    assert_eq!(offset, Position::new(max_x, max_y));
}

#[test]
#[cfg(feature = "dag")]
fn disconnected_pipelines_initial_graph_size_covers_graph_bounds() {
    let state = NodesScrollableWidgetState::new(parallel_pipelines_monitor_model(8));
    let area = Rect::new(0, 0, 80, 20);
    let initial_size = state.estimate_initial_graph_size(area);
    let graph = state.build_graph(initial_size);

    assert!(graph.content_bounds().height <= initial_size.height);
}

#[test]
#[cfg(feature = "dag")]
fn disconnected_pipelines_render_without_alias_fallback() {
    let mut state = NodesScrollableWidgetState::new(parallel_pipelines_monitor_model(8));

    state.ensure_graph_cache(Rect::new(0, 0, 80, 20));

    assert!(state.graph().conn_layout.alias_connections.is_empty());
}

#[test]
fn footer_badges_render_identity_in_requested_order() {
    let badges = footer_badges(
        MonitorFooterIdentity {
            system_name: "robot-alpha".into(),
            subsystem_name: Some("drivetrain".into()),
            mission_name: "autonomous".into(),
            instance_id: 42,
        },
        12846,
    );

    let labels = badges
        .into_iter()
        .map(|badge| badge.inner)
        .collect::<Vec<_>>();
    assert_eq!(
        labels,
        vec![
            " robot-alpha ".to_string(),
            " drivetrain ".to_string(),
            " 42 ".to_string(),
            " autonomous ".to_string(),
            " 00000000000000012846 ".to_string(),
        ]
    );
}

#[test]
fn footer_badges_skip_subsystem_when_absent() {
    let badges = footer_badges(
        MonitorFooterIdentity {
            system_name: "robot-alpha".into(),
            subsystem_name: None,
            mission_name: "autonomous".into(),
            instance_id: 42,
        },
        12846,
    );

    let labels = badges
        .into_iter()
        .map(|badge| badge.inner)
        .collect::<Vec<_>>();
    assert_eq!(
        labels,
        vec![
            " robot-alpha ".to_string(),
            " 42 ".to_string(),
            " autonomous ".to_string(),
            " 00000000000000012846 ".to_string(),
        ]
    );
}

#[test]
fn clip_with_ellipsis_truncates_long_footer_values() {
    assert_eq!(
        clip_with_ellipsis("balancebot-simulator-east", 12),
        "balancebo..."
    );
}

fn test_monitor_model() -> MonitorModel {
    static COMPONENTS: [MonitorComponentMetadata; 3] = [
        MonitorComponentMetadata::new("sensor", ComponentType::Source, Some("Sensor")),
        MonitorComponentMetadata::new("controller", ComponentType::Task, Some("Controller")),
        MonitorComponentMetadata::new("actuator", ComponentType::Sink, Some("Actuator")),
    ];

    let topology = MonitorTopology {
        nodes: vec![
            MonitorNode {
                id: "sensor".to_string(),
                type_name: Some("Sensor".to_string()),
                kind: ComponentType::Source,
                inputs: Vec::new(),
                outputs: vec!["imu".to_string()],
            },
            MonitorNode {
                id: "controller".to_string(),
                type_name: Some("Controller".to_string()),
                kind: ComponentType::Task,
                inputs: vec!["imu".to_string()],
                outputs: vec!["cmd".to_string()],
            },
            MonitorNode {
                id: "actuator".to_string(),
                type_name: Some("Actuator".to_string()),
                kind: ComponentType::Sink,
                inputs: vec!["cmd".to_string()],
                outputs: Vec::new(),
            },
        ],
        connections: Vec::new(),
    };

    MonitorModel::from_parts(&COMPONENTS, CopperListInfo::new(0, 0), topology)
}

#[cfg(feature = "dag")]
fn wide_test_monitor_model() -> MonitorModel {
    static COMPONENTS: [MonitorComponentMetadata; 6] = [
        MonitorComponentMetadata::new("source", ComponentType::Source, Some("Source")),
        MonitorComponentMetadata::new("estimator", ComponentType::Task, Some("Estimator")),
        MonitorComponentMetadata::new("planner", ComponentType::Task, Some("Planner")),
        MonitorComponentMetadata::new("controller", ComponentType::Task, Some("Controller")),
        MonitorComponentMetadata::new("mixer", ComponentType::Task, Some("Mixer")),
        MonitorComponentMetadata::new("actuator", ComponentType::Sink, Some("Actuator")),
    ];

    let ids = [
        "source",
        "estimator",
        "planner",
        "controller",
        "mixer",
        "actuator",
    ];
    let nodes = ids
        .iter()
        .map(|id| MonitorNode {
            id: (*id).to_string(),
            type_name: Some(id.to_string()),
            kind: if *id == "source" {
                ComponentType::Source
            } else if *id == "actuator" {
                ComponentType::Sink
            } else {
                ComponentType::Task
            },
            inputs: if *id == "source" {
                Vec::new()
            } else {
                vec!["in".to_string()]
            },
            outputs: if *id == "actuator" {
                Vec::new()
            } else {
                vec!["out".to_string()]
            },
        })
        .collect();
    let connections = ids
        .windows(2)
        .map(|pair| MonitorConnection {
            src: pair[0].to_string(),
            src_port: Some("out".to_string()),
            dst: pair[1].to_string(),
            dst_port: Some("in".to_string()),
            msg: "msg".to_string(),
        })
        .collect();
    let topology = MonitorTopology { nodes, connections };

    MonitorModel::from_parts(&COMPONENTS, CopperListInfo::new(0, 0), topology)
}

#[cfg(feature = "dag")]
fn parallel_pipelines_monitor_model(pipeline_count: usize) -> MonitorModel {
    let mut components = Vec::with_capacity(pipeline_count * 2);
    let mut nodes = Vec::with_capacity(pipeline_count * 2);
    let mut connections = Vec::with_capacity(pipeline_count);

    for idx in 0..pipeline_count {
        let source_id = format!("source_{idx}");
        let sink_id = format!("sink_{idx}");

        components.push(MonitorComponentMetadata::new(
            Box::leak(source_id.clone().into_boxed_str()),
            ComponentType::Source,
            Some("Source"),
        ));
        components.push(MonitorComponentMetadata::new(
            Box::leak(sink_id.clone().into_boxed_str()),
            ComponentType::Sink,
            Some("Sink"),
        ));

        nodes.push(MonitorNode {
            id: source_id.clone(),
            type_name: Some("Source".to_string()),
            kind: ComponentType::Source,
            inputs: Vec::new(),
            outputs: vec!["out".to_string()],
        });
        nodes.push(MonitorNode {
            id: sink_id.clone(),
            type_name: Some("Sink".to_string()),
            kind: ComponentType::Sink,
            inputs: vec!["in".to_string()],
            outputs: Vec::new(),
        });
        connections.push(MonitorConnection {
            src: source_id,
            src_port: Some("out".to_string()),
            dst: sink_id,
            dst_port: Some("in".to_string()),
            msg: "msg".to_string(),
        });
    }

    let components: &'static [MonitorComponentMetadata] = Box::leak(components.into_boxed_slice());
    let topology = MonitorTopology { nodes, connections };

    MonitorModel::from_parts(components, CopperListInfo::new(0, 0), topology)
}
