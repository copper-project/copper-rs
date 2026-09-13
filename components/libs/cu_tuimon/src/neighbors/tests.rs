//! Regression coverage for topology mapping, navigation and embedded rendering.

use super::*;
use crate::MonitorScreen;
use crate::MonitorUi;
use crate::MonitorUiAction;
#[cfg(feature = "dag")]
use crate::MonitorUiEvent;
use crate::MonitorUiOptions;
use cu29::clock::CuDuration;
use cu29::monitoring::{
    ComponentType, CopperListInfo, MonitorComponentMetadata, MonitorConnection, MonitorNode,
    MonitorTopology,
};
use ratatui::Terminal;
use ratatui::backend::TestBackend;
use ratatui::widgets::Paragraph;

fn model() -> MonitorModel {
    static COMPONENTS: [MonitorComponentMetadata; 3] = [
        MonitorComponentMetadata::new("recorder", ComponentType::Sink, Some("Recorder")),
        MonitorComponentMetadata::new("fusion", ComponentType::Task, Some("Fusion")),
        MonitorComponentMetadata::new("imu", ComponentType::Source, Some("Imu")),
    ];
    let mut nodes: Vec<_> = ["imu", "fusion", "recorder", "isolated", "bridge"]
        .into_iter()
        .map(|id| MonitorNode {
            id: id.into(),
            type_name: Some(format!("Type_{id}")),
            kind: ComponentType::Task,
            inputs: vec!["in".into()],
            outputs: vec!["out".into()],
        })
        .collect();
    nodes[0].kind = ComponentType::Source;
    nodes[4].kind = ComponentType::Bridge;
    let mut connections: Vec<_> = [
        ("imu", "fusion", "accel", "accel"),
        ("imu", "fusion", "gyro", "gyro"),
        ("fusion", "recorder", "pose", "pose"),
        ("recorder", "fusion", "feedback", "feedback"),
        ("recorder", "recorder", "state", "previous"),
        ("missing", "fusion", "bad", "bad"),
    ]
    .into_iter()
    .map(|(src, dst, output, input)| MonitorConnection {
        src: src.into(),
        dst: dst.into(),
        src_port: Some(output.into()),
        dst_port: Some(input.into()),
        msg: "Payload".into(),
    })
    .collect();
    for index in 0..36 {
        let id = format!("observer_{index:02}");
        nodes.push(MonitorNode {
            id: id.clone(),
            type_name: None,
            kind: ComponentType::Sink,
            inputs: vec!["pose".into()],
            outputs: vec![],
        });
        connections.push(MonitorConnection {
            src: "fusion".into(),
            dst: id,
            src_port: None,
            dst_port: None,
            msg: "Pose".into(),
        });
    }
    MonitorModel::from_parts(
        &COMPONENTS,
        CopperListInfo::new(0, 0),
        MonitorTopology { nodes, connections },
    )
}

fn view() -> NeighborsView {
    let mut view = NeighborsView::new(model());
    view.move_selection(true);
    view.column = Column::Outgoing;
    view
}

#[test]
fn test_parallel_feedback_and_self_loop_navigation() {
    let mut view = view();
    assert_eq!(view.neighbors(Column::Incoming).len(), 3);
    assert_eq!(view.neighbors(Column::Outgoing).len(), 37);
    view.follow();
    assert_eq!(view.focus, Some(2));
    view.move_selection(true);
    view.follow(); // Following oneself does not fill the back stack.
    assert_eq!(view.focus, Some(2));
    view.back();
    assert_eq!(view.focus, Some(1));
    assert_eq!(view.lists[2].selected(), Some(0));
    assert_eq!(view.component_ids[1], Some(ComponentId::new(1)));
    assert_eq!(view.component_ids[0], Some(ComponentId::new(2)));
}

#[test]
fn test_filtering_owns_characters_and_preserves_navigation() {
    let mut view = view();
    view.column = Column::Nodes;
    view.handle_key(MonitorUiKey::Char('/'));
    assert!(view.query.is_empty());
    for c in "OBS".chars() {
        assert!(view.handle_key(MonitorUiKey::Char(c)));
    }
    assert_eq!(view.matches.len(), 36);
    assert_eq!(view.focus, Some(5));
    view.handle_key(MonitorUiKey::Down);
    assert_eq!(view.focus, Some(6));
    view.handle_key(MonitorUiKey::Esc);
    assert_eq!(view.lists[0].selected(), Some(6));
    assert!(!view.handle_key(MonitorUiKey::Char('o')));
    view.handle_key(MonitorUiKey::Char('/'));
    for c in "qhjkl123".chars() {
        assert!(view.handle_key(MonitorUiKey::Char(c)));
    }
    assert!(view.matches.is_empty());
    view.handle_key(MonitorUiKey::Down);
    assert_eq!(view.focus, Some(6));
    view.handle_key(MonitorUiKey::Backspace);
    assert_eq!(view.query, "qhjkl12");
    view.handle_key(MonitorUiKey::Esc);
    view.handle_key(MonitorUiKey::Char('/'));
    for c in "isolated".chars() {
        view.handle_key(MonitorUiKey::Char(c));
    }
    view.handle_key(MonitorUiKey::Tab);
    view.handle_key(MonitorUiKey::Enter);
    assert_eq!(view.focus, Some(3));
    assert!(view.neighbors(Column::Incoming).is_empty());
    view.handle_key(MonitorUiKey::Char('/'));
    view.scroll(ScrollDirection::Right, 1);
    view.scroll(ScrollDirection::Left, 1);
    assert!(!view.searching);
}

#[test]
fn test_render_uses_live_component_mapping_and_preserves_error() {
    let model = model();
    model.set_component_error(ComponentId::new(2), "IMU failed");
    model.record_component_latency(ComponentId::new(2), CuDuration::from_micros(42));
    let mut view = NeighborsView::new(model.clone());
    let mut terminal = Terminal::new(TestBackend::new(150, 35)).unwrap();
    let area = Rect::new(4, 3, 140, 30);
    terminal
        .draw(|frame| {
            frame.render_widget(Paragraph::new("outside"), Rect::new(0, 0, 10, 1));
            view.draw(frame, area);
        })
        .unwrap();
    let buffer = terminal.backend().buffer();
    let screen: String = buffer.content.iter().map(|cell| cell.symbol()).collect();
    assert!(screen.contains("IMU failed"));
    assert!(screen.contains("◈ imu"));
    assert!(screen.contains("⚙ fusion"));
    assert!(screen.contains("42"));
    assert!(screen.contains("accel"));
    assert!(screen.contains("gyro"));
    assert_eq!(buffer[(0, 0)].symbol(), "o");
    assert_eq!(view.panel_areas[0].x, area.x);
    assert!(model.inner.component_statuses.lock().unwrap()[2].is_error);
}

#[test]
fn test_neighbor_names_and_message_types_fit_both_columns() {
    let nodes = [
        ("merge_pids", ComponentType::Task),
        ("balance_pid", ComponentType::Task),
        ("motors", ComponentType::Sink),
        ("sensors", ComponentType::Bridge),
        ("telemetry", ComponentType::Bridge),
    ]
    .into_iter()
    .map(|(id, kind)| MonitorNode {
        id: id.into(),
        type_name: None,
        kind,
        inputs: vec!["in".into()],
        outputs: vec![],
    })
    .collect();
    let connections = [
        (
            "balance_pid",
            "merge_pids",
            "out0: cu_pid::PIDControlOutput",
            "in",
            "cu_pid::PIDControlOutput",
        ),
        (
            "merge_pids",
            "motors",
            "out0: cu_rp_sn754410_new::MotorPayload",
            "in",
            "cu_rp_sn754410_new::MotorPayload",
        ),
        (
            "sensors",
            "merge_pids",
            "accel",
            "in",
            "wrapper::Batch<robot::Pose>",
        ),
        (
            "merge_pids",
            "telemetry",
            "out0: robot::Pose",
            "out0",
            "robot::Pose",
        ),
    ]
    .into_iter()
    .map(|(src, dst, output, input, msg)| MonitorConnection {
        src: src.into(),
        dst: dst.into(),
        src_port: Some(output.into()),
        dst_port: Some(input.into()),
        msg: msg.into(),
    })
    .collect();
    let mut view = NeighborsView::new(MonitorModel::from_parts(
        &[],
        CopperListInfo::new(0, 0),
        MonitorTopology { nodes, connections },
    ));
    for width in [100, 160] {
        let mut terminal = Terminal::new(TestBackend::new(width, 25)).unwrap();
        terminal
            .draw(|frame| view.draw(frame, frame.area()))
            .unwrap();
        let buffer = terminal.backend().buffer();
        let panel_text = |column: usize| {
            let area = view.list_areas[column];
            (area.y..area.bottom())
                .map(|y| {
                    (area.x..area.right())
                        .map(|x| buffer[(x, y)].symbol())
                        .collect::<String>()
                })
                .collect::<Vec<_>>()
        };
        let incoming = panel_text(1);
        let outgoing = panel_text(2);
        assert!(incoming[0].contains("⚙ balance_pid"));
        assert!(incoming[1].contains("PIDControlOutput"));
        assert!(incoming[2].contains("⇆ sensors · accel"));
        assert!(incoming[3].contains("Batch<Pose>"));
        assert!(outgoing[0].contains("⭳ motors"));
        assert!(outgoing[1].contains("MotorPayload"));
        // A bridge channel named out0 is meaningful and must remain visible.
        assert!(outgoing[2].contains("⇆ telemetry · out0"));
        assert!(outgoing[3].contains("Pose"));
        assert!(!incoming.join("\n").contains("out0"));
        assert!(!incoming.join("\n").contains("::"));
        assert!(!outgoing.join("\n").contains("::"));
    }
}

#[test]
fn test_fanout_scrolling_hitboxes_resize_and_back() {
    let mut view = view();
    let mut terminal = Terminal::new(TestBackend::new(150, 35)).unwrap();
    let area = Rect::new(4, 3, 140, 30);
    terminal.draw(|frame| view.draw(frame, area)).unwrap();
    let nodes = view.panel_areas[0];
    view.handle_key(MonitorUiKey::Char('/'));
    view.scroll_at(nodes.x + 1, nodes.y + 1, ScrollDirection::Up, 1);
    assert!(view.searching);
    view.scroll_at(nodes.x + 1, nodes.y + 1, ScrollDirection::Down, 1);
    let incoming = view.panel_areas[1];
    assert!(incoming.y + 1 < view.list_areas[1].y);
    view.scroll_at(incoming.x + 1, incoming.y + 1, ScrollDirection::Down, 1);
    assert_eq!(view.column, Column::Incoming);
    assert!(!view.searching);
    assert_eq!(view.lists[1].selected(), Some(1));
    assert_eq!(view.focus, Some(1));
    let outgoing = view.panel_areas[2];
    view.scroll_at(outgoing.x + 1, outgoing.y + 1, ScrollDirection::Down, 36);
    terminal.draw(|frame| view.draw(frame, area)).unwrap();
    let screen: String = terminal
        .backend()
        .buffer()
        .content
        .iter()
        .map(|cell| cell.symbol())
        .collect();
    assert!(screen.contains("observer_35"));
    assert!(view.lists[2].offset() > 0);
    let rows = view.list_areas[2];
    let row = 36 - view.lists[2].offset();
    view.click(rows.x, rows.y + row as u16 * NEIGHBOR_HEIGHT + 1);
    assert_eq!(view.focus, Some(40));
    view.back();
    assert_eq!(view.focus, Some(1));
    assert_eq!(view.lists[2].selected(), Some(36));
    terminal
        .draw(|frame| view.draw(frame, Rect::new(0, 0, 30, 8)))
        .unwrap();
    view.click(rows.x, rows.y);
    view.scroll_at(outgoing.x + 1, outgoing.y + 1, ScrollDirection::Down, 1);
    assert_eq!(view.focus, Some(1));
    assert_eq!(view.list_areas, [Rect::default(); 3]);
}

#[test]
fn test_empty_topology_and_bounded_history() {
    let mut empty = NeighborsView::new(MonitorModel::from_parts(
        &[],
        CopperListInfo::new(0, 0),
        MonitorTopology::default(),
    ));
    let mut terminal = Terminal::new(TestBackend::new(140, 30)).unwrap();
    terminal
        .draw(|frame| empty.draw(frame, frame.area()))
        .unwrap();
    for key in [
        MonitorUiKey::Down,
        MonitorUiKey::Tab,
        MonitorUiKey::Enter,
        MonitorUiKey::Backspace,
    ] {
        empty.handle_key(key);
    }
    assert_eq!(empty.focus, None);
    let mut view = view();
    for _ in 0..HISTORY_CAPACITY * 2 {
        view.follow();
    }
    assert_eq!(view.history.len(), HISTORY_CAPACITY);
}

#[test]
fn test_monitor_tabs_and_input_dispatch() {
    let mut ui = MonitorUi::new(
        model(),
        MonitorUiOptions {
            show_quit_hint: true,
        },
    );
    ui.set_active_screen(MonitorScreen::Neighbors);
    assert_eq!(
        ui.handle_key(MonitorUiKey::Char('q')),
        MonitorUiAction::QuitRequested
    );
    ui.handle_key(MonitorUiKey::Char('/'));
    for key in "q12345".chars() {
        assert_eq!(
            ui.handle_key(MonitorUiKey::Char(key)),
            MonitorUiAction::None
        );
        assert_eq!(ui.active_screen(), MonitorScreen::Neighbors);
    }
    // Deleting the whole query, including Backspace on an empty query, stays in search.
    for _ in 0..7 {
        ui.handle_key(MonitorUiKey::Backspace);
    }
    assert_eq!(
        ui.handle_key(MonitorUiKey::Char('q')),
        MonitorUiAction::None
    );
    ui.handle_key(MonitorUiKey::Enter);
    assert_eq!(
        ui.handle_key(MonitorUiKey::Char('q')),
        MonitorUiAction::QuitRequested
    );
    // Applying a filter restores the same numeric shortcuts as every other screen.
    let mut reference = MonitorUi::new(model(), MonitorUiOptions::default());
    let tab_keys = if cfg!(feature = "dag") {
        "12345"
    } else {
        "1234"
    };
    for key in tab_keys.chars() {
        ui.set_active_screen(MonitorScreen::Neighbors);
        reference.set_active_screen(MonitorScreen::Latency);
        reference.handle_key(MonitorUiKey::Char(key));
        ui.handle_key(MonitorUiKey::Char(key));
        assert_eq!(ui.active_screen(), reference.active_screen());
    }
    ui.set_active_screen(MonitorScreen::Neighbors);
    ui.handle_key(MonitorUiKey::Char('/'));
    ui.handle_key(MonitorUiKey::Esc);
    assert_eq!(
        ui.handle_key(MonitorUiKey::Char('q')),
        MonitorUiAction::QuitRequested
    );
    ui.handle_key(MonitorUiKey::Char('/'));
    ui.handle_key(MonitorUiKey::Tab);
    ui.handle_key(MonitorUiKey::Left);
    assert_eq!(
        ui.handle_key(MonitorUiKey::Char('q')),
        MonitorUiAction::QuitRequested
    );
    let mut terminal = Terminal::new(TestBackend::new(160, 35)).unwrap();
    terminal.draw(|frame| ui.draw(frame)).unwrap();
    let row: String = terminal.backend().buffer().content[..160]
        .iter()
        .map(|cell| cell.symbol())
        .collect();
    assert!(row.contains("HOP"));
    #[cfg(feature = "dag")]
    {
        assert!(row.contains("DAG"));
        ui.handle_event(MonitorUiEvent::MouseDown {
            col: row[..row.find("DAG").unwrap()].chars().count() as u16,
            row: 0,
        });
        assert_eq!(ui.active_screen(), MonitorScreen::Dag);
    }
    #[cfg(not(feature = "dag"))]
    assert!(!row.contains("DAG"));
}

#[cfg(feature = "dag")]
#[test]
fn test_switching_views_does_not_consume_component_errors() {
    let model = model();
    model.set_component_error(ComponentId::new(2), "IMU failed");
    let mut ui = MonitorUi::new(model.clone(), MonitorUiOptions::default());
    let mut terminal = Terminal::new(TestBackend::new(160, 35)).unwrap();
    for screen in [
        MonitorScreen::Dag,
        MonitorScreen::Neighbors,
        MonitorScreen::Dag,
    ] {
        ui.set_active_screen(screen);
        terminal.draw(|frame| ui.draw(frame)).unwrap();
        assert!(model.inner.component_statuses.lock().unwrap()[2].is_error);
    }
    model.clear_component_error(ComponentId::new(2));
    assert!(!model.inner.component_statuses.lock().unwrap()[2].is_error);
}
