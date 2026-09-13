//! Fixed-column neighborhood view using the Catppuccin Mocha palette.

use super::Column;
use super::NEIGHBOR_HEIGHT;
use super::NeighborsView;
use crate::ui::NodeType;
use cu29::monitoring::ComponentType;
use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, List, ListItem, Paragraph, Wrap};

const STATUS: Color = Color::Rgb(245, 194, 231);
const SURFACE: Color = Color::Rgb(49, 50, 68);
const TEXT: Color = Color::Rgb(205, 214, 244);
const MUTED: Color = Color::Rgb(127, 132, 156);
const MAUVE: Color = Color::Rgb(203, 166, 247);
const GREEN: Color = Color::Rgb(166, 227, 161);
const LAVENDER: Color = Color::Rgb(180, 190, 254);

pub(super) fn draw(frame: &mut Frame<'_>, app: &mut NeighborsView, area: Rect) {
    app.list_areas = [Rect::default(); 3];
    app.panel_areas = [Rect::default(); 3];
    let area = area.intersection(frame.area());
    frame.render_widget(
        Block::default().style(Style::default().bg(Color::Rgb(30, 30, 46)).fg(TEXT)),
        area,
    );
    if area.width < 100 || area.height < 16 {
        frame.render_widget(
            Paragraph::new("HOP · enlarge the view to 100 × 16 or larger")
                .wrap(Wrap { trim: true }),
            area,
        );
        return;
    }
    let rows = Layout::vertical([Constraint::Min(8), Constraint::Length(2)]).split(area);
    let body = rows[0];
    let columns = Layout::horizontal([
        Constraint::Percentage(22),
        Constraint::Length(1),
        Constraint::Percentage(27),
        Constraint::Length(3),
        Constraint::Min(16),
        Constraint::Length(3),
        Constraint::Percentage(27),
    ])
    .split(body);
    app.panel_areas = [columns[0], columns[2], columns[6]];
    app.list_areas = app
        .panel_areas
        .map(|area| area.inner(ratatui::layout::Margin::new(1, 1)));
    let tasks = &app.matches;
    let items: Vec<_> = tasks
        .iter()
        .map(|&index| {
            let node = &app.model.topology().nodes[index];
            ListItem::new(format!("{} {}", NodeType::from(node.kind), node.id))
        })
        .collect();
    let title = if app.searching {
        format!(" /{}▏ ", app.query)
    } else if !app.query.is_empty() {
        format!(" /{} ", app.query)
    } else {
        format!(" Nodes ({}) ", tasks.len())
    };
    frame.render_stateful_widget(
        List::new(items)
            .style(Style::default().fg(LAVENDER))
            .block(panel(title, app.column == Column::Nodes, LAVENDER))
            .highlight_symbol("› ")
            .highlight_style(Style::default().add_modifier(Modifier::BOLD)),
        columns[0],
        &mut app.lists[0],
    );
    if tasks.is_empty() {
        frame.render_widget(
            Paragraph::new("No matches").style(Style::default().fg(MUTED)),
            columns[0].inner(ratatui::layout::Margin::new(1, 1)),
        );
    }

    neighbors(frame, app, Column::Incoming, columns[2], MAUVE);
    neighbors(frame, app, Column::Outgoing, columns[6], GREEN);
    rails(
        frame,
        columns[3],
        app.neighbors(Column::Incoming).len(),
        app.lists[1].offset(),
        true,
        MAUVE,
    );
    rails(
        frame,
        columns[5],
        app.neighbors(Column::Outgoing).len(),
        app.lists[2].offset(),
        false,
        GREEN,
    );

    let center = Layout::vertical([
        Constraint::Min(0),
        Constraint::Length(7),
        Constraint::Min(0),
    ])
    .split(columns[4])[1];
    if let Some(focus) = app.focus {
        let node = &app.model.topology().nodes[focus];
        let mut lines = vec![Line::from(node.type_name.as_deref().unwrap_or("unknown"))];
        if let Some(component_id) = app.component_ids[focus] {
            // Snapshot only the focused component and release locks before rendering.
            let status = app
                .model
                .inner
                .component_statuses
                .lock()
                .ok()
                .and_then(|statuses| statuses.get(component_id.index()).cloned());
            if let Some(status) = status {
                let color = if status.is_error {
                    Color::Rgb(243, 139, 168)
                } else {
                    STATUS
                };
                let text = if status.is_error {
                    status.error
                } else {
                    status.status_txt
                };
                lines.push(Line::styled(text.to_string(), Style::default().fg(color)));
            }
            let durations = app
                .model
                .inner
                .component_stats
                .lock()
                .ok()
                .and_then(|stats| {
                    stats
                        .stats
                        .get(component_id.index())
                        .filter(|stat| !stat.is_empty())
                        .map(|stat| (stat.mean(), stat.max()))
                });
            if let Some((mean, max)) = durations {
                lines.push(Line::from(format!("∅ {mean}")));
                lines.push(Line::from(format!("⬆ {max}")));
            } else {
                lines.push(Line::from("No timing samples"));
            }
        } else {
            lines.push(Line::from("No runtime samples"));
        }
        frame.render_widget(
            Paragraph::new(lines)
                .centered()
                .wrap(Wrap { trim: false })
                .block(panel(
                    format!(" {} {} ", NodeType::from(node.kind), node.id),
                    true,
                    LAVENDER,
                )),
            center,
        );
    } else {
        frame.render_widget(Paragraph::new("No topology nodes").centered(), center);
    }

    frame.render_widget(
        Paragraph::new(vec![
            Line::from(
                " ←/→ or Tab: choose list   ↑/↓ or wheel: select   Enter or click: follow neighbor   Backspace: back",
            ),
            Line::from(vec![
                Span::raw(if app.searching {
                    " Search: type to filter · Enter: apply · Esc: clear     "
                } else {
                    " /: search · Number keys: tabs · Esc: clear filter     "
                }),
                Span::styled("output", Style::default().fg(GREEN)),
                Span::styled(" → ", Style::default().fg(MUTED)),
                Span::styled("input", Style::default().fg(MAUVE)),
            ]),
        ])
        .style(Style::default().fg(MUTED)),
        rows[1],
    );
}

fn panel(title: String, active: bool, color: Color) -> Block<'static> {
    Block::bordered()
        .border_type(BorderType::Rounded)
        .title(title)
        .border_style(Style::default().fg(if active { color } else { SURFACE }))
}

fn centered_rows(area: Rect, count: usize) -> Rect {
    let mut rows = area.inner(ratatui::layout::Margin::new(1, 1));
    let height = count.min(usize::from(rows.height)) as u16;
    rows.y += rows.height.saturating_sub(height) / 2;
    rows.height = height;
    rows
}

fn neighbor_rows(area: Rect, count: usize) -> Rect {
    let mut rows = centered_rows(area, count.saturating_mul(usize::from(NEIGHBOR_HEIGHT)));
    // Keep every visible connection complete, including its clickable detail line.
    rows.height -= rows.height % NEIGHBOR_HEIGHT;
    rows
}

fn meaningful_port(port: &str, kind: ComponentType) -> Option<&str> {
    if kind == ComponentType::Bridge {
        return Some(port);
    }
    let name = port.split_once(": ").map_or(port, |(name, _)| name);
    let generated = ["in", "out"].iter().any(|prefix| {
        name.strip_prefix(prefix)
            .is_some_and(|suffix| suffix.chars().all(|c| c.is_ascii_digit()))
    });
    (!generated && name != "input" && name != "output").then_some(name)
}

fn short_type(name: &str) -> String {
    // Shorten each path separately to preserve nested generics, tuples and arrays.
    name.split_inclusive(|c: char| !c.is_alphanumeric() && c != '_' && c != ':')
        .map(|part| part.rsplit("::").next().unwrap_or(part))
        .collect()
}

fn neighbors(
    frame: &mut Frame<'_>,
    app: &mut NeighborsView,
    column: Column,
    area: Rect,
    color: Color,
) {
    let edges = app.neighbors(column);
    let count = edges.len();
    let items: Vec<_> = edges
        .iter()
        .map(|edge| {
            let topology = app.model.topology();
            let connection = &topology.connections[edge.connection];
            let node = &topology.nodes[edge.neighbor];
            let src = if column == Column::Incoming {
                edge.neighbor
            } else {
                app.focus.unwrap_or(0)
            };
            let dst = if column == Column::Incoming {
                app.focus.unwrap_or(0)
            } else {
                edge.neighbor
            };
            let output = connection
                .src_port
                .as_deref()
                .or_else(|| topology.nodes[src].outputs.first().map(String::as_str))
                .unwrap_or("output");
            let input = connection
                .dst_port
                .as_deref()
                .or_else(|| topology.nodes[dst].inputs.first().map(String::as_str))
                .unwrap_or("input");
            let output = meaningful_port(output, topology.nodes[src].kind);
            let input = meaningful_port(input, topology.nodes[dst].kind);
            let (neighbor_port, focus_port) = if column == Column::Incoming {
                (output, input)
            } else {
                (input, output)
            };
            let mut heading = vec![Span::styled(
                format!("{} {}", NodeType::from(node.kind), node.id),
                Style::default().fg(TEXT).add_modifier(Modifier::BOLD),
            )];
            if let Some(port) = neighbor_port {
                heading.push(Span::styled(
                    format!(" · {port}"),
                    Style::default().fg(color),
                ));
            }
            let mut detail = vec![Span::styled(
                short_type(&connection.msg),
                Style::default().fg(color),
            )];
            if let Some(port) = focus_port {
                if column == Column::Incoming {
                    detail.push(Span::styled(
                        format!(" → {port}"),
                        Style::default().fg(MAUVE),
                    ));
                } else {
                    detail.insert(
                        0,
                        Span::styled(format!("{port} → "), Style::default().fg(GREEN)),
                    );
                }
            }
            ListItem::new(vec![Line::from(heading), Line::from(detail)])
        })
        .collect();
    let selected = app.lists[column.index()].selected().unwrap_or(0);
    let direction = if column == Column::Incoming {
        "Incoming"
    } else {
        "Outgoing"
    };
    let position = if count == 0 { 0 } else { selected + 1 };
    frame.render_widget(
        panel(
            format!(" {direction} {position}/{count} "),
            app.column == column,
            color,
        ),
        area,
    );
    let rows = neighbor_rows(area, count);
    app.list_areas[column.index()] = rows;
    frame.render_stateful_widget(
        List::new(items)
            .highlight_symbol("› ")
            .style(Style::default().fg(color))
            .highlight_style(Style::default().add_modifier(Modifier::BOLD)),
        rows,
        &mut app.lists[column.index()],
    );
    if count == 0 {
        frame.render_widget(
            Paragraph::new("No connections").style(Style::default().fg(MUTED)),
            centered_rows(area, 1),
        );
    }
}

fn rails(
    frame: &mut Frame<'_>,
    area: Rect,
    count: usize,
    offset: usize,
    incoming: bool,
    color: Color,
) {
    if count == 0 || area.height < 4 {
        return;
    }
    let rows = neighbor_rows(area, count);
    let visible = count
        .saturating_sub(offset)
        .min(usize::from(rows.height / NEIGHBOR_HEIGHT));
    if visible == 0 {
        return;
    }
    let middle = area.height / 2;
    let row_start = rows.y - area.y + 1;
    let row_end = row_start + (visible as u16 - 1) * NEIGHBOR_HEIGHT;
    let first = row_start.min(middle);
    let last = row_end.max(middle);
    let before = offset > 0;
    let after = offset + visible < count;
    for y in 0..area.height {
        if y < first || y > last {
            if (y < first && before) || (y > last && after) {
                frame.buffer_mut().set_string(
                    area.x + 1,
                    area.y + y,
                    "┆",
                    Style::default().fg(MUTED),
                );
            }
            continue;
        }
        // Connect the message line of each two-line entry.
        let branch =
            y >= row_start && y <= row_end && (y - row_start).is_multiple_of(NEIGHBOR_HEIGHT);
        let west = (incoming && branch) || (!incoming && y == middle);
        let east = (!incoming && branch) || (incoming && y == middle);
        let north = y > first || before;
        let south = y < last || after;
        let joint = match (north, south, west, east) {
            (false, true, true, false) => '╮',
            (false, true, false, true) => '╭',
            (true, false, true, false) => '╯',
            (true, false, false, true) => '╰',
            (true, true, true, false) => '┤',
            (true, true, false, true) => '├',
            (true, true, true, true) => '┼',
            (false, true, true, true) => '┬',
            (true, false, true, true) => '┴',
            (false, false, true, true) => '─',
            _ => '│',
        };
        let symbols = format!(
            "{}{joint}{}",
            if west { '─' } else { ' ' },
            if east { '▶' } else { ' ' }
        );
        frame
            .buffer_mut()
            .set_string(area.x, area.y + y, symbols, Style::default().fg(color));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;

    #[test]
    fn rails_round_the_ends_and_mark_only_hidden_connections() {
        let mut terminal = Terminal::new(TestBackend::new(3, 12)).unwrap();
        terminal
            .draw(|frame| rails(frame, frame.area(), 2, 0, true, MAUVE))
            .unwrap();
        let buffer = terminal.backend().buffer();
        assert_eq!(buffer[(1, 5)].symbol(), "╮");
        assert_eq!(buffer[(1, 6)].symbol(), "├");
        assert_eq!(buffer[(1, 7)].symbol(), "╯");
        assert_eq!(buffer[(1, 0)].symbol(), " ");
        assert_eq!(buffer[(1, 11)].symbol(), " ");
        terminal
            .draw(|frame| rails(frame, frame.area(), 20, 2, false, GREEN))
            .unwrap();
        let buffer = terminal.backend().buffer();
        assert_eq!(buffer[(1, 0)].symbol(), "┆");
        assert_eq!(buffer[(1, 11)].symbol(), "┆");
        terminal
            .draw(|frame| rails(frame, frame.area(), 20, 18, false, GREEN))
            .unwrap();
        assert_eq!(terminal.backend().buffer()[(1, 11)].symbol(), " ");
    }
}
