//! Fixed-column neighborhood view using the Catppuccin Mocha palette.

use super::Column;
use super::NeighborsView;
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
            Paragraph::new("Neighbor explorer · enlarge the view to 100 × 16 or larger")
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
        .map(|&index| ListItem::new(app.model.topology().nodes[index].id.as_str()))
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
                .block(panel(format!(" {} ", node.id), true, LAVENDER)),
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
                Span::raw(" /: search · Type in Nodes to filter · Esc: clear     "),
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
        .enumerate()
        .map(|(row, edge)| {
            let topology = app.model.topology();
            let connection = &topology.connections[edge.connection];
            let name = &topology.nodes[edge.neighbor].id;
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
            let mut ports = vec![
                Span::styled(output.to_owned(), Style::default().fg(GREEN)),
                Span::styled(" → ", Style::default().fg(MUTED)),
                Span::styled(input.to_owned(), Style::default().fg(MAUVE)),
            ];
            let ports_width: usize = ports.iter().map(Span::width).sum();
            let width = usize::from(area.width.saturating_sub(2));
            let name_budget = width.saturating_sub(ports_width + 5);
            let mut short_name = name.clone();
            if Line::from(short_name.as_str()).width() > name_budget {
                while !short_name.is_empty()
                    && Line::from(short_name.as_str()).width() + 1 > name_budget
                {
                    short_name.pop();
                }
                if name_budget > 0 {
                    short_name.push('…');
                }
            }
            let gap =
                width.saturating_sub(ports_width + Line::from(short_name.as_str()).width() + 4);
            let chosen = app.lists[column.index()].selected() == Some(row);
            let spans = if column == Column::Incoming {
                let mut spans = vec![
                    Span::raw(if chosen { "› " } else { "  " }),
                    Span::raw(short_name),
                    Span::raw(" ".repeat(gap + 1)),
                ];
                spans.append(&mut ports);
                spans.push(Span::raw(" "));
                spans
            } else {
                let mut spans = vec![Span::raw(" ")];
                spans.append(&mut ports);
                spans.push(Span::raw(" ".repeat(gap + 1)));
                spans.push(Span::raw(short_name));
                spans.push(Span::raw(if chosen { " ‹" } else { "  " }));
                spans
            };
            ListItem::new(Line::from(spans))
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
    let rows = centered_rows(area, count);
    app.list_areas[column.index()] = rows;
    frame.render_stateful_widget(
        List::new(items)
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
    let visible = count
        .saturating_sub(offset)
        .min(usize::from(area.height - 2));
    if visible == 0 {
        return;
    }
    let middle = area.height / 2;
    let row_start = centered_rows(area, count).y - area.y;
    let row_end = row_start + visible as u16 - 1;
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
        // Each single-line entry meets its connector at the port mapping.
        let branch = y >= row_start && y <= row_end;
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
        assert_eq!(buffer[(1, 6)].symbol(), "┴");
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
