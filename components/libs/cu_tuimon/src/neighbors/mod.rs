//! Immediate-neighbor navigation over the monitor's immutable topology.

mod view;

use crate::MonitorModel;
use crate::MonitorUiKey;
use crate::ScrollDirection;
use cu29::monitoring::ComponentId;
use ratatui::Frame;
use ratatui::layout::Position;
use ratatui::layout::Rect;
use ratatui::widgets::ListState;
use std::collections::HashMap;
use std::collections::VecDeque;

const HISTORY_CAPACITY: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Column {
    Nodes = 0,
    Incoming = 1,
    Outgoing = 2,
}

impl Column {
    const ALL: [Self; 3] = [Self::Nodes, Self::Incoming, Self::Outgoing];

    fn index(self) -> usize {
        self as usize
    }

    fn next(self, forward: bool) -> Self {
        Self::ALL[(self.index() + if forward { 1 } else { 2 }) % 3]
    }
}

struct Edge {
    connection: usize,
    neighbor: usize,
}

struct Navigation {
    focus: Option<usize>,
    column: Column,
    lists: [ListState; 3],
    query: String,
}

pub(crate) struct NeighborsView {
    model: MonitorModel,
    // Topology node order is independent of runtime component order (notably bridges).
    component_ids: Vec<Option<ComponentId>>,
    incoming: Vec<Vec<Edge>>,
    outgoing: Vec<Vec<Edge>>,
    search_names: Vec<String>,
    matches: Vec<usize>,
    focus: Option<usize>,
    column: Column,
    lists: [ListState; 3],
    list_areas: [Rect; 3],
    panel_areas: [Rect; 3],
    query: String,
    searching: bool,
    history: VecDeque<Navigation>,
}

impl NeighborsView {
    pub(crate) fn new(model: MonitorModel) -> Self {
        let topology = model.topology();
        let node_lookup: HashMap<_, _> = topology
            .nodes
            .iter()
            .enumerate()
            .map(|(index, node)| (node.id.as_str(), index))
            .collect();
        let component_lookup: HashMap<_, _> = model
            .components()
            .iter()
            .enumerate()
            .map(|(index, component)| (component.id(), ComponentId::new(index)))
            .collect();
        let component_ids = topology
            .nodes
            .iter()
            .map(|node| component_lookup.get(node.id.as_str()).copied())
            .collect();
        let mut incoming: Vec<Vec<Edge>> = (0..topology.nodes.len()).map(|_| Vec::new()).collect();
        let mut outgoing: Vec<Vec<Edge>> = (0..topology.nodes.len()).map(|_| Vec::new()).collect();
        for (index, connection) in topology.connections.iter().enumerate() {
            let (Some(&src), Some(&dst)) = (
                node_lookup.get(connection.src.as_str()),
                node_lookup.get(connection.dst.as_str()),
            ) else {
                continue;
            };
            incoming[dst].push(Edge {
                connection: index,
                neighbor: src,
            });
            outgoing[src].push(Edge {
                connection: index,
                neighbor: dst,
            });
        }
        let search_names = topology
            .nodes
            .iter()
            .map(|node| node.id.to_lowercase())
            .collect();
        let matches = (0..topology.nodes.len()).collect();
        let focus = (!topology.nodes.is_empty()).then_some(0);
        Self {
            model,
            component_ids,
            incoming,
            outgoing,
            search_names,
            matches,
            focus,
            column: Column::Nodes,
            lists: std::array::from_fn(|_| ListState::default().with_selected(focus)),
            list_areas: [Rect::default(); 3],
            panel_areas: [Rect::default(); 3],
            query: String::new(),
            searching: false,
            history: VecDeque::with_capacity(HISTORY_CAPACITY),
        }
    }

    pub(crate) fn draw(&mut self, frame: &mut Frame<'_>, area: Rect) {
        view::draw(frame, self, area);
    }

    /// Returns whether this view consumed the key before monitor-wide shortcuts.
    pub(crate) fn handle_key(&mut self, key: MonitorUiKey) -> bool {
        if key == MonitorUiKey::Esc {
            self.query.clear();
            self.searching = false;
            self.update_matches();
            self.lists[0].select(self.focus);
            return true;
        }
        if key == MonitorUiKey::Char('/') && !self.searching {
            self.column = Column::Nodes;
            self.searching = true;
            return true;
        }
        if self.column == Column::Nodes {
            match key {
                MonitorUiKey::Char(c) if !c.is_control() => {
                    self.query.push(c);
                    self.searching = true;
                    self.filter();
                    return true;
                }
                MonitorUiKey::Backspace if !self.query.is_empty() => {
                    self.query.pop();
                    self.searching = !self.query.is_empty();
                    self.filter();
                    return true;
                }
                _ => {}
            }
        }
        match key {
            MonitorUiKey::Left | MonitorUiKey::Char('h') => self.column = self.column.next(false),
            MonitorUiKey::Right | MonitorUiKey::Char('l') | MonitorUiKey::Tab => {
                self.column = self.column.next(true)
            }
            MonitorUiKey::Up | MonitorUiKey::Char('k') => self.move_selection(false),
            MonitorUiKey::Down | MonitorUiKey::Char('j') => self.move_selection(true),
            MonitorUiKey::Enter => self.follow(),
            MonitorUiKey::Backspace => self.back(),
            MonitorUiKey::Char('/') => {
                self.column = Column::Nodes;
                self.searching = true;
            }
            _ => return false,
        }
        true
    }

    pub(crate) fn scroll(&mut self, direction: ScrollDirection, steps: usize) {
        for _ in 0..steps {
            match direction {
                ScrollDirection::Up => self.move_selection(false),
                ScrollDirection::Down => self.move_selection(true),
                ScrollDirection::Left => {
                    self.column = self.column.next(false);
                    break;
                }
                ScrollDirection::Right => {
                    self.column = self.column.next(true);
                    break;
                }
            }
        }
    }

    pub(crate) fn scroll_at(&mut self, x: u16, y: u16, direction: ScrollDirection, steps: usize) {
        if let Some(column) = self
            .panel_areas
            .iter()
            .position(|area| area.contains(Position::new(x, y)))
        {
            self.column = Column::ALL[column];
            self.searching = false;
            self.scroll(direction, steps);
        }
    }

    pub(crate) fn click(&mut self, x: u16, y: u16) {
        let Some(column) = self
            .list_areas
            .iter()
            .position(|area| area.contains(Position::new(x, y)))
        else {
            return;
        };
        let row = self.lists[column].offset() + usize::from(y - self.list_areas[column].y);
        let column = Column::ALL[column];
        if row >= self.count(column) {
            return;
        }
        self.searching = false;
        self.column = column;
        self.lists[column.index()].select(Some(row));
        self.follow();
    }

    fn neighbors(&self, column: Column) -> &[Edge] {
        let Some(focus) = self.focus else {
            return &[];
        };
        match column {
            Column::Incoming => &self.incoming[focus],
            Column::Outgoing => &self.outgoing[focus],
            Column::Nodes => &[],
        }
    }

    fn count(&self, column: Column) -> usize {
        if column == Column::Nodes {
            self.matches.len()
        } else {
            self.neighbors(column).len()
        }
    }

    fn update_matches(&mut self) {
        let query = self.query.to_lowercase();
        self.matches.clear();
        self.matches.extend(
            self.search_names
                .iter()
                .enumerate()
                .filter(|(_, name)| name.contains(&query))
                .map(|(index, _)| index),
        );
    }

    fn filter(&mut self) {
        self.update_matches();
        self.lists[0] = ListState::default().with_selected((!self.matches.is_empty()).then_some(0));
        self.follow();
    }

    fn move_selection(&mut self, down: bool) {
        let len = self.count(self.column);
        if len == 0 {
            return;
        }
        let list = &mut self.lists[self.column.index()];
        let current = list.selected().unwrap_or(0);
        let next = if down {
            (current + 1).min(len - 1)
        } else {
            current.saturating_sub(1)
        };
        list.select(Some(next));
        if self.column == Column::Nodes {
            self.follow();
        }
    }

    fn follow(&mut self) {
        let selected = self.lists[self.column.index()].selected().unwrap_or(0);
        let next = if self.column == Column::Nodes {
            self.matches.get(selected).copied()
        } else {
            self.neighbors(self.column)
                .get(selected)
                .map(|edge| edge.neighbor)
        };
        let Some(next) = next else {
            return;
        };
        if self.focus == Some(next) {
            return;
        }
        if self.history.len() == HISTORY_CAPACITY {
            self.history.pop_front();
        }
        self.history.push_back(Navigation {
            focus: self.focus,
            column: self.column,
            lists: self.lists,
            query: self.query.clone(),
        });
        self.focus = Some(next);
        if !self.matches.contains(&next) {
            self.query.clear();
            self.update_matches();
        }
        self.lists[0].select(self.matches.iter().position(|&node| node == next));
        for column in [Column::Incoming, Column::Outgoing] {
            self.lists[column.index()] =
                ListState::default().with_selected((self.count(column) > 0).then_some(0));
        }
    }

    fn back(&mut self) {
        if let Some(previous) = self.history.pop_back() {
            self.focus = previous.focus;
            self.column = previous.column;
            self.lists = previous.lists;
            self.query = previous.query;
            self.searching = false;
            self.update_matches();
            self.lists[0].select(
                self.matches
                    .iter()
                    .position(|&node| Some(node) == self.focus),
            );
        }
    }
}

#[cfg(test)]
mod tests;
