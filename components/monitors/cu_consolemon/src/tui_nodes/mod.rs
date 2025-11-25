// Vendored from tui-nodes v0.9.0 (MIT) to avoid depending on an unpublished fork.

use ratatui::layout::Margin;
use ratatui::style::{Color, Modifier, Style};
use ratatui::{buffer::Buffer, layout::Rect, widgets::Widget};
use std::collections::{BTreeSet as Set, HashMap as Map};

mod connection;
mod graph;
mod node;

pub use connection::{conn_symbol, Connection, ConnectionsLayout};
pub use graph::NodeGraph;
pub use node::NodeLayout;
