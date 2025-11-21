use ratatui::layout::Margin;
use ratatui::style::Color;
use ratatui::style::Modifier;
use ratatui::{buffer::Buffer, layout::Rect, style::Style, widgets::Widget};
use std::collections::BTreeSet as Set;
use std::collections::HashMap as Map;

mod connection;
use connection::*;
pub use connection::{Connection, LineType};
mod node;
pub use node::NodeLayout;
mod graph;
pub use graph::*;
