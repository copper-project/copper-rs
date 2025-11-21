use ratatui::layout::{Position, Size};

use super::*;

const MARGIN: u16 = 10;
const LOOP_OFFSET: i32 = 3;

#[derive(Debug)]
pub struct NodeGraph<'a> {
	nodes: Vec<NodeLayout<'a>>,
	connections: Vec<Connection>,
	placements: Map<usize, Rect>,
	pub conn_layout: ConnectionsLayout,
	width: usize,
	loop_connections: Vec<Connection>,
}

impl<'a> NodeGraph<'a> {
	pub fn new(
		nodes: Vec<NodeLayout<'a>>,
		connections: Vec<Connection>,
		width: usize,
		height: usize,
	) -> Self {
		Self {
			nodes,
			connections,
			conn_layout: ConnectionsLayout::new(width, height),
			placements: Default::default(),
			width,
			loop_connections: Vec::new(),
		}
	}

	pub fn calculate(&mut self) {
		self.placements.clear();
		self.loop_connections.clear();

		// find root nodes
		let mut roots: Set<_> = (0..self.nodes.len()).collect();
		for ea_connection in self.connections.iter() {
			if ea_connection.from_node != ea_connection.to_node {
				roots.remove(&ea_connection.from_node);
			}
		}
		if roots.is_empty() && !self.nodes.is_empty() {
			roots.insert(0);
		}

		// place them and their children (recursively)
		let mut main_chain = Vec::new();
		for ea_root in roots {
			self.place_node(ea_root, 0, 0, &mut main_chain);
			assert!(main_chain.is_empty());
		}

		// calculate connections (eventually, this should be done during node
		// placement, but thats really complicated and i dont wanna deal with that
		// right now. essentially, adding non-trivial connections nudges nodes,
		// and nudging nodes nudges existing connections.)
		let mut conn_map = Map::<(usize, usize), usize>::new();
		let mut next_idx = 1;
		for ea_conn in self.connections.iter() {
			if ea_conn.from_node == ea_conn.to_node {
				self.loop_connections.push(*ea_conn);
				continue;
			}
			let a_pos = self.placements[&ea_conn.from_node];
			let b_pos = self.placements[&ea_conn.to_node];
			// NOTE: don't forget that left and right are swapped
			let a_point = (
				self.width.saturating_sub(a_pos.left().into()),
				a_pos.top() as usize + ea_conn.from_port + 1,
			);
			let b_point = (
				self.width.saturating_sub(b_pos.right() as usize + 1),
				b_pos.top() as usize + ea_conn.to_port + 1,
			);
			self.conn_layout.insert_port(
				false,
				ea_conn.from_node,
				ea_conn.from_port,
				a_point,
			);
			self.conn_layout.insert_port(true, ea_conn.to_node, ea_conn.to_port, b_point);
			let key = (ea_conn.from_node, ea_conn.from_port);
			if !conn_map.contains_key(&key) {
				conn_map.insert(key, next_idx);
				next_idx += 1;
			}
			self.conn_layout.push_connection((*ea_conn, conn_map[&key]));
			self.conn_layout.block_port(a_point, false);
			self.conn_layout.block_port(b_point, true);
		}
		for mut ea_placement in self.placements.values().cloned() {
			ea_placement.x =
				(self.width as u16).saturating_sub(ea_placement.x + ea_placement.width);
			self.conn_layout.block_zone(ea_placement);
		}
		self.conn_layout.calculate();
	}

	/// ATTENTION: x_offs works in the opposite direction (higher values are
	/// further left) and y_offs is the same as tui (higher values are further
	/// down)
	fn place_node(
		&mut self,
		idx_node: usize,
		x: u16,
		y: u16,
		main_chain: &mut Vec<usize>,
	) {
		// place the node
		let size_me = self.nodes[idx_node].size;
		let mut rect_me = Rect { x, y, width: size_me.0, height: size_me.1 };

		// nudge placement. if a node intersects with another node, its entire
		// main chain (largest subset of nodes including this one where every
		// node is the first child of its parent) should be moved down to not
		// intersect.
		//
		// Repeat the for loop until in runs all the way through without any
		// intersections. Surely there's a more efficient way to do this.
		'outer: loop {
			for (_, ea_them) in self.placements.iter() {
				if rect_me.intersects(*ea_them) {
					rect_me.y = rect_me.y.max(ea_them.bottom());
					continue 'outer;
				}
			}
			break;
		}
		for ea_node in main_chain.iter() {
			let y = self.placements[ea_node].y.max(rect_me.y);
			self.placements.get_mut(ea_node).unwrap().y = y;
		}
		self.placements.insert(idx_node, rect_me);

		// find children and order them
		let mut y = y;
		main_chain.push(idx_node);
		for ea_child in get_upstream(&self.connections, idx_node) {
			if ea_child.from_node == idx_node {
				continue;
			}
			if self.placements.contains_key(&ea_child.from_node) {
				// nudge it (if necessary)
				self.nudge(ea_child.from_node, rect_me.x + rect_me.width + MARGIN);
			} else {
				// place it
				self.place_node(
					ea_child.from_node,
					x + rect_me.width + MARGIN,
					y,
					main_chain,
				);
				main_chain.clear();
				y += self.placements[&ea_child.from_node].height;
			}
		}
		main_chain.pop();
	}

	fn nudge(&mut self, idx_node: usize, x: u16) {
		let rect_me = self.placements[&idx_node];
		if rect_me.x < x {
			self.placements.get_mut(&idx_node).unwrap().x = x;
			for ea_child in get_upstream(&self.connections, idx_node) {
				if ea_child.from_node == idx_node {
					continue;
				}
				assert!(self.placements.contains_key(&ea_child.from_node));
				self.nudge(ea_child.from_node, x + rect_me.width + MARGIN);
			}
		}
	}

	pub fn content_bounds(&self) -> Size {
		let mut width = 0;
		let mut height = 0;
		for placement in self.placements.values() {
			width = width.max(placement.right());
			height = height.max(placement.bottom());
		}
		Size::new(width, height)
	}

	pub fn split(&self, area: Rect) -> Vec<Rect> {
		(0..self.nodes.len())
			.map(|idx_node| {
				self.placements
					.get(&idx_node)
					.map(|pos| {
						if pos.right() > area.width || pos.bottom() > area.height {
							return Rect { x: 0, y: 0, width: 0, height: 0 };
						}
						let mut pos = *pos;
						pos.x = area.width - pos.right() + area.x;
						pos.y += area.y;
						pos.inner(Margin { horizontal: 1, vertical: 1 })
					})
					.unwrap_or_default()
			})
			.collect()
	}
}

fn get_upstream(conns: &[Connection], idx_node: usize) -> Vec<Connection> {
	// find children and order them
	let mut upstream: Vec<_> =
		conns.iter().filter(|ea| ea.to_node == idx_node).copied().collect();
	upstream.sort_by(|a, b| a.to_port.cmp(&b.to_port));
	upstream
}

fn get_downstream(conns: &[Connection], idx_node: usize) -> Vec<Connection> {
	// find parents and order them
	let mut downstream: Vec<_> =
		conns.iter().filter(|ea| ea.from_node == idx_node).copied().collect();
	downstream.sort_by(|a, b| a.from_port.cmp(&b.from_port));
	downstream
}

impl<'a> ratatui::widgets::StatefulWidget for NodeGraph<'a> {
	// eventually, this will contain stuff like view position
	//	type State = NodeGraphState;
	type State = ();

	fn render(self, area: Rect, buf: &mut Buffer, _state: &mut Self::State) {
		// draw connections
		self.conn_layout.render(area, buf);
		self.draw_loop_connections(area, buf);

		// draw nodes
		'node: for (idx_node, ea_node) in self.nodes.into_iter().enumerate() {
			if let Some(mut pos) = self.placements.get(&idx_node).copied() {
				if pos.right() > area.width || pos.bottom() > area.height {
					continue 'node;
				}
				// draw box
				pos.x = area.left() + area.width - pos.right();
				pos.y += area.top();
				let block = ea_node.block();
				block.render(pos, buf);
				// draw connection ports
				for ea_conn in get_upstream(&self.connections, idx_node) {
					// draw connection alias
					if let Some(alias_char) = self.conn_layout.alias_connections.get(&(
						true,
						idx_node,
						ea_conn.to_port,
					)) {
						let y = pos.top() + ea_conn.to_port as u16 + 1;
						if pos.left() > 0 && y < area.width {
							buf.cell_mut(Position::new(pos.left() - 1, y))
								.unwrap()
								.set_symbol(alias_char)
								.set_style(
									Style::default()
										.add_modifier(Modifier::BOLD)
										.bg(Color::Red),
								);
						}
					}

					// draw port
					buf.cell_mut(Position::new(
						pos.left(),
						pos.top() + ea_conn.to_port as u16 + 1,
					))
					.unwrap()
					.set_symbol(conn_symbol(
						true,
						ea_node.border_type(),
						ea_conn.line_type(),
					));
				}
				for ea_conn in get_downstream(&self.connections, idx_node) {
					// draw connection alias
					if let Some(alias_char) = self.conn_layout.alias_connections.get(&(
						false,
						idx_node,
						ea_conn.from_port,
					)) {
						buf.cell_mut(Position::new(
							pos.right(),
							pos.top() + ea_conn.from_port as u16 + 1,
						))
						.unwrap()
						.set_symbol(alias_char)
						.set_style(
							Style::default().add_modifier(Modifier::BOLD).bg(Color::Red),
						);
					}

					// draw port
					buf.cell_mut(Position::new(
						pos.right() - 1,
						pos.top() + ea_conn.from_port as u16 + 1,
					))
					.unwrap()
					.set_symbol(conn_symbol(
						false,
						ea_node.border_type(),
						ea_conn.line_type(),
					));
				}
			} else {
				buf.set_string(
					0,
					idx_node as u16,
					format!("{idx_node}"),
					Style::default(),
				);
			}
		}
	}
}

impl<'a> NodeGraph<'a> {
	fn draw_loop_connections(&self, area: Rect, buf: &mut Buffer) {
		for conn in &self.loop_connections {
			if let Some(rect) = self.placements.get(&conn.from_node).copied() {
				if rect.right() > area.width || rect.bottom() > area.height {
					continue;
				}
				let mut pos = rect;
				pos.x = area.left() + area.width - pos.right();
				pos.y += area.top();
				self.draw_loop_path(pos, *conn, area, buf);
			}
		}
	}

	fn draw_loop_path(&self, rect: Rect, conn: Connection, area: Rect, buf: &mut Buffer) {
		let start_y = rect.top().saturating_add(conn.from_port as u16 + 1);
		let end_y = rect.top().saturating_add(conn.to_port as u16 + 1);
		let start_x = rect.right().saturating_sub(1);
		let end_x = rect.left();

		let start_x_i = start_x as i32;
		let end_x_i = end_x as i32;
		let start_y_i = start_y as i32;
		let end_y_i = end_y as i32;
		let area_left = area.left() as i32;
		let area_right = (area.left() + area.width).saturating_sub(1) as i32;
		let area_bottom = (area.top() + area.height).saturating_sub(1) as i32;

		let outer_x = (start_x_i + LOOP_OFFSET).min(area_right);
		if outer_x <= start_x_i {
			return;
		}

		let mut bottom_y = (rect.bottom() as i32 + LOOP_OFFSET).min(area_bottom);
		let max_y = start_y_i.max(end_y_i);
		if bottom_y <= max_y {
			bottom_y = (max_y + 1).min(area_bottom);
		}
		if bottom_y <= max_y {
			return;
		}

		let left_x = (end_x_i - 1).max(area_left);
		if left_x >= end_x_i || left_x >= outer_x {
			return;
		}

		let set = conn.line_type().to_line_set();
		let style = conn.line_style();

		draw_horizontal(
			buf,
			start_y_i,
			start_x_i + 1,
			outer_x,
			set.horizontal,
			style,
			area,
		);
		draw_corner(buf, outer_x, start_y_i, set.top_right, style, area);
		draw_vertical(
			buf,
			outer_x,
			start_y_i + 1,
			bottom_y - 1,
			set.vertical,
			style,
			area,
		);
		draw_corner(buf, outer_x, bottom_y, set.bottom_right, style, area);
		draw_horizontal(
			buf,
			bottom_y,
			left_x + 1,
			outer_x - 1,
			set.horizontal,
			style,
			area,
		);
		draw_corner(buf, left_x, bottom_y, set.bottom_left, style, area);
		draw_vertical(buf, left_x, end_y_i + 1, bottom_y - 1, set.vertical, style, area);
		draw_corner(buf, left_x, end_y_i, set.top_left, style, area);
	}
}

fn draw_horizontal(
	buf: &mut Buffer,
	y: i32,
	mut start: i32,
	mut end: i32,
	symbol: &str,
	style: Style,
	area: Rect,
) {
	if start > end {
		std::mem::swap(&mut start, &mut end);
	}
	for x in start..=end {
		set_cell(buf, x, y, symbol, style, area);
	}
}

fn draw_vertical(
	buf: &mut Buffer,
	x: i32,
	mut start: i32,
	mut end: i32,
	symbol: &str,
	style: Style,
	area: Rect,
) {
	if start > end {
		std::mem::swap(&mut start, &mut end);
	}
	for y in start..=end {
		set_cell(buf, x, y, symbol, style, area);
	}
}

fn draw_corner(buf: &mut Buffer, x: i32, y: i32, symbol: &str, style: Style, area: Rect) {
	set_cell(buf, x, y, symbol, style, area);
}

fn set_cell(buf: &mut Buffer, x: i32, y: i32, symbol: &str, style: Style, area: Rect) {
	let min_x = area.left() as i32;
	let max_x = (area.left() + area.width).saturating_sub(1) as i32;
	let min_y = area.top() as i32;
	let max_y = (area.top() + area.height).saturating_sub(1) as i32;
	if x < min_x || x > max_x || y < min_y || y > max_y {
		return;
	}
	if let Some(cell) = buf.cell_mut(Position::new(x as u16, y as u16)) {
		cell.set_symbol(symbol).set_style(style);
	}
}
