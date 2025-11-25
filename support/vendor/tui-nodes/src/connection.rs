use ratatui::{
	buffer::Buffer,
	layout::{Position, Rect},
	style::Style,
	symbols::line,
	widgets::BorderType,
};
use std::collections::{BTreeMap as Map, BinaryHeap};

const SEARCH_TIMEOUT: usize = 5000;

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LineType {
	#[default]
	Plain,
	Rounded,
	Double,
	Thick,
}

impl LineType {
	pub fn to_line_set(&self) -> line::Set {
		match self {
			LineType::Plain => line::NORMAL,
			LineType::Rounded => line::ROUNDED,
			LineType::Double => line::DOUBLE,
			LineType::Thick => line::THICK,
		}
	}
}

impl From<BorderType> for LineType {
	fn from(value: BorderType) -> Self {
		match value {
			BorderType::Plain => LineType::Plain,
			BorderType::Rounded => LineType::Rounded,
			BorderType::Double => LineType::Double,
			BorderType::Thick => LineType::Thick,
			_ => unimplemented!(),
		}
	}
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Direction {
	North = 0,
	South = 1,
	East = 2,
	West = 3,
}

impl Direction {
	fn is_vertical(&self) -> bool {
		(*self as usize) < 2
	}
	/*
	fn invert(self) -> Self {
		use Direction as D;
		match self {
			D::North => D::South,
			D::South => D::North,
			D::East => D::West,
			D::West => D::East,
		}
	}
	fn rotate(self) -> Self {
		use Direction as D;
		match self {
			D::North => D::East,
			D::East => D::South,
			D::South => D::West,
			D::West => D::North,
		}
	}
	*/
}

impl std::fmt::Debug for Direction {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		let print = match self {
			Direction::North => '↑',
			Direction::South => '↓',
			Direction::East => '→',
			Direction::West => '←',
		};
		write!(f, "{}", print)
	}
}

#[derive(Debug, Clone, Copy)]
pub struct Connection {
	pub from_node: usize,
	pub from_port: usize,
	pub to_node: usize,
	pub to_port: usize,
	line_type: LineType,
	line_style: Style,
}

impl Connection {
	pub fn new(
		from_node: usize,
		from_port: usize,
		to_node: usize,
		to_port: usize,
	) -> Self {
		Self {
			from_node,
			from_port,
			to_node,
			to_port,
			line_type: LineType::Rounded,
			line_style: Style::default(),
		}
	}

	pub fn with_line_type(mut self, line_type: LineType) -> Self {
		self.line_type = line_type;
		self
	}

	pub fn line_type(&self) -> LineType {
		self.line_type
	}

	pub fn with_line_style(mut self, line_style: Style) -> Self {
		self.line_style = line_style;
		self
	}

	pub fn line_style(&self) -> Style {
		self.line_style
	}
}

/// Generate the correct connection symbol for this node
pub fn conn_symbol(
	is_input: bool,
	block_style: BorderType,
	conn_style: LineType,
) -> &'static str {
	let out = match (block_style, conn_style) {
		(BorderType::Plain | BorderType::Rounded, LineType::Thick) => ("┥", "┝"),
		(BorderType::Plain | BorderType::Rounded, LineType::Double) => ("╡", "╞"),
		(
			BorderType::Plain | BorderType::Rounded,
			LineType::Plain | LineType::Rounded,
		) => ("┤", "├"),

		(BorderType::Thick, LineType::Thick) => ("┫", "┣"),
		(BorderType::Thick, LineType::Double) => ("╡", "╞"), // fallback
		(BorderType::Thick, LineType::Plain | LineType::Rounded) => ("┨", "┠"),

		(BorderType::Double, LineType::Thick) => ("╢", "╟"), // fallback
		(BorderType::Double, LineType::Double) => ("╣", "╠"),
		(BorderType::Double, LineType::Plain | LineType::Rounded) => ("╢", "╟"),
		(BorderType::QuadrantInside | BorderType::QuadrantOutside, _) => ("u", "u"),
	};
	if is_input {
		out.0
	} else {
		out.1
	}
}

pub const ALIAS_CHARS: [&str; 24] = [
	"α", "β", "γ", "δ", "ε", "ζ", "η", "θ", "ι", "κ", "λ", "μ", "ν", "ξ", "ο", "π", "ρ",
	"σ", "τ", "υ", "φ", "χ", "ψ", "ω",
];

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
	#[default]
	Empty,
	Blocked,
	Connection(usize),
}
const E: Edge = Edge::Empty;
const B: Edge = Edge::Blocked;

#[derive(Debug)]
pub struct ConnectionsLayout {
	ports: Map<(bool, usize, usize), (usize, usize)>, // (x,y)
	connections: Vec<(Connection, usize)>,            // ((from, to), class)
	edge_field: Betweens<Edge>,
	edge_targets: Betweens<Option<(bool, usize, usize)>>,
	width: usize,
	height: usize,
	pub alias_connections: Map<(bool, usize, usize), &'static str>,
	line_types: Map<usize, LineType>,
	line_styles: Map<usize, Style>,
}

impl ConnectionsLayout {
	pub fn new(width: usize, height: usize) -> Self {
		Self {
			ports: Map::new(),
			connections: Vec::new(),
			edge_field: Betweens::new(width, height),
			edge_targets: Betweens::new(width, height),
			width,
			height,
			alias_connections: Map::new(),
			line_types: Map::new(),
			line_styles: Map::new(),
		}
	}

	pub fn push_connection(&mut self, connection: (Connection, usize)) {
		self.connections.push(connection)
	}

	pub fn insert_port(
		&mut self,
		is_input: bool,
		node: usize,
		port: usize,
		pos: (usize, usize),
	) {
		self.ports.insert((is_input, node, port), pos);
	}

	pub fn block_zone(&mut self, area: Rect) {
		for x in 0..area.width {
			for y in 0..area.height {
				if x != area.width - 1 {
					self.edge_field[(
						((x + area.x) as usize, (y + area.y) as usize),
						Direction::East,
					)
						.into()] = Edge::Blocked;
				}
				if y != area.height - 1 {
					self.edge_field[(
						((x + area.x) as usize, (y + area.y) as usize),
						Direction::South,
					)
						.into()] = Edge::Blocked;
				}
			}
		}
	}

	pub fn block_port(&mut self, coord: (usize, usize), is_input: bool) {
		if is_input {
			return;
		}
		self.edge_field[(coord, Direction::North).into()] = Edge::Blocked;
		self.edge_field[(coord, Direction::South).into()] = Edge::Blocked;
	}

	pub fn calculate(&mut self) {
		let mut idx_next_alias = 0;
		'outer: for ea_conn in &self.connections {
			self.line_types.insert(ea_conn.1, ea_conn.0.line_type());
			self.line_styles.insert(ea_conn.1, ea_conn.0.line_style());
			let start = (
				self.ports[&(false, ea_conn.0.from_node, ea_conn.0.from_port)],
				Direction::West,
			);
			let goal = (
				self.ports[&(true, ea_conn.0.to_node, ea_conn.0.to_port)],
				Direction::East,
			);
			if start.0 .0 > self.edge_field.width || start.0 .1 > self.edge_field.height {
				continue;
			}
			if goal.0 .0 > self.edge_field.width || goal.0 .1 > self.edge_field.height {
				continue;
			}
			//println!("drawing connection {start:?} to {goal:?}");
			let mut frontier = BinaryHeap::new();
			let mut came_from = Betweens::<Option<_>>::new(self.width, self.height);
			let mut cost = Betweens::<isize>::new(self.width, self.height);
			frontier.push(((0, 0), start));
			let mut count = 0;
			while let Some((_, current)) = frontier.pop() {
				count += 1;
				if count > SEARCH_TIMEOUT {
					break;
				}
				if current == goal {
					break;
				}
				for ea_nei in neighbors(current.0, self.width, self.height) {
					let ea_edge = ea_nei.into();
					let current_cost = cost[current.into()];
					//println!("{current_cost}");
					let new_cost = current_cost.saturating_add(self.calc_cost(
						current,
						ea_nei,
						start.0,
						goal.0,
						(true, ea_conn.0.to_node, ea_conn.0.to_port),
						ea_conn.1,
					));
					if came_from[ea_edge].is_none() || new_cost < cost[ea_edge] {
						let prio = (-new_cost, -Self::heuristic(ea_nei.0, goal.0));
						if new_cost != isize::MAX {
							frontier.push((prio, ea_nei));
						}
						came_from[ea_edge] = Some(current);
						cost[ea_edge] = new_cost;
					}
				}
				/*
				print!("\x1b[2J\x1b[1;1H");
				println!("{frontier:?}");
				let mut prio = Betweens::new(self.width, self.height);
				for ea_front in frontier.iter() {
					prio[ea_front.1.into()] = ea_front.0;
				}
				println!("prio\n");
				prio.print_with(4, |ea| print!("{:>4} ", ea.0));
				prio.print_with(4, |ea| print!("{:>4} ", ea.1));
				println!("cost\n");
				cost.print_with(4, |ea| print!("{:>4} ", ea));
				println!("from\n");
				came_from.print_with(1, |ea| {
					if let Some(inner) = ea {
						print!("{:?} ", inner.1);
					}
					else {
						print!("_ ");
					}
				});
				std::io::stdin().read_line(&mut String::new()).unwrap();
				*/
			}
			// first pass: mark connections that didnt reach the goal
			let mut next = goal;
			loop {
				if next == start {
					break;
				}
				if let Some(from) = came_from[next.into()] {
					next = from;
				} else {
					// register alias character
					if !self.alias_connections.contains_key(&(
						false,
						ea_conn.0.from_node,
						ea_conn.0.from_port,
					)) {
						self.alias_connections.insert(
							(false, ea_conn.0.from_node, ea_conn.0.from_port),
							ALIAS_CHARS[idx_next_alias],
						);
						idx_next_alias += 1;
					}
					let alias = self.alias_connections
						[&(false, ea_conn.0.from_node, ea_conn.0.from_port)];
					self.alias_connections
						.insert((true, ea_conn.0.to_node, ea_conn.0.to_port), alias);
					continue 'outer;
				}
			}

			// second pass: draw edges
			let mut next = goal;
			loop {
				if next == start {
					break;
				}
				self.edge_field[next.into()] = Edge::Connection(ea_conn.1);
				self.edge_targets[next.into()] =
					Some((true, ea_conn.0.to_node, ea_conn.0.to_port));
				next = came_from[next.into()].unwrap();
			}
		}
	}

	pub fn render(&self, area: Rect, buf: &mut Buffer) {
		let bor = |idx: Edge| -> line::Set {
			if let Edge::Connection(idx) = idx {
				self.line_types[&idx].to_line_set()
			} else if idx == Edge::Blocked {
				line::THICK
			} else {
				line::Set {
					vertical: " ",
					horizontal: " ",
					top_right: " ",
					top_left: " ",
					bottom_right: " ",
					bottom_left: " ",
					vertical_left: " ",
					vertical_right: " ",
					horizontal_down: " ",
					horizontal_up: " ",
					cross: " ",
				}
			}
		};

		let get_line_style = |idx: Edge| -> Style {
			if let Edge::Connection(idx) = idx {
				self.line_styles[&idx]
			} else {
				Style::default()
			}
		};
		let is_conn = |edge: Edge| matches!(edge, Edge::Connection(_));
		let pick_style = |edges: &[Edge]| {
			edges
				.iter()
				.find_map(|edge| {
					if let Edge::Connection(idx) = edge {
						Some(self.line_styles[&idx])
					} else {
						None
					}
				})
				.unwrap_or_default()
		};
		for y in 0..self.height {
			for x in 0..self.width {
				let pos = (x, y);
				let north = self.edge_field[(pos, Direction::North).into()];
				let south = self.edge_field[(pos, Direction::South).into()];
				let east = self.edge_field[(pos, Direction::East).into()];
				let west = self.edge_field[(pos, Direction::West).into()];
				#[rustfmt::skip]
				let (symbol, line_style) = match (north, south, east, west) {
					(B | E, B | E, B | E, B | E) => continue,
					(n, s, e, w) if n == B || s == B || e == B || w == B => {
						if n == B && s == B && e != E || w != E && e == w {
							(bor(e).horizontal, get_line_style(e))
						} else if e == B && w == B && n != E && s != E && n == s {
							(bor(n).vertical, get_line_style(n))
						} else {
							("*", Style::default())
						}
					}
					(n, E, E, E) => (bor(n).vertical, get_line_style(n)),
					(E, s, E, E) => (bor(s).vertical, get_line_style(s)),
					(E, E, e, E) => (bor(e).horizontal, get_line_style(e)),
					(E, E, E, w) => (bor(w).horizontal, get_line_style(w)),

					(n, s, E, w) if n == s && n == w => (bor(n).vertical_left, get_line_style(n)),
					(n, E, e, w) if n == e && n == w => (bor(n).horizontal_up, get_line_style(n)),
					(n, s, e, E) if n == s && n == e => (bor(n).vertical_right, get_line_style(n)),
					(E, s, e, w) if s == e && s == w => (bor(s).horizontal_down, get_line_style(s)),
					(E, s, E, w) if s == w => (bor(s).top_right, get_line_style(s)),
					(n, E, E, w) if n == w => (bor(n).bottom_right, get_line_style(n)),
					(n, E, e, E) if n == e => (bor(n).bottom_left, get_line_style(n)),
					(E, s, e, E) if s == e => (bor(s).top_left, get_line_style(s)),

					(n, s, E, E) if n == s => (bor(n).vertical, get_line_style(n)),
					(E, E, e, w) if e == w => (bor(e).horizontal, get_line_style(e)),

					(n, s, e, w) if n == s && n == e && n == w => (bor(n).cross, get_line_style(n)),
					// intersections should just be verticals
					(n, s, e, w) if n == s && e == w && n != E && e != E => (bor(n).vertical, get_line_style(n)),
					(n, s, e, w) if is_conn(n) && is_conn(s) && is_conn(e) && !is_conn(w) => {
						(bor(n).vertical_right, pick_style(&[n, s, e]))
					}
					(n, s, e, w) if is_conn(n) && is_conn(s) && !is_conn(e) && is_conn(w) => {
						(bor(n).vertical_left, pick_style(&[n, s, w]))
					}
					(n, s, e, w) if is_conn(n) && !is_conn(s) && is_conn(e) && is_conn(w) => {
						(bor(e).horizontal_up, pick_style(&[n, e, w]))
					}
					(n, s, e, w) if !is_conn(n) && is_conn(s) && is_conn(e) && is_conn(w) => {
						(bor(e).horizontal_down, pick_style(&[s, e, w]))
					}
					(_, _, _, _) => ("?", Style::default()),
				};

				buf.cell_mut(Position::new(
					x as u16 + area.left(),
					y as u16 + area.top(),
				))
				.unwrap()
				.set_symbol(symbol)
				.set_style(line_style);
			}
		}
	}

	fn heuristic(from: (usize, usize), to: (usize, usize)) -> isize {
		(from.0 as isize - to.0 as isize).pow(2)
			+ (from.1 as isize - to.1 as isize).pow(2)
	}

	fn calc_cost(
		&self,
		current: ((usize, usize), Direction),
		neigh: ((usize, usize), Direction),
		start: (usize, usize),
		end: (usize, usize),
		goal: (bool, usize, usize),
		conn_t: usize,
	) -> isize {
		let conn_t = Edge::Connection(conn_t);
		let north = self.edge_field[(current.0, Direction::North).into()];
		let south = self.edge_field[(current.0, Direction::South).into()];
		let east = self.edge_field[(current.0, Direction::East).into()];
		let west = self.edge_field[(current.0, Direction::West).into()];

		let in_dir = self.edge_field[current.into()];
		// TODO: fix
		if !(in_dir == Edge::Empty || in_dir == conn_t) {
			return isize::MAX;
		}
		//	assert!(in_dir == 0 || in_dir == conn_t); // should only calculate cost if its possible
		let out_dir = self.edge_field[neigh.into()];
		if out_dir == conn_t {
			// already exists
			1
		} else if let Edge::Connection(_) = out_dir {
			if self.edge_targets[neigh.into()]
				.is_some_and(|target| target == goal && goal.0)
			{
				3
			} else {
				isize::MAX
			}
		} else if out_dir == Edge::Empty {
			if north == conn_t || south == conn_t || east == conn_t || west == conn_t {
				// intersecting with an existing connection
				2 // maybe multiply with distances?
			} else {
				let in_is_vert = current.1.is_vertical();
				let out_is_vert = neigh.1.is_vertical();
				let straight = in_is_vert == out_is_vert;
				if straight {
					if north == Edge::Empty
						&& south == Edge::Empty
						&& east == Edge::Empty
						&& west == Edge::Empty
					{
						2
					} else {
						4
					}
				} else {
					// curved
					if north != Edge::Empty
						|| south != Edge::Empty
						|| east != Edge::Empty
						|| west != Edge::Empty
					{
						isize::MAX
					} else {
						let ax = current.0 .0 as isize;
						let ay = current.0 .1 as isize;
						let sx = start.0 as isize;
						let sy = start.1 as isize;
						let ex = end.0 as isize;
						let ey = end.1 as isize;
						4 + ((ax - sx).pow(2)
							+ (ay - sy).pow(2) + (ax - ex).pow(2)
							+ (ay - ey).pow(2))
					}
				}
			}
		} else {
			isize::MAX
		}
	}
}

fn neighbors(
	pos: (usize, usize),
	width: usize,
	height: usize,
) -> Vec<((usize, usize), Direction)> {
	let mut out = Vec::new();
	if pos.0 < width - 1 {
		out.push(((pos.0 + 1, pos.1), Direction::West));
	}
	if pos.1 < height - 1 {
		out.push(((pos.0, pos.1 + 1), Direction::North));
	}
	if pos.0 > 0 {
		out.push(((pos.0 - 1, pos.1), Direction::East));
	}
	if pos.1 > 0 {
		out.push(((pos.0, pos.1 - 1), Direction::South));
	}
	out
}

use core::ops::{Index, IndexMut};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct EdgeIdx {
	x: usize,
	y: usize,
	is_vertical: bool,
}
/*
impl EdgeIdx {
	fn pos(self) -> (usize, usize) {
		(self.0, self.1)
	}
}
*/
impl From<((usize, usize), Direction)> for EdgeIdx {
	fn from(value: ((usize, usize), Direction)) -> Self {
		match value.1 {
			Direction::North => Self { x: value.0 .0, y: value.0 .1, is_vertical: true },
			Direction::South => Self {
				x: value.0 .0,
				y: value.0 .1 + 1,
				is_vertical: true,
			},
			Direction::East => Self {
				x: value.0 .0 + 1,
				y: value.0 .1,
				is_vertical: false,
			},
			Direction::West => Self { x: value.0 .0, y: value.0 .1, is_vertical: false },
		}
	}
}

// the outermost values are unnecessary
#[derive(Debug)]
struct Betweens<T: Default> {
	horizontal: Vec<Vec<T>>,
	vertical: Vec<Vec<T>>,
	width: usize,
	height: usize,
}
impl<T: Default> Index<EdgeIdx> for Betweens<T> {
	type Output = T;
	fn index(&self, index: EdgeIdx) -> &Self::Output {
		if index.is_vertical {
			&self.vertical[index.y][index.x]
		} else {
			&self.horizontal[index.y][index.x]
		}
	}
}
impl<T: Default> IndexMut<EdgeIdx> for Betweens<T> {
	fn index_mut(&mut self, index: EdgeIdx) -> &mut T {
		if index.is_vertical {
			&mut self.vertical[index.y][index.x]
		} else {
			&mut self.horizontal[index.y][index.x]
		}
	}
}

impl<T: Default> Betweens<T> {
	fn new(x: usize, y: usize) -> Self {
		let mut out = Self {
			horizontal: Vec::new(),
			vertical: Vec::new(),
			width: 0,
			height: 0,
		};
		out.set_size(x, y);
		out
	}

	fn set_size(&mut self, x: usize, y: usize) {
		self.horizontal.resize_with(y, || {
			let mut inner = Vec::new();
			inner.resize_with(x + 1, Default::default);
			inner
		});
		self.vertical.resize_with(y + 1, || {
			let mut inner = Vec::new();
			inner.resize_with(x, Default::default);
			inner
		});
		self.width = x;
		self.height = y;
	}

	#[allow(unused)]
	fn print_with(&self, width: usize, f: impl Fn(&T)) {
		for y in 0..(self.height + 1) {
			for x in 0..self.width {
				print!("{} ", "-".repeat(width));
				f(&self.vertical[y][x]);
			}
			println!("{}", "-".repeat(width));
			if y < self.height {
				for x in 0..(self.width + 1) {
					f(&self.horizontal[y][x]);
					if x < self.width {
						print!("{} ", "-".repeat(width));
					}
				}
			}
			println!();
		}
	}
}
