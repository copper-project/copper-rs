// boilerplate from from tui-rs examples

use ratatui::{
	backend::CrosstermBackend,
	layout::Rect,
	style::{Color, Style},
	widgets::{BorderType, Paragraph},
	Frame, Terminal,
};

use tui_nodes::*;

struct App {}

impl App {
	fn new() -> Self {
		Self {}
	}
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
	// setup terminal
	let mut out = Vec::new();
	let backend = CrosstermBackend::new(&mut out);
	let mut terminal = Terminal::new(backend)?;

	// create app and run it
	let app = App::new();
	terminal.draw(|f| ui(f, &app))?;

	drop(terminal);

	print!("\x1b[2J\x1b[1;1H");
	println!("{}", std::str::from_utf8(&out).unwrap());

	Ok(())
}

fn ui(f: &mut Frame, _app: &App) {
	let space = Rect { x: 0, y: 0, width: 54, height: 10 };
	let mut graph = NodeGraph::new(
		vec![
			NodeLayout::new((4, 4)).with_border_type(BorderType::Thick),
			NodeLayout::new((4, 4)).with_border_type(BorderType::Thick),
			NodeLayout::new((4, 4)).with_border_type(BorderType::Rounded),
			NodeLayout::new((4, 4)).with_border_type(BorderType::Thick),
			NodeLayout::new((4, 4)).with_border_type(BorderType::Double),
			NodeLayout::new((4, 4)).with_border_type(BorderType::Thick),
			NodeLayout::new((4, 4)).with_border_type(BorderType::Double),
		],
		vec![
			Connection::new(0, 0, 1, 0).with_line_type(LineType::Double), // a | b
			Connection::new(1, 0, 2, 0)
				.with_line_type(LineType::Thick)
				.with_line_style(Style::default().fg(Color::Red)), // b | c
			Connection::new(3, 0, 2, 1).with_line_type(LineType::Double), // d > c
			Connection::new(4, 0, 3, 0),                                  // e | d
			Connection::new(4, 0, 0, 1),                                  // e | d
			Connection::new(5, 0, 1, 1),                                  // f > b
			Connection::new(5, 0, 4, 1)
				.with_line_type(LineType::Thick)
				.with_line_style(Style::default().fg(Color::Red)), // f > e
			Connection::new(6, 0, 0, 0),                                  // g | a
			Connection::new(6, 0, 5, 0).with_line_type(LineType::Double), // g | f
		],
		space.width as usize,
		space.height as usize,
	);
	graph.calculate();
	let zones = graph.split(space);
	for (idx, ea_zone) in zones.into_iter().enumerate() {
		f.render_widget(Paragraph::new(format!("{idx}")), ea_zone);
	}
	f.render_stateful_widget(graph, space, &mut ());
}
