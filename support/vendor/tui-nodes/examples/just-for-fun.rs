// boilerplate from from tui-rs examples

use ratatui::{backend::CrosstermBackend, Frame, Terminal};

use tui_nodes::*;

struct App {}

impl App {
	fn new() -> Self {
		Self {}
	}
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
	print!("\x1b[2J\x1b[1;1H");
	// setup terminal
	let stdout = std::io::stdout();
	let backend = CrosstermBackend::new(stdout);
	let mut terminal = Terminal::new(backend)?;

	// create app and run it
	let app = App::new();
	terminal.draw(|f| ui(f, &app))?;

	Ok(())
}

fn ui(f: &mut Frame, _app: &App) {
	let space = f.area();
	let mut graph = NodeGraph::new(
		vec![
			NodeLayout::new((40, 10)).with_title("a"),
			NodeLayout::new((40, 10)).with_title("b"),
			NodeLayout::new((40, 10)).with_title("c"),
		],
		vec![
			Connection::new(0, 0, 1, 0),
			Connection::new(0, 0, 2, 0),
			Connection::new(1, 0, 2, 1),
		],
		space.width as usize,
		space.height as usize,
	);
	graph.calculate();
	for ea_zone in graph.split(space) {
		let mut minigraph = NodeGraph::new(
			vec![
				NodeLayout::new((2, 3)),
				NodeLayout::new((2, 3)),
				NodeLayout::new((2, 4)),
			],
			vec![
				Connection::new(0, 0, 1, 0),
				Connection::new(0, 0, 2, 0),
				Connection::new(1, 0, 2, 1),
			],
			ea_zone.width as usize,
			ea_zone.height as usize,
		);
		minigraph.calculate();
		f.render_stateful_widget(minigraph, ea_zone, &mut ());
	}
	f.render_stateful_widget(graph, space, &mut ());
}
