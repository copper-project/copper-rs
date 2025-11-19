use ratatui::{
	style::Style,
	widgets::{Block, BorderType, Borders},
};

/// Render information for a single node
#[derive(Debug)]
pub struct NodeLayout<'a> {
	// minimum size of contents (TODO: doc: including borders?)
	pub size: (u16, u16),
	border_type: BorderType,
	title: &'a str,
	border_style: Style,
	//	in_ports: Vec<PortLayout>,
	//	out_ports: Vec<PortLayout>,
}

impl<'a> NodeLayout<'a> {
	pub fn new(size: (u16, u16)) -> Self {
		Self {
			size,
			border_type: BorderType::Plain,
			title: "",
			border_style: Style::default(),
		}
	}

	pub fn with_title(mut self, title: &'a str) -> Self {
		self.title = title;
		self
	}

	pub fn title(&self) -> &str {
		self.title
	}

	pub fn with_border_type(mut self, border: BorderType) -> Self {
		self.border_type = border;
		self
	}

	pub fn border_type(&self) -> BorderType {
		self.border_type
	}

	pub fn with_border_style(mut self, style: Style) -> Self {
		self.border_style = style;
		self
	}

	pub fn block(&self) -> Block<'_> {
		Block::default()
			.borders(Borders::ALL)
			.border_type(self.border_type)
			.border_style(self.border_style)
			.title(self.title)
	}
}
