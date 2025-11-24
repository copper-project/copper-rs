use std::hint::black_box;
use criterion::{criterion_group, criterion_main, Criterion};

use tui_nodes::*;
use ratatui::{
	widgets::BorderType,
	style::{Color, Style},
};

fn tiny(c: &mut Criterion) {
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
		54,
		10,
	);

	c.bench_function("calculate", |b| b.iter(|| black_box(&mut graph).calculate()));
}

fn basic(c: &mut Criterion) {

	let mut graph = NodeGraph::new(
		vec![
			NodeLayout::new((40, 10))
				.with_title("a|b|c")
				.with_border_type(BorderType::Thick),
			NodeLayout::new((40, 10))
				.with_title("b|c")
				.with_border_type(BorderType::Thick),
			NodeLayout::new((40, 10))
				.with_title("c")
				.with_border_type(BorderType::Rounded),
			NodeLayout::new((40, 10))
				.with_title("d>c")
				.with_border_type(BorderType::Thick),
			NodeLayout::new((40, 10))
				.with_title("e|d")
				.with_border_type(BorderType::Double),
			NodeLayout::new((30, 5))
				.with_title("f>(b,e)")
				.with_border_type(BorderType::Thick),
			NodeLayout::new((30, 5))
				.with_title("g|(a,f)")
				.with_border_type(BorderType::Double),
		],
		vec![
			Connection::new(0, 0, 1, 0).with_line_type(LineType::Double), // a | b
			Connection::new(1, 0, 2, 0).with_line_type(LineType::Thick),  // b | c
			Connection::new(3, 0, 2, 1).with_line_type(LineType::Double), // d > c
			Connection::new(4, 0, 3, 0),                                  // e | d
			Connection::new(4, 0, 0, 1),                                  // e | d
			Connection::new(5, 0, 1, 1),                                  // f > b
			Connection::new(5, 0, 4, 6),                                  // f > e
			Connection::new(6, 0, 0, 0).with_line_type(LineType::Double), // g | a
			Connection::new(6, 0, 5, 0).with_line_type(LineType::Double), // g | f
		],
		250,
		100,
	);

	let mut g = c.benchmark_group("basic");
	g.sample_size(10);
	g.bench_function("calculate", |b| b.iter(|| black_box(&mut graph).calculate()));
}

criterion_group!(benches, tiny, basic);
criterion_main!(benches);
