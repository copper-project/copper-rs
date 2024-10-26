use std::io::Write;
use std::process::Command;

use syntect::easy::HighlightLines;
use syntect::highlighting::{Color, Style, Theme, ThemeSet};
use syntect::parsing::SyntaxSet;
use syntect::util::{as_24_bit_terminal_escaped, LinesWithEndings};

/// A utility method to ease up the debugging of the macro generated code by formatting it with rustfmt.
#[allow(dead_code)]
pub(crate) fn rustfmt_generated_code(code: String) -> String {
    let mut rustfmt = Command::new("rustfmt")
        .arg("--emit")
        .arg("stdout")
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .spawn()
        .expect("Failed to spawn rustfmt");

    {
        let stdin = rustfmt.stdin.as_mut().expect("Failed to open stdin");
        stdin
            .write_all(code.as_bytes())
            .expect("Failed to write to stdin");
    }

    let output = rustfmt.wait_with_output().expect("Failed to read stdout");
    String::from_utf8(output.stdout).expect("Output was not valid UTF-8")
}

#[allow(dead_code)]
fn create_black_theme() -> Theme {
    let mut theme = ThemeSet::load_defaults().themes["base16-ocean.dark"].clone();
    theme.settings.background = Some(Color {
        r: 0,
        g: 0,
        b: 0,
        a: 255,
    });
    theme
}

/// A utility method to ease up the debugging of the macro generated code by highlighting it.
#[allow(dead_code)]
pub(crate) fn highlight_rust_code(code: String) -> String {
    let ps = SyntaxSet::load_defaults_newlines();
    let syntax = ps.find_syntax_by_extension("rs").unwrap();
    let theme = create_black_theme();
    let mut h = HighlightLines::new(syntax, &theme);

    let mut highlighted_code = String::new();

    for line in LinesWithEndings::from(&code) {
        let ranges: Vec<(Style, &str)> = h.highlight_line(line, &ps).unwrap();
        let escaped = as_24_bit_terminal_escaped(&ranges[..], true);
        highlighted_code.push_str(&escaped);
    }

    highlighted_code
}
