// DEAD_CODE: This module is mostly used for debugging, we can afford having unused consts and helpers lying around
#![allow(dead_code)]

use ratatui::style::Color;
use ratatui::text::Text;

pub(crate) const BACKGROUND: Color = BLACK;
pub(crate) const FOREGROUND: Color = GRAY;

pub(crate) const BLACK: Color = Color::Rgb(0, 0, 0);
pub(crate) const BLUE: Color = Color::Rgb(13, 115, 204);
pub(crate) const CYAN: Color = Color::Rgb(13, 205, 205);
pub(crate) const DARK_GRAY: Color = Color::Rgb(118, 118, 118);
pub(crate) const GRAY: Color = Color::Rgb(221, 221, 221);
pub(crate) const GREEN: Color = Color::Rgb(25, 203, 0);
pub(crate) const LIGHT_BLUE: Color = Color::Rgb(26, 143, 255);
pub(crate) const LIGHT_CYAN: Color = Color::Rgb(20, 255, 255);
pub(crate) const LIGHT_GREEN: Color = Color::Rgb(35, 253, 0);
pub(crate) const LIGHT_MAGENTA: Color = Color::Rgb(253, 40, 255);
pub(crate) const LIGHT_RED: Color = Color::Rgb(242, 32, 31);
pub(crate) const LIGHT_YELLOW: Color = Color::Rgb(255, 253, 0);
pub(crate) const MAGENTA: Color = Color::Rgb(203, 30, 209);
pub(crate) const RED: Color = Color::Rgb(204, 4, 3);
pub(crate) const WHITE: Color = Color::Rgb(255, 255, 255);
pub(crate) const YELLOW: Color = Color::Rgb(206, 203, 0);

pub(crate) fn normalize_text_colors(text: &mut Text<'_>, fallback_fg: Color, fallback_bg: Color) {
    for line in &mut text.lines {
        for span in &mut line.spans {
            if let Some(fg) = span.style.fg {
                span.style.fg = Some(resolve_color(fg, fallback_fg, fallback_bg, true));
            }
            if let Some(bg) = span.style.bg {
                span.style.bg = Some(resolve_color(bg, fallback_fg, fallback_bg, false));
            }
        }
    }
}

fn indexed_color(index: u8) -> Color {
    match index {
        0 => BLACK,
        1 => RED,
        2 => GREEN,
        3 => YELLOW,
        4 => BLUE,
        5 => MAGENTA,
        6 => CYAN,
        7 => GRAY,
        8 => DARK_GRAY,
        9 => LIGHT_RED,
        10 => LIGHT_GREEN,
        11 => LIGHT_YELLOW,
        12 => LIGHT_BLUE,
        13 => LIGHT_MAGENTA,
        14 => LIGHT_CYAN,
        15 => WHITE,
        16..=231 => {
            let idx = index - 16;
            let r = idx / 36;
            let g = (idx % 36) / 6;
            let b = idx % 6;
            Color::Rgb(cube_level(r), cube_level(g), cube_level(b))
        }
        232..=255 => {
            let level = 8 + (index - 232) * 10;
            Color::Rgb(level, level, level)
        }
    }
}

fn resolve_color(color: Color, fallback_fg: Color, fallback_bg: Color, is_fg: bool) -> Color {
    match color {
        Color::Reset => {
            if is_fg {
                fallback_fg
            } else {
                fallback_bg
            }
        }
        Color::Black => BLACK,
        Color::Red => RED,
        Color::Green => GREEN,
        Color::Yellow => YELLOW,
        Color::Blue => BLUE,
        Color::Magenta => MAGENTA,
        Color::Cyan => CYAN,
        Color::Gray => GRAY,
        Color::DarkGray => DARK_GRAY,
        Color::LightRed => LIGHT_RED,
        Color::LightGreen => LIGHT_GREEN,
        Color::LightYellow => LIGHT_YELLOW,
        Color::LightBlue => LIGHT_BLUE,
        Color::LightMagenta => LIGHT_MAGENTA,
        Color::LightCyan => LIGHT_CYAN,
        Color::White => WHITE,
        Color::Indexed(index) => indexed_color(index),
        Color::Rgb(r, g, b) => Color::Rgb(r, g, b),
    }
}

fn cube_level(component: u8) -> u8 {
    match component {
        0 => 0,
        _ => 55 + component * 40,
    }
}
