//! # Ratatui Software Backend
//!
//! A collection of software rendering backends for [Ratatui] that turn
//! terminal UIs into RGB pixmaps. Perfect for embedding Ratatui into game engines, GUIs, or wherever else.
//!
//! [Ratatui]: https://github.com/ratatui/ratatui
//!
//! ## Features
//!
//! - **Multiple Font Backends**: Choose from various font rendering technologies
//! - **Flexible Output**: Get raw RGB data or RGBA with color-to-alpha support
//! - **Real-time Updates**: Efficient redrawing, perfect for video games
//! - **Ratatui Compatible**: Full implementation of Ratatui's `Backend` trait
//! - **Bevy Examples**: Show you how to get started with the great game engine, also check out [bevy_ratatui]
//!
//!  [bevy_ratatui]: https://github.com/cxreiff/bevy_ratatui
//! [embedded-graphics]: https://github.com/embedded-graphics/embedded-graphics
//!  [cosmic-text]: https://github.com/pop-os/cosmic-text
//! [bdf-parser]: https://github.com/embedded-graphics/bdf
//! [unicodefonts]: https://github.com/j-g00da/embedded-graphics-unicodefonts
//! [embedded-ttf]: https://github.com/peckpeck/embedded-ttf
//!
//! ## Feature Flags
//!
//! - **`unicodefonts`**: Enables embedded-graphics-[unicodefonts] (automatically activates [embedded-graphics], is the default feature)
//! - **`bdf-parser`**: Enables [`Bdf`] backend for bitmap fonts ([bdf-parser])
//! - **`embedded-ttf`**: Enables [`EmbeddedTTF`] backend for TrueType fonts (automatically activates [embedded-graphics]) ([embedded-ttf])
//! - **`cosmic-text`**: Enables [`CosmicText`] backend for advanced text shaping ([cosmic-text])
//! - **`embedded-graphics`**: Enables [`EmbeddedGraphics`] backend
//!
//! ## Performance
//!
//! Generally faster than running ratatui in a terminal with crossterm. Expect hundreds of fps on a normal workload.
//!
//! ## Available Backends
//!
//! | Backend | Feature | Description |
//! |---------|---------|-------------|
//! | [`EmbeddedGraphics`] | `embedded-graphics` | Uses embedded-graphics font atlases |
//! | [`EmbeddedTTF`] | `embedded-ttf` | TrueType font rendering via RustType |
//! | [`Bdf`] | `bdf-parser` | Bitmap Distribution Format fonts |
//! | [`CosmicText`] | `cosmic-text` | Advanced text shaping and layout |
//!
//! ## Quick Start
//!
//!
//! ```bash
//! cargo add soft_ratatui
//! cargo add ratatui
//! ```
//!
//! ### Minimal Example
//!
//! ```rust
//! use soft_ratatui::embedded_graphics_unicodefonts::{
//!     mono_8x13_atlas, mono_8x13_bold_atlas, mono_8x13_italic_atlas,
//! };
//! use ratatui::Terminal;
//! use ratatui::widgets::{Block, Borders, Paragraph, Wrap};
//! use soft_ratatui::{EmbeddedGraphics, SoftBackend};
//!
//! fn main() {
//!     let font_regular = mono_8x13_atlas();
//!     let font_italic = mono_8x13_italic_atlas();
//!     let font_bold = mono_8x13_bold_atlas();
//!     let backend = SoftBackend::<EmbeddedGraphics>::new(
//!         100,
//!         50,
//!         font_regular,
//!         Some(font_bold),
//!         Some(font_italic),
//!     );
//!     let mut terminal = Terminal::new(backend).unwrap();
//!     terminal.clear();
//!
//!     terminal.draw(|frame| {
//!         let area = frame.area();
//!         let textik = format!("Hello soft! The window area is {}", area);
//!         frame.render_widget(
//!             Paragraph::new(textik)
//!                 .block(Block::new().title("Ratatui").borders(Borders::ALL))
//!                 .wrap(Wrap { trim: false }),
//!             area,
//!         );
//!     });
//! }
//! ```
//!
//! ## Usage Patterns
//!
//! ### Getting Rendered Output
//!
//! ```rust,no_run
//! # use soft_ratatui::{SoftBackend, EmbeddedGraphics};
//! # let backend = SoftBackend::<EmbeddedGraphics>::new(80, 24, todo!(), None, None);
//!
//! // Get raw RGB data (3 bytes per pixel: R, G, B)
//! let rgb_data = backend.get_pixmap_data();
//!
//! // Get RGBA data (4 bytes per pixel: R, G, B, A)
//! let rgba_data = backend.get_pixmap_data_as_rgba();
//!
//! // Get RGBA data with one of the colors set to fully transparent
//! let rgba_data = backend.rgb_pixmap.to_rgba_with_color_as_transparent((255,0,255));
//!
//! // Get dimensions
//! let width = backend.get_pixmap_width();
//! let height = backend.get_pixmap_height();
//! ```
//!
//! ## Examples
//!
//! See the `examples/` directory for complete working examples with different
//! backends and font configurations.
//!
//! ## License
//!
//! This project is licensed under the MIT and Apache2.0 license.

pub use pixmap::RgbPixmap;
mod soft_backend;
pub use soft_backend::{RasterBackend, SoftBackend};
mod colors;
mod pixmap;

#[cfg(feature = "embedded-graphics")]
mod embedded_backend;
#[cfg(feature = "embedded-graphics")]
pub use embedded_backend::EmbeddedGraphics;

#[cfg(feature = "cosmic-text")]
pub use cosmic_backend::CosmicText;
#[cfg(feature = "unicodefonts")]
pub use embedded_graphics_unicodefonts;
#[cfg(feature = "embedded-ttf")]
pub use embedded_ttf;
#[cfg(feature = "embedded-ttf")]
pub use rusttype;
#[cfg(feature = "embedded-ttf")]
mod embedded_ttf_backend;
#[cfg(feature = "embedded-ttf")]
pub use embedded_ttf_backend::EmbeddedTTF;
#[cfg(feature = "cosmic-text")]
mod cosmic_backend;
#[cfg(feature = "bdf-parser")]
pub use bdf_backend::Bdf;
#[cfg(feature = "bdf-parser")]
mod bdf_backend;
