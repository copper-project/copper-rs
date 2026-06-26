#[cfg(not(target_arch = "wasm32"))]
use bevy::ecs::message::MessageReader;
#[cfg(not(target_arch = "wasm32"))]
use bevy::window::WindowCreated;
#[cfg(not(target_arch = "wasm32"))]
use bevy::winit::WINIT_WINDOWS;
#[cfg(not(target_arch = "wasm32"))]
use image::ImageFormat;
#[cfg(not(target_arch = "wasm32"))]
use std::sync::OnceLock;
#[cfg(not(target_arch = "wasm32"))]
use winit::window::Icon;

#[cfg(not(target_arch = "wasm32"))]
const COPPER_ICON_PNG: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../doc/static/cu29.png"
));
#[cfg(not(target_arch = "wasm32"))]
static COPPER_WINDOW_ICON: OnceLock<Option<Icon>> = OnceLock::new();

#[cfg(not(target_arch = "wasm32"))]
pub fn set_copper_window_icon(mut created_windows: MessageReader<WindowCreated>) {
    let Some(icon) = copper_window_icon() else {
        return;
    };

    for event in created_windows.read() {
        WINIT_WINDOWS.with_borrow(|winit_windows| {
            if let Some(window) = winit_windows.get_window(event.window) {
                window.set_window_icon(Some(icon.clone()));
            }
        });
    }
}

#[cfg(target_arch = "wasm32")]
pub fn set_copper_window_icon() {}

#[cfg(not(target_arch = "wasm32"))]
fn copper_window_icon() -> Option<Icon> {
    COPPER_WINDOW_ICON
        .get_or_init(load_copper_window_icon)
        .clone()
}

#[cfg(not(target_arch = "wasm32"))]
fn load_copper_window_icon() -> Option<Icon> {
    let image = image::load_from_memory_with_format(COPPER_ICON_PNG, ImageFormat::Png)
        .ok()?
        .into_rgba8();
    let (width, height) = image.dimensions();
    Icon::from_rgba(image.into_raw(), width, height).ok()
}
