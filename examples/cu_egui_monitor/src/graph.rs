//! Background SVG rasterization and a bounded cache of GPU tiles.
use cu29::prelude::CuError;
use cu29::prelude::CuResult;
use eframe::egui;
use resvg::tiny_skia;
use resvg::usvg;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::Condvar;
use std::sync::Mutex;
use std::sync::mpsc;
use std::thread;
use std::time::Instant;

const TILE_SIDE: u32 = 512;
// 64 RGBA tiles consume at most 64 MiB, plus the small overview texture.
const CACHE_TILES: usize = 64;
const RESULT_QUEUE: usize = 4;

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
struct TileKey {
    level: i32,
    x: u32,
    y: u32,
}
impl TileKey {
    fn scale(self) -> f32 {
        2.0_f32.powf(self.level as f32 / 2.0)
    }
    fn rect(self, zoom: f32) -> egui::Rect {
        let side = TILE_SIDE as f32 * zoom / self.scale();
        egui::Rect::from_min_size(
            egui::pos2(self.x as f32 * side, self.y as f32 * side),
            egui::Vec2::splat(side),
        )
    }
}

#[derive(Default)]
struct Requests {
    tiles: Vec<TileKey>,
    stop: bool,
}
struct Rendered {
    key: Option<TileKey>,
    image: egui::ColorImage,
    millis: f64,
}
struct CachedTile {
    texture: egui::TextureHandle,
    touched: u64,
}

pub(crate) struct GraphView {
    size: egui::Vec2,
    zoom: f32,
    displayed_zoom: f32,
    scroll_offset: egui::Vec2,
    viewport: Option<egui::Rect>,
    fit: bool,
    show_tiles: bool,
    visible_ready: usize,
    visible_missing: usize,
    level: i32,
    previous_level: i32,
    epoch: u64,
    cache: HashMap<TileKey, CachedTile>,
    overview: Option<egui::TextureHandle>,
    requests: Arc<(Mutex<Requests>, Condvar)>,
    results: mpsc::Receiver<Rendered>,
    worker: Option<thread::JoinHandle<CuResult<()>>>,
    millis: f64,
    error: Option<String>,
}

impl GraphView {
    pub(crate) fn new(svg: &str, ctx: egui::Context) -> CuResult<Self> {
        let mut options = usvg::Options::default();
        options.fontdb_mut().load_system_fonts();
        let tree = usvg::Tree::from_str(svg, &options)
            .map_err(|error| CuError::new_with_cause("Cannot parse graph SVG", error))?;
        let size = egui::vec2(tree.size().width(), tree.size().height());
        let requests = Arc::new((Mutex::new(Requests::default()), Condvar::new()));
        let (sender, results) = mpsc::sync_channel(RESULT_QUEUE);
        let worker_requests = Arc::clone(&requests);
        let worker = thread::Builder::new()
            .name("graph-raster".into())
            .spawn(move || {
                let result = render_loop(tree, worker_requests, sender, ctx.clone());
                ctx.request_repaint();
                result
            })
            .map_err(|error| CuError::new_with_cause("Cannot start graph renderer", error))?;
        Ok(Self {
            size,
            zoom: 1.0,
            displayed_zoom: 1.0,
            scroll_offset: egui::Vec2::ZERO,
            viewport: None,
            fit: false,
            show_tiles: false,
            visible_ready: 0,
            visible_missing: 0,
            level: 0,
            previous_level: 0,
            epoch: 0,
            cache: HashMap::new(),
            overview: None,
            requests,
            results,
            worker: Some(worker),
            millis: 0.0,
            error: None,
        })
    }

    pub(crate) fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Fit").clicked() {
                self.fit = true;
            }
            if ui.button("100%").clicked() {
                self.zoom = 1.0;
                self.fit = false;
            }
            if ui.button("−").clicked() {
                self.zoom /= 1.25;
                self.fit = false;
            }
            if ui.button("+").clicked() {
                self.zoom *= 1.25;
                self.fit = false;
            }
            if ui
                .add(
                    egui::Slider::new(&mut self.zoom, 0.025..=8.0)
                        .logarithmic(true)
                        .text("Zoom"),
                )
                .changed()
            {
                self.fit = false;
            }
            ui.label(format!("{:.0}%", self.zoom * 100.0));
            ui.checkbox(&mut self.show_tiles, "Tile boundaries");
            ui.label(format!(
                "{} ready / {} waiting · {} cached · last tile {:.1} ms",
                self.visible_ready,
                self.visible_missing,
                self.cache.len(),
                self.millis
            ));
        });
        ui.label("Wheel up/down to zoom · click and drag to pan · pinch to zoom");
        if let Some(error) = &self.error {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
    }

    pub(crate) fn show(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.epoch = self.epoch.wrapping_add(1);
        // Bound uploads per UI frame; the worker's result queue is also bounded.
        for rendered in self.results.try_iter().take(2) {
            self.millis = rendered.millis;
            let texture =
                ctx.load_texture("graph-tile", rendered.image, egui::TextureOptions::LINEAR);
            if let Some(key) = rendered.key {
                if self.cache.len() >= CACHE_TILES
                    && let Some(oldest) = self
                        .cache
                        .iter()
                        .min_by_key(|(_, tile)| tile.touched)
                        .map(|(key, _)| *key)
                {
                    self.cache.remove(&oldest);
                }
                self.cache.insert(
                    key,
                    CachedTile {
                        texture,
                        touched: self.epoch,
                    },
                );
            } else {
                self.overview = Some(texture);
            }
            ctx.request_repaint();
        }
        if self
            .worker
            .as_ref()
            .is_some_and(|worker| worker.is_finished())
            && let Some(worker) = self.worker.take()
        {
            self.error = match worker.join() {
                Ok(Ok(())) => None,
                Ok(Err(error)) => Some(error.to_string()),
                Err(_) => Some("Graph rendering thread panicked".into()),
            };
        }
        if self.fit {
            let available =
                (ui.available_size() - egui::vec2(24.0, 24.0)).max(egui::Vec2::splat(1.0));
            self.zoom = (available.x / self.size.x)
                .min(available.y / self.size.y)
                .clamp(0.025, 8.0);
            self.fit = false;
            self.scroll_offset = egui::Vec2::ZERO;
            self.displayed_zoom = self.zoom;
        }
        // Consume wheel input only inside the graph. The telemetry panel scrolls normally.
        if let Some(viewport) = self.viewport {
            let pointer = ctx.input(|input| input.pointer.hover_pos());
            if pointer.is_some_and(|pointer| viewport.contains(pointer)) {
                let factor = ctx.input_mut(|input| {
                    let factor = (input.smooth_scroll_delta.y * 0.003).exp() * input.zoom_delta();
                    input.smooth_scroll_delta = egui::Vec2::ZERO;
                    factor
                });
                self.zoom *= factor;
            }
            self.zoom = self.zoom.clamp(0.025, 8.0);
            if self.zoom != self.displayed_zoom {
                let anchor = pointer
                    .filter(|pointer| viewport.contains(*pointer))
                    .unwrap_or(viewport.center())
                    - viewport.min;
                self.scroll_offset =
                    zoomed_offset(self.scroll_offset, anchor, self.displayed_zoom, self.zoom);
            }
        }
        self.zoom = self.zoom.clamp(0.025, 8.0);
        self.displayed_zoom = self.zoom;
        let level = ((self.zoom * ctx.pixels_per_point()).log2() * 2.0).round() as i32;
        if level != self.level {
            self.previous_level = self.level;
            self.level = level;
        }
        let output = egui::ScrollArea::both()
            .id_salt("graph")
            .scroll_offset(self.scroll_offset)
            .scroll_source(
                egui::scroll_area::ScrollSource::SCROLL_BAR | egui::scroll_area::ScrollSource::DRAG,
            )
            .auto_shrink([false, false])
            .scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysVisible)
            .show_viewport(ui, |ui, viewport| {
                let (canvas, response) =
                    ui.allocate_exact_size(self.size * self.zoom, egui::Sense::hover());
                response.on_hover_cursor(egui::CursorIcon::Grab);
                let visible = viewport.intersect(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    self.size * self.zoom,
                ));
                let uv = egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(1.0, 1.0));
                if let Some(overview) = &self.overview {
                    ui.painter()
                        .image(overview.id(), canvas, uv, egui::Color32::WHITE);
                } else {
                    ui.painter().text(
                        canvas.min + egui::vec2(16.0, 16.0),
                        egui::Align2::LEFT_TOP,
                        "Preparing graph…",
                        egui::FontId::proportional(16.0),
                        egui::Color32::WHITE,
                    );
                }
                // Scale the preceding zoom level immediately while sharper tiles arrive.
                for (key, tile) in &self.cache {
                    if key.level == self.previous_level
                        && key.level != self.level
                        && key.rect(self.zoom).intersects(visible)
                    {
                        ui.painter().image(
                            tile.texture.id(),
                            key.rect(self.zoom).translate(canvas.min.to_vec2()),
                            uv,
                            egui::Color32::WHITE,
                        );
                    }
                }
                let keys = visible_tiles(visible, self.zoom, self.level);
                let mut missing = Vec::new();
                self.visible_ready = 0;
                for key in keys {
                    let tile_ready = self.cache.contains_key(&key);
                    if let Some(tile) = self.cache.get_mut(&key) {
                        tile.touched = self.epoch;
                        self.visible_ready += 1;
                        ui.painter().image(
                            tile.texture.id(),
                            key.rect(self.zoom).translate(canvas.min.to_vec2()),
                            uv,
                            egui::Color32::WHITE,
                        );
                    } else {
                        missing.push(key);
                    }
                    if self.show_tiles {
                        let rect = key.rect(self.zoom).translate(canvas.min.to_vec2());
                        let color = if tile_ready {
                            egui::Color32::from_rgb(80, 180, 150)
                        } else {
                            egui::Color32::from_rgb(240, 180, 70)
                        };
                        ui.painter().rect_stroke(
                            rect,
                            0.0,
                            egui::Stroke::new(1.0, color),
                            egui::StrokeKind::Inside,
                        );
                        ui.painter().text(
                            rect.min + egui::vec2(4.0, 4.0),
                            egui::Align2::LEFT_TOP,
                            format!("{},{} · L{}", key.x, key.y, key.level),
                            egui::FontId::monospace(11.0),
                            color,
                        );
                    }
                }
                self.visible_missing = missing.len();
                // Replace the plan instead of queuing obsolete scroll positions or zooms.
                let (lock, ready) = &*self.requests;
                let mut requests = lock.lock().unwrap_or_else(|error| error.into_inner());
                requests.tiles = missing;
                ready.notify_one();
            });
        self.scroll_offset = output.state.clamped_offset();
        self.viewport = Some(output.inner_rect);
    }
}

impl Drop for GraphView {
    fn drop(&mut self) {
        let (lock, ready) = &*self.requests;
        lock.lock().unwrap_or_else(|error| error.into_inner()).stop = true;
        ready.notify_one();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

// Preserve the graph coordinate under the pointer when the zoom changes.
fn zoomed_offset(offset: egui::Vec2, anchor: egui::Vec2, before: f32, after: f32) -> egui::Vec2 {
    ((offset + anchor) * (after / before) - anchor).max(egui::Vec2::ZERO)
}

fn visible_tiles(visible: egui::Rect, zoom: f32, level: i32) -> Vec<TileKey> {
    if !visible.is_positive() {
        return Vec::new();
    }
    let side = TILE_SIDE as f32 * zoom / TileKey { level, x: 0, y: 0 }.scale();
    let mut keys = Vec::new();
    for y in (visible.min.y / side).floor() as u32..(visible.max.y / side).ceil() as u32 {
        for x in (visible.min.x / side).floor() as u32..(visible.max.x / side).ceil() as u32 {
            keys.push(TileKey { level, x, y });
        }
    }
    // The worker pops from the back: prioritize the center of the viewport.
    keys.sort_by(|a, b| {
        b.rect(zoom)
            .center()
            .distance_sq(visible.center())
            .total_cmp(&a.rect(zoom).center().distance_sq(visible.center()))
    });
    keys
}

fn rasterize(tree: &usvg::Tree, key: Option<TileKey>) -> CuResult<Rendered> {
    let start = Instant::now();
    let (width, height, transform) = if let Some(key) = key {
        (
            TILE_SIDE,
            TILE_SIDE,
            tiny_skia::Transform::from_row(
                key.scale(),
                0.0,
                0.0,
                key.scale(),
                -((key.x * TILE_SIDE) as f32),
                -((key.y * TILE_SIDE) as f32),
            ),
        )
    } else {
        let scale = (1600.0 / tree.size().width().max(tree.size().height())).min(1.0);
        (
            (tree.size().width() * scale).ceil() as u32,
            (tree.size().height() * scale).ceil() as u32,
            tiny_skia::Transform::from_scale(scale, scale),
        )
    };
    let mut pixmap = tiny_skia::Pixmap::new(width, height)
        .ok_or_else(|| CuError::from("Cannot allocate graph tile"))?;
    resvg::render(tree, transform, &mut pixmap.as_mut());
    Ok(Rendered {
        key,
        image: egui::ColorImage::from_rgba_premultiplied(
            [width as usize, height as usize],
            pixmap.data(),
        ),
        millis: start.elapsed().as_secs_f64() * 1000.0,
    })
}

fn render_loop(
    tree: usvg::Tree,
    requests: Arc<(Mutex<Requests>, Condvar)>,
    sender: mpsc::SyncSender<Rendered>,
    ctx: egui::Context,
) -> CuResult<()> {
    let _ = sender.try_send(rasterize(&tree, None)?);
    ctx.request_repaint();
    loop {
        let key = {
            let (lock, ready) = &*requests;
            let mut state = lock.lock().unwrap_or_else(|error| error.into_inner());
            while state.tiles.is_empty() && !state.stop {
                state = ready.wait(state).unwrap_or_else(|error| error.into_inner());
            }
            if state.stop {
                return Ok(());
            }
            // The wait condition above guarantees a tile unless shutdown was requested.
            state
                .tiles
                .pop()
                .ok_or_else(|| CuError::from("Missing graph tile request"))?
        };
        let rendered = rasterize(&tree, Some(key))?;
        // Drop obsolete duplicate requests submitted while this tile was rendering.
        requests
            .0
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .tiles
            .retain(|pending| *pending != key);
        // Never block shutdown or retain an unbounded queue of pixel buffers.
        if matches!(
            sender.try_send(rendered),
            Err(mpsc::TrySendError::Disconnected(_))
        ) {
            return Ok(());
        }
        ctx.request_repaint();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_coordinates_stay_fixed_while_scrolling() {
        let first = visible_tiles(
            egui::Rect::from_min_size(egui::pos2(1.0, 1.0), egui::vec2(100.0, 100.0)),
            1.0,
            0,
        );
        let second = visible_tiles(
            egui::Rect::from_min_size(egui::pos2(20.0, 20.0), egui::vec2(100.0, 100.0)),
            1.0,
            0,
        );
        assert_eq!(first, second);
        assert_eq!(
            first,
            vec![TileKey {
                level: 0,
                x: 0,
                y: 0
            }]
        );
    }

    #[test]
    fn tiles_cover_viewport_across_boundaries_and_zoom() {
        let rect = egui::Rect::from_min_max(egui::pos2(500.0, 500.0), egui::pos2(600.0, 600.0));
        let keys = visible_tiles(rect, 1.0, 0);
        assert_eq!(keys.len(), 4);
        assert!(keys.contains(&TileKey {
            level: 0,
            x: 1,
            y: 1
        }));
        assert_eq!(visible_tiles(rect, 2.0, 2).len(), 4);
    }

    #[test]
    fn tile_rasterization_translates_into_graph_coordinates() {
        let tree = usvg::Tree::from_str(r##"<svg xmlns="http://www.w3.org/2000/svg" width="1024" height="512"><rect x="512" width="512" height="512" fill="#ff0000"/></svg>"##, &usvg::Options::default()).unwrap();
        let tile = rasterize(
            &tree,
            Some(TileKey {
                level: 0,
                x: 1,
                y: 0,
            }),
        )
        .unwrap();
        assert_eq!(tile.image.pixels[0], egui::Color32::RED);
        assert_eq!(tile.image.size, [512, 512]);
    }

    #[test]
    fn zoom_keeps_the_pointer_over_the_same_graph_point() {
        let offset = egui::vec2(300.0, 400.0);
        let pointer = egui::vec2(100.0, 150.0);
        let zoomed = zoomed_offset(offset, pointer, 1.0, 2.0);
        assert_eq!((offset + pointer) / 1.0, (zoomed + pointer) / 2.0);
        assert_eq!(zoomed_offset(zoomed, pointer, 2.0, 1.0), offset);
    }

    #[test]
    fn worker_shuts_down_when_the_result_channel_is_full() {
        let tree = usvg::Tree::from_str(
            r#"<svg xmlns="http://www.w3.org/2000/svg" width="512" height="512"/>"#,
            &usvg::Options::default(),
        )
        .unwrap();
        let (sender, _receiver) = mpsc::sync_channel(1);
        sender.send(rasterize(&tree, None).unwrap()).unwrap();
        let requests = Arc::new((
            Mutex::new(Requests {
                tiles: Vec::new(),
                stop: true,
            }),
            Condvar::new(),
        ));
        let (finished, completion) = mpsc::channel();
        let worker = thread::spawn(move || {
            let result = render_loop(tree, requests, sender, egui::Context::default());
            let _ = finished.send(result);
        });
        assert!(
            completion
                .recv_timeout(std::time::Duration::from_secs(2))
                .unwrap()
                .is_ok()
        );
        worker.join().unwrap();
    }
}
