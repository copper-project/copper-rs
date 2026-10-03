use std::env;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use eframe::egui::{self, Vec2};
use zed_sdk::{
    Camera, CameraInformation, DepthMode, Mat, MatView, OpenOptions, Resolution, ResolutionPreset,
    Rgba8, RuntimeParameters,
};

const WINDOW_SCALE: f32 = 0.8;
const MIN_WINDOW_WIDTH: f32 = 960.0;
const MIN_WINDOW_HEIGHT: f32 = 720.0;
const FRAME_INTERVAL: Duration = Duration::from_millis(16);
const RETRY_INTERVAL: Duration = Duration::from_millis(100);
const SCAN_STRIDE: usize = 4;
const EDGE_MARGIN: usize = 8;
const OVERLAY_NEAR_PERCENTILE: f32 = 0.10;
const OVERLAY_FAR_PERCENTILE: f32 = 0.90;
const MAX_CLIPPED_DEPTH_FRACTION: f32 = 0.98;
const MIN_OVERLAY_VALID_SAMPLES: usize = 512;
const MIN_OVERLAY_SPAN_M: f32 = 0.35;
const MAX_OVERLAY_RANGE_SHRINK_RATIO: f32 = 0.25;

fn main() -> ExitCode {
    let depth_mode = match env::args().nth(1) {
        Some(value) => match value.parse::<DepthMode>() {
            Ok(mode) => mode,
            Err(_) => {
                eprintln!("unsupported depth mode: {value}");
                return ExitCode::from(2);
            }
        },
        None => DepthMode::Performance,
    };

    let options = OpenOptions::default()
        .camera_device_id(0)
        .resolution(ResolutionPreset::Hd720)
        .fps(30)
        .depth_mode(depth_mode)
        .open_timeout(Duration::from_secs(20));
    let depth_ceiling_m = options.depth_maximum_distance_m_value();

    let camera = match Camera::open(options) {
        Ok(camera) => camera,
        Err(err) => {
            eprintln!("open() failed: {err}");
            return ExitCode::from(1);
        }
    };

    let info = match camera.info() {
        Ok(info) => Some(info),
        Err(err) => {
            eprintln!("camera info unavailable: {err}");
            None
        }
    };
    let resolution = match camera.resolution() {
        Ok(resolution) => resolution,
        Err(err) => {
            eprintln!("{err}");
            return ExitCode::from(1);
        }
    };
    let runtime = RuntimeParameters::default()
        .enable_depth(depth_mode != DepthMode::None)
        .enable_fill_mode(true)
        .confidence_threshold(70)
        .texture_confidence_threshold(70);

    let app = match ZedVizDemoApp::new(
        camera,
        info,
        resolution,
        runtime,
        depth_mode,
        depth_ceiling_m,
    ) {
        Ok(app) => app,
        Err(err) => {
            eprintln!("failed to initialize demo: {err}");
            return ExitCode::from(1);
        }
    };

    let window_size = [
        (resolution.width() as f32 * WINDOW_SCALE).max(MIN_WINDOW_WIDTH),
        (resolution.height() as f32 * WINDOW_SCALE + 140.0).max(MIN_WINDOW_HEIGHT),
    ];
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("ZED Viz Demo")
            .with_inner_size(window_size),
        ..Default::default()
    };

    let mut app = Some(app);
    match eframe::run_native(
        "zed_viz_demo",
        native_options,
        Box::new(move |_cc| Ok(Box::new(app.take().expect("app already initialized")))),
    ) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("failed to start egui app: {err}");
            ExitCode::from(1)
        }
    }
}

struct ZedVizDemoApp {
    camera: Camera,
    info: Option<CameraInformation>,
    resolution: Resolution,
    depth_mode: DepthMode,
    runtime: RuntimeParameters,
    left_image: Mat<Rgba8>,
    depth_map: Mat<f32>,
    texture: Option<egui::TextureHandle>,
    display_rgba: Vec<u8>,
    overlay_alpha: f32,
    depth_ceiling_m: f32,
    last_frame_at: Option<Instant>,
    last_frame_interval: Option<Duration>,
    last_overlay_range: Option<DepthRange>,
    depth_values: Vec<f32>,
}

impl ZedVizDemoApp {
    fn new(
        camera: Camera,
        info: Option<CameraInformation>,
        resolution: Resolution,
        runtime: RuntimeParameters,
        depth_mode: DepthMode,
        depth_ceiling_m: f32,
    ) -> zed_sdk::Result<Self> {
        let pixel_count = resolution.width() as usize * resolution.height() as usize;
        Ok(Self {
            camera,
            info,
            resolution,
            depth_mode,
            runtime,
            left_image: Mat::new_cpu(resolution)?,
            depth_map: Mat::new_cpu(resolution)?,
            texture: None,
            display_rgba: vec![0; pixel_count * 4],
            overlay_alpha: 0.55,
            depth_ceiling_m,
            last_frame_at: None,
            last_frame_interval: None,
            last_overlay_range: None,
            depth_values: Vec::with_capacity(pixel_count / (SCAN_STRIDE * SCAN_STRIDE)),
        })
    }

    fn update_frame(&mut self, ctx: &egui::Context) {
        if self.camera.grab(&self.runtime).is_err() {
            ctx.request_repaint_after(RETRY_INTERVAL);
            return;
        }

        if self.camera.retrieve_left(&mut self.left_image).is_err() {
            ctx.request_repaint_after(RETRY_INTERVAL);
            return;
        }

        if self.camera.retrieve_depth(&mut self.depth_map).is_err() {
            ctx.request_repaint_after(RETRY_INTERVAL);
            return;
        }

        let now = Instant::now();
        {
            let left_view = match self.left_image.view() {
                Ok(view) => view,
                Err(_) => {
                    ctx.request_repaint_after(RETRY_INTERVAL);
                    return;
                }
            };
            let depth_view = match self.depth_map.view() {
                Ok(view) => view,
                Err(_) => {
                    ctx.request_repaint_after(RETRY_INTERVAL);
                    return;
                }
            };

            let overlay_range = depth_overlay_range(
                &depth_view,
                self.depth_ceiling_m,
                self.last_overlay_range,
                &mut self.depth_values,
            );
            if let Some(overlay_range) = overlay_range {
                self.last_overlay_range = Some(overlay_range);
            }

            compose_overlay(
                &mut self.display_rgba,
                &left_view,
                &depth_view,
                overlay_range,
                self.overlay_alpha,
            );
        }

        let color_image = egui::ColorImage::from_rgba_unmultiplied(
            [
                self.resolution.width() as usize,
                self.resolution.height() as usize,
            ],
            &self.display_rgba,
        );
        if let Some(texture) = &mut self.texture {
            texture.set(color_image, egui::TextureOptions::LINEAR);
        } else {
            self.texture = Some(ctx.load_texture(
                "zed_viz_demo_frame",
                color_image,
                egui::TextureOptions::LINEAR,
            ));
        }

        self.last_frame_interval = self.last_frame_at.map(|prev| now.duration_since(prev));
        self.last_frame_at = Some(now);
        ctx.request_repaint_after(FRAME_INTERVAL);
    }

    fn draw_header(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("zed_viz_demo_header").show(ctx, |ui| {
            ui.horizontal(|ui| {
                if let Some(info) = self.info {
                    ui.label(format!("serial {}", info.serial_number));
                    ui.separator();
                    ui.label(format!(
                        "{}x{} @ {:.1} fps",
                        info.resolution.width(),
                        info.resolution.height(),
                        info.fps
                    ));
                    ui.separator();
                } else {
                    ui.label(format!(
                        "{}x{}",
                        self.resolution.width(),
                        self.resolution.height()
                    ));
                    ui.separator();
                }

                ui.label(format!("depth mode {}", self.depth_mode));
                ui.separator();
                ui.label(format!("camera fps {:.1}", self.camera.current_fps()));

                if let Some(interval) = self.last_frame_interval {
                    ui.separator();
                    ui.label(format!(
                        "ui fps {:.1}",
                        1.0 / interval.as_secs_f32().max(0.000_1)
                    ));
                }

                ui.separator();
                ui.add(
                    egui::Slider::new(&mut self.overlay_alpha, 0.0..=1.0)
                        .text("depth overlay")
                        .clamping(egui::SliderClamping::Always),
                );
            });
        });
    }

    fn draw_image(&self, ui: &mut egui::Ui) {
        let Some(texture) = self.texture.as_ref() else {
            ui.centered_and_justified(|ui| {
                ui.label("Waiting for frames...");
            });
            return;
        };

        let available = ui.available_size();
        let full_size = Vec2::new(
            self.resolution.width() as f32,
            self.resolution.height() as f32,
        );
        let scale = (available.x / full_size.x)
            .min(available.y / full_size.y)
            .max(0.1);
        let display_size = full_size * scale;
        ui.add(egui::Image::new(texture).fit_to_exact_size(display_size));
    }
}

impl eframe::App for ZedVizDemoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.update_frame(ctx);
        self.draw_header(ctx);

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.centered_and_justified(|ui| self.draw_image(ui));
        });
    }
}

#[derive(Clone, Copy, Debug)]
struct DepthRange {
    min_depth_m: f32,
    max_depth_m: f32,
}

impl DepthRange {
    fn span_m(self) -> f32 {
        self.max_depth_m - self.min_depth_m
    }
}

fn percentile_index(len: usize, percentile: f32) -> usize {
    ((len.saturating_sub(1)) as f32 * percentile.clamp(0.0, 1.0)).round() as usize
}

fn depth_overlay_range(
    depth_view: &MatView<'_, f32>,
    depth_ceiling_m: f32,
    fallback: Option<DepthRange>,
    depth_values: &mut Vec<f32>,
) -> Option<DepthRange> {
    let width = depth_view.width();
    let height = depth_view.height();
    if width <= EDGE_MARGIN * 2 || height <= EDGE_MARGIN * 2 {
        return fallback;
    }

    depth_values.clear();
    let max_depth_m = depth_ceiling_m * MAX_CLIPPED_DEPTH_FRACTION;

    for y in (EDGE_MARGIN..height - EDGE_MARGIN).step_by(SCAN_STRIDE) {
        let row = depth_view.row(y)?;
        for x in (EDGE_MARGIN..width - EDGE_MARGIN).step_by(SCAN_STRIDE) {
            let depth_m = row[x];
            if !depth_m.is_finite() || depth_m <= 0.0 || depth_m >= max_depth_m {
                continue;
            }

            depth_values.push(depth_m);
        }
    }

    if depth_values.len() < MIN_OVERLAY_VALID_SAMPLES {
        return fallback;
    }

    depth_values.sort_unstable_by(f32::total_cmp);
    let near_index = percentile_index(depth_values.len(), OVERLAY_NEAR_PERCENTILE);
    let far_index = percentile_index(depth_values.len(), OVERLAY_FAR_PERCENTILE);
    let min_depth_m = depth_values[near_index];
    let max_depth_m = depth_values[far_index].max(min_depth_m + 0.1);
    let range = DepthRange {
        min_depth_m,
        max_depth_m,
    };

    if range.span_m() < MIN_OVERLAY_SPAN_M {
        return fallback.or(Some(range));
    }

    if let Some(previous) = fallback
        && range.span_m() < previous.span_m() * MAX_OVERLAY_RANGE_SHRINK_RATIO
    {
        return Some(previous);
    }

    Some(range)
}

fn compose_overlay(
    output: &mut [u8],
    left_view: &MatView<'_, Rgba8>,
    depth_view: &MatView<'_, f32>,
    depth_range: Option<DepthRange>,
    overlay_alpha: f32,
) {
    let alpha = overlay_alpha.clamp(0.0, 1.0);
    let (min_depth, max_depth) = depth_range
        .map(|range| (range.min_depth_m, range.max_depth_m))
        .unwrap_or((0.0, 1.0));
    let span = (max_depth - min_depth).max(0.000_1);

    for y in 0..left_view.height() {
        let image_row = left_view
            .row(y)
            .expect("image row should be available while composing");
        let depth_row = depth_view
            .row(y)
            .expect("depth row should be available while composing");
        let out_row = &mut output[y * left_view.width() * 4..(y + 1) * left_view.width() * 4];

        for (x, (rgba, depth_m)) in image_row.iter().zip(depth_row.iter().copied()).enumerate() {
            let out = &mut out_row[x * 4..x * 4 + 4];
            let [r, g, b] = if depth_m.is_finite() && depth_m > 0.0 && depth_range.is_some() {
                let heat = depth_heat_color(depth_m, min_depth, span);
                blend_rgb([rgba.r, rgba.g, rgba.b], heat, alpha)
            } else {
                [rgba.r, rgba.g, rgba.b]
            };

            out[0] = r;
            out[1] = g;
            out[2] = b;
            out[3] = 255;
        }
    }
}

fn depth_heat_color(depth_m: f32, min_depth: f32, span: f32) -> [u8; 3] {
    let t = ((depth_m - min_depth) / span).clamp(0.0, 1.0);
    if t < 0.5 {
        lerp_rgb([255, 72, 24], [255, 214, 10], t * 2.0)
    } else {
        lerp_rgb([255, 214, 10], [48, 164, 255], (t - 0.5) * 2.0)
    }
}

fn lerp_rgb(a: [u8; 3], b: [u8; 3], t: f32) -> [u8; 3] {
    [
        lerp_channel(a[0], b[0], t),
        lerp_channel(a[1], b[1], t),
        lerp_channel(a[2], b[2], t),
    ]
}

fn lerp_channel(a: u8, b: u8, t: f32) -> u8 {
    (a as f32 + (b as f32 - a as f32) * t.clamp(0.0, 1.0)).round() as u8
}

fn blend_rgb(base: [u8; 3], overlay: [u8; 3], alpha: f32) -> [u8; 3] {
    [
        lerp_channel(base[0], overlay[0], alpha),
        lerp_channel(base[1], overlay[1], alpha),
        lerp_channel(base[2], overlay[2], alpha),
    ]
}
