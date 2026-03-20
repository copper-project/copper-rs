use crate::payloads::MandelbrotRow;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::prelude::*;
use rerun::{ChannelDatatype, ColorModel, Image, RecordingStream, RecordingStreamBuilder};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

const RGB_PIXEL_FORMAT: [u8; 4] = *b"RGB3";
const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

static FRAMES_EMITTED: AtomicU64 = AtomicU64::new(0);
static LAST_FRAME_DIGEST: AtomicU64 = AtomicU64::new(0);

/// Static benchmark parameters carried by the source and reused by the summary.
#[derive(Debug, Clone, Copy)]
pub struct BenchmarkSettings {
    /// Output image width in pixels.
    pub width: u32,
    /// Output image height in pixels.
    pub height: u32,
    /// Number of zoom frames to emit before stopping the runtime.
    pub frames: u32,
    /// Maximum Mandelbrot iterations per pixel.
    pub max_iter: u16,
    /// Center point of the zoom in the complex plane.
    pub center_x: f64,
    /// Center point of the zoom in the complex plane.
    pub center_y: f64,
    /// Initial horizontal span before applying per-frame zoom.
    pub initial_span_x: f64,
    /// Multiplicative zoom applied once per frame.
    pub zoom_ratio: f64,
}

impl BenchmarkSettings {
    pub fn from_component_config(config: &ComponentConfig) -> CuResult<Self> {
        Ok(Self {
            width: required_param::<u64>(config, "width")? as u32,
            height: required_param::<u64>(config, "height")? as u32,
            frames: required_param::<u64>(config, "frames")? as u32,
            max_iter: required_param::<u64>(config, "max_iter")? as u16,
            center_x: required_param::<f64>(config, "center_x")?,
            center_y: required_param::<f64>(config, "center_y")?,
            initial_span_x: required_param::<f64>(config, "initial_span_x")?,
            zoom_ratio: required_param::<f64>(config, "zoom_ratio")?,
        })
    }

    #[inline]
    pub fn total_rows(self) -> u64 {
        self.frames as u64 * self.height as u64
    }
}

#[derive(Debug, Clone, Copy)]
struct IterBandConfig {
    stage_id: u64,
    band_iters: u16,
    finalize: bool,
}

fn required_param<T>(config: &ComponentConfig, key: &str) -> CuResult<T>
where
    T: for<'a> TryFrom<&'a cu29::config::Value, Error = cu29::config::ConfigError>,
{
    config
        .get::<T>(key)
        .map_err(|err| CuError::from(format!("failed to read config key `{key}`: {err}")))?
        .ok_or_else(|| CuError::from(format!("missing required config key `{key}`")))
}

fn optional_param<T>(config: &ComponentConfig, key: &str, default: T) -> CuResult<T>
where
    T: Copy + for<'a> TryFrom<&'a cu29::config::Value, Error = cu29::config::ConfigError>,
{
    Ok(config
        .get::<T>(key)
        .map_err(|err| CuError::from(format!("failed to read config key `{key}`: {err}")))?
        .unwrap_or(default))
}

fn hash_bytes(bytes: &[u8]) -> u64 {
    bytes.iter().fold(FNV_OFFSET, |hash, byte| {
        (hash ^ (*byte as u64)).wrapping_mul(FNV_PRIME)
    })
}

fn colorize_escape(escape_iter: u16, max_iter: u16) -> [u8; 3] {
    if escape_iter == 0 || max_iter == 0 {
        return [0, 0, 0];
    }

    let t = escape_iter as f32 / max_iter as f32;
    let r = (9.0 * (1.0 - t) * t * t * t * 255.0).clamp(0.0, 255.0) as u8;
    let g = (15.0 * (1.0 - t) * (1.0 - t) * t * t * 255.0).clamp(0.0, 255.0) as u8;
    let b = (8.5 * (1.0 - t) * (1.0 - t) * (1.0 - t) * t * 255.0).clamp(0.0, 255.0) as u8;
    [r, g, b]
}

#[inline]
fn advance_escape_shadow(zr: f64, zi: f64, c_re: f64, row_im: f64) -> (f64, f64) {
    // Keep per-band work roughly fixed even after a pixel escaped so the
    // benchmark stresses Copper's scheduler instead of collapsing into a few
    // heavy early bands and many trivial late bands.
    let next_re = zr.mul_add(0.754_877_666_246_692_7, c_re) - zi * 0.569_840_290_998_053_2;
    let next_im = zi.mul_add(0.819_172_513_396_164_4, row_im) + zr * 0.137_631_299_231_935;
    (next_re, next_im)
}

fn request_runtime_stop() -> CuResult<()> {
    let rc = unsafe { libc::raise(libc::SIGINT) };
    if rc == 0 {
        Ok(())
    } else {
        Err(CuError::from(
            "failed to request runtime shutdown via SIGINT",
        ))
    }
}

pub fn reset_benchmark_summary() {
    FRAMES_EMITTED.store(0, Ordering::Release);
    LAST_FRAME_DIGEST.store(0, Ordering::Release);
}

pub fn frames_emitted() -> u64 {
    FRAMES_EMITTED.load(Ordering::Acquire)
}

pub fn last_frame_digest() -> u64 {
    LAST_FRAME_DIGEST.load(Ordering::Acquire)
}

fn source_status(settings: BenchmarkSettings, frame_index: u32, row_index: u32) -> String {
    format!(
        "src f{:02}/{:02} r{:04}",
        frame_index + 1,
        settings.frames,
        row_index + 1
    )
}

fn band_status(stage_id: u64, row: &MandelbrotRow) -> String {
    format!(
        "b{} f{:02} r{:04} i{:03}",
        stage_id,
        row.frame_index + 1,
        row.row_index + 1,
        row.completed_iters
    )
}

fn assembler_status(frame_index: u32, total_frames: u32, row_index: u32) -> String {
    format!(
        "asm f{:02}/{:02} r{:04}",
        frame_index + 1,
        total_frames,
        row_index + 1
    )
}

fn emitted_status(frame_index: u32, total_frames: u32) -> String {
    format!("emit f{:02}/{:02}", frame_index + 1, total_frames)
}

/// Source emitting one Mandelbrot row per CopperList.
///
/// It owns the reusable row-state pools so the compute stages can mutate the
/// same buffers across the pipeline without cloning the heavy arrays.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct MandelbrotRowSource {
    #[reflect(ignore)]
    settings: BenchmarkSettings,
    next_linear_index: u64,
    #[reflect(ignore)]
    z_re_pool: Arc<CuHostMemoryPool<Vec<f64>>>,
    #[reflect(ignore)]
    z_im_pool: Arc<CuHostMemoryPool<Vec<f64>>>,
    #[reflect(ignore)]
    escape_pool: Arc<CuHostMemoryPool<Vec<u16>>>,
    #[reflect(ignore)]
    pixels_pool: Arc<CuHostMemoryPool<Vec<u8>>>,
}

impl Freezable for MandelbrotRowSource {}

impl CuSrcTask for MandelbrotRowSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(MandelbrotRow);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("MandelbrotRowSource requires config"))?;
        let settings = BenchmarkSettings::from_component_config(config)?;
        let pool_slots = optional_param::<u64>(config, "pool_slots", 8)? as usize;
        let width = settings.width as usize;
        let rgb_len = width * 3;

        Ok(Self {
            settings,
            next_linear_index: 0,
            z_re_pool: CuHostMemoryPool::new("mandelbrot-z-re", pool_slots, || vec![0.0; width])?,
            z_im_pool: CuHostMemoryPool::new("mandelbrot-z-im", pool_slots, || vec![0.0; width])?,
            escape_pool: CuHostMemoryPool::new("mandelbrot-escape", pool_slots, || vec![0; width])?,
            pixels_pool: CuHostMemoryPool::new("mandelbrot-rgb", pool_slots, || vec![0; rgb_len])?,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        if self.next_linear_index >= self.settings.total_rows() {
            output.clear_payload();
            return Ok(());
        }

        let linear_index = self.next_linear_index;
        let frame_index = (linear_index / self.settings.height as u64) as u32;
        let row_index = (linear_index % self.settings.height as u64) as u32;
        let span_x =
            self.settings.initial_span_x * self.settings.zoom_ratio.powf(frame_index as f64);

        let z_re = self
            .z_re_pool
            .acquire()
            .ok_or_else(|| CuError::from("mandelbrot z_re pool exhausted"))?;
        let z_im = self
            .z_im_pool
            .acquire()
            .ok_or_else(|| CuError::from("mandelbrot z_im pool exhausted"))?;
        let escape_iter = self
            .escape_pool
            .acquire()
            .ok_or_else(|| CuError::from("mandelbrot escape pool exhausted"))?;
        let pixels_rgb = self
            .pixels_pool
            .acquire()
            .ok_or_else(|| CuError::from("mandelbrot rgb pool exhausted"))?;

        z_re.with_inner_mut(|inner| inner.fill(0.0));
        z_im.with_inner_mut(|inner| inner.fill(0.0));
        escape_iter.with_inner_mut(|inner| inner.fill(0));
        pixels_rgb.with_inner_mut(|inner| inner.fill(0));

        let row = MandelbrotRow {
            frame_index,
            row_index,
            width: self.settings.width,
            height: self.settings.height,
            center_x: self.settings.center_x,
            center_y: self.settings.center_y,
            span_x,
            max_iter: self.settings.max_iter,
            completed_iters: 0,
            z_re,
            z_im,
            escape_iter,
            pixels_rgb,
        };

        output.tov = Tov::Time(ctx.now());
        output.set_payload(row);
        output
            .metadata
            .set_status(source_status(self.settings, frame_index, row_index));

        self.next_linear_index += 1;
        if self.next_linear_index == self.settings.total_rows() {
            request_runtime_stop()?;
        }

        Ok(())
    }
}

/// One stateful Mandelbrot iteration band.
///
/// Each task is intentionally mutable and checks that rows arrive in strict
/// frame-major order, which makes any scheduler-induced state corruption fail
/// immediately instead of silently producing a pretty but invalid image.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct MandelbrotIterBand {
    #[reflect(ignore)]
    band: IterBandConfig,
    expected_linear_index: u64,
}

impl Freezable for MandelbrotIterBand {}

impl CuTask for MandelbrotIterBand {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MandelbrotRow);
    type Output<'m> = output_msg!(MandelbrotRow);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("MandelbrotIterBand requires config"))?;
        Ok(Self {
            band: IterBandConfig {
                stage_id: required_param::<u64>(config, "stage_id")?,
                band_iters: required_param::<u64>(config, "band_iters")? as u16,
                finalize: optional_param::<bool>(config, "finalize", false)?,
            },
            expected_linear_index: 0,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(payload) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let linear_index = payload.linear_index();
        if linear_index != self.expected_linear_index {
            return Err(CuError::from(format!(
                "stage {} observed out-of-order row: expected {}, got {}",
                self.band.stage_id, self.expected_linear_index, linear_index
            )));
        }

        let mut next = payload.clone();
        let target_iters = next
            .completed_iters
            .saturating_add(self.band.band_iters)
            .min(next.max_iter);
        let row_im = next.row_imag();
        let width = next.width as usize;
        let start_iter = next.completed_iters;

        next.z_re.with_inner_mut(|z_re| {
            next.z_im.with_inner_mut(|z_im| {
                next.escape_iter.with_inner_mut(|escape_iter| {
                    for x in 0..width {
                        let c_re = next.pixel_real(x);
                        let mut zr = z_re[x];
                        let mut zi = z_im[x];
                        let mut escaped = escape_iter[x] != 0;

                        for iter in start_iter..target_iters {
                            if escaped {
                                (zr, zi) = advance_escape_shadow(zr, zi, c_re, row_im);
                                continue;
                            }

                            let zr2 = zr * zr;
                            let zi2 = zi * zi;
                            if zr2 + zi2 > 4.0 {
                                escape_iter[x] = iter;
                                escaped = true;
                                (zr, zi) = advance_escape_shadow(zr, zi, c_re, row_im);
                                continue;
                            }

                            zi = 2.0 * zr * zi + row_im;
                            zr = zr2 - zi2 + c_re;
                        }

                        z_re[x] = zr;
                        z_im[x] = zi;
                    }
                });
            });
        });

        next.completed_iters = target_iters;

        if self.band.finalize {
            next.pixels_rgb.with_inner_mut(|pixels| {
                next.escape_iter.with_inner(|escape_iter| {
                    for (x, escape) in escape_iter.iter().copied().enumerate() {
                        let rgb = colorize_escape(escape, next.max_iter);
                        let base = x * 3;
                        pixels[base] = rgb[0];
                        pixels[base + 1] = rgb[1];
                        pixels[base + 2] = rgb[2];
                    }
                });
            });
        }

        let next_status = band_status(self.band.stage_id, &next);
        output.tov = input.tov;
        output.set_payload(next);
        output.metadata.set_status(next_status);
        self.expected_linear_index += 1;
        Ok(())
    }
}

/// Stateful frame assembler that emits a Copper image only when a full frame is complete.
///
/// Intermediate row traffic stays off the unified log; only the completed image
/// messages from this task are logged, which keeps the benchmark visually useful
/// without turning the log into a row-state dump.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct FrameAssembler {
    width: u32,
    height: u32,
    frames: u32,
    expected_linear_index: u64,
    #[reflect(ignore)]
    current_frame: Vec<u8>,
}

impl Freezable for FrameAssembler {}

impl CuTask for FrameAssembler {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MandelbrotRow);
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("FrameAssembler requires config"))?;
        let width = required_param::<u64>(config, "width")? as u32;
        let height = required_param::<u64>(config, "height")? as u32;
        let frames = required_param::<u64>(config, "frames")? as u32;

        Ok(Self {
            width,
            height,
            frames,
            expected_linear_index: 0,
            current_frame: vec![0; width as usize * height as usize * 3],
        })
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(payload) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let linear_index = payload.linear_index();
        if linear_index != self.expected_linear_index {
            return Err(CuError::from(format!(
                "frame assembler observed out-of-order row: expected {}, got {}",
                self.expected_linear_index, linear_index
            )));
        }
        if payload.width != self.width || payload.height != self.height {
            return Err(CuError::from(
                "frame assembler received a row with mismatched dimensions",
            ));
        }

        let row_stride = self.width as usize * 3;
        let frame_offset = payload.row_index as usize * row_stride;
        payload.pixels_rgb.with_inner(|row_pixels| {
            self.current_frame[frame_offset..frame_offset + row_stride]
                .copy_from_slice(&row_pixels[..row_stride]);
        });

        output.clear_payload();
        output.metadata.set_status(assembler_status(
            payload.frame_index,
            self.frames,
            payload.row_index,
        ));
        if payload.row_index + 1 == self.height {
            if payload.frame_index >= self.frames {
                return Err(CuError::from(
                    "frame assembler received a frame past the configured limit",
                ));
            }

            let digest = hash_bytes(&self.current_frame);
            LAST_FRAME_DIGEST.store(digest, Ordering::Release);
            FRAMES_EMITTED.store(payload.frame_index as u64 + 1, Ordering::Release);

            let format = CuImageBufferFormat {
                width: self.width,
                height: self.height,
                stride: self.width * 3,
                pixel_format: RGB_PIXEL_FORMAT,
            };
            let mut image =
                CuImage::new(format, CuHandle::new_detached(self.current_frame.clone()));
            image.seq = payload.frame_index as u64;
            output.tov = Tov::Time(ctx.now());
            output.set_payload(image);
            output
                .metadata
                .set_status(emitted_status(payload.frame_index, self.frames));

            info!(
                "mandelbrot frame complete: frame={} digest=0x{:016x}",
                payload.frame_index, digest
            );
        }

        self.expected_linear_index += 1;
        Ok(())
    }
}

/// Optional live visualization sink used only by the `rerun_live` mission.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct RerunFrameSink {
    #[reflect(ignore)]
    rec: RecordingStream,
}

impl Freezable for RerunFrameSink {}

impl CuSinkTask for RerunFrameSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuImage<Vec<u8>>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let rec = RecordingStreamBuilder::new("Copper Mandelbrot Zoom")
            .spawn()
            .map_err(|err| CuError::new_with_cause("failed to spawn Rerun stream", err))?;
        Ok(Self { rec })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(image_msg) = input.payload() else {
            return Ok(());
        };

        let rgb = image_msg.buffer_handle.with_inner(|inner| inner.to_vec());
        let image = Image::from_color_model_and_bytes(
            rgb,
            [image_msg.format.width, image_msg.format.height],
            ColorModel::RGB,
            ChannelDatatype::U8,
        );

        self.rec
            .log("mandelbrot/frame", &image)
            .map_err(|err| CuError::new_with_cause("failed to log frame to Rerun", err))?;
        Ok(())
    }
}

/// No-op sink used by the `log_only` mission to keep the assembler as a normal task.
#[derive(Reflect)]
pub struct LoggedFrameDrain;

impl Freezable for LoggedFrameDrain {}

impl CuSinkTask for LoggedFrameDrain {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuImage<Vec<u8>>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
