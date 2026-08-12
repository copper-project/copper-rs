//! Synthetic/KITTI stereo input and Rerun depth visualization.

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu_anynet::{STAGES, StereoDepth, StereoPair};
use cu_sensor_payloads::{CuDepthLength, CuDepthMap, CuImage, CuImageBufferFormat};
use cu29::cutask_anytime::quality_to_f32;
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use image::imageops::FilterType;
use rerun::blueprint::{
    Blueprint, BlueprintActivation, Grid, Horizontal, Spatial2DView, Spatial3DView,
    TextDocumentView, TimeSeriesView, Vertical,
};
use rerun::components::{Colormap, ValueRange};
use rerun::datatypes::Range1D;
use rerun::{
    ChannelDatatype, DepthImage, MediaType, Pinhole, RecordingStream, RecordingStreamBuilder,
    Scalars, SeriesLines, SpawnOptions, TextDocument, TextLog,
};
use std::collections::VecDeque;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const DEFAULT_WIDTH: u32 = 640;
const DEFAULT_HEIGHT: u32 = 192;
const POOL_SLOTS: usize = 8;
const POOL_ID: &str = "anynet-stereo-source";

/// Nearest depth kept in the 3D view's colormap range, in meters.
const DEPTH_MIN_METERS: f64 = 0.5;
const DEFAULT_DEPTH_MAX_METERS: f64 = 60.0;
/// Far end of the pinned disparity colormap, in pixels.
///
/// The 2D panes colormap disparity rather than metric depth on purpose: a road
/// scene spreads disparity almost uniformly over the colormap, while linear
/// depth crams most of the image into a few color steps and hides what each
/// refinement changed. This is the jet-style look of the AnyNet paper.
const DEFAULT_DISPARITY_MAX_PIXELS: f64 = 96.0;
/// Largest per-stage disparity correction the residual colormap resolves.
const DELTA_MAX_PIXELS: f64 = 8.0;
/// Longest single step the dilated `time` timeline can take, in seconds.
///
/// Dilation exists to make the within-job stage gaps (tens of milliseconds)
/// watchable; without a cap it also stretches the base-compute wait and the
/// idle stretch between jobs into multi-second freezes on playback.
const MAX_DILATED_STEP_SECS: f64 = 1.0;
/// KITTI's focal length; must match the `anynet` node for the 3D view to be
/// metrically correct.
const DEFAULT_FOCAL_PIXELS: f32 = 721.5;
/// KITTI's stereo baseline; with the focal length it converts the metric depth
/// the task publishes back to the disparity the network regressed.
const DEFAULT_BASELINE_METERS: f32 = 0.54;
/// Recent (tov, seq) pairs kept to match a background-delivered depth, which
/// lags by a few frames, back to the stereo frame it was computed from.
const SEEN_FRAMES: usize = 64;

/// Stereo source using a KITTI directory when configured, otherwise a moving
/// synthetic scene with several disparity planes.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct StereoSrc {
    #[reflect(ignore)]
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    #[reflect(ignore)]
    pairs: Vec<(PathBuf, PathBuf)>,
    next_pair: usize,
    sequence: u64,
    format: CuImageBufferFormat,
}

impl Freezable for StereoSrc {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next_pair, encoder)?;
        Encode::encode(&self.sequence, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next_pair = Decode::decode(decoder)?;
        self.sequence = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for StereoSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(StereoPair);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let width = config
            .map(|config| config.get::<u32>("width"))
            .transpose()?
            .flatten()
            .unwrap_or(DEFAULT_WIDTH);
        let height = config
            .map(|config| config.get::<u32>("height"))
            .transpose()?
            .flatten()
            .unwrap_or(DEFAULT_HEIGHT);
        if width == 0 || height == 0 || !width.is_multiple_of(16) || !height.is_multiple_of(16) {
            return Err(CuError::from(
                "StereoSrc dimensions must be non-zero multiples of 16",
            ));
        }
        let data_dir = config
            .map(|config| config.get::<String>("data_dir"))
            .transpose()?
            .flatten();
        let pairs = data_dir
            .as_deref()
            .map(discover_kitti_pairs)
            .transpose()?
            .unwrap_or_default();
        if data_dir.is_some() && pairs.is_empty() {
            return Err(CuError::from(
                "StereoSrc found no matching files under image_2 and image_3",
            ));
        }
        let format = CuImageBufferFormat {
            width,
            height,
            stride: width * 3,
            pixel_format: *b"RGB3",
        };
        let buffer_size = format.required_bytes();
        let pool = CuHostMemoryPool::new(POOL_ID, POOL_SLOTS, || vec![0; buffer_size])?;
        Ok(Self {
            pool,
            pairs,
            next_pair: 0,
            sequence: 0,
            format,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let left_handle = self.pool.acquire().ok_or_else(|| {
            CuError::from("StereoSrc pool exhausted while acquiring the left image")
        })?;
        let right_handle = self.pool.acquire().ok_or_else(|| {
            CuError::from("StereoSrc pool exhausted while acquiring the right image")
        })?;

        if self.pairs.is_empty() {
            fill_synthetic_pair(&left_handle, &right_handle, self.format, self.sequence);
        } else {
            let (left_path, right_path) = &self.pairs[self.next_pair];
            load_image(left_path, &left_handle, self.format)?;
            load_image(right_path, &right_handle, self.format)?;
            self.next_pair = (self.next_pair + 1) % self.pairs.len();
        }

        self.sequence = self.sequence.wrapping_add(1);
        let mut left = CuImage::new(self.format, left_handle);
        let mut right = CuImage::new(self.format, right_handle);
        left.seq = self.sequence;
        right.seq = self.sequence;
        output.set_payload(StereoPair { left, right });
        output.tov = Tov::Time(ctx.now());
        Ok(())
    }
}

/// Rerun viewer laying out the stereo input, the same frame's disparity after
/// each anytime stage, the residual each refinement applied, and the latency
/// those stages cost. The 2D panes colormap disparity in pixels — the AnyNet
/// paper's jet-style look — because metric depth hides refinement (see
/// [`DEFAULT_DISPARITY_MAX_PIXELS`]). A second `time` timeline stamps each
/// stage at the instant it published, so playing it replays the refinement
/// live inside every frame.
///
/// The stage panes need the `anynet` node to set `stage_snapshots`: refinement
/// overwrites the published depth in place, so without snapshots only the stage
/// the budget actually reached exists by the time this sink runs. Without them
/// the panes degrade to a latest-at view: each pane holds the most recent frame
/// that *ended* at that stage, which is a different frame per pane.
///
/// Two optional config keys make the refinement easier to *watch*:
/// - `time_dilation` stretches the `time` timeline around its first stamp, so
///   playing it in Rerun replays the within-frame refinement in slow motion
///   (the raw stage gaps are tens of milliseconds — sub-perceptual at 1×).
/// - `data_dir` (same KITTI directory as the stereo node) enables measured
///   quality: when `disp_occ_0/` ground truth exists for a frame, the paper's
///   3-pixel error is computed per stage snapshot and logged under
///   `anytime/error`, turning "quality" from a policy label into a measurement.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct DepthViewer {
    #[reflect(ignore)]
    rrd_path: Option<String>,
    #[reflect(ignore)]
    rec: Option<RecordingStream>,
    focal_pixels: f32,
    baseline_meters: f32,
    depth_max_meters: f64,
    disparity_max_pixels: f64,
    time_dilation: f64,
    /// Last real and mapped timestamp on the `time` timeline, in seconds.
    #[reflect(ignore)]
    time_cursor: Option<(f64, f64)>,
    #[reflect(ignore)]
    gt_paths: Vec<Option<PathBuf>>,
    /// Decoded ground truth per `gt_paths` index, filled lazily.
    #[reflect(ignore)]
    gt_cache: Vec<Option<Arc<GtDisparity>>>,
    /// Recent (tov, seq) stereo frames, to resolve a lagging depth's frame.
    #[reflect(ignore)]
    seen: VecDeque<(CuTime, u64)>,
    calibrated: bool,
    cycle: i64,
}

impl Freezable for DepthViewer {}

impl CuSinkTask for DepthViewer {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, StereoPair, StereoDepth);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let rrd_path = config
            .map(|config| config.get::<String>("rrd"))
            .transpose()?
            .flatten();
        let focal_pixels = config
            .map(|config| config.get::<f32>("focal_px"))
            .transpose()?
            .flatten()
            .unwrap_or(DEFAULT_FOCAL_PIXELS);
        let baseline_meters = config
            .map(|config| config.get::<f32>("baseline_m"))
            .transpose()?
            .flatten()
            .unwrap_or(DEFAULT_BASELINE_METERS);
        let depth_max_meters = config
            .map(|config| config.get::<f32>("depth_max_m"))
            .transpose()?
            .flatten()
            .map(f64::from)
            .unwrap_or(DEFAULT_DEPTH_MAX_METERS);
        let disparity_max_pixels = config
            .map(|config| config.get::<f32>("disp_max_px"))
            .transpose()?
            .flatten()
            .map(f64::from)
            .unwrap_or(DEFAULT_DISPARITY_MAX_PIXELS);
        if !focal_pixels.is_finite() || focal_pixels <= 0.0 {
            return Err(CuError::from("DepthViewer: focal_px must be positive"));
        }
        if !baseline_meters.is_finite() || baseline_meters <= 0.0 {
            return Err(CuError::from("DepthViewer: baseline_m must be positive"));
        }
        if !depth_max_meters.is_finite() || depth_max_meters <= DEPTH_MIN_METERS {
            return Err(CuError::from(
                "DepthViewer: depth_max_m must exceed the 0.5 m near plane",
            ));
        }
        if !disparity_max_pixels.is_finite() || disparity_max_pixels <= 0.0 {
            return Err(CuError::from("DepthViewer: disp_max_px must be positive"));
        }
        let time_dilation = config
            .map(|config| config.get::<f32>("time_dilation"))
            .transpose()?
            .flatten()
            .map(f64::from)
            .unwrap_or(1.0);
        if !time_dilation.is_finite() || time_dilation < 1.0 {
            return Err(CuError::from(
                "DepthViewer: time_dilation must be finite and >= 1.0",
            ));
        }
        let gt_paths = config
            .map(|config| config.get::<String>("data_dir"))
            .transpose()?
            .flatten()
            .as_deref()
            .map(discover_kitti_ground_truth)
            .transpose()?
            .unwrap_or_default();
        Ok(Self {
            rrd_path,
            rec: None,
            focal_pixels,
            baseline_meters,
            depth_max_meters,
            disparity_max_pixels,
            time_dilation,
            time_cursor: None,
            gt_cache: vec![None; gt_paths.len()],
            gt_paths,
            seen: VecDeque::new(),
            calibrated: false,
            cycle: 0,
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        let builder = RecordingStreamBuilder::new("cu-anytime-anynet");
        let rec = match &self.rrd_path {
            Some(path) => builder.save(path),
            None => builder.spawn_opts(&SpawnOptions {
                new: true,
                ..Default::default()
            }),
        }
        .map_err(|error| CuError::new_with_cause("Failed to open the Rerun stream", error))?;
        send_blueprint(&rec)?;
        style_curves(&rec)?;
        self.rec = Some(rec);
        Ok(())
    }

    fn process(&mut self, ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (stereo_msg, depth_msg): (&CuMsg<StereoPair>, &CuMsg<StereoDepth>) = *input;
        self.cycle += 1;
        let cycle = self.cycle;
        let rec = self
            .rec
            .as_ref()
            .ok_or("DepthViewer: Rerun stream not started")?
            .clone();
        rec.set_time_sequence("copperlist", cycle);

        // Everything is also stamped on a `time` timeline with the moment it
        // actually happened: the left frame at its time of validity and each
        // stage at the instant it published. Scrubbing `copperlist` compares
        // finished frames; playing `time` replays the refinement live.
        //
        // All `time` stamps derive from tovs and snapshot elapsed times, which
        // share the app clock. `metadata.process_time` is not used for stamps:
        // under `sysclock-perf` it lives on a different clock epoch, and in
        // background mode it times the delivery poll, not the job.
        let frame_time = match stereo_msg.tov {
            Tov::Time(time) => time,
            _ => ctx.now(),
        };
        // A background-delivered depth lags the frame beside it; its tov is
        // the tov of the stereo frame the job actually consumed.
        let depth_time = match depth_msg.tov {
            Tov::Time(time) => time,
            _ => frame_time,
        };

        let stamped_frame_time = self.stamp(frame_time);
        rec.set_duration_secs("time", stamped_frame_time);
        if let Some(stereo) = stereo_msg.payload() {
            if !self.calibrated {
                // A pinhole on the parent path back-projects the depth image
                // below it into the 3D view; the resolution is only known once
                // a frame has arrived.
                let format = stereo.left.format;
                let pinhole = Pinhole::from_focal_length_and_resolution(
                    [self.focal_pixels, self.focal_pixels],
                    [format.width as f32, format.height as f32],
                );
                rec.log_static("world/cam", &pinhole).map_err(rerun_error)?;
                self.calibrated = true;
            }
            stereo.left.mark_touched();
            rec.log("world/cam/image", &stereo.left)
                .map_err(rerun_error)?;
            self.seen.push_back((frame_time, stereo.left.seq));
            if self.seen.len() > SEEN_FRAMES {
                self.seen.pop_front();
            }
        }

        // Ground truth for the frame the depth was computed from — matched by
        // tov, so a lagging background result is never scored against the
        // wrong frame's labels.
        let depth_seq = self
            .seen
            .iter()
            .rev()
            .find(|(tov, _)| *tov == depth_time)
            .map(|(_, seq)| *seq);
        let ground_truth = depth_seq.and_then(|seq| self.ground_truth(seq));

        let mut frame = FrameFacts::default();
        if let Some(depth) = depth_msg.payload() {
            let job_duration = depth
                .snapshots
                .iter()
                .flatten()
                .last()
                .map(|snapshot| snapshot.elapsed)
                .or_else(|| process_duration(depth_msg.metadata.process_time))
                .unwrap_or_default();
            frame.total_millis = Some(millis(job_duration));
            if depth.snapshots.iter().all(Option::is_none) {
                // Latest-at fallback: only the stage this frame reached exists.
                let stamped = self.stamp(depth_time + job_duration);
                rec.set_duration_secs("time", stamped);
                let image = self.log_result(&rec, &depth.depth, &mut frame)?;
                rec.log(stage_path("stage", depth.stage), &image)
                    .map_err(rerun_error)?;
                if let Some(gt) = &ground_truth
                    && let Some(error) = three_pixel_error(&depth.depth, gt, self.disparity_scale())
                {
                    rec.log(
                        stage_path("anytime/error", depth.stage),
                        &Scalars::single(f64::from(error)),
                    )
                    .map_err(rerun_error)?;
                    if let Some(slot) = stage_slot(depth.stage) {
                        frame.errors[slot] = Some(error);
                    }
                }
            } else {
                let mut previous: Option<&CuDepthMap<Vec<Length>, CuDepthLength>> = None;
                for snapshot in depth.snapshots.iter().flatten() {
                    let stamped = self.stamp(depth_time + snapshot.elapsed);
                    rec.set_duration_secs("time", stamped);
                    // The result entities are rewritten per stage: on `time`
                    // they and the 3D back-projection visibly sharpen inside
                    // each frame; on `copperlist` the last write is the stage
                    // the budget bought.
                    let image = self.log_result(&rec, &snapshot.depth, &mut frame)?;
                    rec.log(stage_path("stage", snapshot.stage), &image)
                        .map_err(rerun_error)?;
                    rec.log(
                        stage_path("anytime/latency", snapshot.stage),
                        &Scalars::single(millis(snapshot.elapsed)),
                    )
                    .map_err(rerun_error)?;
                    if let Some(gt) = &ground_truth
                        && let Some(error) =
                            three_pixel_error(&snapshot.depth, gt, self.disparity_scale())
                    {
                        rec.log(
                            stage_path("anytime/error", snapshot.stage),
                            &Scalars::single(f64::from(error)),
                        )
                        .map_err(rerun_error)?;
                        if let Some(slot) = stage_slot(snapshot.stage) {
                            frame.errors[slot] = Some(error);
                        }
                    }
                    if let Some(previous) = previous
                        && previous.format == snapshot.depth.format
                    {
                        let (image, largest) =
                            delta_image(&snapshot.depth, previous, self.disparity_scale());
                        rec.log(stage_path("delta", snapshot.stage), &image)
                            .map_err(rerun_error)?;
                        if let Some(slot) = stage_slot(snapshot.stage) {
                            frame.corrections[slot] = Some(largest);
                        }
                    }
                    previous = Some(&snapshot.depth);
                }
            }
            let stamped = self.stamp(depth_time + job_duration);
            rec.set_duration_secs("time", stamped);
            rec.log("anytime/stage", &Scalars::single(depth.stage as f64))
                .map_err(rerun_error)?;
            rec.log(
                "anytime/quality",
                &Scalars::single(quality_to_f32(depth.quality) as f64),
            )
            .map_err(rerun_error)?;
        }

        if let Some(elapsed) = frame.total_millis {
            rec.log("anytime/latency/total", &Scalars::single(elapsed))
                .map_err(rerun_error)?;
        }
        let status = depth_msg.metadata.status_txt.0.as_str();
        rec.log("anytime/status", &TextLog::new(status))
            .map_err(rerun_error)?;
        rec.log(
            "anytime/summary",
            &TextDocument::new(summary(cycle, depth_msg.payload(), status, &frame))
                .with_media_type(MediaType::markdown()),
        )
        .map_err(rerun_error)?;
        Ok(())
    }
}

impl DepthViewer {
    /// Focal length times baseline: divides metric depth back to disparity.
    fn disparity_scale(&self) -> f32 {
        self.focal_pixels * self.baseline_meters
    }

    /// Maps a real timestamp onto the `time` timeline. Every step between
    /// consecutive stamps is stretched by `time_dilation` but capped at
    /// [`MAX_DILATED_STEP_SECS`], so the stage-to-stage refinement plays in
    /// slow motion while the idle stretches between jobs stay short instead of
    /// freezing playback for seconds. At the default 1.0 the mapping is the
    /// identity, which also keeps out-of-order stamps honest — a background
    /// job's stages lie before the frame delivered beside them, and the
    /// cursor-based dilation below would collapse them onto one instant
    /// (dilation is a foreground-mode feature).
    fn stamp(&mut self, time: CuTime) -> f64 {
        let real = secs(time);
        if self.time_dilation <= 1.0 {
            return real;
        }
        let Some((last_real, last_mapped)) = self.time_cursor else {
            self.time_cursor = Some((real, real));
            return real;
        };
        let delta = (real - last_real).max(0.0);
        // Never cap below the real delta: a genuine multi-second stall
        // must still advance the timeline past it.
        let step = (delta * self.time_dilation).min(MAX_DILATED_STEP_SECS.max(delta));
        let mapped = last_mapped + step;
        self.time_cursor = Some((real.max(last_real), mapped));
        mapped
    }

    /// Ground-truth disparity for the given frame sequence, when the viewer
    /// knows the KITTI directory and that frame has a labeled map (only `_10`
    /// frames do). Each map is decoded once and cached; an unreadable one is
    /// warned about once and never retried.
    fn ground_truth(&mut self, sequence: u64) -> Option<Arc<GtDisparity>> {
        if self.gt_paths.is_empty() || sequence == 0 {
            return None;
        }
        let index = (sequence - 1) as usize % self.gt_paths.len();
        if self.gt_cache[index].is_none() {
            let path = self.gt_paths[index].take()?;
            match load_gt_disparity(&path) {
                Ok(gt) => self.gt_cache[index] = Some(Arc::new(gt)),
                Err(error) => warning!(
                    "DepthViewer: skipping unreadable ground truth: {}",
                    error.to_string()
                ),
            }
        }
        self.gt_cache[index].clone()
    }

    /// Disparity pane on the pinned colormap range, so a stage that changed
    /// nothing looks unchanged instead of being re-normalized per frame.
    fn disparity_image(
        &self,
        depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
    ) -> (DepthImage, Option<(f32, f32)>) {
        let (bytes, range) = disparity_bytes(depth, self.disparity_scale());
        let image = depth_image(
            bytes,
            depth.format.width,
            depth.format.height,
            (0.0, self.disparity_max_pixels),
            Colormap::Turbo,
        );
        (image, range)
    }

    /// Metric depth under the pinhole, which the 3D view back-projects.
    fn depth_image(&self, depth: &CuDepthMap<Vec<Length>, CuDepthLength>) -> DepthImage {
        depth_image(
            depth_bytes(depth),
            depth.format.width,
            depth.format.height,
            (DEPTH_MIN_METERS, self.depth_max_meters),
            Colormap::Turbo,
        )
    }

    /// The two live result entities: the disparity pane and the metric depth
    /// the 3D view back-projects. Returns the disparity image so the caller
    /// can reuse it for the stage pane instead of recomputing it.
    fn log_result(
        &self,
        rec: &RecordingStream,
        depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
        frame: &mut FrameFacts,
    ) -> CuResult<DepthImage> {
        let (image, range) = self.disparity_image(depth);
        frame.disparity_range = range;
        rec.log("result/disparity", &image).map_err(rerun_error)?;
        rec.log("world/cam/depth", &self.depth_image(depth))
            .map_err(rerun_error)?;
        Ok(image)
    }
}

/// Numbers the summary panel reports that the panes alone cannot convey.
#[derive(Default)]
struct FrameFacts {
    /// Range of the finite disparity samples in the published map, in pixels.
    disparity_range: Option<(f32, f32)>,
    /// Largest disparity correction each refinement applied, in pixels,
    /// indexed by stage - 1.
    corrections: [Option<f32>; STAGES],
    /// KITTI 3-pixel error of each stage against ground truth, in percent,
    /// indexed by stage - 1.
    errors: [Option<f32>; STAGES],
    total_millis: Option<f64>,
}

/// A KITTI ground-truth disparity map at its native resolution, kept in the
/// raw `256 × disparity` encoding; `0` marks unlabeled pixels.
struct GtDisparity {
    width: u32,
    height: u32,
    values: Vec<u16>,
}

/// Fixed viewport shaped like the AnyNet paper's figure: the input and the
/// same frame's disparity after each stage across the top, coarse to fine;
/// the live result, its 3D back-projection, and the per-stage residuals on
/// the second row; the anytime signals below.
fn send_blueprint(rec: &RecordingStream) -> CuResult<()> {
    let mut panes = vec![
        Spatial2DView::new("left")
            .with_origin("world/cam/image")
            .into(),
    ];
    for stage in 1..=STAGES as u8 {
        panes.push(
            Spatial2DView::new(format!("stage {stage}"))
                .with_origin(stage_path("stage", stage))
                .into(),
        );
    }
    panes.push(
        Spatial2DView::new("anytime result")
            .with_origin("result/disparity")
            .into(),
    );
    panes.push(Spatial3DView::new("depth 3D").with_origin("world").into());
    for stage in 2..=STAGES as u8 {
        panes.push(
            Spatial2DView::new(format!("residual {} to {stage}", stage - 1))
                .with_origin(stage_path("delta", stage))
                .into(),
        );
    }
    // Milliseconds, a 0..1 quality, and an error percentage share no sensible
    // axis, so they get one view each rather than one view that flattens the
    // traces. The error view stays empty unless the viewer has KITTI ground
    // truth (`data_dir` with a `disp_occ_0/` directory).
    let signals = Horizontal::new([
        TimeSeriesView::new("latency (ms)")
            .with_origin("anytime/latency")
            .into(),
        TimeSeriesView::new("quality and stage")
            .with_contents(["anytime/quality", "anytime/stage"])
            .into(),
        TimeSeriesView::new("3-px error (%)")
            .with_origin("anytime/error")
            .into(),
        TextDocumentView::new("frame")
            .with_origin("anytime/summary")
            .into(),
    ]);
    Blueprint::new(Vertical::new([
        Grid::new(panes).with_grid_columns(4).into(),
        signals.into(),
    ]))
    .send(rec, BlueprintActivation::default())
    .map_err(rerun_error)
}

/// Names and colors the curves once so the legend reads without hovering.
fn style_curves(rec: &RecordingStream) -> CuResult<()> {
    let curves: [(&str, &str, [u8; 3]); 9] = [
        ("anytime/stage", "stage reached", [120, 180, 255]),
        ("anytime/quality", "quality", [255, 214, 102]),
        ("anytime/latency/stage1", "stage 1 at", [140, 220, 160]),
        ("anytime/latency/stage2", "stage 2 at", [90, 180, 120]),
        ("anytime/latency/stage3", "stage 3 at", [50, 140, 90]),
        ("anytime/latency/total", "job total", [235, 120, 120]),
        ("anytime/error/stage1", "stage 1 error", [255, 175, 120]),
        ("anytime/error/stage2", "stage 2 error", [235, 125, 80]),
        ("anytime/error/stage3", "stage 3 error", [200, 80, 50]),
    ];
    for (path, name, color) in curves {
        rec.log_static(
            path,
            &SeriesLines::new().with_names([name]).with_colors([color]),
        )
        .map_err(rerun_error)?;
    }
    Ok(())
}

fn stage_path(prefix: &str, stage: u8) -> String {
    format!("{prefix}/stage{stage}")
}

/// `stage - 1` as an array index, `None` for a stage outside `1..=STAGES` —
/// the stage is a wire value, not something to index with unchecked.
fn stage_slot(stage: u8) -> Option<usize> {
    (1..=STAGES as u8)
        .contains(&stage)
        .then(|| stage as usize - 1)
}

/// Per-frame markdown panel: what the budget bought for this copperlist.
///
/// The depth range and the per-stage correction sizes are here because the
/// panes cannot show them: a flat depth map, an all-zero correction, and an
/// empty pane are three different things that all render as one flat color.
fn summary(cycle: i64, depth: Option<&StereoDepth>, status: &str, frame: &FrameFacts) -> String {
    let mut text = format!("### frame {cycle}\n\n`{status}`\n\n| | |\n| --- | --- |\n");
    match depth {
        Some(depth) => {
            text.push_str(&format!("| stage | {} of {STAGES} |\n", depth.stage));
            text.push_str(&format!(
                "| quality | {:.2} |\n",
                quality_to_f32(depth.quality)
            ));
            text.push_str(&match frame.disparity_range {
                Some((low, high)) if (high - low).abs() < f32::EPSILON => {
                    format!("| disparity | flat at {low:.1} px |\n")
                }
                Some((low, high)) => format!("| disparity | {low:.1} to {high:.1} px |\n"),
                None => "| disparity | no valid samples |\n".to_owned(),
            });
            for snapshot in depth.snapshots.iter().flatten() {
                text.push_str(&format!(
                    "| stage {} at | {:.1} ms |\n",
                    snapshot.stage,
                    millis(snapshot.elapsed),
                ));
            }
            for (index, largest) in frame.corrections.iter().enumerate() {
                if let Some(largest) = largest {
                    text.push_str(&format!(
                        "| residual {} to {} | max {largest:.2} px |\n",
                        index,
                        index + 1
                    ));
                }
            }
            for (index, error) in frame.errors.iter().enumerate() {
                if let Some(error) = error {
                    text.push_str(&format!(
                        "| 3-px error, stage {} | {error:.1}% |\n",
                        index + 1
                    ));
                }
            }
        }
        None => text.push_str("| stage | nothing published |\n"),
    }
    if let Some(total) = frame.total_millis {
        text.push_str(&format!("| job total | {total:.1} ms |\n"));
    }
    text
}

/// Serializes a depth map in meters for the 3D back-projection. Rows are
/// walked through the stride: the sample slice includes row padding, while the
/// serialized image is `width` pixels wide.
fn depth_bytes(depth: &CuDepthMap<Vec<Length>, CuDepthLength>) -> Vec<u8> {
    let (width, height, stride) = row_layout(depth);
    let mut bytes = Vec::with_capacity(width * height * 4);
    depth.with_samples(|samples, _format| {
        for row in samples.chunks(stride).take(height) {
            for sample in &row[..width] {
                bytes.extend_from_slice(&sample.get::<meter>().to_le_bytes());
            }
        }
    });
    bytes
}

fn row_layout(depth: &CuDepthMap<Vec<Length>, CuDepthLength>) -> (usize, usize, usize) {
    (
        depth.format.width as usize,
        depth.format.height as usize,
        depth.format.stride as usize,
    )
}

/// Serializes a depth map as disparity in pixels and reports the range of its
/// finite samples.
///
/// The range is what the summary panel reports: a flat map and a broken one
/// both render as one uniform color, and only the numbers tell them apart.
fn disparity_bytes(
    depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
    disparity_scale: f32,
) -> (Vec<u8>, Option<(f32, f32)>) {
    let (width, height, stride) = row_layout(depth);
    let mut bytes = Vec::with_capacity(width * height * 4);
    let mut range: Option<(f32, f32)> = None;
    depth.with_samples(|samples, _format| {
        for row in samples.chunks(stride).take(height) {
            for sample in &row[..width] {
                let meters = sample.get::<meter>();
                let pixels = if meters.is_finite() && meters > 0.0 {
                    disparity_scale / meters
                } else {
                    f32::NAN
                };
                bytes.extend_from_slice(&pixels.to_le_bytes());
                if pixels.is_finite() {
                    range = Some(match range {
                        Some((low, high)) => (low.min(pixels), high.max(pixels)),
                        None => (pixels, pixels),
                    });
                }
            }
        }
    });
    (bytes, range)
}

/// Absolute per-pixel disparity correction one refinement applied, with the
/// largest correction it contains, both in pixels.
///
/// This is the paper's grayscale residual map: what the eye cannot pull out of
/// two near-identical disparity panes — and the reported maximum is what
/// distinguishes "refined nothing" from "no data".
fn delta_image(
    current: &CuDepthMap<Vec<Length>, CuDepthLength>,
    previous: &CuDepthMap<Vec<Length>, CuDepthLength>,
    disparity_scale: f32,
) -> (DepthImage, f32) {
    let pixels = |meters: f32| {
        if meters.is_finite() && meters > 0.0 {
            disparity_scale / meters
        } else {
            f32::NAN
        }
    };
    let (width, height, stride) = row_layout(current);
    let mut before = Vec::with_capacity(previous.format.required_elements());
    previous.with_samples(|samples, _format| before.extend_from_slice(samples));
    let mut bytes = Vec::with_capacity(width * height * 4);
    let mut largest = 0.0f32;
    current.with_samples(|samples, _format| {
        for (row, before_row) in samples
            .chunks(stride)
            .zip(before.chunks(stride))
            .take(height)
        {
            for (sample, before) in row[..width].iter().zip(&before_row[..width]) {
                let delta = (pixels(sample.get::<meter>()) - pixels(before.get::<meter>())).abs();
                bytes.extend_from_slice(&delta.to_le_bytes());
                if delta.is_finite() && delta > largest {
                    largest = delta;
                }
            }
        }
    });
    let image = depth_image(
        bytes,
        current.format.width,
        current.format.height,
        (0.0, DELTA_MAX_PIXELS),
        Colormap::Grayscale,
    );
    (image, largest)
}

fn depth_image(
    bytes: Vec<u8>,
    width: u32,
    height: u32,
    range: (f64, f64),
    colormap: Colormap,
) -> DepthImage {
    DepthImage::from_data_type_and_bytes(bytes, [width, height], ChannelDatatype::F32)
        .with_meter(1.0)
        .with_colormap(colormap)
        .with_depth_range(ValueRange(Range1D([range.0, range.1])))
}

fn millis(duration: CuDuration) -> f64 {
    duration.as_nanos() as f64 / 1_000_000.0
}

fn secs(time: CuTime) -> f64 {
    time.as_nanos() as f64 / 1_000_000_000.0
}

fn process_duration(range: PartialCuTimeRange) -> Option<CuDuration> {
    if range.start.is_none() || range.end.is_none() {
        return None;
    }
    Some(range.end.unwrap() - range.start.unwrap())
}

fn discover_kitti_pairs(data_dir: &str) -> CuResult<Vec<(PathBuf, PathBuf)>> {
    let root = Path::new(data_dir);
    let left_dir = root.join("image_2");
    let right_dir = root.join("image_3");
    let entries = fs::read_dir(&left_dir).map_err(|error| {
        CuError::new_with_cause("Failed to read the KITTI image_2 directory", error)
    })?;
    let mut pairs = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|error| {
            CuError::new_with_cause("Failed to read a KITTI directory entry", error)
        })?;
        let right = right_dir.join(entry.file_name());
        if entry.path().is_file() && right.is_file() {
            pairs.push((entry.path(), right));
        }
    }
    pairs.sort();
    Ok(pairs)
}

/// Ground-truth disparity path per stereo pair, in the same order as
/// [`discover_kitti_pairs`] so the source's frame sequence indexes both.
/// KITTI 2015 labels only the `_10` frame of each scene, so `_11` entries
/// are `None`.
fn discover_kitti_ground_truth(data_dir: &str) -> CuResult<Vec<Option<PathBuf>>> {
    let gt_dir = Path::new(data_dir).join("disp_occ_0");
    Ok(discover_kitti_pairs(data_dir)?
        .into_iter()
        .map(|(left, _right)| {
            let candidate = gt_dir.join(left.file_name()?);
            candidate.is_file().then_some(candidate)
        })
        .collect())
}

/// KITTI disparity maps are 16-bit PNG images holding `256 × disparity`; zero marks
/// pixels without a label.
fn load_gt_disparity(path: &Path) -> CuResult<GtDisparity> {
    let image = image::open(path)
        .map_err(|error| {
            CuError::new_with_cause("Failed to load a KITTI ground-truth disparity map", error)
        })?
        .to_luma16();
    let (width, height) = image.dimensions();
    let values = image.into_raw();
    Ok(GtDisparity {
        width,
        height,
        values,
    })
}

/// KITTI's 3-pixel error: the share of labeled pixels whose disparity is off
/// by more than 3 px *and* more than 5%, in percent — the paper's per-stage
/// quality metric. The published map is nearest-neighbor matched against the
/// native-resolution ground truth and its disparities are rescaled to native
/// pixel units, so the 3 px threshold keeps its KITTI meaning.
fn three_pixel_error(
    depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
    gt: &GtDisparity,
    disparity_scale: f32,
) -> Option<f32> {
    let width = depth.format.width as usize;
    let height = depth.format.height as usize;
    let stride = depth.format.stride as usize;
    let gt_width = gt.width as usize;
    let gt_height = gt.height as usize;
    if width == 0 || height == 0 || gt_width == 0 || gt_height == 0 {
        return None;
    }
    let to_native = gt.width as f32 / width as f32;
    let mut labeled = 0u32;
    let mut bad = 0u32;
    depth.with_samples(|samples, _format| {
        for y in 0..height {
            let gt_y = y * gt_height / height;
            for x in 0..width {
                let gt_x = x * gt_width / width;
                let raw = gt.values[gt_y * gt_width + gt_x];
                if raw == 0 {
                    continue;
                }
                let truth = raw as f32 / 256.0;
                labeled += 1;
                let meters = samples[y * stride + x].get::<meter>();
                let predicted = if meters.is_finite() && meters > 0.0 {
                    disparity_scale / meters * to_native
                } else {
                    // An invalid prediction over a labeled pixel is wrong.
                    f32::INFINITY
                };
                let error = (predicted - truth).abs();
                if error > 3.0 && error / truth > 0.05 {
                    bad += 1;
                }
            }
        }
    });
    (labeled > 0).then(|| 100.0 * bad as f32 / labeled as f32)
}

fn load_image(
    path: &Path,
    handle: &CuHandle<Vec<u8>>,
    format: CuImageBufferFormat,
) -> CuResult<()> {
    let image = image::open(path)
        .map_err(|error| CuError::new_with_cause("Failed to load a stereo image", error))?
        .resize_exact(format.width, format.height, FilterType::Triangle)
        .to_rgb8();
    handle.with_inner_mut(|buffer| buffer.copy_from_slice(image.as_raw()));
    Ok(())
}

fn fill_synthetic_pair(
    left: &CuHandle<Vec<u8>>,
    right: &CuHandle<Vec<u8>>,
    format: CuImageBufferFormat,
    sequence: u64,
) {
    let phase = sequence as usize % 64;
    left.with_inner_mut(|left_buffer| {
        right.with_inner_mut(|right_buffer| {
            for y in 0..format.height as usize {
                let disparity = match y * 3 / format.height as usize {
                    0 => 4,
                    1 => 8,
                    _ => 16,
                };
                for x in 0..format.width as usize {
                    let left_pixel = synthetic_pixel(x, y, phase);
                    let right_pixel = synthetic_pixel(x + disparity, y, phase);
                    let index = y * format.stride as usize + x * 3;
                    left_buffer[index..index + 3].copy_from_slice(&left_pixel);
                    right_buffer[index..index + 3].copy_from_slice(&right_pixel);
                }
            }
        });
    });
}

fn synthetic_pixel(x: usize, y: usize, phase: usize) -> [u8; 3] {
    let checker = (((x + phase) / 24 + y / 24) % 2) as u8;
    let stripe = (((x + phase * 2) / 9) % 2) as u8;
    [
        45 + checker * 170,
        35 + stripe * 190,
        ((x.wrapping_mul(3) + y.wrapping_mul(5) + phase) % 256) as u8,
    ]
}

fn rerun_error(error: rerun::RecordingStreamError) -> CuError {
    CuError::new_with_cause("Failed to log to Rerun", error)
}
