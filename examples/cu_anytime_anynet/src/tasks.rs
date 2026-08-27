//! Synthetic/KITTI stereo input and Rerun depth visualization.

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu_anynet::{STAGES, StereoDepth, StereoPair};
use cu_sensor_payloads::{CuDepthLength, CuDepthMap, CuImage, CuImageBufferFormat};
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use image::imageops::FilterType;
use rerun::blueprint::{
    Blueprint, BlueprintActivation, Grid, Spatial2DView, Spatial3DView, TimeSeriesView, Vertical,
};
use rerun::components::{Colormap, ValueRange};
use rerun::datatypes::Range1D;
use rerun::{
    ChannelDatatype, DepthImage, LineStrips3D, Points3D, RecordingStream, RecordingStreamBuilder,
    Scalars, SeriesLines, SpawnOptions, ViewCoordinates,
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
/// Keep one depth sample per square of this many source pixels in the 3D pane.
/// This is visualization-only: inference and KITTI scoring still use every
/// model output pixel.
const DEPTH_POINT_STEP: usize = 4;
/// The 3D pane is for reading foreground structure, not inspecting the
/// network's unconstrained far field. Keeping distant predictions hides the
/// cars inside a large fan of low-disparity noise.
const DEPTH_DISPLAY_MAX_METERS: f32 = 35.0;
/// Fixed screen-space point size: large enough for the source RGB texture to
/// remain recognizable without reconnecting the cloud into an opaque sheet.
const DEPTH_POINT_RADIUS: f32 = 2.0;
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

struct SeenFrame {
    tov: CuTime,
    sequence: u64,
    rgb: Arc<SampledRgb>,
}

struct SampledRgb {
    columns: usize,
    colors: Vec<[u8; 3]>,
}

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

/// Rerun viewer centered on the stereo input, KITTI ground truth, the current
/// anytime result, and its 3D reconstruction. A compact diagnostics strip keeps
/// the three stage snapshots, their latency, and the KITTI 3-pixel error visible
/// without giving debug detail the same weight as the primary comparison.
///
/// All disparity panes share the same model-resolution pixel scale and pinned
/// colormap. A second `time` timeline stamps each stage at the instant it
/// published, so playing it replays both the prediction and its benchmark
/// outlier mask refining live inside every frame.
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
    /// Recent stereo metadata and decimated RGB, used to color a lagging
    /// depth result with the frame it actually reconstructed.
    #[reflect(ignore)]
    seen: VecDeque<SeenFrame>,
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
                let format = stereo.left.format;
                rec.log_static("world", &ViewCoordinates::RDF())
                    .map_err(rerun_error)?;
                rec.log_static(
                    "world/reconstruction_extent",
                    &reconstruction_extent(
                        format,
                        self.focal_pixels,
                        self.display_depth_max_meters(),
                    ),
                )
                .map_err(rerun_error)?;
                self.calibrated = true;
            }
            stereo.left.mark_touched();
            rec.log("world/cam/image", &stereo.left)
                .map_err(rerun_error)?;
            self.seen.push_back(SeenFrame {
                tov: frame_time,
                sequence: stereo.left.seq,
                rgb: Arc::new(sample_rgb(&stereo.left, DEPTH_POINT_STEP)?),
            });
            if self.seen.len() > SEEN_FRAMES {
                self.seen.pop_front();
            }
        }

        // Ground truth for the frame the depth was computed from — matched by
        // tov, so a lagging background result is never scored against the
        // wrong frame's labels.
        let matched_frame = self
            .seen
            .iter()
            .rev()
            .find(|frame| frame.tov == depth_time)
            .map(|frame| (frame.sequence, Arc::clone(&frame.rgb)));
        let ground_truth = matched_frame
            .as_ref()
            .and_then(|(sequence, _rgb)| self.ground_truth(*sequence));
        let rgb = matched_frame.as_ref().map(|(_sequence, rgb)| rgb.as_ref());

        if let Some(depth) = depth_msg.payload() {
            let job_duration = depth
                .snapshots
                .iter()
                .flatten()
                .last()
                .map(|snapshot| snapshot.elapsed)
                .or_else(|| process_duration(depth_msg.metadata.process_time))
                .unwrap_or_default();

            // Ground truth and the empty initial comparison belong to the
            // frame the inference consumed, not necessarily the frame beside
            // a lagging background result.
            let stamped = self.stamp(depth_time);
            rec.set_duration_secs("time", stamped);
            match &ground_truth {
                Some(gt) => rec.log(
                    "reference/ground_truth",
                    &ground_truth_image(
                        gt,
                        depth.depth.format.width,
                        depth.depth.format.height,
                        self.disparity_max_pixels,
                    ),
                ),
                None => rec.log("reference/ground_truth", &DepthImage::clear_fields()),
            }
            .map_err(rerun_error)?;
            rec.log("comparison/outliers", &DepthImage::clear_fields())
                .map_err(rerun_error)?;

            if depth.snapshots.iter().all(Option::is_none) {
                // Latest-at fallback: only the stage this frame reached exists.
                let stamped = self.stamp(depth_time + job_duration);
                rec.set_duration_secs("time", stamped);
                let image = self.log_result(&rec, &depth.depth, rgb)?;
                rec.log(stage_path("stage", depth.stage), &image)
                    .map_err(rerun_error)?;
                self.log_benchmark(&rec, &depth.depth, depth.stage, ground_truth.as_deref())?;
            } else {
                for snapshot in depth.snapshots.iter().flatten() {
                    let stamped = self.stamp(depth_time + snapshot.elapsed);
                    rec.set_duration_secs("time", stamped);
                    // The result entities are rewritten per stage: on `time`
                    // they and the 3D back-projection visibly sharpen inside
                    // each frame; on `copperlist` the last write is the stage
                    // the budget bought.
                    let image = self.log_result(&rec, &snapshot.depth, rgb)?;
                    rec.log(stage_path("stage", snapshot.stage), &image)
                        .map_err(rerun_error)?;
                    rec.log(
                        stage_path("anytime/latency", snapshot.stage),
                        &Scalars::single(millis(snapshot.elapsed)),
                    )
                    .map_err(rerun_error)?;
                    self.log_benchmark(
                        &rec,
                        &snapshot.depth,
                        snapshot.stage,
                        ground_truth.as_deref(),
                    )?;
                }
            }
            let stamped = self.stamp(depth_time + job_duration);
            rec.set_duration_secs("time", stamped);
            rec.log(
                "anytime/latency/total",
                &Scalars::single(millis(job_duration)),
            )
            .map_err(rerun_error)?;
        }
        Ok(())
    }
}

impl DepthViewer {
    /// Focal length times baseline: divides metric depth back to disparity.
    fn disparity_scale(&self) -> f32 {
        self.focal_pixels * self.baseline_meters
    }

    fn display_depth_max_meters(&self) -> f32 {
        (self.depth_max_meters as f32).min(DEPTH_DISPLAY_MAX_METERS)
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

    /// The two live result entities: the disparity pane and an RGB-colored,
    /// display-filtered 3D point cloud. Returns the disparity image so the
    /// caller can reuse it for the stage pane instead of recomputing it.
    fn log_result(
        &self,
        rec: &RecordingStream,
        depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
        rgb: Option<&SampledRgb>,
    ) -> CuResult<DepthImage> {
        let (image, _range) = self.disparity_image(depth);
        rec.log("result/disparity", &image).map_err(rerun_error)?;
        rec.log(
            "world/reconstruction",
            &rgb_point_cloud(
                depth,
                rgb,
                self.focal_pixels,
                DEPTH_POINT_STEP,
                DEPTH_MIN_METERS as f32,
                self.display_depth_max_meters(),
            ),
        )
        .map_err(rerun_error)?;
        Ok(image)
    }

    /// Logs both the scalar KITTI score and the spatial outlier mask for one
    /// stage. The mask shares the stage timestamp, so it refines alongside the
    /// anytime result on the `time` timeline.
    fn log_benchmark(
        &self,
        rec: &RecordingStream,
        depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
        stage: u8,
        ground_truth: Option<&GtDisparity>,
    ) -> CuResult<()> {
        let Some(gt) = ground_truth else {
            return Ok(());
        };
        let Some((bytes, error)) = benchmark_outlier_bytes(depth, gt, self.disparity_scale())
        else {
            return Ok(());
        };
        rec.log(
            stage_path("anytime/error", stage),
            &Scalars::single(f64::from(error)),
        )
        .map_err(rerun_error)?;
        rec.log(
            "comparison/outliers",
            &depth_image(
                bytes,
                depth.format.width,
                depth.format.height,
                (0.0, 1.0),
                Colormap::Grayscale,
            ),
        )
        .map_err(rerun_error)
    }
}

/// A KITTI ground-truth disparity map at its native resolution, kept in the
/// raw `256 × disparity` encoding; `0` marks unlabeled pixels.
struct GtDisparity {
    width: u32,
    height: u32,
    values: Vec<u16>,
}

/// Primary views occupy most of the window: input, truth, live anytime result,
/// and the reconstructed scene. Stage snapshots and benchmark diagnostics use
/// a shorter two-row grid below; each is roughly one quarter the area of a
/// primary pane.
fn send_blueprint(rec: &RecordingStream) -> CuResult<()> {
    let primary = Grid::new([
        Spatial2DView::new("left")
            .with_origin("world/cam/image")
            .into(),
        Spatial2DView::new("KITTI ground truth")
            .with_origin("reference/ground_truth")
            .into(),
        Spatial2DView::new("anytime result")
            .with_origin("result/disparity")
            .into(),
        Spatial3DView::new("RGB depth reconstruction")
            .with_origin("world")
            .with_contents(["world/reconstruction", "world/reconstruction_extent"])
            .into(),
    ])
    .with_grid_columns(2);

    let mut details = Vec::with_capacity(STAGES + 3);
    for stage in 1..=STAGES as u8 {
        details.push(
            Spatial2DView::new(format!("stage {stage}"))
                .with_origin(stage_path("stage", stage))
                .into(),
        );
    }
    details.extend([
        Spatial2DView::new("3-px outliers (white = wrong)")
            .with_origin("comparison/outliers")
            .into(),
        TimeSeriesView::new("latency (ms)")
            .with_origin("anytime/latency")
            .into(),
        TimeSeriesView::new("3-px error (%)")
            .with_origin("anytime/error")
            .into(),
    ]);

    Blueprint::new(
        Vertical::new([
            primary.into(),
            Grid::new(details).with_grid_columns(3).into(),
        ])
        .with_row_shares([4.0, 1.5]),
    )
    .send(rec, BlueprintActivation::default())
    .map_err(rerun_error)
}

/// Names and colors the curves once so the legend reads without hovering.
fn style_curves(rec: &RecordingStream) -> CuResult<()> {
    let curves: [(&str, &str, [u8; 3]); 7] = [
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

/// Copies only the RGB samples used by the display point cloud. Retaining this
/// compact texture rather than an image handle keeps background results tied
/// to their real source frame without exhausting the source's buffer pool.
fn sample_rgb(image: &CuImage<Vec<u8>>, point_step: usize) -> CuResult<SampledRgb> {
    if point_step == 0 {
        return Err(CuError::from("DepthViewer: point step must be non-zero"));
    }
    let bgr = match &image.format.pixel_format {
        b"RGB3" | b"RGB " => false,
        b"BGR3" | b"BGR " => true,
        _ => {
            return Err(CuError::from(
                "DepthViewer: RGB reconstruction requires RGB3 or BGR3 input",
            ));
        }
    };
    let width = image.format.width as usize;
    let height = image.format.height as usize;
    image.with_plane_bytes(0, |bytes, plane| {
        let stride = plane.stride_bytes as usize;
        let columns = width.div_ceil(point_step);
        let mut colors = Vec::with_capacity(columns * height.div_ceil(point_step));
        for y in (0..height).step_by(point_step) {
            for x in (0..width).step_by(point_step) {
                let index = y * stride + x * 3;
                let pixel = [bytes[index], bytes[index + 1], bytes[index + 2]];
                colors.push(if bgr {
                    [pixel[2], pixel[1], pixel[0]]
                } else {
                    pixel
                });
            }
        }
        SampledRgb { columns, colors }
    })
}

/// An RGB-colored point cloud is much easier to parse than a depth-colored
/// sheet: the car remains visually a car while its metric shape is visible.
/// A small median over the sampled grid suppresses isolated depth spikes, and
/// the foreground cutoff removes the unstable low-disparity far field. Both
/// operations are display-only; inference and scoring retain the raw map.
fn rgb_point_cloud(
    depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
    rgb: Option<&SampledRgb>,
    focal_pixels: f32,
    point_step: usize,
    depth_min_meters: f32,
    depth_max_meters: f32,
) -> Points3D {
    let (width, height, stride) = row_layout(depth);
    let capacity = width.div_ceil(point_step) * height.div_ceil(point_step);
    let mut positions = Vec::with_capacity(capacity);
    let mut colors = Vec::with_capacity(capacity);
    let center_x = 0.5 * width as f32;
    let center_y = 0.5 * height as f32;
    depth.with_samples(|samples, _format| {
        for y in (0..height).step_by(point_step) {
            for x in (0..width).step_by(point_step) {
                let Some(z) = median_depth(
                    samples,
                    width,
                    height,
                    stride,
                    x,
                    y,
                    point_step,
                    depth_min_meters,
                    depth_max_meters,
                ) else {
                    continue;
                };
                positions.push([
                    (x as f32 - center_x) * z / focal_pixels,
                    (y as f32 - center_y) * z / focal_pixels,
                    z,
                ]);
                let color_index = y / point_step * width.div_ceil(point_step) + x / point_step;
                colors.push(
                    rgb.filter(|sampled| sampled.columns == width.div_ceil(point_step))
                        .and_then(|sampled| sampled.colors.get(color_index))
                        .copied()
                        .unwrap_or([190, 200, 210]),
                );
            }
        }
    });
    Points3D::new(positions)
        .with_colors(colors)
        .with_radii([rerun::Radius::new_ui_points(DEPTH_POINT_RADIUS)])
}

#[allow(clippy::too_many_arguments)]
fn median_depth(
    samples: &[Length],
    width: usize,
    height: usize,
    stride: usize,
    x: usize,
    y: usize,
    point_step: usize,
    depth_min_meters: f32,
    depth_max_meters: f32,
) -> Option<f32> {
    let mut neighbors = [0.0; 9];
    let mut count = 0;
    let step = point_step as isize;
    for dy in [-step, 0, step] {
        for dx in [-step, 0, step] {
            let sample_x = x as isize + dx;
            let sample_y = y as isize + dy;
            if sample_x < 0
                || sample_y < 0
                || sample_x >= width as isize
                || sample_y >= height as isize
            {
                continue;
            }
            let meters = samples[sample_y as usize * stride + sample_x as usize].get::<meter>();
            if meters.is_finite() && (depth_min_meters..=depth_max_meters).contains(&meters) {
                neighbors[count] = meters;
                count += 1;
            }
        }
    }
    if count < 3 {
        return None;
    }
    neighbors[..count].sort_by(f32::total_cmp);
    Some(neighbors[count / 2])
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

/// KITTI truth resampled to the network output dimensions and converted from
/// native-image pixels to model-image pixels. It therefore shares exactly the
/// same 0..`disparity_max_pixels` colormap as the prediction panes.
fn ground_truth_image(
    gt: &GtDisparity,
    width: u32,
    height: u32,
    disparity_max_pixels: f64,
) -> DepthImage {
    depth_image(
        ground_truth_bytes(gt, width, height),
        width,
        height,
        (0.0, disparity_max_pixels),
        Colormap::Turbo,
    )
}

fn ground_truth_bytes(gt: &GtDisparity, width: u32, height: u32) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(width as usize * height as usize * 4);
    let to_native = gt.width as f32 / width as f32;
    for y in 0..height as usize {
        let gt_y = y * gt.height as usize / height as usize;
        for x in 0..width as usize {
            let gt_x = x * gt.width as usize / width as usize;
            let raw = gt.values[gt_y * gt.width as usize + gt_x];
            let disparity = if raw == 0 {
                f32::NAN
            } else {
                raw as f32 / 256.0 / to_native
            };
            bytes.extend_from_slice(&disparity.to_le_bytes());
        }
    }
    bytes
}

/// KITTI's 3-pixel benchmark as a spatial mask and aggregate percentage.
/// White pixels are outliers, black pixels are correct, and unlabeled pixels
/// are NaN. Thresholding remains in native KITTI pixels even though the mask
/// is displayed at the network's resized resolution.
fn benchmark_outlier_bytes(
    depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
    gt: &GtDisparity,
    disparity_scale: f32,
) -> Option<(Vec<u8>, f32)> {
    let (width, height, stride) = row_layout(depth);
    let gt_width = gt.width as usize;
    let gt_height = gt.height as usize;
    if width == 0 || height == 0 || gt_width == 0 || gt_height == 0 {
        return None;
    }
    let to_native = gt.width as f32 / width as f32;
    let mut bytes = Vec::with_capacity(width * height * 4);
    let mut labeled = 0u32;
    let mut bad = 0u32;
    depth.with_samples(|samples, _format| {
        for y in 0..height {
            let gt_y = y * gt_height / height;
            for x in 0..width {
                let gt_x = x * gt_width / width;
                let raw = gt.values[gt_y * gt_width + gt_x];
                if raw == 0 {
                    bytes.extend_from_slice(&f32::NAN.to_le_bytes());
                    continue;
                }
                let truth = raw as f32 / 256.0;
                labeled += 1;
                let meters = samples[y * stride + x].get::<meter>();
                let predicted = if meters.is_finite() && meters > 0.0 {
                    disparity_scale / meters * to_native
                } else {
                    f32::INFINITY
                };
                let error = (predicted - truth).abs();
                let is_bad = error > 3.0 && error / truth > 0.05;
                bad += u32::from(is_bad);
                bytes.extend_from_slice(&f32::from(is_bad).to_le_bytes());
            }
        }
    });
    (labeled > 0).then(|| (bytes, 100.0 * bad as f32 / labeled as f32))
}

/// A subtle wireframe around the volume reconstructed from the depth image.
/// Besides making the camera geometry legible, this gives Rerun a stable
/// region of interest: depth-image clouds alone do not currently participate
/// in the initial 3D eye's automatic framing.
fn reconstruction_extent(
    format: CuImageBufferFormat,
    focal_pixels: f32,
    far_meters: f32,
) -> LineStrips3D {
    let half_width = 0.5 * format.width as f32 * far_meters / focal_pixels;
    let half_height = 0.5 * format.height as f32 * far_meters / focal_pixels;
    let origin = [0.0, 0.0, 0.0];
    let top_left = [-half_width, -half_height, far_meters];
    let top_right = [half_width, -half_height, far_meters];
    let bottom_right = [half_width, half_height, far_meters];
    let bottom_left = [-half_width, half_height, far_meters];
    LineStrips3D::new([
        vec![origin, top_left],
        vec![origin, top_right],
        vec![origin, bottom_right],
        vec![origin, bottom_left],
        vec![top_left, top_right, bottom_right, bottom_left, top_left],
    ])
    .with_colors([0x6B728080])
    .with_radii([rerun::Radius::new_ui_points(0.75)])
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
        // KITTI scene flow provides `_11` as the following temporal frame.
        // Stereo disparity ground truth and benchmark inputs are the `_10`
        // frames, which are the useful sequence for this depth demo.
        if !entry.file_name().to_string_lossy().ends_with("_10.png") {
            continue;
        }
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
/// The source deliberately selects KITTI's labeled `_10` benchmark frames, so
/// every discovered pair should have a ground-truth entry in a training set.
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

#[cfg(test)]
mod tests {
    use super::*;
    use cu_sensor_payloads::CuDepthMapFormat;

    fn decode_f32(bytes: &[u8]) -> Vec<f32> {
        bytes
            .chunks_exact(4)
            .map(|chunk| f32::from_le_bytes(chunk.try_into().unwrap()))
            .collect()
    }

    #[test]
    fn ground_truth_is_resampled_into_model_pixel_units() {
        let gt = GtDisparity {
            width: 4,
            height: 1,
            values: vec![8 * 256, 0, 20 * 256, 0],
        };

        let values = decode_f32(&ground_truth_bytes(&gt, 2, 1));

        assert_eq!(values, vec![4.0, 10.0]);
    }

    #[test]
    fn benchmark_mask_matches_kitti_three_pixel_rule() {
        let depth = CuDepthMap::new(
            CuDepthMapFormat {
                width: 2,
                height: 1,
                stride: 2,
            },
            CuHandle::new_detached(vec![Length::new::<meter>(2.5), Length::new::<meter>(5.0)]),
        );
        let gt = GtDisparity {
            width: 4,
            height: 1,
            values: vec![8 * 256, 0, 20 * 256, 0],
        };

        let (bytes, error) = benchmark_outlier_bytes(&depth, &gt, 10.0).unwrap();

        assert_eq!(decode_f32(&bytes), vec![0.0, 1.0]);
        assert_eq!(error, 50.0);
    }

    #[test]
    fn benchmark_mask_marks_unlabeled_pixels_as_nan() {
        let depth = CuDepthMap::new(
            CuDepthMapFormat {
                width: 2,
                height: 1,
                stride: 2,
            },
            CuHandle::new_detached(vec![Length::new::<meter>(1.0), Length::new::<meter>(1.25)]),
        );
        let gt = GtDisparity {
            width: 2,
            height: 1,
            values: vec![0, 8 * 256],
        };

        let (bytes, error) = benchmark_outlier_bytes(&depth, &gt, 10.0).unwrap();
        let values = decode_f32(&bytes);
        assert!(values[0].is_nan());
        assert_eq!(values[1], 0.0);
        assert_eq!(error, 0.0);
    }

    #[test]
    fn display_median_suppresses_an_isolated_depth_spike() {
        let mut samples = vec![Length::new::<meter>(12.0); 9];
        samples[4] = Length::new::<meter>(34.0);

        let filtered = median_depth(&samples, 3, 3, 3, 1, 1, 1, 0.5, 35.0);

        assert_eq!(filtered, Some(12.0));
    }
}
