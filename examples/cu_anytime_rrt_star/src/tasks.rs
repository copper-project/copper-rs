//! The Copper side of the demo: a navigation simulator ahead of the planner
//! and a Rerun viewer behind it.
//!
//! The feedback loop closes outside the graph: the viewer stores the newest
//! published path in [`LATEST_PATH`], and the simulator follows it one cycle
//! later. Foreground execution runs nav -> planner -> viewer in order within
//! one copperlist, so the hand-off is deterministic.

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu_rrt_star::{PlanPath, PlanRequest, Point2, World};
use cu29::prelude::*;
use rerun::{
    Color, LineStrips2D, Points2D, Radius, RecordingStream, RecordingStreamBuilder, Scalars,
};
use std::sync::Mutex;

/// Where the robot starts, in free space on the depot map.
const START: Point2 = Point2::new(0.5, 0.5);

/// The goals the robot patrols, all in free space on the depot map. They are
/// placed so a pillar blocks the straight line of every leg: each leg needs a
/// detour, and refinement has visible work on every trajectory.
const PATROL: [Point2; 3] = [
    Point2::new(9.5, 9.5),
    Point2::new(0.5, 7.0),
    Point2::new(6.5, 0.5),
];

/// The newest path the viewer saw, followed by the simulator one cycle later.
static LATEST_PATH: Mutex<Option<PlanPath>> = Mutex::new(None);

/// A point robot that replans while it drives.
///
/// Each copperlist it advances along the newest published path, then asks for
/// a fresh plan from wherever it is now. When it reaches the current patrol
/// goal it picks the next one. While no path is published - the first cycle,
/// or a job under the quality floor - it holds position and retries with a
/// new seed.
#[derive(Reflect)]
pub struct NavSim {
    pose: Point2,
    goal_index: u32,
    seed: u64,
    speed_mps: f32,
    goal_threshold: f32,
    #[reflect(ignore)]
    last_now: Option<CuTime>,
}

impl Freezable for NavSim {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.pose, encoder)?;
        Encode::encode(&self.goal_index, encoder)?;
        Encode::encode(&self.seed, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.pose = Decode::decode(decoder)?;
        self.goal_index = Decode::decode(decoder)?;
        self.seed = Decode::decode(decoder)?;
        Ok(())
    }
}

impl NavSim {
    /// Advances the pose by `budget` meters along `path`. Waypoint 0 is where
    /// the path was planned from - last cycle's pose - so the walk starts at
    /// waypoint 1.
    fn follow(&mut self, path: &PlanPath, mut budget: f32) {
        let len = path.len as usize;
        let mut next = 1;
        while budget > 0.0 && next < len {
            let target = path.waypoints[next];
            let distance = self.pose.distance(target);
            if distance <= budget {
                self.pose = target;
                budget -= distance;
                next += 1;
            } else {
                let ratio = budget / distance;
                self.pose = Point2::new(
                    self.pose.x + ratio * (target.x - self.pose.x),
                    self.pose.y + ratio * (target.y - self.pose.y),
                );
                break;
            }
        }
    }
}

impl CuSrcTask for NavSim {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(PlanRequest);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let mut seed = 1u64;
        let mut speed_mps = 1.5f32;
        let mut goal_threshold = 0.3f32;
        if let Some(config) = config {
            if let Some(value) = config.get::<u32>("seed")? {
                seed = value as u64;
            }
            if let Some(value) = config.get::<f32>("speed_mps")? {
                speed_mps = value;
            }
            if let Some(value) = config.get::<f32>("goal_threshold")? {
                goal_threshold = value;
            }
        }
        Ok(Self {
            pose: START,
            goal_index: 0,
            seed,
            speed_mps,
            goal_threshold,
            last_now: None,
        })
    }

    fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let now = ctx.clock.now();
        let dt = match self.last_now {
            Some(last) => {
                let CuDuration(nanos) = now - last;
                nanos as f32 / 1e9
            }
            None => 0.0,
        };
        self.last_now = Some(now);

        let path = LATEST_PATH.lock().expect("path poisoned").clone();
        if let Some(path) = path {
            self.follow(&path, self.speed_mps * dt);
        }

        if self.pose.distance(PATROL[self.goal_index as usize]) <= self.goal_threshold {
            self.goal_index = (self.goal_index + 1) % PATROL.len() as u32;
            // A path toward the old goal must not be driven.
            *LATEST_PATH.lock().expect("path poisoned") = None;
        }

        self.seed = self.seed.wrapping_add(1);
        new_msg.set_payload(PlanRequest {
            world: World::depot(),
            start: self.pose,
            goal: PATROL[self.goal_index as usize],
            seed: self.seed,
        });
        new_msg.tov = Tov::Time(now);
        Ok(())
    }
}

/// Draws the world, the robot and the published path into a Rerun viewer,
/// and stores the path for [`NavSim`] to follow.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct RerunViewer {
    #[reflect(ignore)]
    rec: RecordingStream,
    cycle: i64,
    world_logged: bool,
    trail: Vec<Point2>,
}

impl Freezable for RerunViewer {}

impl CuSinkTask for RerunViewer {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, PlanRequest, PlanPath);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let builder = RecordingStreamBuilder::new("cu-anytime-rrt-star");
        // `rrd` writes the recording to a file instead of spawning a viewer,
        // for headless runs.
        let rec = match config.and_then(|c| c.get::<String>("rrd").transpose()) {
            Some(path) => builder.save(path?),
            None => builder.spawn(),
        }
        .map_err(|e| CuError::new_with_cause("Failed to open the Rerun stream", e))?;
        Ok(Self {
            rec,
            cycle: 0,
            world_logged: false,
            trail: Vec::new(),
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        // Input order follows the cnx order in the RON: the request first.
        let (request_msg, path_msg): (&CuMsg<PlanRequest>, &CuMsg<PlanPath>) = *input;
        let Some(request) = request_msg.payload() else {
            return Ok(());
        };

        self.cycle += 1;
        self.rec.set_time_sequence("copperlist", self.cycle);
        apply_tov(&self.rec, &request_msg.tov);

        if !self.world_logged {
            log_world(&self.rec, &request.world)?;
            self.world_logged = true;
        }

        self.trail.push(request.start);
        log(
            &self.rec,
            "world/robot",
            &Points2D::new([(request.start.x, request.start.y)])
                .with_radii([Radius::new_scene_units(0.15)])
                .with_colors([Color::from([255, 140, 0])]),
        )?;
        log(
            &self.rec,
            "world/trail",
            &LineStrips2D::new([self.trail.iter().map(|p| (p.x, p.y)).collect::<Vec<_>>()])
                .with_colors([Color::from([255, 200, 120])]),
        )?;
        log(
            &self.rec,
            "world/goal",
            &Points2D::new([(request.goal.x, request.goal.y)])
                .with_radii([Radius::new_scene_units(0.2)])
                .with_colors([Color::from([0, 200, 0])]),
        )?;

        // An empty strip when nothing was published: a job under the quality
        // floor visibly clears the path.
        let strip: Vec<(f32, f32)> = path_msg.payload().map_or_else(Vec::new, |path| {
            path.waypoints[..path.len as usize]
                .iter()
                .map(|p| (p.x, p.y))
                .collect()
        });
        log(
            &self.rec,
            "world/path",
            &LineStrips2D::new([strip]).with_colors([Color::from([0, 128, 255])]),
        )?;

        // The same quality definition as the planner: straight line over cost.
        let quality = path_msg.payload().map_or(0.0, |path| {
            (request.start.distance(request.goal) / path.cost) as f64
        });
        log(&self.rec, "curves/quality", &Scalars::single(quality))?;
        let iterations = path_msg
            .payload()
            .map_or(0.0, |path| path.iterations as f64);
        log(&self.rec, "curves/iterations", &Scalars::single(iterations))?;

        if let Some(path) = path_msg.payload() {
            *LATEST_PATH.lock().expect("path poisoned") = Some(path.clone());
        }
        Ok(())
    }
}

fn apply_tov(rec: &RecordingStream, tov: &Tov) {
    match tov {
        Tov::Time(t) => rec.set_duration_secs("tov", t.0 as f64 / 1e9),
        Tov::Range(r) => rec.set_duration_secs("tov", r.start.0 as f64 / 1e9),
        Tov::None => rec.reset_time(),
    }
}

fn log(rec: &RecordingStream, path: &str, entity: &impl rerun::AsComponents) -> CuResult<()> {
    rec.log(path, entity)
        .map_err(|e| CuError::new_with_cause("Failed to log to Rerun", e))
}

/// The static part of the scene: the world bounds and the obstacles.
fn log_world(rec: &RecordingStream, world: &World) -> CuResult<()> {
    let (w, h) = (world.width, world.height);
    rec.log_static(
        "world/bounds",
        &LineStrips2D::new([[(0.0, 0.0), (w, 0.0), (w, h), (0.0, h), (0.0, 0.0)]])
            .with_colors([Color::from([180, 180, 180])]),
    )
    .map_err(|e| CuError::new_with_cause("Failed to log the world bounds", e))?;

    let obstacles = &world.obstacles[..world.obstacle_count as usize];
    rec.log_static(
        "world/obstacles",
        &Points2D::new(obstacles.iter().map(|o| (o.center.x, o.center.y)))
            .with_radii(obstacles.iter().map(|o| Radius::new_scene_units(o.radius)))
            .with_colors([Color::from([100, 100, 100])]),
    )
    .map_err(|e| CuError::new_with_cause("Failed to log the obstacles", e))
}
