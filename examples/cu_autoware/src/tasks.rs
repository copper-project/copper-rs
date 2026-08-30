//! The six generic node types of the Autoware reference system.
//!
//! Every node is parameterized from its RON `config:` block: `crunch_limit` sets the
//! busy work, `period_ms` the tick of the timed ones. Timing constants are load-bearing
//! in a benchmark, so a missing or mistyped key is an error rather than a default.

#[cfg(feature = "hybrid-background")]
use crate::payload::{REGION_CALLBACK_CAPACITY, RefRegionSample};
use crate::payload::{RefLaneSample, RefSample};
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use std::hint::black_box;
use std::marker::PhantomData;

fn cfg_u64(config: Option<&ComponentConfig>, task: &str, key: &str) -> CuResult<u64> {
    config
        .map(|c| c.get::<u64>(key))
        .transpose()
        .map_err(|e| CuError::from(format!("{task}: config key '{key}': {e}")))?
        .flatten()
        .ok_or_else(|| CuError::from(format!("{task}: missing required config key '{key}'")))
}

/// Port of the fork's `number_cruncher`, off-by-one included: the inner loop stops
/// *below* the truncated square root, so prime squares and a few semiprimes are counted
/// as prime (32 below 100, not 25). The benchmark calibrates against this cost curve, so
/// the bug is preserved deliberately.
fn number_cruncher(limit: u64) -> u64 {
    let mut primes = 0u64;
    for i in 3..limit {
        let bound = (i as f64).sqrt() as u64;
        let mut is_prime = true;
        for n in 2..bound {
            if i.is_multiple_of(n) {
                is_prime = false;
                break;
            }
        }
        primes += u64::from(is_prime);
    }
    primes
}

/// The reference system's number crunch. `limit` is calibrated per node to a target
/// duration.
///
/// Never inlined: the app would otherwise get one copy per call site and `calibrate`
/// another, and the same limit then costs a few percent more in one binary than the other.
#[inline(never)]
pub fn crunch(limit: u64) {
    black_box(number_cruncher(limit));
}

/// Period gating for the timed nodes. Copper has no per-task period, so the whole
/// multi-rate mechanism is confined here and stays swappable.
///
/// Deliberate choice: clock-based pacing, equivalent to an rclcpp timer. The next
/// deadline is rescheduled as `due + period`, and resynced to `now + period` once a
/// whole period has been missed, so a lagging loop never fires a catch-up burst.
/// Replay is therefore only deterministic at copperlist granularity.
#[derive(Reflect)]
pub struct Pacer {
    period: CuDuration,
    due: Option<CuTime>,
}

impl Pacer {
    pub fn new(config: Option<&ComponentConfig>, task: &str) -> CuResult<Self> {
        Ok(Self {
            period: CuDuration::from_millis(cfg_u64(config, task, "period_ms")?),
            due: None,
        })
    }

    /// True on the cycles where the node is due to run.
    pub fn fire(&mut self, now: CuTime) -> bool {
        let Some(due) = self.due else {
            self.due = Some(now + self.period);
            return true;
        };
        if now < due {
            return false;
        }
        let next = due + self.period;
        self.due = Some(if next > now { next } else { now + self.period });
        true
    }
}

impl Freezable for Pacer {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.due, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.due = Decode::decode(decoder)?;
        Ok(())
    }
}

/// The runtime reuses the copperlist slots, so a suppressed output has to drop the
/// previous cycle's time of validity along with its payload.
fn suppress<P: CuMsgPayload>(output: &mut CuMsg<P>) {
    output.clear_payload();
    output.tov = Tov::default();
}

/// Callback names in execution order for each independent region. The order keeps the
/// reported critical continuation ahead of non-critical fan-out work.
#[cfg(feature = "hybrid-background")]
pub fn region_callbacks(region: usize) -> &'static [&'static str] {
    match region {
        0 => &[
            "front_lidar",
            "points_transformer_front",
            "point_cloud_fusion",
            "ray_ground_filter",
            "euclidean_cluster_detector",
            "object_collision_estimator",
            "behavior_planner_input_0",
            "voxel_grid_downsampler",
            "ndt_localizer_voxel_callback",
        ],
        1 => &[
            "rear_lidar",
            "points_transformer_rear",
            "point_cloud_fusion_rear_callback",
        ],
        2 => &[
            "point_cloud_map",
            "point_cloud_map_loader",
            "ndt_localizer",
            "lanelet2_global_planner_ndt_callback",
            "behavior_planner_input_1",
        ],
        3 => &[
            "visualizer",
            "lanelet2_global_planner",
            "lanelet2_map_loader_global_callback",
            "behavior_planner_input_2",
        ],
        4 => &[
            "lanelet2_map",
            "lanelet2_map_loader",
            "behavior_planner_input_3",
            "parking_planner",
            "behavior_planner_input_4",
            "lane_planner",
            "behavior_planner_input_5",
        ],
        5 => &[
            "euclidean_cluster_settings",
            "euclidean_cluster_detector_settings_callback",
            "intersection_output",
        ],
        6 => &[
            "behavior_planner",
            "mpc_controller",
            "vehicle_interface",
            "vehicle_dbw",
            "vehicle_interface_behavior_callback",
        ],
        _ => &[],
    }
}

#[cfg(feature = "hybrid-background")]
const fn region_endpoint_index(region: usize) -> Option<usize> {
    match region {
        0 => Some(6),
        1 => Some(2),
        6 => Some(3),
        _ => None,
    }
}

/// A profile-guided-style static region: its periodic root is scheduled in the
/// background, then every causal continuation runs inline on that same worker.
#[cfg(feature = "hybrid-background")]
#[derive(Reflect)]
pub struct RefRegion<const REGION: usize> {
    pacer: Pacer,
    crunch_limits: [u64; REGION_CALLBACK_CAPACITY],
    seq: u64,
}

#[cfg(feature = "hybrid-background")]
impl<const REGION: usize> Freezable for RefRegion<REGION> {}

#[cfg(feature = "hybrid-background")]
impl<const REGION: usize> CuSrcTask for RefRegion<REGION> {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(RefRegionSample);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let callbacks = region_callbacks(REGION);
        if callbacks.is_empty() || callbacks.len() > REGION_CALLBACK_CAPACITY {
            return Err(CuError::from("invalid static Autoware callback region"));
        }
        let mut crunch_limits = [0; REGION_CALLBACK_CAPACITY];
        for (index, callback) in callbacks.iter().enumerate() {
            crunch_limits[index] = cfg_u64(config, "RefRegion", callback)?;
        }
        Ok(Self {
            pacer: Pacer::new(config, "RefRegion")?,
            crunch_limits,
            seq: 0,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let now = ctx.now();
        if !self.pacer.fire(now) {
            suppress(output);
            return Ok(());
        }
        self.seq += 1;
        let mut callback_ns = [0; REGION_CALLBACK_CAPACITY];
        let mut endpoint_ns = 0;
        for (index, elapsed_ns) in callback_ns
            .iter_mut()
            .enumerate()
            .take(region_callbacks(REGION).len())
        {
            let started = ctx.now();
            crunch(self.crunch_limits[index]);
            *elapsed_ns = (ctx.now() - started).as_nanos();
            if region_endpoint_index(REGION) == Some(index) {
                endpoint_ns = (ctx.now() - now).as_nanos();
            }
        }
        output.tov = Tov::Time(now);
        output.set_payload(RefRegionSample {
            seq: self.seq,
            endpoint_ns,
            callback_ns,
            ..Default::default()
        });
        Ok(())
    }
}

/// Timer-driven sensor: emits a fresh sample on its own period.
#[derive(Reflect)]
pub struct RefSensor {
    pacer: Pacer,
    crunch_limit: u64,
    seq: u64,
}

impl Freezable for RefSensor {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.seq, encoder)?;
        self.pacer.freeze(encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.seq = Decode::decode(decoder)?;
        self.pacer.thaw(decoder)
    }
}

impl CuSrcTask for RefSensor {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(RefSample);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            pacer: Pacer::new(config, "RefSensor")?,
            crunch_limit: cfg_u64(config, "RefSensor", "crunch_limit")?,
            seq: 0,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let now = ctx.now();
        if !self.pacer.fire(now) {
            suppress(output);
            return Ok(());
        }
        crunch(self.crunch_limit);
        self.seq += 1;
        output.tov = Tov::Time(now);
        output.set_payload(RefSample {
            seq: self.seq,
            ..Default::default()
        });
        Ok(())
    }
}

/// One in, one out: runs only on a fresh input and forwards its provenance.
#[derive(Reflect)]
pub struct RefTransform {
    crunch_limit: u64,
}

impl Freezable for RefTransform {}

impl CuTask for RefTransform {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(RefSample);
    type Output<'m> = output_msg!(RefSample);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            crunch_limit: cfg_u64(config, "RefTransform", "crunch_limit")?,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(sample) = input.payload() else {
            suppress(output);
            return Ok(());
        };
        crunch(self.crunch_limit);
        output.tov = input.tov;
        output.set_payload(sample.clone());
        Ok(())
    }
}

/// Two in, one out. Input 0 is the trigger; a fresh input 1 is only charged
/// `cache_crunch_limit`, the way the reference system's secondary callback is.
///
/// The reference node also has a warm-up gate that waits for both inputs before it first
/// emits. The fork comments out its `.reset()`, so steady state is identical and it is
/// left out here rather than reimplemented.
#[derive(Reflect)]
pub struct RefFusion {
    crunch_limit: u64,
    cache_crunch_limit: u64,
}

impl Freezable for RefFusion {}

impl RefFusion {
    /// `charge` is `crunch` in production and a recorder in the tests.
    fn fuse(
        &self,
        input: &[&CuMsg<RefSample>; 2],
        output: &mut CuMsg<RefSample>,
        mut charge: impl FnMut(u64),
    ) {
        if input[1].payload().is_some() {
            charge(self.cache_crunch_limit);
        }
        let Some(trigger) = input[0].payload() else {
            suppress(output);
            return;
        };
        charge(self.crunch_limit);
        output.tov = input[0].tov;
        output.set_payload(trigger.clone());
    }
}

impl CuTask for RefFusion {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, RefSample, RefSample);
    type Output<'m> = output_msg!(RefSample);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            crunch_limit: cfg_u64(config, "RefFusion", "crunch_limit")?,
            cache_crunch_limit: cfg_u64(config, "RefFusion", "cache_crunch_limit")?,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        self.fuse(&[input.0, input.1], output, crunch);
        Ok(())
    }
}

/// Six inputs, driven by its own period instead of by its inputs. Each fresh input is
/// only charged `cache_crunch_limit`.
#[derive(Reflect)]
pub struct RefCyclic {
    pacer: Pacer,
    crunch_limit: u64,
    cache_crunch_limit: u64,
    seq: u64,
}

impl Freezable for RefCyclic {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.seq, encoder)?;
        self.pacer.freeze(encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.seq = Decode::decode(decoder)?;
        self.pacer.thaw(decoder)
    }
}

impl RefCyclic {
    /// `charge` is `crunch` in production and a recorder in the tests.
    fn tick(
        &mut self,
        now: CuTime,
        input: &[&CuMsg<RefSample>; 6],
        output: &mut CuMsg<RefSample>,
        mut charge: impl FnMut(u64),
    ) {
        for msg in input {
            if msg.payload().is_some() {
                charge(self.cache_crunch_limit);
            }
        }
        if !self.pacer.fire(now) {
            suppress(output);
            return;
        }
        charge(self.crunch_limit);
        self.seq += 1;
        output.tov = Tov::Time(now);
        output.set_payload(RefSample {
            seq: self.seq,
            ..Default::default()
        });
    }
}

impl CuTask for RefCyclic {
    type Resources<'r> = ();
    type Input<'m> =
        input_msg!('m, RefSample, RefSample, RefSample, RefSample, RefSample, RefSample);
    type Output<'m> = output_msg!(RefSample);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            pacer: Pacer::new(config, "RefCyclic")?,
            crunch_limit: cfg_u64(config, "RefCyclic", "crunch_limit")?,
            cache_crunch_limit: cfg_u64(config, "RefCyclic", "cache_crunch_limit")?,
            seq: 0,
        })
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let input = [input.0, input.1, input.2, input.3, input.4, input.5];
        self.tick(ctx.now(), &input, output, crunch);
        Ok(())
    }
}

/// Two independent lanes, in0 -> out0 and in1 -> out1, each with its own budget.
#[derive(Reflect)]
pub struct RefIntersection {
    crunch_limit0: u64,
    crunch_limit1: u64,
}

impl Freezable for RefIntersection {}

impl CuTask for RefIntersection {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, RefSample, RefSample);
    type Output<'m> = output_msg!(RefSample, RefLaneSample);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            crunch_limit0: cfg_u64(config, "RefIntersection", "crunch_limit0")?,
            crunch_limit1: cfg_u64(config, "RefIntersection", "crunch_limit1")?,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        lane(self.crunch_limit0, input.0, &mut output.0);
        lane(self.crunch_limit1, input.1, &mut output.1);
        Ok(())
    }
}

fn lane<P: CuMsgPayload + From<RefSample>>(
    crunch_limit: u64,
    input: &CuMsg<RefSample>,
    output: &mut CuMsg<P>,
) {
    let Some(sample) = input.payload() else {
        suppress(output);
        return;
    };
    crunch(crunch_limit);
    output.tov = input.tov;
    output.set_payload(sample.clone().into());
}

/// Terminal command node, over whichever lane type reaches it.
// No `type_path = false`: the derived per-instantiation TypePath is what
// `CuSinkTask::debug_state_type_path` needs, and it keeps the two lanes distinguishable.
#[derive(Reflect)]
pub struct RefCommand<P: CuMsgPayload + AsRef<RefSample>> {
    crunch_limit: u64,
    #[reflect(ignore)]
    _payload: PhantomData<fn() -> P>,
}

impl<P: CuMsgPayload + AsRef<RefSample>> Freezable for RefCommand<P> {}

impl<P: CuMsgPayload + AsRef<RefSample>> CuSinkTask for RefCommand<P> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(P);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            crunch_limit: cfg_u64(config, "RefCommand", "crunch_limit")?,
            _payload: PhantomData,
        })
    }

    fn process(&mut self, ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(sample) = input.payload() else {
            return Ok(());
        };
        crunch(self.crunch_limit);
        // A sink has no output message to stamp, so the status goes to the text log
        // (debug builds only: release compiles debug! out).
        debug!(ctx, "command seq {}", sample.as_ref().seq);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "hybrid-background")]
    #[test]
    fn hybrid_regions_cover_every_callback_once() {
        let callbacks: Vec<_> = (0..7)
            .flat_map(|region| region_callbacks(region).iter().copied())
            .collect();
        assert_eq!(callbacks.len(), 36);
        let unique: std::collections::HashSet<_> = callbacks.iter().copied().collect();
        assert_eq!(unique.len(), callbacks.len());
        assert_eq!(region_endpoint_index(0), Some(6));
        assert_eq!(region_endpoint_index(1), Some(2));
        assert_eq!(region_endpoint_index(6), Some(3));
    }
    use bincode::config::standard;
    use bincode::de::DecoderImpl;
    use bincode::de::read::SliceReader;
    use bincode::encode_to_vec;

    fn config(entries: &[(&str, u64)]) -> ComponentConfig {
        let mut config = ComponentConfig::new();
        for (key, value) in entries {
            config.set(key, *value);
        }
        config
    }

    fn sample(seq: u64, tov_ms: u64) -> CuMsg<RefSample> {
        let mut msg = CuMsg::new(Some(RefSample {
            seq,
            ..Default::default()
        }));
        msg.tov = Tov::Time(CuTime::from_millis(tov_ms));
        msg
    }

    fn sensor(period_ms: u64) -> RefSensor {
        RefSensor::new(
            Some(&config(&[("period_ms", period_ms), ("crunch_limit", 0)])),
            (),
        )
        .unwrap()
    }

    fn transform() -> RefTransform {
        RefTransform::new(Some(&config(&[("crunch_limit", 0)])), ()).unwrap()
    }

    /// Distinct sentinels so a recorded charge says which callback ran.
    const CRUNCH: u64 = 7;
    const CACHE: u64 = 3;

    fn fusion() -> RefFusion {
        RefFusion::new(
            Some(&config(&[
                ("crunch_limit", CRUNCH),
                ("cache_crunch_limit", CACHE),
            ])),
            (),
        )
        .unwrap()
    }

    fn cyclic() -> RefCyclic {
        RefCyclic::new(
            Some(&config(&[
                ("period_ms", 100),
                ("crunch_limit", CRUNCH),
                ("cache_crunch_limit", CACHE),
            ])),
            (),
        )
        .unwrap()
    }

    fn fusion_charges(task: &RefFusion, input: &[&CuMsg<RefSample>; 2]) -> Vec<u64> {
        let mut charged = Vec::new();
        let mut output = CuMsg::new(None);
        task.fuse(input, &mut output, |limit| charged.push(limit));
        charged
    }

    fn thaw_from<T: Freezable>(task: &mut T, frozen: &[u8]) {
        let mut decoder = DecoderImpl::new(SliceReader::new(frozen), standard(), ());
        task.thaw(&mut decoder).unwrap();
    }

    #[test]
    fn test_crunch_prime_count_is_stable() {
        // Anchors the shape the step 3 calibration maps milliseconds onto. 32, not 25:
        // the fork's off-by-one lets 4 and 9 through, and this port keeps it.
        assert_eq!(number_cruncher(100), 32);
    }

    #[test]
    fn test_missing_config_key_is_an_error() {
        let err = RefTransform::new(Some(&config(&[])), ())
            .err()
            .expect("a missing key must fail");
        assert!(err.to_string().contains("crunch_limit"));
        // A float where an integer is expected must not silently read as 0 either.
        let mut float_config = ComponentConfig::new();
        float_config.set("crunch_limit", 200.0f64);
        assert!(RefTransform::new(Some(&float_config), ()).is_err());
    }

    #[test]
    fn test_pacer_fires_once_per_period() {
        let mut pacer = Pacer::new(Some(&config(&[("period_ms", 200)])), "t").unwrap();
        // 5ms loop over one second: fire at t=0 then every 200ms.
        let fired: Vec<u64> = (0..200)
            .map(|step| CuTime::from_millis(step * 5))
            .filter(|now| pacer.fire(*now))
            .map(|now| now.as_nanos() / 1_000_000)
            .collect();
        assert_eq!(fired, [0, 200, 400, 600, 800]);
    }

    #[test]
    fn test_pacer_resyncs_after_a_missed_period() {
        let mut pacer = Pacer::new(Some(&config(&[("period_ms", 200)])), "t").unwrap();
        assert!(pacer.fire(CuTime::from_millis(0)));
        // The loop stalls for more than two periods: one fire, no catch-up burst.
        assert!(pacer.fire(CuTime::from_millis(700)));
        assert!(!pacer.fire(CuTime::from_millis(800)));
        assert!(pacer.fire(CuTime::from_millis(900)));
    }

    #[test]
    fn test_transform_skips_empty_input() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = transform();
        let mut output = CuMsg::new(Some(RefSample::default()));
        output.tov = Tov::Time(CuTime::from_millis(1));

        task.process(&ctx, &CuMsg::new(None), &mut output).unwrap();
        assert!(output.payload().is_none());
        assert_eq!(
            output.tov,
            Tov::None,
            "a suppressed output must drop its tov"
        );

        task.process(&ctx, &sample(7, 42), &mut output).unwrap();
        assert_eq!(output.payload().unwrap().seq, 7);
    }

    #[test]
    fn test_fusion_triggers_on_input_zero_only() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = fusion();
        let mut output = CuMsg::new(None);

        task.process(&ctx, &(&CuMsg::new(None), &sample(1, 10)), &mut output)
            .unwrap();
        assert!(output.payload().is_none());

        task.process(&ctx, &(&sample(9, 90), &CuMsg::new(None)), &mut output)
            .unwrap();
        assert_eq!(output.payload().unwrap().seq, 9);
    }

    #[test]
    fn test_fusion_charge_model() {
        let task = fusion();
        let empty = CuMsg::new(None);
        let fresh = sample(1, 10);

        // The cache callback is charged for a fresh input 1 whether or not anything is emitted.
        assert_eq!(fusion_charges(&task, &[&empty, &fresh]), [CACHE]);
        assert_eq!(fusion_charges(&task, &[&fresh, &empty]), [CRUNCH]);
        assert_eq!(fusion_charges(&task, &[&fresh, &fresh]), [CACHE, CRUNCH]);
        assert!(fusion_charges(&task, &[&empty, &empty]).is_empty());
    }

    #[test]
    fn test_cyclic_charge_model() {
        let empty = CuMsg::new(None);
        let fresh = sample(1, 0);
        for fresh_inputs in 0..=6 {
            let mut task = cyclic();
            let input: [&CuMsg<RefSample>; 6] =
                std::array::from_fn(|i| if i < fresh_inputs { &fresh } else { &empty });
            let mut output = CuMsg::new(None);

            // The first tick fires the pacer, so the timer callback is charged too.
            let mut charged = Vec::new();
            task.tick(CuTime::from_millis(0), &input, &mut output, |limit| {
                charged.push(limit)
            });
            let mut expected = vec![CACHE; fresh_inputs];
            expected.push(CRUNCH);
            assert_eq!(charged, expected);

            // The next one does not, but the input callbacks still run.
            let mut charged = Vec::new();
            task.tick(CuTime::from_millis(10), &input, &mut output, |limit| {
                charged.push(limit)
            });
            assert_eq!(charged, vec![CACHE; fresh_inputs]);
        }
    }

    #[test]
    fn test_config_pins_the_trigger_lane_of_every_multi_input_node() {
        let config = read_configuration("copperconfig.ron").unwrap();
        let graph = config.get_graph(None).unwrap();
        // Input order is connection order, which only the edge order carries.
        let inputs = |id: &str| -> Vec<String> {
            let node_id = graph.get_node_id_by_name(id).unwrap();
            let mut edges: Vec<&Cnx> = graph
                .get_dst_edges(node_id)
                .unwrap()
                .iter()
                .map(|edge_id| graph.edge(*edge_id).unwrap())
                .collect();
            edges.sort_by_key(|edge| edge.order);
            edges.iter().map(|edge| edge.src.clone()).collect()
        };

        assert_eq!(
            inputs("point_cloud_fusion"),
            ["points_transformer_front", "points_transformer_rear"]
        );
        assert_eq!(
            inputs("ndt_localizer"),
            ["point_cloud_map_loader", "voxel_grid_downsampler"]
        );
        assert_eq!(
            inputs("lanelet2_global_planner"),
            ["visualizer", "ndt_localizer"]
        );
        assert_eq!(
            inputs("lanelet2_map_loader"),
            ["lanelet2_map", "lanelet2_global_planner"]
        );
        assert_eq!(
            inputs("euclidean_cluster_detector"),
            ["ray_ground_filter", "euclidean_cluster_settings"]
        );
        assert_eq!(
            inputs("behavior_planner"),
            [
                "object_collision_estimator",
                "ndt_localizer",
                "lanelet2_global_planner",
                "lanelet2_map_loader",
                "parking_planner",
                "lane_planner",
            ]
        );
        assert_eq!(
            inputs("vehicle_interface"),
            ["mpc_controller", "behavior_planner"]
        );
    }

    #[test]
    fn test_intersection_lanes_are_independent() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = RefIntersection::new(
            Some(&config(&[("crunch_limit0", 0), ("crunch_limit1", 0)])),
            (),
        )
        .unwrap();
        let mut output = (CuMsg::new(None), CuMsg::new(None));

        task.process(&ctx, &(&sample(1, 10), &CuMsg::new(None)), &mut output)
            .unwrap();
        assert_eq!(output.0.payload().unwrap().seq, 1);
        assert!(output.1.payload().is_none());

        task.process(&ctx, &(&CuMsg::new(None), &sample(2, 20)), &mut output)
            .unwrap();
        assert!(output.0.payload().is_none());
        assert_eq!(output.1.payload().unwrap().0.seq, 2);
    }

    #[test]
    fn test_cyclic_ticks_on_its_own_period() {
        let (ctx, mock) = CuContext::new_mock_clock();
        let mut task = cyclic();
        let empty = CuMsg::new(None);
        let fresh = sample(1, 0);
        let mut output = CuMsg::new(None);

        // First cycle ticks, the next one does not even though every input is fresh.
        let all_fresh = (&fresh, &fresh, &fresh, &fresh, &fresh, &fresh);
        task.process(&ctx, &all_fresh, &mut output).unwrap();
        assert_eq!(output.payload().unwrap().seq, 1);
        mock.increment(CuDuration::from_millis(10));
        task.process(&ctx, &all_fresh, &mut output).unwrap();
        assert!(output.payload().is_none());

        // ... and it ticks on its period even with no input at all.
        mock.increment(CuDuration::from_millis(100));
        let none = (&empty, &empty, &empty, &empty, &empty, &empty);
        task.process(&ctx, &none, &mut output).unwrap();
        assert_eq!(output.payload().unwrap().seq, 2);
    }

    #[test]
    fn test_sensor_freeze_thaw_restores_state() {
        let (ctx, mock) = CuContext::new_mock_clock();
        let mut task = sensor(200);
        let mut output = CuMsg::new(None);
        mock.increment(CuDuration::from_millis(500));
        task.process(&ctx, &mut output).unwrap();

        let frozen = encode_to_vec(BincodeAdapter(&task), standard()).unwrap();
        // Drift the state away from what was frozen.
        mock.increment(CuDuration::from_millis(400));
        task.process(&ctx, &mut output).unwrap();
        assert_eq!(output.payload().unwrap().seq, 2);

        thaw_from(&mut task, &frozen);
        assert_eq!(task.pacer.due, Some(CuTime::from_millis(700)));
        mock.increment(CuDuration::from_millis(400));
        task.process(&ctx, &mut output).unwrap();
        assert_eq!(
            output.payload().unwrap().seq,
            2,
            "seq resumed from the keyframe"
        );
    }

    #[test]
    fn test_cyclic_freeze_thaw_restores_state() {
        let mut task = cyclic();
        task.seq = 12;
        task.pacer.due = Some(CuTime::from_millis(340));
        let frozen = encode_to_vec(BincodeAdapter(&task), standard()).unwrap();

        task.seq = 99;
        task.pacer.due = None;
        thaw_from(&mut task, &frozen);
        assert_eq!(task.seq, 12);
        assert_eq!(task.pacer.due, Some(CuTime::from_millis(340)));
    }

    #[test]
    fn test_seq_and_tov_propagate_to_fusion() {
        let (ctx, mock) = CuContext::new_mock_clock();
        mock.increment(CuDuration::from_millis(300));
        let mut sensor = sensor(200);
        let mut transform = transform();
        let mut fusion = fusion();

        let mut sensed = CuMsg::new(None);
        let mut transformed = CuMsg::new(None);
        let mut fused = CuMsg::new(None);
        sensor.process(&ctx, &mut sensed).unwrap();
        transform.process(&ctx, &sensed, &mut transformed).unwrap();
        fusion
            .process(&ctx, &(&transformed, &CuMsg::new(None)), &mut fused)
            .unwrap();

        assert_eq!(sensed.payload().unwrap().seq, 1);
        assert_eq!(sensed.tov, Tov::Time(CuTime::from_millis(300)));
        assert_eq!(fused.payload().unwrap().seq, 1);
        assert_eq!(fused.tov, sensed.tov);
    }
}
