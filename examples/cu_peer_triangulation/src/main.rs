use std::fs;
use std::path::Path;

use cu_peer_triangulation::LocalPosition3d;
use cu_sensor_payloads::{PeerRangeObservation, RangePeerId};
use cu29::bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::prelude::app::CuSimApplication;
use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use cu29::units::si::length::meter;

#[derive(Reflect)]
pub struct RangeScenarioSource {
    next: usize,
}

impl Freezable for RangeScenarioSource {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for RangeScenarioSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(PeerRangeObservation);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { next: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let (peer_id, distance_m) = SCENARIO[self.next % SCENARIO.len()];
        self.next += 1;
        output.tov = Tov::Time(CuTime::from_millis(self.next as u64 * 10));
        output.set_payload(PeerRangeObservation::from_meters(
            RangePeerId::new(peer_id).map_err(|_| CuError::from("invalid scenario peer id"))?,
            distance_m,
            None,
        ));
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct PositionSink {
    latest: Option<LocalPosition3d>,
}

impl Freezable for PositionSink {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.latest, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.latest = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSinkTask for PositionSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(LocalPosition3d);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        self.latest = input.payload().copied();
        if let Some(position) = self.latest {
            if (position.x.get::<meter>() - 2.0).abs() > 0.001
                || (position.y.get::<meter>() - 1.0).abs() > 0.001
                || (position.z.get::<meter>() - 1.0).abs() > 0.001
            {
                return Err(CuError::from("triangulated position drifted from scenario"));
            }
            println!(
                "triangulated x={:.3}m y={:.3}m z={:.3}m residual={:.4}m",
                position.x.get::<meter>(),
                position.y.get::<meter>(),
                position.z.get::<meter>(),
                position.rms_residual.get::<meter>()
            );
        }
        Ok(())
    }
}

const SCENARIO: [(&str, f32); 4] = [
    ("A", 2.449_489_8),
    ("B", 2.449_489_8),
    ("C", 3.0),
    ("D", 3.741_657_5),
];

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct App {}

fn main() -> CuResult<()> {
    let mut callback = |step: <App as CuSimApplication<
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
    >>::Step<'_>|
     -> SimOverride {
        match step {
            default::SimStep::Ranges(CuTaskCallbackState::Process(..)) => {
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::Snapshot(CuTaskCallbackState::Process(_, _)) => {
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::Triangulate(CuTaskCallbackState::Process(_, _)) => {
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::Sink(CuTaskCallbackState::Process(..)) => {
                SimOverride::ExecuteByRuntime
            }
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    let logger_path = "logs/peer-triangulation.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("failed to create logs directory");
    }

    let (robot_clock, _) = RobotClock::mock();
    let mut app = App::builder()
        .with_clock(robot_clock)
        .with_log_path(logger_path, Some(64 * 1024 * 1024))
        .expect("failed to setup logger")
        .with_sim_callback(&mut callback)
        .build()
        .expect("failed to create runtime");

    app.start_all_tasks(&mut callback)?;
    for _ in 0..6 {
        app.run_one_iteration(&mut callback)?;
    }
    app.stop_all_tasks(&mut callback)?;

    Ok(())
}
