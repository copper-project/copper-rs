use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct TickPayload {
    pub tick: u64,
    pub phase: f32,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct TrajectoryPayload {
    pub tick: u64,
    pub phase: f32,
    pub target_x: f32,
    pub target_z: f32,
    pub height: f32,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct VizPayload {
    pub tick: u64,
    pub phase: f32,
    pub energy: f32,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct MotorCommandPayload {
    pub tick: u64,
    pub torque: f32,
    pub armed: bool,
}

#[derive(Reflect)]
pub struct ClockSource {
    tick: u64,
    phase_step: f32,
}

impl Freezable for ClockSource {}

impl CuSrcTask for ClockSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(TickPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            tick: 0,
            phase_step: cfg_f32(config, "phase_step", 0.17)?,
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.tick = self.tick.wrapping_add(1);
        let phase = self.tick as f32 * self.phase_step;

        output.set_payload(TickPayload {
            tick: self.tick,
            phase,
        });
        output.metadata.set_status(format!("tick {}", self.tick));

        if self.tick.is_multiple_of(12) {
            debug!(
                "clock source emitted synthetic tick={} phase={phase:.2}",
                self.tick
            );
        }

        Ok(())
    }
}

#[derive(Reflect)]
pub struct PlannerTask {
    radius: f32,
    height_bias: f32,
}

impl Freezable for PlannerTask {}

impl CuTask for PlannerTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(TickPayload);
    type Output<'m> = output_msg!(TrajectoryPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            radius: cfg_f32(config, "radius", 1.8)?,
            height_bias: cfg_f32(config, "height_bias", 0.2)?,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(tick) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let target_x = (tick.phase * 0.6).sin() * self.radius;
        let target_z = (tick.phase * 0.4).cos() * self.radius * 0.7;
        let height = 1.0 + self.height_bias + (tick.phase * 0.3).sin() * 0.25;

        output.set_payload(TrajectoryPayload {
            tick: tick.tick,
            phase: tick.phase,
            target_x,
            target_z,
            height,
        });
        output
            .metadata
            .set_status(format!("target x={target_x:+.2}"));

        if tick.tick.is_multiple_of(18) {
            info!(
                "planner retargeted synthetic path: x={target_x:+.2} z={target_z:+.2} h={height:.2}"
            );
        }

        Ok(())
    }
}

#[derive(Reflect)]
pub struct ControllerTask {
    torque_scale: f32,
    warn_torque: f32,
    hold_every: u64,
}

impl Freezable for ControllerTask {}

impl CuTask for ControllerTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(TrajectoryPayload);
    type Output<'m> = output_msg!(VizPayload, MotorCommandPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            torque_scale: cfg_f32(config, "torque_scale", 0.82)?,
            warn_torque: cfg_f32(config, "warn_torque", 0.9)?,
            hold_every: cfg_u64(config, "hold_every", 48)?.max(1),
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(plan) = input.payload() else {
            output.0.clear_payload();
            output.1.clear_payload();
            return Ok(());
        };

        let phase = plan.phase.rem_euclid(std::f32::consts::TAU);
        let torque = ((plan.target_x * 0.32) - (plan.target_z * 0.18)
            + ((plan.height - 1.0) * 0.7))
            * self.torque_scale;
        let armed = !plan.tick.is_multiple_of(self.hold_every);
        let clamped_torque = torque.clamp(-1.2, 1.2);

        output.0.set_payload(VizPayload {
            tick: plan.tick,
            phase,
            energy: clamped_torque.abs() * 0.9 + plan.height * 0.1,
        });
        output
            .0
            .metadata
            .set_status(format!("mesh phase {phase:.2}"));

        output.1.set_payload(MotorCommandPayload {
            tick: plan.tick,
            torque: clamped_torque,
            armed,
        });
        output.1.metadata.set_status(if armed {
            format!("cmd tau={clamped_torque:+.2}")
        } else {
            "hold".to_string()
        });

        if plan.tick.is_multiple_of(24) && clamped_torque.abs() > self.warn_torque {
            warning!("controller torque nearing clamp: {clamped_torque:+.2}");
        }

        if plan.tick.is_multiple_of(45) {
            error!(
                "controller synthetic estimator glitch at tick {}; continuing in degraded mode",
                plan.tick
            );
        }

        Ok(())
    }
}

#[derive(Reflect)]
pub struct VizSink {
    overlay_label: String,
}

impl Freezable for VizSink {}

impl CuSinkTask for VizSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(VizPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            overlay_label: cfg_string(config, "overlay_label", "sim-overlay")?,
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(viz) = input.payload() else {
            return Ok(());
        };

        if viz.tick.is_multiple_of(30) {
            debug!(
                "viz sink refreshed '{}' with phase={:.2} energy={:.2}",
                self.overlay_label.as_str(),
                viz.phase,
                viz.energy
            );
        }

        Ok(())
    }
}

#[derive(Reflect)]
pub struct ActuatorSink {
    hard_limit: f32,
}

impl Freezable for ActuatorSink {}

impl CuSinkTask for ActuatorSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MotorCommandPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            hard_limit: cfg_f32(config, "hard_limit", 1.0)?,
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(cmd) = input.payload() else {
            return Ok(());
        };

        if cmd.tick.is_multiple_of(20) {
            info!(
                "actuator frame accepted: armed={} torque={:+.2}",
                cmd.armed, cmd.torque
            );
        }

        if !cmd.armed {
            warning!("actuator hold requested at tick {}", cmd.tick);
        }

        if cmd.tick.is_multiple_of(60) || cmd.torque.abs() > self.hard_limit {
            error!(
                "actuator telemetry timeout simulated at tick {}; last torque={:+.2}",
                cmd.tick, cmd.torque
            );
        }

        Ok(())
    }
}

fn cfg_f32(config: Option<&ComponentConfig>, key: &str, default: f32) -> CuResult<f32> {
    let value = match config {
        Some(cfg) => cfg.get::<f64>(key)?,
        None => None,
    };
    Ok(value.map(|v| v as f32).unwrap_or(default))
}

fn cfg_u64(config: Option<&ComponentConfig>, key: &str, default: u64) -> CuResult<u64> {
    let value = match config {
        Some(cfg) => cfg.get::<u64>(key)?,
        None => None,
    };
    Ok(value.unwrap_or(default))
}

fn cfg_string(config: Option<&ComponentConfig>, key: &str, default: &str) -> CuResult<String> {
    let value = match config {
        Some(cfg) => cfg.get::<String>(key)?,
        None => None,
    };
    Ok(value.unwrap_or_else(|| default.to_string()))
}
