//! MCU-side AUTO mission, navigation, and ViTFly command tracking.

use super::*;
use crate::messages::{
    AutoMissionLeg, AutoMissionTarget, AutonomyContext, AutonomyControl, AutonomyVelocityCommand,
    ControlInputs, FlightMode, GeographicHeading, NavigationState,
};
use cu_ahrs::AhrsPose;
use cu_gnss_payloads::{GeodeticPosition, GnssFixSolution};
use cu29::units::si::angle::{degree, radian};
use cu29::units::si::f32::{Length, Ratio, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::ratio::ratio;
use cu29::units::si::velocity::meter_per_second;

const EARTH_RADIUS_M: f64 = 6_371_008.8;
const AUTO_DISTANCE_M: f64 = 500.0;
const AUTO_ALTITUDE_OFFSET_M: f32 = 50.0;
const AUTO_ARRIVAL_HORIZONTAL_M: f64 = 12.0;
const AUTO_ARRIVAL_VERTICAL_M: f32 = 5.0;
const NAV_FIX_TIMEOUT: CuDuration = CuDuration(2_000_000_000);
const NAV_ATTITUDE_TIMEOUT: CuDuration = CuDuration(250_000_000);
const CONTEXT_PERIOD: CuDuration = CuDuration(33_000_000);
const COMMAND_TIMEOUT: CuDuration = CuDuration(250_000_000);
const AUTO_MAX_SPEED_MPS: f32 = 4.0;
const AUTO_HOVER_THROTTLE: f32 = 0.48;
// ViTFly was evaluated with Flightmare's geometric controller, whose XY
// velocity-error gain is 3 m/s^2 per m/s. With this app's 60-degree attitude
// limit, the equivalent small-angle stick ratio is approximately
// (3 / 9.81) / radians(60) = 0.29 per m/s.
const AUTO_XY_VELOCITY_TO_TILT_RATIO_PER_MPS: f32 = 0.29;
const AUTO_MAX_TILT_RATIO: f32 = 0.35;

#[derive(Reflect, Default)]
pub struct NavigationStateTask {
    last_fix: Option<GnssFixSolution>,
    last_fix_at: Option<CuTime>,
    last_pose: Option<AhrsPose>,
    last_pose_at: Option<CuTime>,
    last_heading: Option<GeographicHeading>,
    last_heading_at: Option<CuTime>,
}

impl Freezable for NavigationStateTask {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.last_fix, encoder)?;
        Encode::encode(&self.last_fix_at, encoder)?;
        Encode::encode(&self.last_pose, encoder)?;
        Encode::encode(&self.last_pose_at, encoder)?;
        Encode::encode(&self.last_heading, encoder)?;
        Encode::encode(&self.last_heading_at, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.last_fix = Decode::decode(decoder)?;
        self.last_fix_at = Decode::decode(decoder)?;
        self.last_pose = Decode::decode(decoder)?;
        self.last_pose_at = Decode::decode(decoder)?;
        self.last_heading = Decode::decode(decoder)?;
        self.last_heading_at = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for NavigationStateTask {
    type Input<'m> = input_msg!('m, GnssFixSolution, AhrsPose, GeographicHeading);
    type Output<'m> = CuMsg<NavigationState>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let now = ctx.now();
        if let Some(fix) = input.0.payload().copied() {
            self.last_fix = Some(fix);
            self.last_fix_at = Some(now);
        }
        if let Some(pose) = input.1.payload().copied() {
            self.last_pose = Some(pose);
            self.last_pose_at = Some(now);
        }
        if let Some(heading) = input.2.payload().copied() {
            self.last_heading = Some(heading);
            self.last_heading_at = Some(now);
        }

        let fix_fresh = is_fresh(now, self.last_fix_at, NAV_FIX_TIMEOUT);
        let pose_fresh = is_fresh(now, self.last_pose_at, NAV_ATTITUDE_TIMEOUT);
        let heading_fresh = is_fresh(now, self.last_heading_at, NAV_ATTITUDE_TIMEOUT);
        let (fix, pose, heading) = (
            self.last_fix.unwrap_or_default(),
            self.last_pose.unwrap_or_default(),
            self.last_heading.unwrap_or_default(),
        );
        let valid = fix_fresh
            && pose_fresh
            && heading_fresh
            && fix.gnss_fix_ok
            && !fix.invalid_llh
            && navigation_values_are_finite(&fix, &pose, &heading);

        output.tov = Tov::Time(now);
        output.set_payload(NavigationState {
            valid,
            position: fix.position,
            altitude_msl: fix.height_msl,
            velocity_north: fix.velocity_north,
            velocity_east: fix.velocity_east,
            velocity_down: fix.velocity_down,
            heading: heading.heading,
            pose,
        });
        Ok(())
    }
}

#[derive(Reflect, Default)]
pub struct AutoMission {
    active: bool,
    generation: u32,
    leg: AutoMissionLeg,
    home: GeodeticPosition,
    away: GeodeticPosition,
    cruise_altitude_msl: Length,
}

impl Freezable for AutoMission {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.active, encoder)?;
        Encode::encode(&self.generation, encoder)?;
        Encode::encode(&self.leg, encoder)?;
        Encode::encode(&self.home, encoder)?;
        Encode::encode(&self.away, encoder)?;
        Encode::encode(&self.cruise_altitude_msl, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.active = Decode::decode(decoder)?;
        self.generation = Decode::decode(decoder)?;
        self.leg = Decode::decode(decoder)?;
        self.home = Decode::decode(decoder)?;
        self.away = Decode::decode(decoder)?;
        self.cruise_altitude_msl = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for AutoMission {
    type Input<'m> = input_msg!('m, ControlInputs, NavigationState);
    type Output<'m> = CuMsg<AutoMissionTarget>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let enabled = input
            .0
            .payload()
            .is_some_and(|controls| controls.armed && controls.auto);
        let navigation = input.1.payload().copied().unwrap_or_default();

        if !enabled {
            self.active = false;
        } else if !self.active && navigation.valid {
            self.active = true;
            self.generation = self.generation.wrapping_add(1).max(1);
            self.leg = AutoMissionLeg::Outbound;
            self.home = navigation.position;
            self.away = offset_position(
                navigation.position,
                navigation.heading.get::<degree>() as f64,
                AUTO_DISTANCE_M,
            );
            self.cruise_altitude_msl = Length::new::<meter>(
                navigation.altitude_msl.get::<meter>() + AUTO_ALTITUDE_OFFSET_M,
            );
        }

        if self.active && navigation.valid && self.has_arrived(&navigation) {
            self.leg = match self.leg {
                AutoMissionLeg::Outbound => AutoMissionLeg::Return,
                AutoMissionLeg::Return => AutoMissionLeg::Outbound,
            };
        }

        let target_position = match self.leg {
            AutoMissionLeg::Outbound => self.away,
            AutoMissionLeg::Return => self.home,
        };
        output.tov = Tov::Time(ctx.now());
        output.set_payload(AutoMissionTarget {
            active: self.active,
            generation: self.generation,
            leg: self.leg,
            position: target_position,
            altitude_msl: self.cruise_altitude_msl,
        });
        Ok(())
    }
}

impl AutoMission {
    fn has_arrived(&self, navigation: &NavigationState) -> bool {
        let target = match self.leg {
            AutoMissionLeg::Outbound => self.away,
            AutoMissionLeg::Return => self.home,
        };
        position_distance_m(navigation.position, target) <= AUTO_ARRIVAL_HORIZONTAL_M
            && (navigation.altitude_msl.get::<meter>() - self.cruise_altitude_msl.get::<meter>())
                .abs()
                <= AUTO_ARRIVAL_VERTICAL_M
    }
}

#[derive(Reflect, Default)]
pub struct AutonomyContextTask {
    sequence: u64,
    last_emit: Option<CuTime>,
    last_active: bool,
}

impl Freezable for AutonomyContextTask {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.sequence, encoder)?;
        Encode::encode(&self.last_emit, encoder)?;
        Encode::encode(&self.last_active, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.sequence = Decode::decode(decoder)?;
        self.last_emit = Decode::decode(decoder)?;
        self.last_active = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for AutonomyContextTask {
    type Input<'m> = input_msg!('m, ControlInputs, NavigationState, AutoMissionTarget);
    type Output<'m> = CuMsg<AutonomyContext>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let now = ctx.now();
        let controls = input.0.payload();
        let navigation = input.1.payload().copied().unwrap_or_default();
        let target = input.2.payload().copied().unwrap_or_default();
        let active = controls.is_some_and(|controls| controls.armed && controls.auto)
            && navigation.valid
            && target.active;
        let emit_due = self
            .last_emit
            .is_none_or(|last_emit| now - last_emit >= CONTEXT_PERIOD);
        output.tov = Tov::Time(now);
        if !emit_due && active == self.last_active {
            output.clear_payload();
            return Ok(());
        }

        self.sequence = self.sequence.wrapping_add(1);
        self.last_emit = Some(now);
        self.last_active = active;
        let distance_m = position_distance_m(navigation.position, target.position) as f32;
        // Keep the ViTFly preview live at its normal cruise reference even when
        // AUTO is inactive. The active flag, not a zero speed, gates actuation.
        let desired_speed_mps = if active {
            (distance_m * 0.1).clamp(1.0, AUTO_MAX_SPEED_MPS)
        } else {
            AUTO_MAX_SPEED_MPS
        };
        output.set_payload(AutonomyContext {
            sequence: self.sequence,
            mission_generation: target.generation,
            active,
            pose: navigation.pose,
            desired_speed: Velocity::new::<meter_per_second>(desired_speed_mps),
        });
        Ok(())
    }
}

#[derive(Reflect, Default)]
pub struct AutoVelocityController {
    last_command: Option<AutonomyVelocityCommand>,
    last_command_at: Option<CuTime>,
}

impl Freezable for AutoVelocityController {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.last_command, encoder)?;
        Encode::encode(&self.last_command_at, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.last_command = Decode::decode(decoder)?;
        self.last_command_at = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for AutoVelocityController {
    type Input<'m> = input_msg!(
        'm,
        ControlInputs,
        NavigationState,
        AutoMissionTarget,
        AutonomyVelocityCommand
    );
    type Output<'m> = CuMsg<AutonomyControl>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let now = ctx.now();
        if let Some(command) = input.3.payload().copied() {
            let is_newer = self.last_command.is_none_or(|last| {
                sequence_is_newer(command.context_sequence, last.context_sequence)
            });
            if is_newer && command_values_are_finite(&command) {
                self.last_command = Some(command);
                self.last_command_at = Some(now);
            }
        }

        let raw = input.0.payload().copied().unwrap_or_default();
        let navigation = input.1.payload().copied().unwrap_or_default();
        let target = input.2.payload().copied().unwrap_or_default();
        let command = self.last_command.unwrap_or_default();
        let valid = raw.armed
            && raw.auto
            && navigation.valid
            && target.active
            && command.mission_generation == target.generation
            && is_fresh(now, self.last_command_at, COMMAND_TIMEOUT);

        let controls = if valid {
            velocity_command_to_controls(&raw, &navigation, &target, &command)
        } else {
            auto_hover_controls(&raw)
        };
        output.tov = Tov::Time(now);
        output.set_payload(AutonomyControl { valid, controls });
        Ok(())
    }
}

#[derive(Reflect, Default)]
pub struct ModeSupervisor;

impl Freezable for ModeSupervisor {}

impl CuTask for ModeSupervisor {
    type Input<'m> = input_msg!('m, ControlInputs, AutonomyControl);
    type Output<'m> = CuMsg<ControlInputs>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(raw) = input.0.payload().copied() else {
            output.tov = Tov::Time(ctx.now());
            output.clear_payload();
            return Ok(());
        };
        let controls = if raw.auto && raw.armed {
            input
                .1
                .payload()
                .map_or_else(|| auto_hover_controls(&raw), |auto| auto.controls)
        } else {
            raw
        };
        output.tov = input.0.tov;
        output.set_payload(controls);
        Ok(())
    }
}

fn auto_hover_controls(raw: &ControlInputs) -> ControlInputs {
    ControlInputs {
        roll: Ratio::new::<ratio>(0.0),
        pitch: Ratio::new::<ratio>(0.0),
        yaw: Ratio::new::<ratio>(0.0),
        throttle: Ratio::new::<ratio>(if raw.armed { AUTO_HOVER_THROTTLE } else { 0.0 }),
        armed: raw.armed,
        mode: FlightMode::PositionHold,
        auto: raw.auto,
    }
}

fn velocity_command_to_controls(
    raw: &ControlInputs,
    navigation: &NavigationState,
    target: &AutoMissionTarget,
    command: &AutonomyVelocityCommand,
) -> ControlInputs {
    let heading_rad = navigation.heading.get::<radian>();
    let sin_heading = libm::sinf(heading_rad);
    let cos_heading = libm::cosf(heading_rad);
    let velocity_north = navigation.velocity_north.get::<meter_per_second>();
    let velocity_west = -navigation.velocity_east.get::<meter_per_second>();
    let (measured_forward, measured_left) =
        world_flu_to_body(velocity_north, velocity_west, sin_heading, cos_heading);
    let desired_forward = command.forward.get::<meter_per_second>();
    let desired_left = command.left.get::<meter_per_second>();
    let measured_up = -navigation.velocity_down.get::<meter_per_second>();

    let forward_error = desired_forward - measured_forward;
    let left_error = desired_left - measured_left;
    let altitude_error =
        target.altitude_msl.get::<meter>() - navigation.altitude_msl.get::<meter>();
    let desired_up =
        (command.up.get::<meter_per_second>() + altitude_error * 0.25).clamp(-3.0, 3.0);
    let up_error = desired_up - measured_up;

    let desired_heading_deg = bearing_degrees(navigation.position, target.position) as f32;
    let heading_error_deg =
        wrap_signed_degrees(desired_heading_deg - navigation.heading.get::<degree>());

    ControlInputs {
        roll: Ratio::new::<ratio>(
            (-left_error * AUTO_XY_VELOCITY_TO_TILT_RATIO_PER_MPS)
                .clamp(-AUTO_MAX_TILT_RATIO, AUTO_MAX_TILT_RATIO),
        ),
        // Positive FC pitch is forward (the same convention produced by W through RcMapper).
        pitch: Ratio::new::<ratio>(
            (forward_error * AUTO_XY_VELOCITY_TO_TILT_RATIO_PER_MPS)
                .clamp(-AUTO_MAX_TILT_RATIO, AUTO_MAX_TILT_RATIO),
        ),
        // FC yaw and compass heading have opposite signs.
        yaw: Ratio::new::<ratio>((-heading_error_deg * 0.005).clamp(-0.35, 0.35)),
        throttle: Ratio::new::<ratio>((AUTO_HOVER_THROTTLE + up_error * 0.06).clamp(0.25, 0.75)),
        armed: raw.armed,
        mode: FlightMode::PositionHold,
        auto: true,
    }
}

/// Rotate a world-aligned FLU vector into the current body FLU frame.
/// Compass heading is positive toward the east, while FLU's lateral axis is positive west/left.
fn world_flu_to_body(
    world_forward: f32,
    world_left: f32,
    sin_heading: f32,
    cos_heading: f32,
) -> (f32, f32) {
    (
        world_forward * cos_heading - world_left * sin_heading,
        world_forward * sin_heading + world_left * cos_heading,
    )
}

fn is_fresh(now: CuTime, last: Option<CuTime>, timeout: CuDuration) -> bool {
    last.is_some_and(|last| now - last <= timeout)
}

fn navigation_values_are_finite(
    fix: &GnssFixSolution,
    pose: &AhrsPose,
    heading: &GeographicHeading,
) -> bool {
    fix.position.latitude_degrees().is_finite()
        && fix.position.longitude_degrees().is_finite()
        && fix.height_msl.get::<meter>().is_finite()
        && fix.velocity_north.get::<meter_per_second>().is_finite()
        && fix.velocity_east.get::<meter_per_second>().is_finite()
        && fix.velocity_down.get::<meter_per_second>().is_finite()
        && pose.roll.get::<radian>().is_finite()
        && pose.pitch.get::<radian>().is_finite()
        && pose.yaw.get::<radian>().is_finite()
        && heading.heading.get::<radian>().is_finite()
}

fn command_values_are_finite(command: &AutonomyVelocityCommand) -> bool {
    command.forward.get::<meter_per_second>().is_finite()
        && command.left.get::<meter_per_second>().is_finite()
        && command.up.get::<meter_per_second>().is_finite()
}

fn sequence_is_newer(candidate: u64, previous: u64) -> bool {
    candidate != previous && candidate.wrapping_sub(previous) < (1_u64 << 63)
}

fn offset_position(
    start: GeodeticPosition,
    heading_degrees: f64,
    distance_m: f64,
) -> GeodeticPosition {
    let latitude = start.latitude_degrees().to_radians();
    let longitude = start.longitude_degrees().to_radians();
    let bearing = heading_degrees.to_radians();
    let angular_distance = distance_m / EARTH_RADIUS_M;
    let sin_latitude = libm::sin(latitude);
    let cos_latitude = libm::cos(latitude);
    let sin_distance = libm::sin(angular_distance);
    let cos_distance = libm::cos(angular_distance);
    let latitude_out =
        libm::asin(sin_latitude * cos_distance + cos_latitude * sin_distance * libm::cos(bearing));
    let longitude_out = longitude
        + libm::atan2(
            libm::sin(bearing) * sin_distance * cos_latitude,
            cos_distance - sin_latitude * libm::sin(latitude_out),
        );
    GeodeticPosition::from_degrees(latitude_out.to_degrees(), longitude_out.to_degrees())
}

fn position_distance_m(from: GeodeticPosition, to: GeodeticPosition) -> f64 {
    let latitude_from = from.latitude_degrees().to_radians();
    let latitude_to = to.latitude_degrees().to_radians();
    let north = (latitude_to - latitude_from) * EARTH_RADIUS_M;
    let east = (to.longitude_degrees() - from.longitude_degrees()).to_radians()
        * libm::cos((latitude_from + latitude_to) * 0.5)
        * EARTH_RADIUS_M;
    libm::sqrt(north * north + east * east)
}

fn bearing_degrees(from: GeodeticPosition, to: GeodeticPosition) -> f64 {
    let latitude_from = from.latitude_degrees().to_radians();
    let latitude_to = to.latitude_degrees().to_radians();
    let north = latitude_to - latitude_from;
    let east = (to.longitude_degrees() - from.longitude_degrees()).to_radians()
        * libm::cos((latitude_from + latitude_to) * 0.5);
    rem_euclid_f64(libm::atan2(east, north).to_degrees(), 360.0)
}

fn wrap_signed_degrees(value: f32) -> f32 {
    rem_euclid_f32(value + 180.0, 360.0) - 180.0
}

fn rem_euclid_f32(value: f32, modulus: f32) -> f32 {
    let remainder = libm::fmodf(value, modulus);
    if remainder < 0.0 {
        remainder + modulus
    } else {
        remainder
    }
}

fn rem_euclid_f64(value: f64, modulus: f64) -> f64 {
    let remainder = libm::fmod(value, modulus);
    if remainder < 0.0 {
        remainder + modulus
    } else {
        remainder
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn start() -> GeodeticPosition {
        GeodeticPosition::from_degrees(30.389_861_114_639_405, -97.693_168_273_800_47)
    }

    #[test]
    fn waypoint_is_500_meters_ahead_for_cardinal_headings() {
        let start = start();
        let north = offset_position(start, 0.0, AUTO_DISTANCE_M);
        let east = offset_position(start, 90.0, AUTO_DISTANCE_M);

        assert!((position_distance_m(start, north) - AUTO_DISTANCE_M).abs() < 0.1);
        assert!((position_distance_m(start, east) - AUTO_DISTANCE_M).abs() < 0.1);
        assert!(north.latitude_degrees() > start.latitude_degrees());
        assert!(east.longitude_degrees() > start.longitude_degrees());
    }

    #[test]
    fn bearing_and_heading_wrap_follow_compass_convention() {
        let start = start();
        let east = offset_position(start, 90.0, AUTO_DISTANCE_M);
        assert!((bearing_degrees(start, east) - 90.0).abs() < 0.1);
        assert!((wrap_signed_degrees(10.0 - 350.0) - 20.0).abs() < 0.001);
    }

    #[test]
    fn body_frame_vitfly_command_drives_forward_at_cardinal_headings() {
        let start = start();
        let raw = ControlInputs {
            armed: true,
            auto: true,
            ..ControlInputs::default()
        };

        for heading_deg in [0.0_f32, 90.0, 180.0, 270.0] {
            let navigation = NavigationState {
                valid: true,
                position: start,
                heading: cu29::units::si::f32::Angle::new::<degree>(heading_deg),
                ..NavigationState::default()
            };
            let target = AutoMissionTarget {
                active: true,
                position: offset_position(start, heading_deg as f64, AUTO_DISTANCE_M),
                ..AutoMissionTarget::default()
            };
            let command = AutonomyVelocityCommand {
                forward: Velocity::new::<meter_per_second>(4.0),
                left: Velocity::new::<meter_per_second>(0.0),
                ..AutonomyVelocityCommand::default()
            };

            let controls = velocity_command_to_controls(&raw, &navigation, &target, &command);
            assert!(
                controls.pitch.get::<ratio>() > 0.0,
                "heading {heading_deg} should command forward pitch"
            );
            assert!(
                controls.roll.get::<ratio>().abs() < 1.0e-5,
                "heading {heading_deg} introduced lateral roll"
            );
        }
    }

    #[test]
    fn auto_xy_velocity_gain_matches_flightmare_geometric_controller() {
        let start = start();
        let raw = ControlInputs {
            armed: true,
            auto: true,
            ..ControlInputs::default()
        };
        let navigation = NavigationState {
            valid: true,
            position: start,
            heading: cu29::units::si::f32::Angle::new::<degree>(0.0),
            ..NavigationState::default()
        };
        let target = AutoMissionTarget {
            active: true,
            position: offset_position(start, 0.0, AUTO_DISTANCE_M),
            ..AutoMissionTarget::default()
        };
        let command = AutonomyVelocityCommand {
            forward: Velocity::new::<meter_per_second>(1.0),
            left: Velocity::new::<meter_per_second>(1.0),
            ..AutonomyVelocityCommand::default()
        };

        let controls = velocity_command_to_controls(&raw, &navigation, &target, &command);
        assert!(
            (controls.pitch.get::<ratio>() - AUTO_XY_VELOCITY_TO_TILT_RATIO_PER_MPS).abs() < 1.0e-6
        );
        assert!(
            (controls.roll.get::<ratio>() + AUTO_XY_VELOCITY_TO_TILT_RATIO_PER_MPS).abs() < 1.0e-6
        );
    }

    #[test]
    fn auto_mission_alternates_forever_and_reseeds_on_reentry() {
        let (ctx, clock_mock) = CuContext::new_mock_clock();
        let start = start();
        let mut mission = <AutoMission as CuTask>::new(None, ()).expect("create mission");
        let mut controls = CuMsg::new(Some(ControlInputs {
            armed: true,
            auto: true,
            ..ControlInputs::default()
        }));
        let mut navigation_payload = NavigationState {
            valid: true,
            position: start,
            altitude_msl: Length::new::<meter>(212.0),
            heading: cu29::units::si::f32::Angle::new::<degree>(90.0),
            ..NavigationState::default()
        };
        let mut navigation = CuMsg::new(Some(navigation_payload));
        let mut output = CuMsg::default();

        mission
            .process(&ctx, &(&controls, &navigation), &mut output)
            .expect("activate mission");
        let first = output.payload().copied().expect("outbound target");
        assert_eq!(first.leg, AutoMissionLeg::Outbound);
        assert_eq!(first.generation, 1);
        assert!((position_distance_m(start, first.position) - AUTO_DISTANCE_M).abs() < 0.1);
        assert!((first.altitude_msl.get::<meter>() - 262.0).abs() < f32::EPSILON);

        for expected_leg in [
            AutoMissionLeg::Return,
            AutoMissionLeg::Outbound,
            AutoMissionLeg::Return,
            AutoMissionLeg::Outbound,
        ] {
            let current_target = output.payload().copied().expect("current target");
            navigation_payload.position = current_target.position;
            navigation_payload.altitude_msl = current_target.altitude_msl;
            navigation.set_payload(navigation_payload);
            clock_mock.increment(CuDuration::from_millis(10));
            mission
                .process(&ctx, &(&controls, &navigation), &mut output)
                .expect("advance mission leg");
            assert_eq!(output.payload().expect("next target").leg, expected_leg);
        }

        controls.payload_mut().as_mut().expect("controls").auto = false;
        mission
            .process(&ctx, &(&controls, &navigation), &mut output)
            .expect("leave auto");
        assert!(!output.payload().expect("inactive target").active);

        let new_start = offset_position(start, 180.0, 100.0);
        navigation_payload.position = new_start;
        navigation_payload.altitude_msl = Length::new::<meter>(230.0);
        navigation_payload.heading = cu29::units::si::f32::Angle::new::<degree>(180.0);
        navigation.set_payload(navigation_payload);
        controls.payload_mut().as_mut().expect("controls").auto = true;
        mission
            .process(&ctx, &(&controls, &navigation), &mut output)
            .expect("reenter auto");
        let reseeded = output.payload().expect("reseeded target");
        assert_eq!(reseeded.generation, 2);
        assert_eq!(reseeded.leg, AutoMissionLeg::Outbound);
        assert!((position_distance_m(new_start, reseeded.position) - AUTO_DISTANCE_M).abs() < 0.1);
        assert!((reseeded.altitude_msl.get::<meter>() - 280.0).abs() < f32::EPSILON);
    }
}
