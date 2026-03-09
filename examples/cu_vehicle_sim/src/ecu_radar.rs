//! Toyota ADAS Radar ECU simulation task.
//!
//! Implements a `CuSrcTask` that generates all 34 CAN messages defined in the
//! toyota_tss2_adas.dbc file. The task produces one CAN frame per `process()` call,
//! cycling through all messages in round-robin order.
//!
//! Each radar track simulates a vehicle at a given distance and relative speed,
//! with values that evolve over time to produce a realistic CAN trace.

use cu29::prelude::*;
use cu_automotive_payloads::can::{CanFrame, CanId};

use crate::dbc_generated::{DBC_MESSAGES, DBC_MESSAGE_COUNT};
use crate::signal_pack::pack_signal;
use crate::toyota_checksum::apply_toyota_checksum;

/// Number of radar tracks (0..15).
const NUM_TRACKS: usize = 16;

/// Simulated state for one radar track.
#[derive(Clone, Copy, Debug)]
struct TrackState {
    /// Is this track currently valid/active?
    valid: bool,
    /// Longitudinal distance to target [m], range 0..300.
    long_dist: f64,
    /// Lateral distance to target [m], range -50..50.
    lat_dist: f64,
    /// Relative speed [m/s], range -100..100.
    rel_speed: f64,
    /// Relative acceleration (for TRACK_B).
    rel_accel: f64,
    /// Track confidence score 0..100 (for TRACK_B).
    score: f64,
    /// Whether this is a newly acquired track.
    new_track: bool,
}

impl Default for TrackState {
    fn default() -> Self {
        Self {
            valid: false,
            long_dist: 0.0,
            lat_dist: 0.0,
            rel_speed: 0.0,
            rel_accel: 0.0,
            score: 0.0,
            new_track: false,
        }
    }
}

/// Toyota ADAS Radar ECU — produces CAN frames for all DBC messages.
///
/// # Configuration (RON)
/// ```ron
/// config: {
///     "active_tracks": 6,       // How many tracks to simulate as active (default: 6)
///     "base_speed_kph": 100.0,  // Simulated ego vehicle speed in km/h (default: 100)
/// }
/// ```
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct ToyotaRadarEcu {
    /// Per-message counters (0..255 wrapping).
    counters: [u8; DBC_MESSAGE_COUNT],
    /// Index into DBC_MESSAGES for round-robin scheduling.
    msg_index: usize,
    /// Overall simulation tick counter.
    tick: u64,
    /// Simulated radar track states.
    tracks: [TrackState; NUM_TRACKS],
    /// How many tracks are active.
    _active_tracks: usize,
    /// Simulated ego speed (km/h), affects relative speeds.
    _base_speed_kph: f64,
}

impl Freezable for ToyotaRadarEcu {}

impl ToyotaRadarEcu {
    /// Initialize track simulation state with realistic driving scenario.
    fn init_tracks(active: usize, _base_speed: f64) -> [TrackState; NUM_TRACKS] {
        let mut tracks = [TrackState::default(); NUM_TRACKS];

        for i in 0..active.min(NUM_TRACKS) {
            tracks[i] = TrackState {
                valid: true,
                // Distribute cars at various distances: 20m, 45m, 80m, 120m, ...
                long_dist: 20.0 + (i as f64) * 25.0,
                // Spread across lanes: -3m to +3m
                lat_dist: -3.0 + (i as f64) * (6.0 / active.max(1) as f64),
                // Most cars moving at similar speed (small relative speed)
                rel_speed: -2.0 + (i as f64) * 0.8,
                rel_accel: 0.0,
                score: 80.0 + (i as f64) * 2.0,
                new_track: i == 0, // First track starts as "new"
            };
        }

        tracks
    }

    /// Evolve simulation state — called each full cycle through all messages.
    fn tick_simulation(&mut self) {
        let dt = 0.05; // ~50ms per full message cycle (34 msgs at ~1.5ms each)

        for track in self.tracks.iter_mut() {
            if !track.valid {
                continue;
            }

            // Update distance based on relative speed
            track.long_dist += track.rel_speed * dt;
            track.long_dist = track.long_dist.clamp(0.0, 300.0);

            // Slight lateral drift (simulating lane change or curve)
            track.lat_dist += 0.01 * ((self.tick as f64 * 0.1).sin());
            track.lat_dist = track.lat_dist.clamp(-50.0, 50.0);

            // Relative speed slowly changes (traffic flow)
            track.rel_speed += track.rel_accel * dt;
            track.rel_speed = track.rel_speed.clamp(-100.0, 100.0);

            // Small random-ish acceleration changes
            track.rel_accel = 0.5 * ((self.tick as f64 * 0.05 + track.long_dist * 0.01).sin());
            track.rel_accel = track.rel_accel.clamp(-63.0, 63.0);

            // Score evolves based on distance
            track.score = (100.0 - track.long_dist * 0.2).clamp(0.0, 100.0);

            // New track flag clears after a while
            track.new_track = false;

            // If a car gets too close or too far, reset it
            if track.long_dist < 5.0 || track.long_dist > 295.0 {
                track.long_dist = 50.0 + (self.tick as f64 * 0.1 % 200.0);
                track.rel_speed = -1.0;
                track.new_track = true;
            }
        }
    }

    /// Pack a TRACK_A message for track index `track_idx`.
    fn pack_track_a(&self, msg_idx: usize, track_idx: usize, data: &mut [u8]) {
        let msg_def = &DBC_MESSAGES[msg_idx];
        let track = &self.tracks[track_idx];

        for sig in msg_def.signals.iter() {
            if sig.is_counter || sig.is_checksum {
                continue; // Handled separately
            }
            let value = match sig.name {
                "LONG_DIST" => track.long_dist,
                "LAT_DIST" => track.lat_dist,
                "REL_SPEED" => track.rel_speed,
                "VALID" => if track.valid { 1.0 } else { 0.0 },
                "NEW_TRACK" => if track.new_track { 1.0 } else { 0.0 },
                _ => 0.0,
            };
            pack_signal(data, sig, value);
        }
    }

    /// Pack a TRACK_B message for track index `track_idx`.
    fn pack_track_b(&self, msg_idx: usize, track_idx: usize, data: &mut [u8]) {
        let msg_def = &DBC_MESSAGES[msg_idx];
        let track = &self.tracks[track_idx];

        for sig in msg_def.signals.iter() {
            if sig.is_counter || sig.is_checksum {
                continue;
            }
            let value = match sig.name {
                "REL_ACCEL" => track.rel_accel,
                "SCORE" => track.score,
                _ => 0.0,
            };
            pack_signal(data, sig, value);
        }
    }

    /// Pack a NEW_MSG message (generic ADAS signals).
    fn pack_new_msg(&self, msg_idx: usize, data: &mut [u8]) {
        let msg_def = &DBC_MESSAGES[msg_idx];

        for sig in msg_def.signals.iter() {
            if sig.is_counter || sig.is_checksum {
                continue;
            }
            let value = match sig.name {
                // Simulated signal values that change slowly
                "NEW_SIGNAL_1" => ((self.tick as f64 * 0.02).sin() * 50.0 + 60.0).clamp(0.0, 127.0),
                "NEW_SIGNAL_2" => ((self.tick as f64 * 0.03).cos() * 100.0 + 128.0).clamp(0.0, 255.0),
                _ => 0.0,
            };
            pack_signal(data, sig, value);
        }
    }
}

impl CuSrcTask for ToyotaRadarEcu {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CanFrame);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let active_tracks = match config {
            Some(cfg) => cfg.get::<i64>("active_tracks")?.unwrap_or(6) as usize,
            None => 6,
        };
        let base_speed_kph = match config {
            Some(cfg) => cfg.get::<f64>("base_speed_kph")?.unwrap_or(100.0),
            None => 100.0,
        };

        Ok(Self {
            counters: [0u8; DBC_MESSAGE_COUNT],
            msg_index: 0,
            tick: 0,
            tracks: Self::init_tracks(active_tracks, base_speed_kph),
            _active_tracks: active_tracks,
            _base_speed_kph: base_speed_kph,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let idx = self.msg_index;
        let msg_def = &DBC_MESSAGES[idx];
        let mut data = [0u8; 8];

        // Determine which message group this belongs to and pack signals
        match idx {
            0..=15 => self.pack_track_a(idx, idx, &mut data),         // TRACK_A_0..15
            16..=31 => self.pack_track_b(idx, idx - 16, &mut data),   // TRACK_B_0..15
            32..=33 => self.pack_new_msg(idx, &mut data),             // NEW_MSG_1, NEW_MSG_2
            _ => {}
        }

        // Pack COUNTER signal (auto-incrementing)
        if let Some(counter_sig) = msg_def.signals.iter().find(|s| s.is_counter) {
            pack_signal(&mut data, counter_sig, self.counters[idx] as f64);
            self.counters[idx] = self.counters[idx].wrapping_add(1);
        }

        // Compute and apply Toyota checksum (always in last byte for Toyota)
        if msg_def.signals.iter().any(|s| s.is_checksum) {
            apply_toyota_checksum(msg_def.msg_id, &mut data);
        }

        // Build the CAN frame
        let frame = CanFrame {
            id: CanId::Standard(msg_def.msg_id as u16),
            dlc: msg_def.dlc,
            flags: cu_automotive_payloads::can::CanFlags::NONE,
            data: {
                let mut d = [0u8; 8];
                d[..msg_def.dlc as usize].copy_from_slice(&data[..msg_def.dlc as usize]);
                d
            },
        };

        output.set_payload(frame);
        output.tov = Tov::Time(ctx.clock.now());

        // Advance round-robin index
        self.msg_index = (self.msg_index + 1) % DBC_MESSAGE_COUNT;

        // Every full cycle, evolve the simulation
        if self.msg_index == 0 {
            self.tick += 1;
            self.tick_simulation();
        }

        Ok(())
    }
}
