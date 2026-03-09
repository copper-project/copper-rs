//! CAN Bus Spy — decodes and logs CAN frames using DBC signal definitions.
//!
//! This `CuSinkTask` receives CAN frames, looks up the message definition from the compiled-in
//! DBC database, unpacks all signals to physical values, and prints a human-readable summary.
//! Validates Toyota checksums and counter continuity.

use cu29::prelude::*;
use cu_automotive_payloads::can::CanFrame;

use crate::dbc_generated::{DBC_MESSAGES, DBC_MESSAGE_COUNT, DbcMessageDef};
use crate::signal_pack::unpack_signal;
use crate::toyota_checksum::toyota_checksum;

/// CAN Bus Spy — decodes all received CAN frames using the DBC database.
///
/// # Configuration (RON)
/// ```ron
/// config: {
///     "verbose": true,       // Print every frame (default: false, prints summary every N frames)
///     "summary_interval": 34, // Print summary every N frames (default: 34 = one full cycle)
/// }
/// ```
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CanBusSpy {
    /// Total frames received.
    frame_count: u64,
    /// Frames with checksum errors.
    checksum_errors: u64,
    /// Frames with counter discontinuities.
    counter_errors: u64,
    /// Last seen counter value per message index.
    last_counters: [Option<u8>; DBC_MESSAGE_COUNT],
    /// Whether to print every frame.
    verbose: bool,
    /// Print summary every N frames.
    summary_interval: u64,
    /// Last decoded values for summary display (one f64 per signal, flattened).
    /// We store up to 10 signals per message × 34 messages = 340 slots.
    last_values: [f64; 340],
}

impl Freezable for CanBusSpy {}

impl CanBusSpy {
    /// Find the DBC message definition matching a CAN ID.
    fn find_message(can_id: u32) -> Option<(usize, &'static DbcMessageDef)> {
        DBC_MESSAGES
            .iter()
            .enumerate()
            .find(|(_, m)| m.msg_id == can_id)
    }

    /// Print a decoded frame.
    fn print_frame(msg_def: &DbcMessageDef, data: &[u8], cksum_ok: bool, counter_ok: bool) {
        let status = match (cksum_ok, counter_ok) {
            (true, true) => "OK",
            (false, _) => "CKSUM_ERR",
            (_, false) => "CTR_ERR",
        };

        let mut sig_strs = Vec::new();
        for sig in msg_def.signals.iter() {
            if sig.is_checksum || sig.is_counter {
                continue;
            }
            let val = unpack_signal(data, sig);
            if sig.unit.is_empty() {
                sig_strs.push(format!("{}={:.2}", sig.name, val));
            } else {
                sig_strs.push(format!("{}={:.2}{}", sig.name, val, sig.unit));
            }
        }

        println!(
            "[CAN] 0x{:03X} {:16} [{}] {}",
            msg_def.msg_id,
            msg_def.name,
            status,
            sig_strs.join(" "),
        );
    }

    /// Print a summary of all tracked signals.
    fn print_summary(&self) {
        println!("╔══════════════════════════════════════════════════════════════════╗");
        println!("║  Vehicle CAN Bus Summary — {} frames received                ║",
               self.frame_count);
        println!("║  Checksum errors: {}  Counter errors: {}                      ║",
               self.checksum_errors, self.counter_errors);
        println!("╠══════════════════════════════════════════════════════════════════╣");

        // Print a few interesting tracks
        for track_idx in 0..4 {
            if let Some(msg_def) = DBC_MESSAGES.get(track_idx) {
                let base = track_idx * 10;
                let mut sig_vals = Vec::new();
                for (i, sig) in msg_def.signals.iter().enumerate() {
                    if sig.is_checksum || sig.is_counter {
                        continue;
                    }
                    if base + i < self.last_values.len() {
                        let val = self.last_values[base + i];
                        if sig.unit.is_empty() {
                            sig_vals.push(format!("{}={:.1}", sig.name, val));
                        } else {
                            sig_vals.push(format!("{}={:.1}{}", sig.name, val, sig.unit));
                        }
                    }
                }
                println!("║ {:12} │ {}", msg_def.name, sig_vals.join(" "));
            }
        }

        println!("╚══════════════════════════════════════════════════════════════════╝");
    }
}

impl CuSinkTask for CanBusSpy {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CanFrame);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let verbose = match config {
            Some(cfg) => cfg.get::<bool>("verbose")?.unwrap_or(false),
            None => false,
        };
        let summary_interval = match config {
            Some(cfg) => cfg.get::<i64>("summary_interval")?.unwrap_or(34) as u64,
            None => 34,
        };

        Ok(Self {
            frame_count: 0,
            checksum_errors: 0,
            counter_errors: 0,
            last_counters: [None; DBC_MESSAGE_COUNT],
            verbose,
            summary_interval,
            last_values: [0.0; 340],
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let frame = match input.payload() {
            Some(f) => f,
            None => return Ok(()),
        };

        let can_id = match frame.id {
            cu_automotive_payloads::can::CanId::Standard(id) => id as u32,
            cu_automotive_payloads::can::CanId::Extended(id) => id,
        };

        let data = &frame.data[..frame.dlc as usize];

        // Look up message in DBC database
        let (msg_idx, msg_def) = match Self::find_message(can_id) {
            Some(found) => found,
            None => {
                if self.verbose {
                    println!("[CAN] 0x{:03X} UNKNOWN {} bytes", can_id, frame.dlc);
                }
                self.frame_count += 1;
                return Ok(());
            }
        };

        // Validate Toyota checksum
        let cksum_ok = if msg_def.signals.iter().any(|s| s.is_checksum) {
            let expected = toyota_checksum(msg_def.msg_id, data);
            let actual = data[data.len() - 1];
            if expected != actual {
                self.checksum_errors += 1;
                false
            } else {
                true
            }
        } else {
            true
        };

        // Validate counter continuity
        let counter_ok = if let Some(counter_sig) = msg_def.signals.iter().find(|s| s.is_counter) {
            let raw = unpack_signal(data, counter_sig) as u8;
            let ok = match self.last_counters[msg_idx] {
                Some(last) => raw == last.wrapping_add(1),
                None => true, // First frame, no reference
            };
            self.last_counters[msg_idx] = Some(raw);
            if !ok {
                self.counter_errors += 1;
            }
            ok
        } else {
            true
        };

        // Store decoded signal values
        let base = msg_idx * 10;
        for (i, sig) in msg_def.signals.iter().enumerate() {
            if base + i < self.last_values.len() {
                self.last_values[base + i] = unpack_signal(data, sig);
            }
        }

        // Print frame or summary
        if self.verbose {
            Self::print_frame(msg_def, data, cksum_ok, counter_ok);
        }

        self.frame_count += 1;

        if self.summary_interval > 0 && self.frame_count % self.summary_interval == 0 {
            self.print_summary();
        }

        Ok(())
    }
}
