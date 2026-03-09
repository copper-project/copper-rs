//! SOME/IP Service Discovery (SD) Monitor.
//!
//! Listens for SOME/IP-SD multicast announcements (OfferService, StopOffer)
//! and tracks which services are available on the network.

use cu29::prelude::*;
use cu_automotive_payloads::someip::{
    SdEntryType, SdServiceEntry, SomeIpMessage, SomeIpServiceStatus,
};

/// Maximum tracked services.
const MAX_TRACKED_SERVICES: usize = 32;

/// Known service availability record.
#[derive(Clone, Copy, Debug, Default)]
struct TrackedService {
    service_id: u16,
    instance_id: u16,
    available: bool,
    ttl_remaining: u32,
    #[allow(dead_code)]
    active: bool,
}

/// SOME/IP Service Discovery Monitor.
///
/// Receives SOME/IP messages (typically from a SomeIpSource bound to the
/// SD multicast group), parses SD entries, and outputs service availability
/// status changes.
///
/// This is modeled as a CuTask: Input = raw SOME/IP messages, Output = service status.
///
/// # Config
/// - `sd_service_id` (i64): SD service ID. Default: `0xFFFF`.
/// - `sd_method_id` (i64): SD method ID. Default: `0x8100`.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SomeIpSdMonitor {
    tracked: [TrackedService; MAX_TRACKED_SERVICES],
    tracked_count: usize,
    sd_service_id: u16,
    sd_method_id: u16,
}

impl Freezable for SomeIpSdMonitor {}

impl SomeIpSdMonitor {
    /// Parse SD entries from the SOME/IP payload.
    /// This is a simplified parser for OfferService entries.
    fn parse_sd_entries(payload: &[u8]) -> alloc::vec::Vec<SdServiceEntry> {
        let mut entries = alloc::vec::Vec::new();
        if payload.len() < 12 {
            return entries;
        }
        // SD payload format:
        // [0..3] flags (reboot, unicast)
        // [4..7] length of entries array
        let entries_len = u32::from_be_bytes([
            payload[4], payload[5], payload[6], payload[7],
        ]) as usize;

        let entry_data = &payload[8..];
        let mut offset = 0;
        while offset + 16 <= entry_data.len().min(8 + entries_len) {
            let entry_type_byte = entry_data[offset];
            let service_id =
                u16::from_be_bytes([entry_data[offset + 4], entry_data[offset + 5]]);
            let instance_id =
                u16::from_be_bytes([entry_data[offset + 6], entry_data[offset + 7]]);
            let major_version = entry_data[offset + 8];
            let ttl = u32::from_be_bytes([
                0,
                entry_data[offset + 9],
                entry_data[offset + 10],
                entry_data[offset + 11],
            ]);
            let minor_version = u32::from_be_bytes([
                entry_data[offset + 12],
                entry_data[offset + 13],
                entry_data[offset + 14],
                entry_data[offset + 15],
            ]);

            let entry_type = match entry_type_byte {
                0x00 => SdEntryType::FindService,
                0x01 => {
                    if ttl > 0 {
                        SdEntryType::OfferService
                    } else {
                        SdEntryType::StopOfferService
                    }
                }
                0x06 => SdEntryType::SubscribeEventgroup,
                0x07 => SdEntryType::SubscribeEventgroupAck,
                _ => SdEntryType::FindService,
            };

            entries.push(SdServiceEntry {
                entry_type,
                service_id,
                instance_id,
                major_version,
                minor_version,
                ttl,
            });
            offset += 16;
        }
        entries
    }

    /// Update tracked service table with an SD entry.
    fn update_tracked(&mut self, entry: &SdServiceEntry) -> Option<SomeIpServiceStatus> {
        let available = matches!(entry.entry_type, SdEntryType::OfferService);

        // Look for existing entry
        for i in 0..self.tracked_count {
            if self.tracked[i].service_id == entry.service_id
                && self.tracked[i].instance_id == entry.instance_id
            {
                let changed = self.tracked[i].available != available;
                self.tracked[i].available = available;
                self.tracked[i].ttl_remaining = entry.ttl;
                if changed {
                    return Some(SomeIpServiceStatus {
                        service_id: entry.service_id,
                        instance_id: entry.instance_id,
                        available,
                    });
                }
                return None;
            }
        }

        // New entry
        if self.tracked_count < MAX_TRACKED_SERVICES {
            self.tracked[self.tracked_count] = TrackedService {
                service_id: entry.service_id,
                instance_id: entry.instance_id,
                available,
                ttl_remaining: entry.ttl,
                active: true,
            };
            self.tracked_count += 1;
            return Some(SomeIpServiceStatus {
                service_id: entry.service_id,
                instance_id: entry.instance_id,
                available,
            });
        }
        None
    }
}

impl CuTask for SomeIpSdMonitor {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(SomeIpMessage);
    type Output<'m> = output_msg!(SomeIpServiceStatus);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let sd_service_id = match config {
            Some(cfg) => cfg.get::<i64>("sd_service_id")?.unwrap_or(0xFFFF) as u16,
            None => 0xFFFF,
        };
        let sd_method_id = match config {
            Some(cfg) => cfg.get::<i64>("sd_method_id")?.unwrap_or(0x8100) as u16,
            None => 0x8100,
        };

        Ok(Self {
            tracked: [TrackedService::default(); MAX_TRACKED_SERVICES],
            tracked_count: 0,
            sd_service_id,
            sd_method_id,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if let Some(msg) = input.payload() {
            // Only process SD messages
            if msg.header.service_id != self.sd_service_id
                || msg.header.method_id != self.sd_method_id
            {
                return Ok(());
            }

            let entries = Self::parse_sd_entries(msg.payload_data());
            for entry in &entries {
                if let Some(status) = self.update_tracked(entry) {
                    output.set_payload(status);
                    output.tov = Tov::Time(ctx.now());
                    // Note: only outputs the last status change per cycle.
                    // For batch, we'd need a batch output type.
                }
            }
        }
        Ok(())
    }
}
