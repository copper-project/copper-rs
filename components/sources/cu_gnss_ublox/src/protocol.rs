use alloc::string::String;
use alloc::vec::Vec;

use cu_gnss_payloads::{
    GnssAccuracy, GnssAckKind, GnssCommandAck, GnssEpochTime, GnssEvent, GnssFixSolution,
    GnssFixType, GnssInfoSeverity, GnssInfoText, GnssNavEpoch, GnssRawUbxFrame, GnssRfBlockStatus,
    GnssRfStatus, GnssSatelliteInfo, GnssSatelliteState, GnssSignalInfo, GnssSignalState,
};
use uom::si::angle::degree;
use uom::si::f32::{Angle, Length, Time, Velocity};
use uom::si::length::meter;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

const UBX_SYNC_1: u8 = 0xB5;
const UBX_SYNC_2: u8 = 0x62;

const CLASS_ACK: u8 = 0x05;
const CLASS_INF: u8 = 0x04;
const CLASS_MON: u8 = 0x0A;
const CLASS_NAV: u8 = 0x01;

const ID_ACK_NAK: u8 = 0x00;
const ID_ACK_ACK: u8 = 0x01;

const ID_NAV_PVT: u8 = 0x07;
const ID_NAV_SAT: u8 = 0x35;
const ID_NAV_SIG: u8 = 0x43;
const ID_MON_RF: u8 = 0x38;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct UbxFrame {
    pub class_id: u8,
    pub msg_id: u8,
    pub payload: Vec<u8>,
}

impl UbxFrame {
    pub fn from_message(class_id: u8, msg_id: u8, payload: &[u8]) -> Self {
        Self {
            class_id,
            msg_id,
            payload: payload.to_vec(),
        }
    }

    pub fn to_wire_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.payload.len() + 8);
        out.push(UBX_SYNC_1);
        out.push(UBX_SYNC_2);
        out.push(self.class_id);
        out.push(self.msg_id);
        let len = self.payload.len() as u16;
        out.extend_from_slice(&len.to_le_bytes());
        out.extend_from_slice(&self.payload);
        let (ck_a, ck_b) = checksum(&out[2..]);
        out.push(ck_a);
        out.push(ck_b);
        out
    }
}

pub fn extract_next_ubx_frame(buffer: &mut Vec<u8>) -> Option<UbxFrame> {
    loop {
        let sync_pos = match buffer
            .windows(2)
            .position(|w| w[0] == UBX_SYNC_1 && w[1] == UBX_SYNC_2)
        {
            Some(pos) => pos,
            None => {
                if buffer.last().copied() == Some(UBX_SYNC_1) {
                    buffer.drain(0..buffer.len().saturating_sub(1));
                } else {
                    buffer.clear();
                }
                return None;
            }
        };

        if sync_pos > 0 {
            buffer.drain(0..sync_pos);
        }

        if buffer.len() < 8 {
            return None;
        }

        let payload_len = le_u16(buffer, 4) as usize;
        let frame_len = payload_len + 8;
        if buffer.len() < frame_len {
            return None;
        }

        let class_id = buffer[2];
        let msg_id = buffer[3];
        let payload_end = 6 + payload_len;
        let expected_ck_a = buffer[payload_end];
        let expected_ck_b = buffer[payload_end + 1];
        let (actual_ck_a, actual_ck_b) = checksum(&buffer[2..payload_end]);

        if expected_ck_a != actual_ck_a || expected_ck_b != actual_ck_b {
            buffer.drain(0..1);
            continue;
        }

        let payload = buffer[6..payload_end].to_vec();
        buffer.drain(0..frame_len);
        return Some(UbxFrame {
            class_id,
            msg_id,
            payload,
        });
    }
}

pub fn decode_frame_to_event(frame: &UbxFrame, emit_raw_unknown: bool) -> Option<GnssEvent> {
    let event = match (frame.class_id, frame.msg_id) {
        (CLASS_NAV, ID_NAV_PVT) => decode_nav_pvt(&frame.payload).map(GnssEvent::NavEpoch),
        (CLASS_NAV, ID_NAV_SAT) => decode_nav_sat(&frame.payload).map(GnssEvent::SatelliteState),
        (CLASS_NAV, ID_NAV_SIG) => decode_nav_sig(&frame.payload).map(GnssEvent::SignalState),
        (CLASS_MON, ID_MON_RF) => decode_mon_rf(&frame.payload).map(GnssEvent::RfStatus),
        (CLASS_INF, msg_id) => Some(GnssEvent::InfoText(decode_info(msg_id, &frame.payload))),
        (CLASS_ACK, ID_ACK_ACK) => decode_ack(GnssAckKind::Ack, &frame.payload),
        (CLASS_ACK, ID_ACK_NAK) => decode_ack(GnssAckKind::Nak, &frame.payload),
        _ => None,
    };

    if event.is_some() {
        return event;
    }

    if emit_raw_unknown {
        return Some(GnssEvent::RawUbx(GnssRawUbxFrame {
            class_id: frame.class_id,
            msg_id: frame.msg_id,
            payload: frame.payload.clone(),
        }));
    }

    None
}

fn decode_ack(kind: GnssAckKind, payload: &[u8]) -> Option<GnssEvent> {
    if payload.len() < 2 {
        return None;
    }
    Some(GnssEvent::CommandAck(GnssCommandAck {
        kind,
        class_id: payload[0],
        msg_id: payload[1],
    }))
}

fn decode_info(msg_id: u8, payload: &[u8]) -> GnssInfoText {
    let severity = match msg_id {
        0x00 => GnssInfoSeverity::Error,
        0x01 => GnssInfoSeverity::Warning,
        0x02 => GnssInfoSeverity::Notice,
        0x03 => GnssInfoSeverity::Test,
        0x04 => GnssInfoSeverity::Debug,
        _ => GnssInfoSeverity::Notice,
    };
    let text = String::from_utf8_lossy(payload)
        .trim_end_matches('\0')
        .to_string();
    GnssInfoText { severity, text }
}

fn decode_nav_pvt(payload: &[u8]) -> Option<GnssNavEpoch> {
    if payload.len() < 92 {
        return None;
    }

    let valid = payload[11];
    let flags = payload[21];
    let flags3 = le_u16(payload, 78);

    let time = GnssEpochTime {
        itow_ms: le_u32(payload, 0),
        year: le_u16(payload, 4),
        month: payload[6],
        day: payload[7],
        hour: payload[8],
        minute: payload[9],
        second: payload[10],
        valid_date: (valid & 0x01) != 0,
        valid_time: (valid & 0x02) != 0,
        fully_resolved: (valid & 0x04) != 0,
        valid_magnetic_declination: (valid & 0x08) != 0,
    };

    let head_motion_deg = le_i32(payload, 64) as f32 * 1e-5;
    let head_vehicle_deg = le_i32(payload, 84) as f32 * 1e-5;

    let fix = GnssFixSolution {
        fix_type: GnssFixType::from(payload[20]),
        gnss_fix_ok: (flags & 0x01) != 0,
        differential_solution: (flags & 0x02) != 0,
        carrier_solution: (flags >> 6) & 0x03,
        invalid_llh: (flags3 & 0x0001) != 0,
        num_satellites_used: payload[23],
        longitude: Angle::new::<degree>(le_i32(payload, 24) as f32 * 1e-7),
        latitude: Angle::new::<degree>(le_i32(payload, 28) as f32 * 1e-7),
        height_ellipsoid: Length::new::<meter>(le_i32(payload, 32) as f32 / 1000.0),
        height_msl: Length::new::<meter>(le_i32(payload, 36) as f32 / 1000.0),
        velocity_north: Velocity::new::<meter_per_second>(le_i32(payload, 48) as f32 / 1000.0),
        velocity_east: Velocity::new::<meter_per_second>(le_i32(payload, 52) as f32 / 1000.0),
        velocity_down: Velocity::new::<meter_per_second>(le_i32(payload, 56) as f32 / 1000.0),
        ground_speed: Velocity::new::<meter_per_second>(le_i32(payload, 60) as f32 / 1000.0),
        heading_motion: Angle::new::<degree>(head_motion_deg),
        heading_vehicle: Angle::new::<degree>(head_vehicle_deg),
        magnetic_declination: Angle::new::<degree>(le_i16(payload, 88) as f32 * 1e-2),
        psm_state: (flags >> 2) & 0x07,
        correction_age_bucket: ((flags3 >> 1) & 0x0F) as u8,
    };

    let accuracy = GnssAccuracy {
        horizontal: Length::new::<meter>(le_u32(payload, 40) as f32 / 1000.0),
        vertical: Length::new::<meter>(le_u32(payload, 44) as f32 / 1000.0),
        speed: Velocity::new::<meter_per_second>(le_u32(payload, 68) as f32 / 1000.0),
        heading: Angle::new::<degree>(le_u32(payload, 72) as f32 * 1e-5),
        time: Time::new::<second>(le_u32(payload, 12) as f32 * 1e-9),
        position_dop: le_u16(payload, 76) as f32 * 0.01,
    };

    Some(GnssNavEpoch {
        time,
        fix,
        accuracy,
    })
}

fn decode_nav_sat(payload: &[u8]) -> Option<GnssSatelliteState> {
    if payload.len() < 8 {
        return None;
    }

    let itow_ms = le_u32(payload, 0);
    let reported = payload[5] as usize;
    let available = (payload.len().saturating_sub(8)) / 12;
    let count = reported.min(available);

    let mut satellites = Vec::with_capacity(count);
    for idx in 0..count {
        let base = 8 + idx * 12;
        let flags = le_u32(payload, base + 8);

        satellites.push(GnssSatelliteInfo {
            gnss_id: payload[base],
            sv_id: payload[base + 1],
            cno_dbhz: payload[base + 2],
            elevation_deg: payload[base + 3] as i8,
            azimuth_deg: le_i16(payload, base + 4),
            pseudorange_residual_dm: le_i16(payload, base + 6),
            quality_ind: (flags & 0x07) as u8,
            used_for_navigation: (flags & (1 << 3)) != 0,
            health: ((flags >> 4) & 0x03) as u8,
            differential_correction_available: (flags & (1 << 6)) != 0,
            pseudorange_smoothed: (flags & (1 << 7)) != 0,
            orbit_source: ((flags >> 8) & 0x07) as u8,
            sbas_corr_used: (flags & (1 << 16)) != 0,
            rtcm_corr_used: (flags & (1 << 17)) != 0,
            slas_corr_used: (flags & (1 << 18)) != 0,
            spartn_corr_used: (flags & (1 << 19)) != 0,
        });
    }

    Some(GnssSatelliteState {
        itow_ms,
        satellites,
    })
}

fn decode_nav_sig(payload: &[u8]) -> Option<GnssSignalState> {
    if payload.len() < 8 {
        return None;
    }

    let itow_ms = le_u32(payload, 0);
    let reported = payload[5] as usize;
    let available = (payload.len().saturating_sub(8)) / 16;
    let count = reported.min(available);

    let mut signals = Vec::with_capacity(count);
    for idx in 0..count {
        let base = 8 + idx * 16;
        let sig_flags = le_u16(payload, base + 10);

        signals.push(GnssSignalInfo {
            gnss_id: payload[base],
            sv_id: payload[base + 1],
            signal_id: payload[base + 2],
            frequency_id: payload[base + 3],
            pseudorange_residual_dm: le_i16(payload, base + 4),
            cno_dbhz: payload[base + 6],
            quality_ind: payload[base + 7],
            correction_source: payload[base + 8],
            iono_model: payload[base + 9],
            health: (sig_flags & 0x03) as u8,
            pseudorange_smoothed: (sig_flags & (1 << 2)) != 0,
            pseudorange_used: (sig_flags & (1 << 3)) != 0,
            carrier_used: (sig_flags & (1 << 4)) != 0,
            doppler_used: (sig_flags & (1 << 5)) != 0,
            pseudorange_correction_used: (sig_flags & (1 << 6)) != 0,
            carrier_correction_used: (sig_flags & (1 << 7)) != 0,
            doppler_correction_used: (sig_flags & (1 << 8)) != 0,
        });
    }

    Some(GnssSignalState { itow_ms, signals })
}

fn decode_mon_rf(payload: &[u8]) -> Option<GnssRfStatus> {
    if payload.len() < 4 {
        return None;
    }

    let n_blocks = payload[1] as usize;
    let available = (payload.len().saturating_sub(4)) / 24;
    let count = n_blocks.min(available);

    let mut blocks = Vec::with_capacity(count);
    for idx in 0..count {
        let base = 4 + idx * 24;
        let flags = payload[base + 1];
        blocks.push(GnssRfBlockStatus {
            block_id: payload[base],
            jamming_state: flags & 0x03,
            antenna_status: payload[base + 2],
            antenna_power: payload[base + 3],
            post_status: le_u32(payload, base + 4),
            noise_per_ms: le_u16(payload, base + 12),
            agc_count: le_u16(payload, base + 14),
            cw_jam_indicator: payload[base + 16],
            i_imbalance: payload[base + 17] as i8,
            i_magnitude: payload[base + 18],
            q_imbalance: payload[base + 19] as i8,
            q_magnitude: payload[base + 20],
        });
    }

    Some(GnssRfStatus { blocks })
}

fn checksum(payload: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for b in payload {
        ck_a = ck_a.wrapping_add(*b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

#[inline]
fn le_u16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

#[inline]
fn le_i16(bytes: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

#[inline]
fn le_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

#[inline]
fn le_i32(bytes: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

#[cfg(test)]
mod tests {
    use super::{UbxFrame, extract_next_ubx_frame};

    #[test]
    fn drops_full_noise_buffer_when_no_sync_is_present() {
        let mut buffer = vec![0x00, 0x11, 0x22, 0x33, 0x44];
        assert!(extract_next_ubx_frame(&mut buffer).is_none());
        assert!(buffer.is_empty());
    }

    #[test]
    fn keeps_trailing_sync_lead_byte_for_cross_read_reassembly() {
        let frame = UbxFrame::from_message(0x01, 0x07, &[]);
        let wire = frame.to_wire_bytes();

        let mut buffer = vec![0xAA, 0xBB, 0xB5];
        assert!(extract_next_ubx_frame(&mut buffer).is_none());
        assert_eq!(buffer, vec![0xB5]);

        buffer.extend_from_slice(&wire[1..]);

        let parsed = extract_next_ubx_frame(&mut buffer).expect("expected reconstructed frame");
        assert_eq!(parsed.class_id, 0x01);
        assert_eq!(parsed.msg_id, 0x07);
        assert!(parsed.payload.is_empty());
        assert!(buffer.is_empty());
    }
}
