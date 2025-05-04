use bytemuck::{Pod, Zeroable};
use chrono::{DateTime, MappedLocalTime, TimeZone, Utc};
use cu29::prelude::{CuDuration, CuTime};
use std::error::Error;
use std::fmt;
use std::fmt::{Debug, Formatter};
use std::mem::size_of;
use uom::fmt::DisplayStyle::Abbreviation;
use uom::num_traits::ToPrimitive;
use uom::si::angle::degree;
use uom::si::angular_velocity::revolution_per_minute;
use uom::si::f32::{Angle, Ratio};
use uom::si::f32::{AngularVelocity, Length};
use uom::si::length::millimeter;
use uom::si::ratio::{percent, ratio};
use uom::si::Unit;
use uom::ConversionFactor;

#[derive(Debug)]
pub enum HesaiError {
    InvalidPacket(String),
    InvalidTimestamp(String),
}

impl fmt::Display for HesaiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HesaiError::InvalidPacket(msg) => write!(f, "Invalid packet: {msg}"),
            HesaiError::InvalidTimestamp(msg) => write!(f, "Invalid timestamp: {msg}"),
        }
    }
}

impl Error for HesaiError {}

// ╭──────────────────────────────────────────────────────────────────────────────╮
// │                              Pre-Header (6 bytes)                            │
// ├──────────────────────────────┬─────────┬─────────────────────────────────────┤
// │ Field                        │ Bytes   │ Description                         │
// ├──────────────────────────────┼─────────┼─────────────────────────────────────┤
// │ 0xEE (SOP)                   │ 1       │ Start of packet (constant: 0xEE)    │
// │ 0xFF (SOP)                   │ 1       │ Start of packet (constant: 0xFF)    │
// │ Protocol Version Major       │ 1       │ PandarXT series uses 0x06           │
// │ Protocol Version Minor       │ 1       │ Current protocol version (0x01)     │
// │ Reserved                     │ 2       │ Reserved bytes                      │
// ╰──────────────────────────────┴─────────┴─────────────────────────────────────╯
#[repr(C, packed)]
#[derive(Copy, Clone, Zeroable, Pod)]
pub struct PreHeader {
    sop1: u8,
    sop2: u8,
    protocol_version_major: u8,
    protocol_version_minor: u8,
    reserved: [u8; 2],
}

impl Debug for PreHeader {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("Magic: {:2X}{:2X}", self.sop1, self.sop2))?;
        f.write_fmt(format_args!(
            "\nVersion {}.{}",
            self.protocol_version_major, self.protocol_version_minor
        ))
    }
}

// ╭──────────────────────────────────────────────────────────────────────────────╮
// │                                 Header (6 bytes)                             │
// ├──────────────────────────────┬─────────┬─────────────────────────────────────┤
// │ Field                        │ Bytes   │ Description                         │
// ├──────────────────────────────┼─────────┼─────────────────────────────────────┤
// │ Laser Num                    │ 1       │ Constant 0x20 (32 channels)         │
// │ Block Num                    │ 1       │ Constant 0x08 (8 blocks per packet) │
// │ First Block Return           │ 1       │ 0x00 = Single Return                │
// │                              │         │ 0x01 = Last Return in Dual Return   │
// │ Dis Unit                     │ 1       │ Constant 0x04 (4 mm)                │
// │ Return Number                │ 1       │ 0x01 = One return (max)             │
// │                              │         │ 0x02 = Two returns (max             │
// │ UDP Seq                      │ 1       │ Always 0x01 for PandarXT            │
// ╰──────────────────────────────┴─────────┴─────────────────────────────────────╯
#[repr(C, packed)]
#[derive(Copy, Clone, Zeroable, Pod)]
pub struct Header {
    laser_num: u8,
    block_num: u8,
    first_block_return: u8,
    dis_unit: u8,
    return_number: u8,
    udp_seq: u8,
}

impl Header {
    pub fn is_dual_return(self) -> bool {
        self.return_number == 1
    }

    pub fn distance_unit(self) -> Length {
        Length::new::<millimeter>(self.dis_unit as f32)
    }

    pub fn check_invariants(self) -> Result<(), HesaiError> {
        if self.laser_num != 0x20 {
            return Err(HesaiError::InvalidPacket(format!(
                "Invalid laser number: 0x{:x}",
                self.laser_num
            )));
        }
        if self.block_num != 0x08 {
            return Err(HesaiError::InvalidPacket(format!(
                "Invalid block number: 0x{:x}",
                self.block_num
            )));
        }
        if self.dis_unit != 0x04 {
            return Err(HesaiError::InvalidPacket(format!(
                "Invalid distance unit: 0x{:x}",
                self.dis_unit
            )));
        }
        if self.udp_seq != 0x01 {
            return Err(HesaiError::InvalidPacket(format!(
                "Invalid UDP sequence: 0x{:x}",
                self.udp_seq
            )));
        }
        Ok(())
    }
}

impl Debug for Header {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Laser num: {:02x}", self.laser_num)?;
        writeln!(f, "Block num: {:02x}", self.block_num)?;
        writeln!(f, "First Block return: {}", self.is_dual_return())?;
        writeln!(
            f,
            "Distance unit: {}",
            self.distance_unit()
                .into_format_args(millimeter, Abbreviation)
        )?;
        writeln!(f, "UDP Seq: {}", self.udp_seq)
    }
}

// Body Block Definition (each block of 130 bytes)
//
// ╭──────────────────────────────────────────────────────────────────────────────╮
// │                             Body (1040 bytes)                                │
// │                            (8 blocks, 130 bytes each)                        │
// ├──────────────────────────────┬─────────┬─────────────────────────────────────┤
// │ Field                        │ Bytes   │ Description                         │
// ├──────────────────────────────┼─────────┼─────────────────────────────────────┤
// │ Azimuth                      │ 2       │ Azimuth angle (in hundredths of a   │
// │                              │         │ degree, little-endian)              │
// │ Channels 1 to 32             │ 128     │ Distance (2 bytes), Reflectivity    │
// │                              │         │ (1 byte), Reserved (1 byte)         │
// ├──────────────────────────────┴─────────┴─────────────────────────────────────┤
// │ Notes:                                                                       │
// │ - Each block consists of 130 bytes, where Azimuth data and Channels are      │
// │   stored. The Distance value must be multiplied by 4 to get mm.              │
// │ - Reflectivity is stored as a 1-byte percentage (0-255).                     │
// ╰──────────────────────────────────────────────────────────────────────────────╯
#[repr(C, packed)]
#[derive(Copy, Clone, Zeroable, Pod)]
pub struct Block {
    azimuth: u16,                // Azimuth Angle
    pub channels: [Channel; 32], // 32 channels per block
}

impl Block {
    pub fn azimuth(&self) -> Angle {
        // it is in 100th of degrees.
        Angle::new::<degree>(self.azimuth as f32 / 100.0)
    }

    pub fn check_invariants(self) -> Result<(), HesaiError> {
        if self.azimuth > 36000 {
            return Err(HesaiError::InvalidPacket(format!(
                "Invalid azimuth: {}",
                self.azimuth().into_format_args(degree, Abbreviation)
            )));
        }
        for channel in self.channels.iter() {
            channel.check_invariants()?;
        }
        Ok(())
    }
}

impl Debug for Block {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Azimuth: {:>06.2}{}",
            self.azimuth().get::<degree>().value().to_f64().unwrap(),
            degree::abbreviation()
        )?;
        writeln!(f, "Channels:\n{:?}", self.channels)
    }
}

// Channel Definition
//
// ╭──────────────────────────────────────────────────────────────────────────────╮
// │                             Channel (4 bytes)                                │
// ├──────────────────────────────┬─────────┬─────────────────────────────────────┤
// │ Field                        │ Bytes   │ Description                         │
// ├──────────────────────────────┼─────────┼─────────────────────────────────────┤
// │ Distance                     │ 2       │ Distance / 4mm (little-endian)      │
// │ Reflectivity                 │ 1       │ Reflectivity in percentage          │
// │ Reserved                     │ 1       │ Reserved byte                       │
// ╰──────────────────────────────┴─────────┴─────────────────────────────────────╯
//
#[repr(C, packed)]
#[derive(Copy, Clone, Zeroable, Pod)]
pub struct Channel {
    distance: u16,    // !! raw endianness
    reflectivity: u8, // Reflectivity in percentage
    reserved: u8,     // Reserved byte
}

impl Channel {
    pub fn distance(&self) -> Length {
        Length::new::<millimeter>(u16_endianness(self.distance) as f32 * 4.0) // unharcode 4mm if we port this to another sensor.
    }
    pub fn reflectivity(&self) -> Ratio {
        Ratio::new::<ratio>(self.reflectivity as f32 / 255.0)
    }
    pub fn check_invariants(&self) -> Result<(), HesaiError> {
        // TODO: determine the valid range for distance
        Ok(())
    }
}

impl Debug for Channel {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Distance: {}",
            self.distance().into_format_args(millimeter, Abbreviation)
        )?;
        writeln!(
            f,
            "Reflectivity: {:>06.5}",
            self.reflectivity().into_format_args(percent, Abbreviation)
        )
    }
}

// Tail Definition
//
// ╭──────────────────────────────────────────────────────────────────────────────╮
// │                                  Tail (24 bytes)                             │
// ├──────────────────────────────┬─────────┬─────────────────────────────────────┤
// │ Field                        │ Bytes   │ Description                         │
// ├──────────────────────────────┼─────────┼─────────────────────────────────────┤
// │ Reserved                     │ 10      │ Reserved bytes                      │
// │ Return Mode                  │ 1       │ 0x37 = Strongest Return             │
// │                              │         │ 0x38 = Last Return                  │
// │                              │         │ 0x39 = Dual Return                  │
// │ High Temp Shutdown Flag      │ 1       │ 0x01 = High temp, 0x00 = Normal     │
// │ Motor Speed                  │ 2       │ Motor speed in RPM                  │
// │ Date & Time                  │ 6       │ Timestamp (year, month, day, etc.)  │
// │ Timestamp (µs)               │ 4       │ Timestamp in microseconds           │
// │ Factory Info                 │ 1       │ Factory-specific info  (0x42)       │
// ╰──────────────────────────────┴─────────┴─────────────────────────────────────╯

#[repr(C, packed)]
#[derive(Copy, Clone, Zeroable, Pod)]
pub struct Tail {
    reserved: [u8; 10],
    return_mode: u8,
    motor_speed: u16, // !! raw endianness, use u16_endianness to convert

    // ╭─────────────────────────────────────────────╮
    // │ The absolute UTC time of this data packet,  │
    // │ accurate to the second.                     │
    // ├───────────────────────┬─────────────────────┤
    // │       Each Byte       │        Range        │
    // ├───────────────────────┼─────────────────────┤
    // │ Year (current year    │       ≥70           │
    // │ minus 1900)           │                     │
    // │ Month                 │       1 to 12       │
    // │ Day                   │       1 to 31       │
    // │ Hour                  │       0 to 23       │
    // │ Minute                │       0 to 59       │
    // │ Second                │       0 to 59       │
    // ╰───────────────────────┴─────────────────────╯
    date_time: [u8; 6],

    // The "μs time" part of the absolute time of this data packet (defined in Appendix II)
    // Unit: μs
    // Range: 0 to 1000000 μs (1 s)
    timestamp: u32,

    // Should be 0x42
    factory_info: u8,
}

#[derive(Debug)]
enum ReturnMode {
    First,
    Strongest,
    Last,
    LastAndStrongest, // Default
    LastAndFirst,
    FirstAndStrongest,
}

impl Tail {
    fn motor_speed(&self) -> AngularVelocity {
        AngularVelocity::new::<revolution_per_minute>(u16_endianness(self.motor_speed) as f32)
    }

    fn return_mode(&self) -> Result<ReturnMode, HesaiError> {
        match self.return_mode {
            0x33 => Ok(ReturnMode::First),
            0x37 => Ok(ReturnMode::Strongest),
            0x38 => Ok(ReturnMode::Last),
            0x39 => Ok(ReturnMode::LastAndStrongest),
            0x3B => Ok(ReturnMode::LastAndFirst),
            0x3C => Ok(ReturnMode::FirstAndStrongest),
            _ => Err(HesaiError::InvalidPacket(format!(
                "Invalid return mode: 0x{:x}",
                self.return_mode
            ))),
        }
    }

    fn utc_tov(&self) -> Result<DateTime<Utc>, HesaiError> {
        match Utc.with_ymd_and_hms(
            self.date_time[0] as i32 + 1900,
            self.date_time[1] as u32,
            self.date_time[2] as u32,
            self.date_time[3] as u32,
            self.date_time[4] as u32,
            self.date_time[5] as u32,
        ) {
            MappedLocalTime::None => Err(HesaiError::InvalidTimestamp("No such local time".into())),
            MappedLocalTime::Single(t) => {
                Ok(t + chrono::Duration::microseconds(self.timestamp as i64))
            }
            MappedLocalTime::Ambiguous(_t1, _t2) => {
                Err(HesaiError::InvalidTimestamp("Ambiguous time".into()))
                // If that happens, good luck. ¯\_(ツ)_/¯
            }
        }
    }

    // Lidar timestamp to monotonic time of validity.
    // RefTime is a tuple of (DateTime<Utc>, CuTime) to convert the UTC time to a monotonic time.
    // You can create it and update it doing DateTime<Utc>::now() and CuTime::now() respectively
    // if the system clock is precise enough and sync'ed with the lidar.
    fn tov(&self, reftime: &(DateTime<Utc>, CuTime)) -> Result<CuTime, HesaiError> {
        // This hesai API is terrible and based on UTC, here we give a function to convert it to a monotonic robot time.
        // UTC is corrected to match earth rotation so it is NOT suitable for robotic applications.
        let (ref_date, ref_cu_time) = reftime;
        let utc_tov = self.utc_tov()?;

        let elapsed = utc_tov
            .signed_duration_since(*ref_date)
            .num_nanoseconds()
            .unwrap() as u64;

        let cu_time = *ref_cu_time + CuTime::from(elapsed);
        Ok(cu_time)
    }
}

impl Debug for Tail {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Return Mode: {:?}", self.return_mode())?;
        writeln!(
            f,
            "Motor Speed: {}",
            self.motor_speed()
                .into_format_args(revolution_per_minute, Abbreviation)
        )?;
        writeln!(f, "UTC Time: {:?}", self.utc_tov())?;
        writeln!(f, "Factory Info: {:x}", self.factory_info)
    }
}

#[inline(always)]
fn u16_endianness(val: u16) -> u16 {
    if cfg!(target_endian = "little") {
        val
    } else {
        u16::from_le(val)
    }
}

#[allow(dead_code)]
#[inline(always)]
fn u32_endianness(val: u32) -> u32 {
    if cfg!(target_endian = "little") {
        val
    } else {
        u32::from_le(val)
    }
}

// Type to map the lidar timestamp to the robot monotonic time.
pub type RefTime = (DateTime<Utc>, CuTime);

#[repr(C, packed)]
#[derive(Copy, Clone, Zeroable, Pod, Debug)]
pub struct Packet {
    pub pre_header: PreHeader,
    pub header: Header,
    pub blocks: [Block; 8],
    pub tail: Tail,
}

const FIRING_OFFSET: CuDuration = CuDuration(5_632); // this is in ns
const FIRING_DELAY: CuDuration = CuDuration(50_000);
const DUAL_RETURN_OFFSETS: [i32; 8] = [3, 3, 2, 2, 1, 1, 0, 0];
const SINGLE_RETURN_OFFSETS: [i32; 8] = [7, 6, 5, 4, 3, 2, 1, 0];

impl Packet {
    // ┌──────────────────────────────────────────────────────────────────────┐
    // │ Start time of each block                                             │
    // │ Given the absolute time of point cloud data packets as t₀, the start │
    // │ time of each block (i.e., the time when the first firing starts) can │
    // │ be calculated.                                                       │
    // ├──────────────────────────────────────────────────────────────────────┤
    // │ Single return mode                                                   │
    // ├───────────┬──────────────────────────────────────────────────────────┤
    // │  Block    │ Start time (µs)                                          │
    // ├───────────┼──────────────────────────────────────────────────────────┤
    // │  Block 8  │ t₀ + 5.632                                               │
    // │  Block N  │ t₀ + 5.632 − 50 × (8 − N)                                │
    // │  Block 3  │ t₀ + 5.632 − 50 × 5                                      │
    // │  Block 2  │ t₀ + 5.632 − 50 × 6                                      │
    // │  Block 1  │ t₀ + 5.632 − 50 × 7                                      │
    // ├───────────┴──────────────────────────────────────────────────────────┤
    // │ Dual return mode                                                     │
    // ├─────────────┬────────────────────────────────────────────────────────┤
    // │    Block    │ Start time (µs)                                        │
    // ├─────────────┼────────────────────────────────────────────────────────┤
    // │ Blocks 8&7  │ t₀ + 5.632                                             │
    // │ Blocks 6&5  │ t₀ + 5.632 − 50 × 1                                    │
    // │ Blocks 4&3  │ t₀ + 5.632 − 50 × 2                                    │
    // │ Blocks 2&1  │ t₀ + 5.632 − 50 × 3                                    │
    // └─────────────┴────────────────────────────────────────────────────────┘
    pub fn block_ts(self, reftime: &RefTime) -> Result<[CuTime; 8], HesaiError> {
        let t_zero = self.tail.tov(reftime)? + FIRING_OFFSET;
        let offsets = if self.header.is_dual_return() {
            DUAL_RETURN_OFFSETS
        } else {
            SINGLE_RETURN_OFFSETS
        };
        let result = offsets.map(|offset| t_zero - offset * FIRING_DELAY);
        Ok(result)
    }

    pub fn check_invariants(self) -> Result<(), HesaiError> {
        self.header.check_invariants()?;
        self.blocks
            .iter()
            .try_for_each(|block| block.check_invariants())?;
        Ok(())
    }
}

pub fn parse_packet(data: &[u8]) -> Result<&Packet, HesaiError> {
    if data[0] != 0xEE || data[1] != 0xFF {
        return Err(HesaiError::InvalidPacket(format!(
            "Not an Xt32 packet: {:2X}{:2X}",
            data[0], data[1],
        )));
    }

    if data.len() < size_of::<Packet>() {
        return Err(HesaiError::InvalidPacket(format!(
            "Packet too short: {} < {}",
            data.len(),
            size_of::<Packet>()
        )));
    }
    if data.len() > size_of::<Packet>() {
        return Err(HesaiError::InvalidPacket(format!(
            "Packet too long: {} > {}",
            data.len(),
            size_of::<Packet>()
        )));
    }
    let packet: &Packet = bytemuck::from_bytes(data);
    packet.check_invariants()?;
    Ok(packet)
}

/// Generate the default elevation calibration for the Xt32 Hesai sensor.
/// The sensor has 32 channels, each with a different elevation angle.
/// The elevation angles are in degrees and range from 15 to -16.
pub fn generate_default_elevation_calibration() -> [Angle; 32] {
    let mut elevations = [Angle::default(); 32];
    elevations.iter_mut().enumerate().for_each(|(i, x)| {
        *x = Angle::new::<degree>(15.0 - i as f32);
    });
    elevations
}

#[cfg(test)]
mod tests {
    use crate::parser::{parse_packet, Packet, RefTime};
    use cu29::prelude::RobotClock;

    #[test]
    fn test_packet() {
        // Taken from a real world packet
        let packet: [u8; 1122] = [
            0xB4, 0x96, 0x91, 0x72, 0x1D, 0x12, 0xEC, 0x9F, 0x0D, 0x01, 0x00, 0x69, 0x08, 0x00,
            0x45, 0x00, 0x04, 0x54, 0x57, 0xF2, 0x40, 0x00, 0x40, 0x11, 0xC6, 0xDF, 0x0A, 0xDE,
            0x01, 0x0B, 0x0A, 0xDE, 0x01, 0x01, 0x27, 0x10, 0x09, 0x40, 0x04, 0x40, 0x1B, 0xAC,
            0xEE, 0xFF, 0x06, 0x01, 0x00, 0x00, 0x20, 0x08, 0x01, 0x04, 0x02, 0x01, 0x8B, 0x77,
            0xBB, 0x01, 0x0A, 0xFF, 0xC1, 0x01, 0x0C, 0xFF, 0xC8, 0x01, 0x0B, 0xFF, 0xCD, 0x01,
            0x0B, 0xFF, 0xD5, 0x01, 0x0C, 0xFF, 0xDB, 0x01, 0x0C, 0xFF, 0xE3, 0x01, 0x0B, 0xFF,
            0xEA, 0x01, 0x0B, 0xFF, 0xF2, 0x01, 0x0B, 0xFF, 0xFA, 0x01, 0x0A, 0xFF, 0x02, 0x02,
            0x0A, 0xFF, 0x14, 0x02, 0x06, 0xFF, 0x42, 0x02, 0x06, 0xFF, 0x7C, 0x02, 0x07, 0xFF,
            0x8B, 0x02, 0x08, 0xFF, 0x9F, 0x02, 0x0A, 0xFF, 0xAA, 0x02, 0x08, 0xFF, 0xBA, 0x02,
            0x09, 0xFF, 0xCE, 0x02, 0x08, 0xFF, 0xDA, 0x02, 0x07, 0xFF, 0xEC, 0x02, 0x07, 0xFF,
            0x01, 0x03, 0x0A, 0xFF, 0x1C, 0x03, 0x07, 0xFF, 0x2F, 0x03, 0x05, 0xFF, 0x47, 0x03,
            0x07, 0xFF, 0x64, 0x03, 0x07, 0xFF, 0x80, 0x03, 0x07, 0xFF, 0xA4, 0x03, 0x08, 0xFF,
            0xBE, 0x03, 0x0A, 0xFF, 0xE5, 0x03, 0x06, 0xFF, 0x15, 0x04, 0x0B, 0xFF, 0x3D, 0x04,
            0x06, 0xFF, 0x8B, 0x77, 0xBB, 0x01, 0x0A, 0xFF, 0xC1, 0x01, 0x0C, 0xFF, 0xC8, 0x01,
            0x0B, 0xFF, 0xCD, 0x01, 0x0B, 0xFF, 0xD5, 0x01, 0x0C, 0xFF, 0xDB, 0x01, 0x0C, 0xFF,
            0xE3, 0x01, 0x0B, 0xFF, 0xEA, 0x01, 0x0B, 0xFF, 0xF2, 0x01, 0x0B, 0xFF, 0xFA, 0x01,
            0x0A, 0xFF, 0x02, 0x02, 0x0A, 0xFF, 0x14, 0x02, 0x06, 0xFF, 0x42, 0x02, 0x06, 0xFF,
            0x7C, 0x02, 0x07, 0xFF, 0x8B, 0x02, 0x08, 0xFF, 0x9F, 0x02, 0x0A, 0xFF, 0xAA, 0x02,
            0x08, 0xFF, 0xBA, 0x02, 0x09, 0xFF, 0xCE, 0x02, 0x08, 0xFF, 0xDA, 0x02, 0x07, 0xFF,
            0xEC, 0x02, 0x07, 0xFF, 0x01, 0x03, 0x0A, 0xFF, 0x1C, 0x03, 0x07, 0xFF, 0x2F, 0x03,
            0x05, 0xFF, 0x47, 0x03, 0x07, 0xFF, 0x64, 0x03, 0x07, 0xFF, 0x80, 0x03, 0x07, 0xFF,
            0xA4, 0x03, 0x08, 0xFF, 0xBE, 0x03, 0x0A, 0xFF, 0xE5, 0x03, 0x06, 0xFF, 0x15, 0x04,
            0x0B, 0xFF, 0x3D, 0x04, 0x06, 0xFF, 0x9D, 0x77, 0xBB, 0x01, 0x09, 0xFF, 0xC2, 0x01,
            0x0C, 0xFF, 0xC9, 0x01, 0x0C, 0xFF, 0xCF, 0x01, 0x0B, 0xFF, 0xD6, 0x01, 0x0C, 0xFF,
            0xDB, 0x01, 0x0B, 0xFF, 0xE3, 0x01, 0x0B, 0xFF, 0xEB, 0x01, 0x0B, 0xFF, 0xF6, 0x01,
            0x0B, 0xFF, 0xFC, 0x01, 0x0B, 0xFF, 0x02, 0x02, 0x08, 0xFF, 0x2C, 0x02, 0x06, 0xFF,
            0x47, 0x02, 0x06, 0xFF, 0x81, 0x02, 0x08, 0xFF, 0x8D, 0x02, 0x09, 0xFF, 0xA1, 0x02,
            0x0A, 0xFF, 0xAC, 0x02, 0x08, 0xFF, 0xBB, 0x02, 0x09, 0xFF, 0xD1, 0x02, 0x08, 0xFF,
            0xDE, 0x02, 0x07, 0xFF, 0xF1, 0x02, 0x07, 0xFF, 0x03, 0x03, 0x09, 0xFF, 0x1F, 0x03,
            0x07, 0xFF, 0x35, 0x03, 0x06, 0xFF, 0x4B, 0x03, 0x07, 0xFF, 0x68, 0x03, 0x07, 0xFF,
            0x82, 0x03, 0x08, 0xFF, 0xA8, 0x03, 0x09, 0xFF, 0xC3, 0x03, 0x0A, 0xFF, 0xE9, 0x03,
            0x06, 0xFF, 0x15, 0x04, 0x0B, 0xFF, 0x45, 0x04, 0x07, 0xFF, 0x9D, 0x77, 0xBB, 0x01,
            0x09, 0xFF, 0xC2, 0x01, 0x0C, 0xFF, 0xC9, 0x01, 0x0C, 0xFF, 0xCF, 0x01, 0x0B, 0xFF,
            0xD6, 0x01, 0x0C, 0xFF, 0xDB, 0x01, 0x0B, 0xFF, 0xE3, 0x01, 0x0B, 0xFF, 0xEB, 0x01,
            0x0B, 0xFF, 0xF6, 0x01, 0x0B, 0xFF, 0xFC, 0x01, 0x0B, 0xFF, 0x02, 0x02, 0x08, 0xFF,
            0x2C, 0x02, 0x06, 0xFF, 0x47, 0x02, 0x06, 0xFF, 0x81, 0x02, 0x08, 0xFF, 0x8D, 0x02,
            0x09, 0xFF, 0xA1, 0x02, 0x0A, 0xFF, 0xAC, 0x02, 0x08, 0xFF, 0xBB, 0x02, 0x09, 0xFF,
            0xD1, 0x02, 0x08, 0xFF, 0xDE, 0x02, 0x07, 0xFF, 0xF1, 0x02, 0x07, 0xFF, 0x03, 0x03,
            0x09, 0xFF, 0x1F, 0x03, 0x07, 0xFF, 0x35, 0x03, 0x06, 0xFF, 0x4B, 0x03, 0x07, 0xFF,
            0x68, 0x03, 0x07, 0xFF, 0x82, 0x03, 0x08, 0xFF, 0xA8, 0x03, 0x09, 0xFF, 0xC3, 0x03,
            0x0A, 0xFF, 0xE9, 0x03, 0x06, 0xFF, 0x15, 0x04, 0x0B, 0xFF, 0x45, 0x04, 0x07, 0xFF,
            0xAF, 0x77, 0xBE, 0x01, 0x0A, 0xFF, 0xC2, 0x01, 0x0C, 0xFF, 0xC9, 0x01, 0x0B, 0xFF,
            0xD0, 0x01, 0x0B, 0xFF, 0xD5, 0x01, 0x0C, 0xFF, 0xDD, 0x01, 0x0C, 0xFF, 0xE4, 0x01,
            0x0B, 0xFF, 0xEE, 0x01, 0x0B, 0xFF, 0xF5, 0x01, 0x0B, 0xFF, 0xFC, 0x01, 0x0B, 0xFF,
            0x04, 0x02, 0x07, 0xFF, 0x32, 0x02, 0x05, 0xFF, 0x64, 0x02, 0x06, 0xFF, 0x86, 0x02,
            0x08, 0xFF, 0x8F, 0x02, 0x08, 0xFF, 0xA2, 0x02, 0x09, 0xFF, 0xAE, 0x02, 0x09, 0xFF,
            0xBD, 0x02, 0x08, 0xFF, 0xD1, 0x02, 0x08, 0xFF, 0xDF, 0x02, 0x07, 0xFF, 0xF3, 0x02,
            0x07, 0xFF, 0x06, 0x03, 0x09, 0xFF, 0x21, 0x03, 0x07, 0xFF, 0x3A, 0x03, 0x05, 0xFF,
            0x4F, 0x03, 0x07, 0xFF, 0x6B, 0x03, 0x08, 0xFF, 0x8A, 0x03, 0x07, 0xFF, 0xAB, 0x03,
            0x09, 0xFF, 0xC6, 0x03, 0x08, 0xFF, 0xED, 0x03, 0x07, 0xFF, 0x1C, 0x04, 0x0B, 0xFF,
            0x4C, 0x04, 0x07, 0xFF, 0xAF, 0x77, 0xBE, 0x01, 0x0A, 0xFF, 0xC2, 0x01, 0x0C, 0xFF,
            0xC9, 0x01, 0x0B, 0xFF, 0xD0, 0x01, 0x0B, 0xFF, 0xD5, 0x01, 0x0C, 0xFF, 0xDD, 0x01,
            0x0C, 0xFF, 0xE4, 0x01, 0x0B, 0xFF, 0xEE, 0x01, 0x0B, 0xFF, 0xF5, 0x01, 0x0B, 0xFF,
            0xFC, 0x01, 0x0B, 0xFF, 0x04, 0x02, 0x07, 0xFF, 0x32, 0x02, 0x05, 0xFF, 0x64, 0x02,
            0x06, 0xFF, 0x86, 0x02, 0x08, 0xFF, 0x8F, 0x02, 0x08, 0xFF, 0xA2, 0x02, 0x09, 0xFF,
            0xAE, 0x02, 0x09, 0xFF, 0xBD, 0x02, 0x08, 0xFF, 0xD1, 0x02, 0x08, 0xFF, 0xDF, 0x02,
            0x07, 0xFF, 0xF3, 0x02, 0x07, 0xFF, 0x06, 0x03, 0x09, 0xFF, 0x21, 0x03, 0x07, 0xFF,
            0x3A, 0x03, 0x05, 0xFF, 0x4F, 0x03, 0x07, 0xFF, 0x6B, 0x03, 0x08, 0xFF, 0x8A, 0x03,
            0x07, 0xFF, 0xAB, 0x03, 0x09, 0xFF, 0xC6, 0x03, 0x08, 0xFF, 0xED, 0x03, 0x07, 0xFF,
            0x1C, 0x04, 0x0B, 0xFF, 0x4C, 0x04, 0x07, 0xFF, 0xC0, 0x77, 0xBC, 0x01, 0x0B, 0xFF,
            0xC3, 0x01, 0x0C, 0xFF, 0xCA, 0x01, 0x0B, 0xFF, 0xD0, 0x01, 0x0B, 0xFF, 0xD7, 0x01,
            0x0C, 0xFF, 0xDE, 0x01, 0x0B, 0xFF, 0xE6, 0x01, 0x0B, 0xFF, 0xEE, 0x01, 0x0B, 0xFF,
            0xF6, 0x01, 0x0B, 0xFF, 0xFE, 0x01, 0x0B, 0xFF, 0x0F, 0x02, 0x07, 0xFF, 0x37, 0x02,
            0x06, 0xFF, 0x72, 0x02, 0x08, 0xFF, 0x89, 0x02, 0x08, 0xFF, 0x91, 0x02, 0x09, 0xFF,
            0xA4, 0x02, 0x09, 0xFF, 0xB0, 0x02, 0x09, 0xFF, 0xBF, 0x02, 0x08, 0xFF, 0xD4, 0x02,
            0x08, 0xFF, 0xE3, 0x02, 0x07, 0xFF, 0xF5, 0x02, 0x07, 0xFF, 0x08, 0x03, 0x09, 0xFF,
            0x23, 0x03, 0x08, 0xFF, 0x3D, 0x03, 0x06, 0xFF, 0x55, 0x03, 0x08, 0xFF, 0x6F, 0x03,
            0x07, 0xFF, 0x8D, 0x03, 0x07, 0xFF, 0xAE, 0x03, 0x0A, 0xFF, 0xC9, 0x03, 0x06, 0xFF,
            0xF2, 0x03, 0x06, 0xFF, 0x1D, 0x04, 0x0B, 0xFF, 0x4F, 0x04, 0x08, 0xFF, 0xC0, 0x77,
            0xBC, 0x01, 0x0B, 0xFF, 0xC3, 0x01, 0x0C, 0xFF, 0xCA, 0x01, 0x0B, 0xFF, 0xD0, 0x01,
            0x0B, 0xFF, 0xD7, 0x01, 0x0C, 0xFF, 0xDE, 0x01, 0x0B, 0xFF, 0xE6, 0x01, 0x0B, 0xFF,
            0xEE, 0x01, 0x0B, 0xFF, 0xF6, 0x01, 0x0B, 0xFF, 0xFE, 0x01, 0x0B, 0xFF, 0x0F, 0x02,
            0x07, 0xFF, 0x37, 0x02, 0x06, 0xFF, 0x72, 0x02, 0x08, 0xFF, 0x89, 0x02, 0x08, 0xFF,
            0x91, 0x02, 0x09, 0xFF, 0xA4, 0x02, 0x09, 0xFF, 0xB0, 0x02, 0x09, 0xFF, 0xBF, 0x02,
            0x08, 0xFF, 0xD4, 0x02, 0x08, 0xFF, 0xE3, 0x02, 0x07, 0xFF, 0xF5, 0x02, 0x07, 0xFF,
            0x08, 0x03, 0x09, 0xFF, 0x23, 0x03, 0x08, 0xFF, 0x3D, 0x03, 0x06, 0xFF, 0x55, 0x03,
            0x08, 0xFF, 0x6F, 0x03, 0x07, 0xFF, 0x8D, 0x03, 0x07, 0xFF, 0xAE, 0x03, 0x0A, 0xFF,
            0xC9, 0x03, 0x06, 0xFF, 0xF2, 0x03, 0x06, 0xFF, 0x1D, 0x04, 0x0B, 0xFF, 0x4F, 0x04,
            0x08, 0xFF, 0x00, 0x00, 0x86, 0xF4, 0x07, 0x83, 0x07, 0x00, 0x01, 0x00, 0x3C, 0x58,
            0x02, 0x7C, 0x09, 0x11, 0x0F, 0x2F, 0x0C, 0x37, 0x73, 0x0A, 0x00, 0x42, 0x96, 0x13,
            0x85, 0x01,
        ];

        let (robot_clock, mock) = RobotClock::mock();
        // push the time by 1s because the first emulated test packet could end up in negative time.
        mock.increment(std::time::Duration::new(1, 0));

        let udp_header_size = 0x2A;
        if packet.len() < udp_header_size + size_of::<Packet>() {
            panic!("Packet too short: {}", packet.len());
        }
        let packet_data = &packet[udp_header_size..udp_header_size + size_of::<Packet>()];
        let packet = parse_packet(packet_data).unwrap();

        let rt: RefTime = (
            packet.tail.utc_tov().unwrap(), // emulates a packet coming in recently
            robot_clock.now(),
        );
        for (bid, ts) in packet.block_ts(&rt).unwrap().iter().enumerate() {
            println!("Block {bid} tov: {ts}");
        }
    }
}
