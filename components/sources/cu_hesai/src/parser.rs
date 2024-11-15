use bytemuck::{Pod, Zeroable};
use chrono::{DateTime, MappedLocalTime, TimeZone, Utc};
use cu29_clock::{CuDuration, CuTime};
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
use uom::si::ratio::percent;
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
            HesaiError::InvalidPacket(msg) => write!(f, "Invalid packet: {}", msg),
            HesaiError::InvalidTimestamp(msg) => write!(f, "Invalid timestamp: {}", msg),
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
    distance: u16,    // !! raw endianess
    reflectivity: u8, // Reflectivity in percentage
    reserved: u8,     // Reserved byte
}

impl Channel {
    pub fn distance(&self) -> Length {
        Length::new::<millimeter>(u16_endianess(self.distance) as f32 * 4.0) // unharcode 4mm if we port this to another sensor.
    }
    pub fn reflectivity(&self) -> Ratio {
        Ratio::new::<percent>(self.reflectivity as f32 / 255.0)
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
    motor_speed: u16, // !! raw endianess, use u16_endianess to convert

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
        AngularVelocity::new::<revolution_per_minute>(u16_endianess(self.motor_speed) as f32)
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
            MappedLocalTime::None => {
                return Err(HesaiError::InvalidTimestamp("No such local time".into()))
            }
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
fn u16_endianess(val: u16) -> u16 {
    if cfg!(target_endian = "little") {
        val
    } else {
        u16::from_le(val)
    }
}

#[allow(dead_code)]
#[inline(always)]
fn u32_endianess(val: u32) -> u32 {
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
    use super::*;
    use cu29_clock::RobotClock;
    use pcap::Capture;

    #[test]
    fn test_packet() {
        let (robot_clock, mock) = RobotClock::mock();
        // push the time by 1s because the first emulated test packet could end up in negative time.
        mock.increment(std::time::Duration::new(1, 0));

        let mut cap =
            Capture::from_file("tests/hesai-xt32-small.pcap").expect("Failed to open pcap file");

        if let Ok(packet) = cap.next_packet() {
            let udp_header_size = 0x2A;
            if packet.data.len() < udp_header_size + size_of::<Packet>() {
                panic!("Packet too short: {}", packet.data.len());
            }
            let packet_data = &packet.data[udp_header_size..udp_header_size + size_of::<Packet>()];
            let packet = parse_packet(packet_data).unwrap();

            let rt: RefTime = (
                packet.tail.utc_tov().unwrap(), // emulates a packet coming in recently
                robot_clock.now(),
            );
            for (bid, ts) in packet.block_ts(&rt).unwrap().iter().enumerate() {
                println!("Block {} tov: {}", bid, ts);
            }
        }
    }
}
