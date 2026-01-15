#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

pub mod commands;
pub mod structs;

use core::{
    fmt::{self, Debug, Display, Formatter},
    mem,
};
#[cfg(feature = "std")]
use std::error::Error;

use crc_any::CRCu8;
use heapless::Vec as HeaplessVec;
use packed_struct::PackedStruct;

#[derive(Clone, PartialEq)]
pub struct MspPacketData(pub(crate) MspPacketDataBuffer);
pub const MSP_MAX_PAYLOAD_LEN: usize = 255;
const MSP_V2_FRAME_ID: u8 = 255;
pub(crate) type MspPacketDataBuffer = HeaplessVec<u8, MSP_MAX_PAYLOAD_LEN>;

impl MspPacketData {
    pub(crate) fn new() -> MspPacketData {
        MspPacketData(MspPacketDataBuffer::new())
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        let MspPacketData(data) = self;
        data.as_mut_slice()
    }
}
// By definition an MSP packet cannot be larger than 255 bytes

impl Debug for MspPacketData {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let MspPacketData(data) = self;
        if data.is_empty() {
            write!(f, "empty")?;
            return Ok(());
        }
        write!(f, "0x")?;
        for byte in data {
            write!(f, "{byte:02X}")?;
        }
        Ok(())
    }
}

impl From<&[u8]> for MspPacketData {
    fn from(data: &[u8]) -> Self {
        let data = MspPacketDataBuffer::from_slice(data)
            .expect("MSP packet payload exceeds MSP_MAX_PAYLOAD_LEN");
        MspPacketData(data)
    }
}

impl MspPacketData {
    pub fn as_slice(&self) -> &[u8] {
        let Self(data) = self;
        data
    }
}

/// Packet parsing error
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum MspPacketParseError {
    OutputBufferSizeMismatch,
    CrcMismatch { expected: u8, calculated: u8 },
    InvalidData,
    InvalidHeader1,
    InvalidHeader2,
    InvalidDirection,
    InvalidDataLength,
}

impl Display for MspPacketParseError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            MspPacketParseError::OutputBufferSizeMismatch => {
                write!(f, "Output buffer size mismatch")
            }
            MspPacketParseError::CrcMismatch {
                expected,
                calculated,
            } => write!(
                f,
                "CRC mismatch, expected: 0x{expected:02X}, calculated: 0x{calculated:02X}"
            ),
            MspPacketParseError::InvalidData => write!(f, "Invalid data"),
            MspPacketParseError::InvalidHeader1 => write!(f, "Invalid header 1"),
            MspPacketParseError::InvalidHeader2 => write!(f, "Invalid header 2"),
            MspPacketParseError::InvalidDirection => write!(f, "Invalid direction"),
            MspPacketParseError::InvalidDataLength => write!(f, "Invalid data length"),
        }
    }
}

#[cfg(feature = "std")]
impl Error for MspPacketParseError {}

#[cfg(not(feature = "std"))]
impl core::error::Error for MspPacketParseError {}

/// Packet's desired destination
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum MspPacketDirection {
    /// Network byte '<'
    ToFlightController,
    /// Network byte '>'
    FromFlightController,
    /// Network byte '!'
    Unsupported,
}

impl MspPacketDirection {
    /// To network byte
    pub fn to_byte(&self) -> u8 {
        let b = match *self {
            MspPacketDirection::ToFlightController => '<',
            MspPacketDirection::FromFlightController => '>',
            MspPacketDirection::Unsupported => '!',
        };
        b as u8
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MspParsedVersion {
    V1,
    V2Native,
    V2OverV1,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MspPacketInfo {
    pub version: MspParsedVersion,
    pub cmd: u16,
}

#[derive(Debug, Clone, PartialEq)]
/// A decoded MSP packet, with a command code, direction and payload
pub struct MspPacket {
    pub cmd: u16,
    pub direction: MspPacketDirection,
    pub data: MspPacketData,
}

#[derive(Copy, Clone, PartialEq, Debug)]
enum MspParserState {
    Header1,
    Header2,
    Direction,
    FlagV2,
    DataLength,
    DataLengthV2,
    Command,
    CommandV2,
    Data,
    DataV2,
    Crc,
}

#[derive(Copy, Clone, PartialEq, Debug)]
enum MspVersion {
    V1,
    V2,
}

/// Parser that can find packets from a raw byte stream
pub struct MspParser {
    state: MspParserState,
    packet_version: MspVersion,
    packet_direction: MspPacketDirection,
    packet_cmd: u16,
    packet_data_length_remaining: usize,
    packet_data: MspPacketData,
    packet_crc: u8,
    packet_crc_v2: CRCu8,
    last_packet_info: Option<MspPacketInfo>,
}

impl MspParser {
    /// Create a new parser
    pub fn new() -> MspParser {
        Self {
            state: MspParserState::Header1,
            packet_version: MspVersion::V1,
            packet_direction: MspPacketDirection::ToFlightController,
            packet_data_length_remaining: 0,
            packet_cmd: 0,
            packet_data: MspPacketData::new(),
            packet_crc: 0,
            packet_crc_v2: CRCu8::crc8dvb_s2(),
            last_packet_info: None,
        }
    }

    /// Are we waiting for the header of a brand new packet?
    pub fn state_is_between_packets(&self) -> bool {
        self.state == MspParserState::Header1
    }

    pub fn last_packet_info(&self) -> Option<MspPacketInfo> {
        self.last_packet_info
    }

    /// Parse the next input byte. Returns a valid packet whenever a full packet is received, otherwise
    /// restarts the state of the parser.
    pub fn parse(&mut self, input: u8) -> Result<Option<MspPacket>, MspPacketParseError> {
        match self.state {
            MspParserState::Header1 => {
                if input == b'$' {
                    self.state = MspParserState::Header2;
                } else {
                    self.reset();
                }
            }

            MspParserState::Header2 => {
                self.packet_version = match input as char {
                    'M' => MspVersion::V1,
                    'X' => MspVersion::V2,
                    _ => {
                        self.reset();
                        return Err(MspPacketParseError::InvalidHeader2);
                    }
                };

                self.state = MspParserState::Direction;
            }

            MspParserState::Direction => {
                match input {
                    60 => self.packet_direction = MspPacketDirection::ToFlightController, // '>'
                    62 => self.packet_direction = MspPacketDirection::FromFlightController, // '<'
                    33 => self.packet_direction = MspPacketDirection::Unsupported, // '!' error
                    _ => {
                        self.reset();
                        return Err(MspPacketParseError::InvalidDirection);
                    }
                }

                self.state = match self.packet_version {
                    MspVersion::V1 => MspParserState::DataLength,
                    MspVersion::V2 => MspParserState::FlagV2,
                };
            }

            MspParserState::FlagV2 => {
                // uint8, flag, usage to be defined (set to zero)
                self.state = MspParserState::CommandV2;
                self.packet_data = MspPacketData::new();
                self.packet_crc_v2.digest(&[input]);
            }

            MspParserState::CommandV2 => {
                let overflow = {
                    let MspPacketData(data) = &mut self.packet_data;
                    data.push(input).is_err()
                };
                if overflow {
                    self.reset();
                    return Err(MspPacketParseError::InvalidDataLength);
                }

                let MspPacketData(data) = &mut self.packet_data;
                if data.len() == 2 {
                    let mut s = [0u8; core::mem::size_of::<u16>()];
                    s.copy_from_slice(data);
                    self.packet_cmd = u16::from_le_bytes(s);

                    self.packet_crc_v2.digest(&data);
                    data.clear();
                    self.state = MspParserState::DataLengthV2;
                }
            }

            MspParserState::DataLengthV2 => {
                let overflow = {
                    let MspPacketData(data) = &mut self.packet_data;
                    data.push(input).is_err()
                };
                if overflow {
                    self.reset();
                    return Err(MspPacketParseError::InvalidDataLength);
                }

                let MspPacketData(data) = &mut self.packet_data;
                if data.len() == 2 {
                    let mut s = [0u8; core::mem::size_of::<u16>()];
                    s.copy_from_slice(data);
                    self.packet_data_length_remaining = u16::from_le_bytes(s).into();
                    if self.packet_data_length_remaining > MSP_MAX_PAYLOAD_LEN {
                        self.reset();
                        return Err(MspPacketParseError::InvalidDataLength);
                    }
                    self.packet_crc_v2.digest(data);
                    data.clear();
                    if self.packet_data_length_remaining == 0 {
                        self.state = MspParserState::Crc;
                    } else {
                        self.state = MspParserState::DataV2;
                    }
                }
            }

            MspParserState::DataV2 => {
                let overflow = {
                    let MspPacketData(data) = &mut self.packet_data;
                    data.push(input).is_err()
                };
                if overflow {
                    self.reset();
                    return Err(MspPacketParseError::InvalidDataLength);
                }
                self.packet_data_length_remaining -= 1;

                if self.packet_data_length_remaining == 0 {
                    self.state = MspParserState::Crc;
                }
            }

            MspParserState::DataLength => {
                let MspPacketData(data) = &mut self.packet_data;
                self.packet_data_length_remaining = input as usize;
                self.state = MspParserState::Command;
                self.packet_crc ^= input;
                data.clear();
            }

            MspParserState::Command => {
                self.packet_cmd = input as u16;

                if self.packet_data_length_remaining == 0 {
                    self.state = MspParserState::Crc;
                } else {
                    self.state = MspParserState::Data;
                }

                self.packet_crc ^= input;
            }

            MspParserState::Data => {
                let overflow = {
                    let MspPacketData(data) = &mut self.packet_data;
                    data.push(input).is_err()
                };
                if overflow {
                    self.reset();
                    return Err(MspPacketParseError::InvalidDataLength);
                }
                self.packet_data_length_remaining -= 1;

                self.packet_crc ^= input;

                if self.packet_data_length_remaining == 0 {
                    self.state = MspParserState::Crc;
                }
            }

            MspParserState::Crc => {
                let MspPacketData(data) = &mut self.packet_data;
                if self.packet_version == MspVersion::V2 {
                    self.packet_crc_v2.digest(data);
                    self.packet_crc = self.packet_crc_v2.get_crc();
                }

                let packet_crc = self.packet_crc;
                if input != packet_crc {
                    self.reset();
                    return Err(MspPacketParseError::CrcMismatch {
                        expected: input,
                        calculated: packet_crc,
                    });
                }

                let mut n = MspPacketData::new();
                mem::swap(&mut self.packet_data, &mut n);

                let packet = if self.packet_version == MspVersion::V1
                    && self.packet_cmd == u16::from(MSP_V2_FRAME_ID)
                {
                    let raw = n.as_slice();
                    if raw.len() < 6 {
                        self.reset();
                        return Err(MspPacketParseError::InvalidDataLength);
                    }
                    let cmd = u16::from_le_bytes([raw[1], raw[2]]);
                    let payload_len = u16::from_le_bytes([raw[3], raw[4]]) as usize;
                    let v2_end = 5 + payload_len;
                    if raw.len() < v2_end + 1 {
                        self.reset();
                        return Err(MspPacketParseError::InvalidDataLength);
                    }
                    let v2_crc = raw[v2_end];
                    let mut crc = CRCu8::crc8dvb_s2();
                    crc.digest(&raw[..v2_end]);
                    let calculated = crc.get_crc();
                    if v2_crc != calculated {
                        self.reset();
                        return Err(MspPacketParseError::CrcMismatch {
                            expected: v2_crc,
                            calculated,
                        });
                    }
                    self.last_packet_info = Some(MspPacketInfo {
                        version: MspParsedVersion::V2OverV1,
                        cmd,
                    });
                    MspPacket {
                        cmd,
                        direction: self.packet_direction,
                        data: MspPacketData::from(&raw[5..v2_end]),
                    }
                } else {
                    let version = match self.packet_version {
                        MspVersion::V1 => MspParsedVersion::V1,
                        MspVersion::V2 => MspParsedVersion::V2Native,
                    };
                    self.last_packet_info = Some(MspPacketInfo {
                        version,
                        cmd: self.packet_cmd,
                    });
                    MspPacket {
                        cmd: self.packet_cmd,
                        direction: self.packet_direction,
                        data: n,
                    }
                };

                self.reset();

                return Ok(Some(packet));
            }
        }

        Ok(None)
    }

    pub fn reset(&mut self) {
        let MspPacketData(data) = &mut self.packet_data;
        self.state = MspParserState::Header1;
        self.packet_direction = MspPacketDirection::ToFlightController;
        self.packet_data_length_remaining = 0;
        self.packet_cmd = 0;
        data.clear();
        self.packet_crc = 0;
        self.packet_crc_v2.reset();
    }
}

impl Default for MspParser {
    fn default() -> Self {
        Self::new()
    }
}

impl MspPacket {
    /// Number of bytes that this packet requires to be packed
    pub fn packet_size_bytes(&self) -> usize {
        if self.cmd > u16::from(MSP_V2_FRAME_ID) {
            return self.packet_size_bytes_v2_over_v1();
        }

        let MspPacketData(data) = &self.data;
        6 + data.len()
    }

    /// Number of bytes that this packet requires to be packed
    pub fn packet_size_bytes_v2(&self) -> usize {
        let MspPacketData(data) = &self.data;
        9 + data.len()
    }

    /// Number of bytes that this packet requires to be packed as MSPv2 over MSPv1.
    pub fn packet_size_bytes_v2_over_v1(&self) -> usize {
        let MspPacketData(data) = &self.data;
        12 + data.len()
    }

    /// Serialize to network bytes
    pub fn serialize(&self, output: &mut [u8]) -> Result<(), MspPacketParseError> {
        if self.cmd > u16::from(MSP_V2_FRAME_ID) {
            return self.serialize_v2_over_v1(output);
        }

        let MspPacketData(data) = &self.data;
        let l = output.len();

        if l != self.packet_size_bytes() {
            return Err(MspPacketParseError::OutputBufferSizeMismatch);
        }

        output[0] = b'$';
        output[1] = b'M';
        output[2] = self.direction.to_byte();
        output[3] = data.len() as u8;
        output[4] = self.cmd as u8;

        output[5..l - 1].copy_from_slice(data);

        let mut crc = output[3] ^ output[4];
        for b in data {
            crc ^= *b;
        }
        output[l - 1] = crc;

        Ok(())
    }

    /// Serialize to network bytes
    pub fn serialize_v2(&self, output: &mut [u8]) -> Result<(), MspPacketParseError> {
        let MspPacketData(data) = &self.data;
        let l = output.len();

        if l != self.packet_size_bytes_v2() {
            return Err(MspPacketParseError::OutputBufferSizeMismatch);
        }

        output[0] = b'$';
        output[1] = b'X';
        output[2] = self.direction.to_byte();
        output[3] = 0;
        output[4..6].copy_from_slice(&self.cmd.to_le_bytes());
        output[6..8].copy_from_slice(&(data.len() as u16).to_le_bytes());

        output[8..l - 1].copy_from_slice(data);

        let mut crc = CRCu8::crc8dvb_s2();
        crc.digest(&output[3..l - 1]);
        output[l - 1] = crc.get_crc();

        Ok(())
    }

    /// Serialize to MSPv2-over-MSPv1 (MSP_V2_FRAME) bytes.
    pub fn serialize_v2_over_v1(&self, output: &mut [u8]) -> Result<(), MspPacketParseError> {
        let MspPacketData(data) = &self.data;
        let l = output.len();

        if l != self.packet_size_bytes_v2_over_v1() {
            return Err(MspPacketParseError::OutputBufferSizeMismatch);
        }

        let v1_payload_len = data.len() + 6;
        output[0] = b'$';
        output[1] = b'M';
        output[2] = self.direction.to_byte();
        output[3] = v1_payload_len as u8;
        output[4] = MSP_V2_FRAME_ID;

        output[5] = 0; // flags
        output[6..8].copy_from_slice(&self.cmd.to_le_bytes());
        output[8..10].copy_from_slice(&(data.len() as u16).to_le_bytes());
        output[10..10 + data.len()].copy_from_slice(data);

        let mut crc_v2 = CRCu8::crc8dvb_s2();
        crc_v2.digest(&output[5..10 + data.len()]);
        output[10 + data.len()] = crc_v2.get_crc();

        let mut crc = output[3] ^ output[4];
        for b in &output[5..11 + data.len()] {
            crc ^= *b;
        }
        output[l - 1] = crc;

        Ok(())
    }

    pub fn decode_as<T: PackedStruct>(&self) -> Result<T, packed_struct::PackingError> {
        let expected_size = core::mem::size_of::<T::ByteArray>();

        if self.data.0.len() < expected_size {
            return Err(packed_struct::PackingError::BufferSizeMismatch {
                expected: expected_size,
                actual: self.data.0.len(),
            });
        }

        let byte_array: &T::ByteArray = unsafe { &*(self.data.0.as_ptr() as *const T::ByteArray) };

        T::unpack(byte_array)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_serialize() {
        let packet = MspPacket {
            cmd: 2,
            direction: MspPacketDirection::ToFlightController,
            data: MspPacketData::from(&[0xbeu8, 0xef][..]),
        };

        let size = packet.packet_size_bytes();
        assert_eq!(8, size);

        #[cfg(feature = "std")]
        let mut output = std::vec![0; size];
        #[cfg(not(feature = "std"))]
        let mut output = alloc::vec![0; size];
        packet.serialize(&mut output).unwrap();
        let expected = [b'$', b'M', b'<', 2, 2, 0xbe, 0xef, 81];
        assert_eq!(&expected, output.as_slice());

        let mut packet_parsed = None;
        let mut parser = MspParser::new();
        for b in output {
            let s = parser.parse(b);
            if let Ok(Some(p)) = s {
                packet_parsed = Some(p);
                break;
            }
        }

        assert_eq!(packet, packet_parsed.unwrap());
    }

    #[test]
    fn test_roundtrip() {
        fn roundtrip(packet: &MspPacket) {
            let size = packet.packet_size_bytes();
            #[cfg(feature = "std")]
            let mut output = std::vec![0; size];
            #[cfg(not(feature = "std"))]
            let mut output = alloc::vec![0; size];

            packet.serialize(&mut output).unwrap();
            let mut parser = MspParser::new();
            let mut packet_parsed = None;
            for b in output {
                let s = parser.parse(b);
                if let Ok(Some(p)) = s {
                    packet_parsed = Some(p);
                    break;
                }
            }
            assert_eq!(packet, &packet_parsed.unwrap());
        }

        {
            let packet = MspPacket {
                cmd: 1,
                direction: MspPacketDirection::ToFlightController,
                data: MspPacketData::from(&[0x00u8, 0x00, 0x00][..]),
            };
            roundtrip(&packet);
        }

        {
            let packet = MspPacket {
                cmd: 200,
                direction: MspPacketDirection::FromFlightController,
                data: MspPacketData::new(),
            };
            roundtrip(&packet);
        }

        {
            let packet = MspPacket {
                cmd: 100,
                direction: MspPacketDirection::Unsupported,
                data: MspPacketData::from(&[0x44u8, 0x20, 0x00, 0x80][..]),
            };
            roundtrip(&packet);
        }
    }
}
