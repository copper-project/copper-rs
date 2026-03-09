//! AUTOSAR SOME/IP protocol types.
//!
//! Covers the fixed 16-byte SOME/IP header, message type enums,
//! return codes, and a complete SOME/IP message payload.
//! Also includes Service Discovery (SD) entry types.

use bincode::{Decode, Encode};
use cu29::prelude::*;
use core::fmt;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

/// Maximum SOME/IP payload size (UDP practical limit minus header).
pub const SOMEIP_MAX_PAYLOAD_SIZE: usize = 1400;
/// SOME/IP header size in bytes.
pub const SOMEIP_HEADER_SIZE: usize = 16;
/// SOME/IP-SD well-known multicast port.
pub const SOMEIP_SD_PORT: u16 = 30490;

// ---------------------------------------------------------------------------
// Message Type
// ---------------------------------------------------------------------------

/// SOME/IP message type field.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[repr(u8)]
pub enum SomeIpMessageType {
    #[default]
    Request = 0x00,
    RequestNoReturn = 0x01,
    Notification = 0x02,
    Response = 0x80,
    Error = 0x81,
    TpRequest = 0x20,
    TpRequestNoReturn = 0x21,
    TpNotification = 0x22,
    TpResponse = 0xA0,
    TpError = 0xA1,
}

impl From<u8> for SomeIpMessageType {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::Request,
            0x01 => Self::RequestNoReturn,
            0x02 => Self::Notification,
            0x80 => Self::Response,
            0x81 => Self::Error,
            0x20 => Self::TpRequest,
            0x21 => Self::TpRequestNoReturn,
            0x22 => Self::TpNotification,
            0xA0 => Self::TpResponse,
            0xA1 => Self::TpError,
            _ => Self::Request,
        }
    }
}

// ---------------------------------------------------------------------------
// Return Code
// ---------------------------------------------------------------------------

/// SOME/IP return code.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[repr(u8)]
pub enum SomeIpReturnCode {
    #[default]
    Ok = 0x00,
    NotOk = 0x01,
    UnknownService = 0x02,
    UnknownMethod = 0x03,
    NotReady = 0x04,
    NotReachable = 0x05,
    Timeout = 0x06,
    WrongProtocolVersion = 0x07,
    WrongInterfaceVersion = 0x08,
    MalformedMessage = 0x09,
    WrongMessageType = 0x0A,
}

impl From<u8> for SomeIpReturnCode {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::Ok,
            0x01 => Self::NotOk,
            0x02 => Self::UnknownService,
            0x03 => Self::UnknownMethod,
            0x04 => Self::NotReady,
            0x05 => Self::NotReachable,
            0x06 => Self::Timeout,
            0x07 => Self::WrongProtocolVersion,
            0x08 => Self::WrongInterfaceVersion,
            0x09 => Self::MalformedMessage,
            0x0A => Self::WrongMessageType,
            _ => Self::NotOk,
        }
    }
}

// ---------------------------------------------------------------------------
// SOME/IP Header (16 bytes on-wire)
// ---------------------------------------------------------------------------

/// The fixed 16-byte SOME/IP header.
///
/// ```text
/// Bytes 0-3:   Service ID (16 bits) | Method/Event ID (16 bits)
/// Bytes 4-7:   Length (payload + 8 bytes of remaining header)
/// Bytes 8-9:   Client ID (16 bits)
/// Bytes 10-11: Session ID (16 bits)
/// Byte  12:    Protocol Version (always 0x01)
/// Byte  13:    Interface Version
/// Byte  14:    Message Type
/// Byte  15:    Return Code
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct SomeIpHeader {
    pub service_id: u16,
    pub method_id: u16,
    pub length: u32,
    pub client_id: u16,
    pub session_id: u16,
    pub protocol_version: u8,
    pub interface_version: u8,
    pub message_type: SomeIpMessageType,
    pub return_code: SomeIpReturnCode,
}

impl SomeIpHeader {
    /// SOME/IP protocol version (always 1).
    pub const PROTOCOL_VERSION: u8 = 0x01;

    /// Creates a request header for the given service/method.
    pub fn request(service_id: u16, method_id: u16, client_id: u16, session_id: u16) -> Self {
        Self {
            service_id,
            method_id,
            length: 8, // will be updated when payload is added
            client_id,
            session_id,
            protocol_version: Self::PROTOCOL_VERSION,
            interface_version: 1,
            message_type: SomeIpMessageType::Request,
            return_code: SomeIpReturnCode::Ok,
        }
    }

    /// Creates a response header echoing the request session.
    pub fn response(req: &SomeIpHeader) -> Self {
        Self {
            service_id: req.service_id,
            method_id: req.method_id,
            length: 8,
            client_id: req.client_id,
            session_id: req.session_id,
            protocol_version: Self::PROTOCOL_VERSION,
            interface_version: req.interface_version,
            message_type: SomeIpMessageType::Response,
            return_code: SomeIpReturnCode::Ok,
        }
    }

    /// Is this a request message?
    pub fn is_request(&self) -> bool {
        matches!(
            self.message_type,
            SomeIpMessageType::Request | SomeIpMessageType::TpRequest
        )
    }

    /// Is this a notification/event?
    pub fn is_notification(&self) -> bool {
        matches!(
            self.message_type,
            SomeIpMessageType::Notification | SomeIpMessageType::TpNotification
        )
    }

    /// Serialize header to 16 bytes (big-endian, on-wire format).
    pub fn to_bytes(&self, buf: &mut [u8; SOMEIP_HEADER_SIZE]) {
        buf[0..2].copy_from_slice(&self.service_id.to_be_bytes());
        buf[2..4].copy_from_slice(&self.method_id.to_be_bytes());
        buf[4..8].copy_from_slice(&self.length.to_be_bytes());
        buf[8..10].copy_from_slice(&self.client_id.to_be_bytes());
        buf[10..12].copy_from_slice(&self.session_id.to_be_bytes());
        buf[12] = self.protocol_version;
        buf[13] = self.interface_version;
        buf[14] = self.message_type as u8;
        buf[15] = self.return_code as u8;
    }

    /// Deserialize header from 16 bytes (big-endian, on-wire format).
    pub fn from_bytes(buf: &[u8; SOMEIP_HEADER_SIZE]) -> Self {
        Self {
            service_id: u16::from_be_bytes([buf[0], buf[1]]),
            method_id: u16::from_be_bytes([buf[2], buf[3]]),
            length: u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]),
            client_id: u16::from_be_bytes([buf[8], buf[9]]),
            session_id: u16::from_be_bytes([buf[10], buf[11]]),
            protocol_version: buf[12],
            interface_version: buf[13],
            message_type: SomeIpMessageType::from(buf[14]),
            return_code: SomeIpReturnCode::from(buf[15]),
        }
    }
}

impl fmt::Display for SomeIpHeader {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SOME/IP [0x{:04X}.0x{:04X}] {:?} client=0x{:04X} session=0x{:04X}",
            self.service_id, self.method_id, self.message_type, self.client_id, self.session_id,
        )
    }
}

// ---------------------------------------------------------------------------
// SOME/IP Message (header + payload)
// ---------------------------------------------------------------------------

/// A complete SOME/IP message with fixed-size payload buffer.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct SomeIpMessage {
    /// SOME/IP header.
    pub header: SomeIpHeader,
    /// Payload data.
    #[serde(with = "BigArray")]
    pub payload: [u8; SOMEIP_MAX_PAYLOAD_SIZE],
    /// Number of valid bytes in `payload`.
    pub payload_len: u16,
}

impl Default for SomeIpMessage {
    fn default() -> Self {
        Self {
            header: SomeIpHeader::default(),
            payload: [0u8; SOMEIP_MAX_PAYLOAD_SIZE],
            payload_len: 0,
        }
    }
}

impl SomeIpMessage {
    /// Create a request message.
    pub fn request(
        service_id: u16,
        method_id: u16,
        client_id: u16,
        session_id: u16,
        payload: &[u8],
    ) -> Self {
        let mut msg = Self {
            header: SomeIpHeader::request(service_id, method_id, client_id, session_id),
            ..Default::default()
        };
        let len = payload.len().min(SOMEIP_MAX_PAYLOAD_SIZE);
        msg.payload[..len].copy_from_slice(&payload[..len]);
        msg.payload_len = len as u16;
        msg.header.length = 8 + len as u32;
        msg
    }

    /// Create a response echoing the request.
    pub fn response(req: &SomeIpMessage, payload: &[u8]) -> Self {
        let mut msg = Self {
            header: SomeIpHeader::response(&req.header),
            ..Default::default()
        };
        let len = payload.len().min(SOMEIP_MAX_PAYLOAD_SIZE);
        msg.payload[..len].copy_from_slice(&payload[..len]);
        msg.payload_len = len as u16;
        msg.header.length = 8 + len as u32;
        msg
    }

    /// Create an error response.
    pub fn error(req: &SomeIpMessage, return_code: SomeIpReturnCode) -> Self {
        let mut msg = Self {
            header: SomeIpHeader::response(&req.header),
            ..Default::default()
        };
        msg.header.message_type = SomeIpMessageType::Error;
        msg.header.return_code = return_code;
        msg.header.length = 8;
        msg
    }

    /// Create a notification (event) message.
    pub fn notification(service_id: u16, event_id: u16, payload: &[u8]) -> Self {
        let mut msg = Self::default();
        msg.header.service_id = service_id;
        msg.header.method_id = event_id;
        msg.header.message_type = SomeIpMessageType::Notification;
        msg.header.protocol_version = SomeIpHeader::PROTOCOL_VERSION;
        let len = payload.len().min(SOMEIP_MAX_PAYLOAD_SIZE);
        msg.payload[..len].copy_from_slice(&payload[..len]);
        msg.payload_len = len as u16;
        msg.header.length = 8 + len as u32;
        msg
    }

    /// Returns the valid payload slice.
    pub fn payload_data(&self) -> &[u8] {
        &self.payload[..self.payload_len as usize]
    }

    /// Serialize the entire message (header + payload) to wire format.
    pub fn to_wire(&self, buf: &mut [u8]) -> usize {
        let mut hdr_buf = [0u8; SOMEIP_HEADER_SIZE];
        self.header.to_bytes(&mut hdr_buf);
        buf[..SOMEIP_HEADER_SIZE].copy_from_slice(&hdr_buf);
        let plen = self.payload_len as usize;
        buf[SOMEIP_HEADER_SIZE..SOMEIP_HEADER_SIZE + plen]
            .copy_from_slice(&self.payload[..plen]);
        SOMEIP_HEADER_SIZE + plen
    }

    /// Parse a message from wire bytes.
    pub fn from_wire(buf: &[u8]) -> Option<Self> {
        if buf.len() < SOMEIP_HEADER_SIZE {
            return None;
        }
        let hdr_arr: [u8; SOMEIP_HEADER_SIZE] = buf[..SOMEIP_HEADER_SIZE].try_into().ok()?;
        let header = SomeIpHeader::from_bytes(&hdr_arr);
        let payload_size = header.length.saturating_sub(8) as usize;
        let available = buf.len() - SOMEIP_HEADER_SIZE;
        let copy_len = payload_size.min(available).min(SOMEIP_MAX_PAYLOAD_SIZE);
        let mut msg = Self {
            header,
            ..Default::default()
        };
        msg.payload[..copy_len]
            .copy_from_slice(&buf[SOMEIP_HEADER_SIZE..SOMEIP_HEADER_SIZE + copy_len]);
        msg.payload_len = copy_len as u16;
        Some(msg)
    }
}

impl fmt::Display for SomeIpMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ({} bytes payload)", self.header, self.payload_len)
    }
}

// ---------------------------------------------------------------------------
// SOME/IP-SD Service Entry
// ---------------------------------------------------------------------------

/// SOME/IP Service Discovery entry type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum SdEntryType {
    #[default]
    FindService,
    OfferService,
    StopOfferService,
    SubscribeEventgroup,
    StopSubscribeEventgroup,
    SubscribeEventgroupAck,
    SubscribeEventgroupNack,
}

/// A SOME/IP-SD service entry.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct SdServiceEntry {
    pub entry_type: SdEntryType,
    pub service_id: u16,
    pub instance_id: u16,
    pub major_version: u8,
    pub minor_version: u32,
    pub ttl: u32,
}

/// Current SOME/IP service availability (output of SD task).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct SomeIpServiceStatus {
    pub service_id: u16,
    pub instance_id: u16,
    pub available: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_round_trip() {
        let hdr = SomeIpHeader::request(0x1234, 0x0001, 0xABCD, 0x0001);
        let mut buf = [0u8; SOMEIP_HEADER_SIZE];
        hdr.to_bytes(&mut buf);
        let decoded = SomeIpHeader::from_bytes(&buf);
        assert_eq!(hdr.service_id, decoded.service_id);
        assert_eq!(hdr.method_id, decoded.method_id);
        assert_eq!(hdr.client_id, decoded.client_id);
        assert_eq!(hdr.session_id, decoded.session_id);
        assert_eq!(decoded.protocol_version, 0x01);
    }

    #[test]
    fn message_wire_round_trip() {
        let msg = SomeIpMessage::request(0x1234, 0x0001, 0x00FF, 0x0001, &[1, 2, 3, 4]);
        let mut buf = [0u8; 2048];
        let len = msg.to_wire(&mut buf);
        let parsed = SomeIpMessage::from_wire(&buf[..len]).unwrap();
        assert_eq!(parsed.header.service_id, 0x1234);
        assert_eq!(parsed.payload_data(), &[1, 2, 3, 4]);
    }

    #[test]
    fn response_echoes_session() {
        let req = SomeIpMessage::request(0xABCD, 0x0002, 0x1111, 0x0042, &[10]);
        let resp = SomeIpMessage::response(&req, &[20, 30]);
        assert_eq!(resp.header.session_id, 0x0042);
        assert_eq!(resp.header.client_id, 0x1111);
        assert_eq!(resp.header.message_type, SomeIpMessageType::Response);
    }
}
