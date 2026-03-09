//! ISO 14229 — Unified Diagnostic Services (UDS) types.
//!
//! Message payloads for UDS request/response communication, session management,
//! and Negative Response Codes (NRC).

use bincode::{Decode, Encode};
use cu29::prelude::*;
use core::fmt;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

/// Maximum UDS message payload size (excluding SID and sub-function).
pub const UDS_MAX_PAYLOAD_SIZE: usize = 4093;

// ---------------------------------------------------------------------------
// Session types
// ---------------------------------------------------------------------------

/// UDS diagnostic session type (ISO 14229-1 §9.2).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[repr(u8)]
pub enum UdsSessionType {
    #[default]
    Default = 0x01,
    Programming = 0x02,
    Extended = 0x03,
}

impl From<u8> for UdsSessionType {
    fn from(v: u8) -> Self {
        match v {
            0x02 => Self::Programming,
            0x03 => Self::Extended,
            _ => Self::Default,
        }
    }
}

// ---------------------------------------------------------------------------
// Service IDs
// ---------------------------------------------------------------------------

/// Well-known UDS Service Identifiers (SID).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[repr(u8)]
pub enum UdsServiceId {
    DiagnosticSessionControl = 0x10,
    EcuReset = 0x11,
    SecurityAccess = 0x27,
    CommunicationControl = 0x28,
    TesterPresent = 0x3E,
    ControlDtcSetting = 0x85,
    ReadDataByIdentifier = 0x22,
    WriteDataByIdentifier = 0x2E,
    RoutineControl = 0x31,
    RequestDownload = 0x34,
    RequestUpload = 0x35,
    TransferData = 0x36,
    RequestTransferExit = 0x37,
    ReadDtcInformation = 0x19,
    ClearDtcInformation = 0x14,
    InputOutputControlByIdentifier = 0x2F,
    NegativeResponse = 0x7F,
}

impl Default for UdsServiceId {
    fn default() -> Self {
        Self::TesterPresent
    }
}

impl From<u8> for UdsServiceId {
    fn from(v: u8) -> Self {
        match v {
            0x10 => Self::DiagnosticSessionControl,
            0x11 => Self::EcuReset,
            0x27 => Self::SecurityAccess,
            0x28 => Self::CommunicationControl,
            0x3E => Self::TesterPresent,
            0x85 => Self::ControlDtcSetting,
            0x22 => Self::ReadDataByIdentifier,
            0x2E => Self::WriteDataByIdentifier,
            0x31 => Self::RoutineControl,
            0x34 => Self::RequestDownload,
            0x35 => Self::RequestUpload,
            0x36 => Self::TransferData,
            0x37 => Self::RequestTransferExit,
            0x19 => Self::ReadDtcInformation,
            0x14 => Self::ClearDtcInformation,
            0x2F => Self::InputOutputControlByIdentifier,
            0x7F => Self::NegativeResponse,
            _ => Self::TesterPresent, // fallback
        }
    }
}

// ---------------------------------------------------------------------------
// Negative Response Codes
// ---------------------------------------------------------------------------

/// UDS Negative Response Code (NRC) — ISO 14229-1 Annex A.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[repr(u8)]
pub enum Nrc {
    #[default]
    PositiveResponse = 0x00,
    GeneralReject = 0x10,
    ServiceNotSupported = 0x11,
    SubFunctionNotSupported = 0x12,
    IncorrectMessageLengthOrInvalidFormat = 0x13,
    ResponseTooLong = 0x14,
    BusyRepeatRequest = 0x21,
    ConditionsNotCorrect = 0x22,
    RequestSequenceError = 0x24,
    NoResponseFromSubnetComponent = 0x25,
    FailurePreventsExecutionOfRequestedAction = 0x26,
    RequestOutOfRange = 0x31,
    SecurityAccessDenied = 0x33,
    AuthenticationRequired = 0x34,
    InvalidKey = 0x35,
    ExceededNumberOfAttempts = 0x36,
    RequiredTimeDelayNotExpired = 0x37,
    UploadDownloadNotAccepted = 0x70,
    TransferDataSuspended = 0x71,
    GeneralProgrammingFailure = 0x72,
    WrongBlockSequenceCounter = 0x73,
    ResponsePending = 0x78,
    SubFunctionNotSupportedInActiveSession = 0x7E,
    ServiceNotSupportedInActiveSession = 0x7F,
}

impl From<u8> for Nrc {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::PositiveResponse,
            0x10 => Self::GeneralReject,
            0x11 => Self::ServiceNotSupported,
            0x12 => Self::SubFunctionNotSupported,
            0x13 => Self::IncorrectMessageLengthOrInvalidFormat,
            0x14 => Self::ResponseTooLong,
            0x21 => Self::BusyRepeatRequest,
            0x22 => Self::ConditionsNotCorrect,
            0x24 => Self::RequestSequenceError,
            0x25 => Self::NoResponseFromSubnetComponent,
            0x26 => Self::FailurePreventsExecutionOfRequestedAction,
            0x31 => Self::RequestOutOfRange,
            0x33 => Self::SecurityAccessDenied,
            0x34 => Self::AuthenticationRequired,
            0x35 => Self::InvalidKey,
            0x36 => Self::ExceededNumberOfAttempts,
            0x37 => Self::RequiredTimeDelayNotExpired,
            0x70 => Self::UploadDownloadNotAccepted,
            0x71 => Self::TransferDataSuspended,
            0x72 => Self::GeneralProgrammingFailure,
            0x73 => Self::WrongBlockSequenceCounter,
            0x78 => Self::ResponsePending,
            0x7E => Self::SubFunctionNotSupportedInActiveSession,
            0x7F => Self::ServiceNotSupportedInActiveSession,
            _ => Self::GeneralReject,
        }
    }
}

impl fmt::Display for Nrc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?} (0x{:02X})", self, *self as u8)
    }
}

// ---------------------------------------------------------------------------
// UDS Request / Response messages
// ---------------------------------------------------------------------------

/// A UDS request message.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct UdsRequest {
    /// Service Identifier.
    pub service_id: u8,
    /// Sub-function byte (if applicable).
    pub sub_function: u8,
    /// Whether sub-function is present for this service.
    pub has_sub_function: bool,
    /// Target ECU logical address.
    pub target_addr: u32,
    /// Request data (after SID and optional sub-function).
    #[serde(with = "BigArray")]
    pub data: [u8; UDS_MAX_PAYLOAD_SIZE],
    /// Number of valid bytes in `data`.
    pub data_len: u16,
}

impl Default for UdsRequest {
    fn default() -> Self {
        Self {
            service_id: 0x3E, // TesterPresent
            sub_function: 0,
            has_sub_function: false,
            target_addr: 0,
            data: [0u8; UDS_MAX_PAYLOAD_SIZE],
            data_len: 0,
        }
    }
}

impl UdsRequest {
    /// Builds the raw ISO-TP PDU bytes for this request.
    pub fn to_bytes(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        buf[pos] = self.service_id;
        pos += 1;
        if self.has_sub_function {
            buf[pos] = self.sub_function;
            pos += 1;
        }
        let data_len = self.data_len as usize;
        buf[pos..pos + data_len].copy_from_slice(&self.data[..data_len]);
        pos + data_len
    }

    /// Parse a UDS request from raw bytes.
    pub fn from_bytes(target_addr: u32, bytes: &[u8]) -> Option<Self> {
        if bytes.is_empty() {
            return None;
        }
        let mut req = Self {
            service_id: bytes[0],
            target_addr,
            ..Default::default()
        };
        let payload = &bytes[1..];
        let copy_len = payload.len().min(UDS_MAX_PAYLOAD_SIZE);
        req.data[..copy_len].copy_from_slice(&payload[..copy_len]);
        req.data_len = copy_len as u16;
        Some(req)
    }
}

/// A UDS response message.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct UdsResponse {
    /// Positive response SID (request SID + 0x40), or 0x7F for negative.
    pub service_id: u8,
    /// Negative Response Code (only meaningful when `is_negative` is true).
    pub nrc: Nrc,
    /// Whether this is a negative response.
    pub is_negative: bool,
    /// Source ECU address.
    pub source_addr: u32,
    /// Response data.
    #[serde(with = "BigArray")]
    pub data: [u8; UDS_MAX_PAYLOAD_SIZE],
    /// Number of valid bytes in `data`.
    pub data_len: u16,
}

impl Default for UdsResponse {
    fn default() -> Self {
        Self {
            service_id: 0x7E, // TesterPresent positive
            nrc: Nrc::PositiveResponse,
            is_negative: false,
            source_addr: 0,
            data: [0u8; UDS_MAX_PAYLOAD_SIZE],
            data_len: 0,
        }
    }
}

impl UdsResponse {
    /// Creates a positive response with the given data.
    pub fn positive(request_sid: u8, source_addr: u32, data: &[u8]) -> Self {
        let mut resp = Self {
            service_id: request_sid + 0x40,
            nrc: Nrc::PositiveResponse,
            is_negative: false,
            source_addr,
            ..Default::default()
        };
        let len = data.len().min(UDS_MAX_PAYLOAD_SIZE);
        resp.data[..len].copy_from_slice(&data[..len]);
        resp.data_len = len as u16;
        resp
    }

    /// Creates a negative response.
    pub fn negative(request_sid: u8, source_addr: u32, nrc: Nrc) -> Self {
        Self {
            service_id: request_sid,
            nrc,
            is_negative: true,
            source_addr,
            data: [0u8; UDS_MAX_PAYLOAD_SIZE],
            data_len: 0,
        }
    }

    /// Build raw bytes for transmission.
    pub fn to_bytes(&self, buf: &mut [u8]) -> usize {
        if self.is_negative {
            buf[0] = 0x7F;
            buf[1] = self.service_id;
            buf[2] = self.nrc as u8;
            3
        } else {
            buf[0] = self.service_id;
            let data_len = self.data_len as usize;
            buf[1..1 + data_len].copy_from_slice(&self.data[..data_len]);
            1 + data_len
        }
    }

    /// Parse a response from raw bytes.
    pub fn from_bytes(source_addr: u32, bytes: &[u8]) -> Option<Self> {
        if bytes.is_empty() {
            return None;
        }
        if bytes[0] == 0x7F && bytes.len() >= 3 {
            Some(Self {
                service_id: bytes[1],
                nrc: Nrc::from(bytes[2]),
                is_negative: true,
                source_addr,
                ..Default::default()
            })
        } else {
            let mut resp = Self {
                service_id: bytes[0],
                is_negative: false,
                source_addr,
                ..Default::default()
            };
            let payload = &bytes[1..];
            let len = payload.len().min(UDS_MAX_PAYLOAD_SIZE);
            resp.data[..len].copy_from_slice(&payload[..len]);
            resp.data_len = len as u16;
            Some(resp)
        }
    }
}

// ---------------------------------------------------------------------------
// Session state (shared between tasks)
// ---------------------------------------------------------------------------

/// Current UDS session state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct UdsSessionState {
    /// Current active session.
    pub session_type: UdsSessionType,
    /// Security access level (0 = locked).
    pub security_level: u8,
    /// Whether S3 timer (session keep-alive) has expired.
    pub s3_expired: bool,
}

// ---------------------------------------------------------------------------
// DID (Data Identifier) types
// ---------------------------------------------------------------------------

/// A UDS Data Identifier (2-byte DID).
pub type Did = u16;

/// Standard DID ranges (ISO 14229-1 Annex C).
pub mod did_ranges {
    /// Vehicle manufacturer specific DIDs.
    pub const MANUFACTURER_START: u16 = 0xF100;
    pub const MANUFACTURER_END: u16 = 0xF1FF;
    /// System supplier specific DIDs.
    pub const SUPPLIER_START: u16 = 0xF200;
    pub const SUPPLIER_END: u16 = 0xF2FF;
    /// VIN.
    pub const VIN: u16 = 0xF190;
    /// ECU serial number.
    pub const ECU_SERIAL: u16 = 0xF18C;
    /// ECU hardware version.
    pub const ECU_HW_VERSION: u16 = 0xF191;
    /// ECU software version.
    pub const ECU_SW_VERSION: u16 = 0xF195;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uds_request_round_trip() {
        let req = UdsRequest {
            service_id: 0x22,
            has_sub_function: false,
            data: {
                let mut d = [0u8; UDS_MAX_PAYLOAD_SIZE];
                d[0] = 0xF1;
                d[1] = 0x90;
                d
            },
            data_len: 2,
            ..Default::default()
        };
        let mut buf = [0u8; 256];
        let len = req.to_bytes(&mut buf);
        assert_eq!(&buf[..len], &[0x22, 0xF1, 0x90]);
    }

    #[test]
    fn uds_positive_response() {
        let resp = UdsResponse::positive(0x22, 0x7E8, &[0xF1, 0x90, 0x41, 0x42]);
        assert!(!resp.is_negative);
        assert_eq!(resp.service_id, 0x62);
        assert_eq!(resp.data_len, 4);
    }

    #[test]
    fn uds_negative_response() {
        let resp = UdsResponse::negative(0x22, 0x7E8, Nrc::ServiceNotSupported);
        assert!(resp.is_negative);
        let mut buf = [0u8; 16];
        let len = resp.to_bytes(&mut buf);
        assert_eq!(&buf[..len], &[0x7F, 0x22, 0x11]);
    }

    #[test]
    fn uds_response_parse_negative() {
        let bytes = [0x7F, 0x22, 0x31];
        let resp = UdsResponse::from_bytes(0x7E8, &bytes).unwrap();
        assert!(resp.is_negative);
        assert_eq!(resp.nrc, Nrc::RequestOutOfRange);
    }
}
