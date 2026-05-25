use core::fmt;

const ANCHOR_RCV_PREFIX: &str = "+ANCHOR_RCV=";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AnchorReceiveEvent<'a> {
    pub peer_id: &'a str,
    pub distance_cm: u32,
    pub rssi_dbm: Option<i16>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModemEvent<'a> {
    AnchorReceive(AnchorReceiveEvent<'a>),
    CommandOk,
    Ready,
    Reset,
    Error,
    Other(&'a str),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    MissingComma,
    InvalidUtf8,
    InvalidPayloadLength,
    InvalidPayloadBytes,
    InvalidDistance,
    InvalidRssi,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingComma => write!(f, "missing comma in modem line"),
            Self::InvalidUtf8 => write!(f, "modem line is not valid UTF-8"),
            Self::InvalidPayloadLength => write!(f, "invalid anchor payload length"),
            Self::InvalidPayloadBytes => write!(f, "anchor payload bytes do not match length"),
            Self::InvalidDistance => write!(f, "invalid anchor distance field"),
            Self::InvalidRssi => write!(f, "invalid anchor RSSI field"),
        }
    }
}

impl core::error::Error for ParseError {}

pub fn parse_line(line: &str) -> Result<ModemEvent<'_>, ParseError> {
    let line = line.trim();

    if line.is_empty() {
        return Ok(ModemEvent::Other(line));
    }
    if line == "+OK" {
        return Ok(ModemEvent::CommandOk);
    }
    if line == "+READY" {
        return Ok(ModemEvent::Ready);
    }
    if line == "+RESET" {
        return Ok(ModemEvent::Reset);
    }
    if line.starts_with("+ERR") {
        return Ok(ModemEvent::Error);
    }
    if let Some(rest) = line.strip_prefix(ANCHOR_RCV_PREFIX) {
        return parse_anchor_receive(rest).map(ModemEvent::AnchorReceive);
    }

    Ok(ModemEvent::Other(line))
}

fn parse_anchor_receive(rest: &str) -> Result<AnchorReceiveEvent<'_>, ParseError> {
    let first_comma = rest.find(',').ok_or(ParseError::MissingComma)?;
    let peer_id = &rest[..first_comma];

    let remaining = &rest[first_comma + 1..];
    let second_comma = remaining.find(',').ok_or(ParseError::MissingComma)?;
    let payload_len: usize = remaining[..second_comma]
        .trim()
        .parse()
        .map_err(|_| ParseError::InvalidPayloadLength)?;

    let payload_and_tail = &remaining[second_comma + 1..];
    let payload_and_tail_bytes = payload_and_tail.as_bytes();
    if payload_and_tail_bytes.len() < payload_len + 1 {
        return Err(ParseError::InvalidPayloadBytes);
    }
    if payload_and_tail_bytes[payload_len] != b',' {
        return Err(ParseError::InvalidPayloadBytes);
    }
    let _payload = core::str::from_utf8(&payload_and_tail_bytes[..payload_len])
        .map_err(|_| ParseError::InvalidUtf8)?;

    let tail = core::str::from_utf8(&payload_and_tail_bytes[payload_len + 1..])
        .map_err(|_| ParseError::InvalidUtf8)?;
    let (distance_field, rssi_field) = match tail.split_once(',') {
        Some((distance, rssi)) => (distance, Some(rssi)),
        None => (tail, None),
    };

    let distance_cm = parse_distance_cm(distance_field)?;
    let rssi_dbm = rssi_field.map(parse_rssi_dbm).transpose()?;

    Ok(AnchorReceiveEvent {
        peer_id: peer_id.trim(),
        distance_cm,
        rssi_dbm,
    })
}

fn parse_distance_cm(field: &str) -> Result<u32, ParseError> {
    let field = field.trim();
    let field = field.strip_suffix("cm").unwrap_or(field);
    let field = field.trim();
    field.parse().map_err(|_| ParseError::InvalidDistance)
}

fn parse_rssi_dbm(field: &str) -> Result<i16, ParseError> {
    let field = field.trim();
    let field = field.strip_suffix("dBm").unwrap_or(field);
    let field = field.strip_suffix("dbm").unwrap_or(field);
    let field = field.trim();
    field.parse().map_err(|_| ParseError::InvalidRssi)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_anchor_receive_without_rssi() {
        let event = parse_line("+ANCHOR_RCV=DAVID123,5,HELLO,40 cm").unwrap();
        assert_eq!(
            event,
            ModemEvent::AnchorReceive(AnchorReceiveEvent {
                peer_id: "DAVID123",
                distance_cm: 40,
                rssi_dbm: None,
            })
        );
    }

    #[test]
    fn parses_anchor_receive_with_rssi_and_comma_payload() {
        let event = parse_line("+ANCHOR_RCV=DAVID123,5,HE,LO,40 cm,-71 dBm").unwrap();
        assert_eq!(
            event,
            ModemEvent::AnchorReceive(AnchorReceiveEvent {
                peer_id: "DAVID123",
                distance_cm: 40,
                rssi_dbm: Some(-71),
            })
        );
    }
}
