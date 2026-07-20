use core::{convert::TryFrom, fmt, net::SocketAddr, str::FromStr};

const PROTO_SEPARATOR: char = '/';
const METADATA_SEPARATOR: char = '?';
const CONFIG_SEPARATOR: char = '#';

fn protocol(s: &str) -> &str {
    let pdix = s.find(PROTO_SEPARATOR).unwrap_or(s.len());
    &s[..pdix]
}

fn address(s: &str) -> &str {
    let pdix = s.find(PROTO_SEPARATOR).unwrap_or(s.len());
    let midx = s.find(METADATA_SEPARATOR).unwrap_or(s.len());
    let cidx = s.find(CONFIG_SEPARATOR).unwrap_or(s.len());
    &s[pdix + 1..midx.min(cidx)]
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub enum ProtocolId {
    Tcp,
    Udp,
    WebSocket,
    Serial,
}

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub struct Protocol<'a>(&'a str);

impl<'a> Protocol<'a> {
    pub fn as_str(&self) -> &'_ str {
        self.0
    }
}

impl AsRef<str> for Protocol<'_> {
    fn as_ref(&self) -> &str {
        self.as_str()
    }
}

impl fmt::Display for Protocol<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl fmt::Debug for Protocol<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}

impl TryFrom<Protocol<'_>> for ProtocolId {
    type Error = crate::EndpointError;

    fn try_from(value: Protocol<'_>) -> Result<Self, Self::Error> {
        Ok(match value.as_str() {
            "tcp" => Self::Tcp,
            "udp" => Self::Udp,
            "ws" => Self::WebSocket,
            "serial" => Self::Serial,
            _ => crate::zbail!(crate::EndpointError::CouldNotParseProtocol),
        })
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub struct Address<'a>(&'a str);

impl<'a> Address<'a> {
    pub fn as_str(&self) -> &'_ str {
        self.0
    }
}

impl AsRef<str> for Address<'_> {
    fn as_ref(&self) -> &str {
        self.as_str()
    }
}

impl fmt::Display for Address<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl fmt::Debug for Address<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}

impl<'a> From<&'a str> for Address<'a> {
    fn from(value: &'a str) -> Self {
        Address(value)
    }
}

impl TryFrom<Address<'_>> for SocketAddr {
    type Error = crate::EndpointError;

    fn try_from(value: Address<'_>) -> Result<Self, Self::Error> {
        SocketAddr::from_str(value.as_str()).map_err(|_| crate::EndpointError::CouldNotParseAddress)
    }
}

#[derive(Clone, PartialEq, Eq, Hash)]
pub struct Endpoint<'a> {
    pub(super) inner: &'a str,
}

impl Endpoint<'_> {
    pub fn protocol(&self) -> Protocol<'_> {
        Protocol(protocol(self.inner))
    }

    pub fn address(&self) -> Address<'_> {
        Address(address(self.inner))
    }
}

impl fmt::Display for Endpoint<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.inner)
    }
}

impl fmt::Debug for Endpoint<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}

impl<'a> TryFrom<&'a str> for Endpoint<'a> {
    type Error = crate::EndpointError;

    fn try_from(s: &'a str) -> Result<Self, Self::Error> {
        let pidx = s
            .find(PROTO_SEPARATOR)
            .and_then(|i| (!s[..i].is_empty() && !s[i + 1..].is_empty()).then_some(i))
            .ok_or(crate::EndpointError::NoProtocolSeparator)?;

        match (s.find(METADATA_SEPARATOR), s.find(CONFIG_SEPARATOR)) {
            (None, None) => Ok(Endpoint { inner: s }),

            (Some(midx), None) if midx > pidx && !s[midx + 1..].is_empty() => {
                crate::zbail!(crate::EndpointError::MetadataNotSupported)
            }

            (None, Some(cidx)) if cidx > pidx && !s[cidx + 1..].is_empty() => {
                crate::zbail!(crate::EndpointError::ConfigNotSupported)
            }

            (Some(midx), Some(cidx))
                if midx > pidx
                    && cidx > midx
                    && !s[midx + 1..cidx].is_empty()
                    && !s[cidx + 1..].is_empty() =>
            {
                crate::zbail!(crate::EndpointError::MetadataNotSupported)
            }
            _ => Err(crate::EndpointError::MetadataNotSupported),
        }
    }
}
