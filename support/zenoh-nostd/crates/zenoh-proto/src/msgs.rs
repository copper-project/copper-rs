pub mod exts;
pub mod fields;

mod err;
mod put;
mod query;
mod reply;

mod declare;
mod interest;
mod push;
mod request;
mod response;

mod close;
mod frame;
mod init;
mod keepalive;
mod open;

pub use err::*;
pub use put::*;
pub use query::*;
pub use reply::*;

pub use declare::*;
pub use interest::*;
pub use push::*;
pub use request::*;
pub use response::*;

pub use close::*;
pub use frame::*;
pub use init::*;
pub use keepalive::*;
pub use open::*;

use zenoh_derive::ZEnum;

#[derive(ZEnum, Debug, PartialEq)]
pub enum NetworkBody<'a> {
    Push(Push<'a>),
    Request(Request<'a>),
    Response(Response<'a>),
    ResponseFinal(ResponseFinal),
    Interest(Interest<'a>),
    InterestFinal(InterestFinal),
    Declare(Declare<'a>),
}

impl NetworkBody<'_> {
    pub fn as_ref(&self) -> NetworkBodyRef<'_> {
        match self {
            NetworkBody::Push(x) => NetworkBodyRef::Push(x),
            NetworkBody::Request(x) => NetworkBodyRef::Request(x),
            NetworkBody::Response(x) => NetworkBodyRef::Response(x),
            NetworkBody::ResponseFinal(x) => NetworkBodyRef::ResponseFinal(x),
            NetworkBody::Interest(x) => NetworkBodyRef::Interest(x),
            NetworkBody::InterestFinal(x) => NetworkBodyRef::InterestFinal(x),
            NetworkBody::Declare(x) => NetworkBodyRef::Declare(x),
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum NetworkBodyRef<'a> {
    Push(&'a Push<'a>),
    Request(&'a Request<'a>),
    Response(&'a Response<'a>),
    ResponseFinal(&'a ResponseFinal),
    Interest(&'a Interest<'a>),
    InterestFinal(&'a InterestFinal),
    Declare(&'a Declare<'a>),
}

impl<'a> crate::ZBodyLen for NetworkBodyRef<'a> {
    fn z_body_len(&self) -> usize {
        match self {
            Self::Push(x) => <Push as crate::ZBodyLen>::z_body_len(x),
            Self::Request(x) => <Request as crate::ZBodyLen>::z_body_len(x),
            Self::Response(x) => <Response as crate::ZBodyLen>::z_body_len(x),
            Self::ResponseFinal(x) => <ResponseFinal as crate::ZBodyLen>::z_body_len(x),
            Self::Interest(x) => <Interest as crate::ZBodyLen>::z_body_len(x),
            Self::InterestFinal(x) => <InterestFinal as crate::ZBodyLen>::z_body_len(x),
            Self::Declare(x) => <Declare as crate::ZBodyLen>::z_body_len(x),
        }
    }
}

impl<'a> crate::ZLen for NetworkBodyRef<'a> {
    fn z_len(&self) -> usize {
        1 + <Self as crate::ZBodyLen>::z_body_len(self)
    }
}

impl<'a> crate::ZBodyEncode for NetworkBodyRef<'a> {
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        match self {
            Self::Push(x) => <Push as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::Request(x) => <Request as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::Response(x) => <Response as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::ResponseFinal(x) => <ResponseFinal as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::Interest(x) => <Interest as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::InterestFinal(x) => <InterestFinal as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::Declare(x) => <Declare as crate::ZBodyEncode>::z_body_encode(x, w),
        }
    }
}

impl<'a> crate::ZEncode for NetworkBodyRef<'a> {
    fn z_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        match self {
            Self::Push(x) => <Push as crate::ZEncode>::z_encode(x, w),
            Self::Request(x) => <Request as crate::ZEncode>::z_encode(x, w),
            Self::Response(x) => <Response as crate::ZEncode>::z_encode(x, w),
            Self::ResponseFinal(x) => <ResponseFinal as crate::ZEncode>::z_encode(x, w),
            Self::Interest(x) => <Interest as crate::ZEncode>::z_encode(x, w),
            Self::InterestFinal(x) => <InterestFinal as crate::ZEncode>::z_encode(x, w),
            Self::Declare(x) => <Declare as crate::ZEncode>::z_encode(x, w),
        }
    }
}

#[derive(Debug, PartialEq)]
pub struct NetworkMessage<'a> {
    pub reliability: fields::Reliability,
    pub qos: exts::QoS,
    pub body: NetworkBody<'a>,
}

impl NetworkMessage<'_> {
    pub fn as_ref(&self) -> NetworkMessageRef<'_> {
        NetworkMessageRef {
            reliability: self.reliability,
            qos: self.qos,
            body: self.body.as_ref(),
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct NetworkMessageRef<'a> {
    pub reliability: fields::Reliability,
    pub qos: exts::QoS,
    pub body: NetworkBodyRef<'a>,
}

#[derive(ZEnum, Debug, PartialEq)]
pub enum TransportMessage<'a> {
    Close(Close),
    InitSyn(InitSyn<'a>),
    InitAck(InitAck<'a>),
    KeepAlive(KeepAlive),
    OpenSyn(OpenSyn<'a>),
    OpenAck(OpenAck<'a>),
}

impl TransportMessage<'_> {
    pub fn as_ref(&self) -> TransportMessageRef<'_> {
        match self {
            TransportMessage::Close(x) => TransportMessageRef::Close(x),
            TransportMessage::InitSyn(x) => TransportMessageRef::InitSyn(x),
            TransportMessage::InitAck(x) => TransportMessageRef::InitAck(x),
            TransportMessage::KeepAlive(x) => TransportMessageRef::KeepAlive(x),
            TransportMessage::OpenSyn(x) => TransportMessageRef::OpenSyn(x),
            TransportMessage::OpenAck(x) => TransportMessageRef::OpenAck(x),
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum TransportMessageRef<'a> {
    Close(&'a Close),
    InitSyn(&'a InitSyn<'a>),
    InitAck(&'a InitAck<'a>),
    KeepAlive(&'a KeepAlive),
    OpenSyn(&'a OpenSyn<'a>),
    OpenAck(&'a OpenAck<'a>),
}

impl<'a> crate::ZBodyLen for TransportMessageRef<'a> {
    fn z_body_len(&self) -> usize {
        match self {
            Self::Close(x) => <Close as crate::ZBodyLen>::z_body_len(x),
            Self::InitSyn(x) => <InitSyn as crate::ZBodyLen>::z_body_len(x),
            Self::InitAck(x) => <InitAck as crate::ZBodyLen>::z_body_len(x),
            Self::KeepAlive(x) => <KeepAlive as crate::ZBodyLen>::z_body_len(x),
            Self::OpenSyn(x) => <OpenSyn as crate::ZBodyLen>::z_body_len(x),
            Self::OpenAck(x) => <OpenAck as crate::ZBodyLen>::z_body_len(x),
        }
    }
}

impl<'a> crate::ZLen for TransportMessageRef<'a> {
    fn z_len(&self) -> usize {
        1 + <Self as crate::ZBodyLen>::z_body_len(self)
    }
}

impl<'a> crate::ZBodyEncode for TransportMessageRef<'a> {
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        match self {
            Self::Close(x) => <Close as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::InitSyn(x) => <InitSyn as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::InitAck(x) => <InitAck as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::KeepAlive(x) => <KeepAlive as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::OpenSyn(x) => <OpenSyn as crate::ZBodyEncode>::z_body_encode(x, w),
            Self::OpenAck(x) => <OpenAck as crate::ZBodyEncode>::z_body_encode(x, w),
        }
    }
}

impl<'a> crate::ZEncode for TransportMessageRef<'a> {
    fn z_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        match self {
            Self::Close(x) => <Close as crate::ZEncode>::z_encode(x, w),
            Self::InitSyn(x) => <InitSyn as crate::ZEncode>::z_encode(x, w),
            Self::InitAck(x) => <InitAck as crate::ZEncode>::z_encode(x, w),
            Self::KeepAlive(x) => <KeepAlive as crate::ZEncode>::z_encode(x, w),
            Self::OpenSyn(x) => <OpenSyn as crate::ZEncode>::z_encode(x, w),
            Self::OpenAck(x) => <OpenAck as crate::ZEncode>::z_encode(x, w),
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum Message<'a> {
    Network(NetworkMessage<'a>),
    Transport(TransportMessage<'a>),
}

#[derive(Debug, PartialEq)]
pub enum MessageRef<'a> {
    Network(NetworkMessageRef<'a>),
    Transport(TransportMessageRef<'a>),
}

impl Message<'_> {
    pub fn as_ref(&self) -> MessageRef<'_> {
        match self {
            Message::Transport(msg) => MessageRef::Transport(msg.as_ref()),
            Message::Network(msg) => MessageRef::Network(msg.as_ref()),
        }
    }
}
