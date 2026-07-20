//! This module defines all the errors that should exists in a `zenoh` context.

crate::declare_zerror! {
    #[doc = "Errors related to zenoh bytes."]
    enum BytesError {
        #[doc = "Source buffer is empty."]
        #[err = "source buffer is empty"]
        SrcIsEmpty = 10,
        #[doc = "Destination buffer is full."]
        #[err = "destination buffer is full"]
        DstIsFull = 11,
        #[doc = "Destination buffer is too small."]
        #[err = "destination buffer is too small"]
        DstIsTooSmall = 12,
        #[doc = "Source buffer is too small."]
        #[err = "source buffer is too small"]
        SrcIsTooSmall = 13,
        // Reserved: 14-19 for future BytesError variants
    }

    #[doc = "Errors related to zenoh codec."]
    enum CodecError: BytesError {
        #[doc = "Could not complete a read operation."]
        #[err = "could not read"]
        CouldNotRead = 20,
        #[doc = "Could not complete a write operation."]
        #[err = "could not write"]
        CouldNotWrite = 21,
        #[doc = "Could not parse the header."]
        #[err = "could not parse header"]
        CouldNotParseHeader = 22,
        #[doc = "Could not parse the field."]
        #[err = "could not parse field"]
        CouldNotParseField = 23,
        #[doc = "Could not read the extension."]
        #[err = "could not read extension"]
        CouldNotReadExtension = 24,
        // Reserved: 25-39 for future CodecError variants
    }

    #[doc = "Errors related to zenoh key expression parsing."]
    pub enum KeyexprError {
        #[doc = "A lone `$*` was found in an expression."]
        #[err = "lone '$*' in expression"]
        LoneDollarStar = 40,
        #[doc = "A single `*` was found after a `**` in an expression."]
        #[err = "single '*' after '**' in expression"]
        SingleStarAfterDoubleStar = 41,
        #[doc = "A double `**` was found after a `**` in an expression."]
        #[err = "double '**' after '**' in expression"]
        DoubleStarAfterDoubleStar = 42,
        #[doc = "An empty chunk was found in an expression."]
        #[err = "empty chunk in expression"]
        EmptyChunk = 43,
        #[doc = "A `*` was found in the middle of a chunk in an expression."]
        #[err = "'*' in middle of chunk in expression"]
        StarInChunk = 44,
        #[doc = "A `$` was found after another `$` in an expression."]
        #[err = "'$' after '$' in expression"]
        DollarAfterDollar = 45,
        #[doc = "A `#` or `?` was found in an expression."]
        #[err = "'#' or '?' in expression"]
        SharpOrQMark = 46,
        #[doc = "An unbound `$n` was found in an expression."]
        #[err = "unbound '$n' in expression"]
        UnboundDollar = 47,
        #[doc = "A wildcard chunk was found where it is not allowed."]
        #[err = "wildcard chunk not allowed"]
        WildChunk = 48,
        // Reserved: 49-59 for future KeyexprError variants
    }

    #[doc = "Errors related to zenoh endpoints."]
    enum EndpointError {
        #[doc = "Missing protocol separator in endpoint."]
        #[err = "missing protocol separator in endpoint"]
        NoProtocolSeparator = 60,
        #[doc = "Metadata is not supported in endpoint."]
        #[err = "metadata not supported in endpoint"]
        MetadataNotSupported = 61,
        #[doc = "Configuration is not supported in endpoint."]
        #[err = "configuration not supported in endpoint"]
        ConfigNotSupported = 62,
        #[doc = "Could not parse the endpoint address."]
        #[err = "could not parse endpoint address"]
        CouldNotParseAddress = 63,
        #[doc = "Could not parse the endpoint protocol."]
        #[err = "could not parse endpoint protocol"]
        CouldNotParseProtocol = 64,
        // Reserved: 65-79 for future EndpointError variants
    }

    #[doc = "Errors related to zenoh transports."]
    enum TransportError: CodecError {
        #[doc = "The RX part of the transport is full (not large enough to receive this data)."]
        #[err = "transport rx is full"]
        TransportRxFull = 80,
        #[doc = "The Tx part of the transport is full (not large enough to encode this msg)."]
        #[err = "transport tx is too small"]
        TransportTxFull = 81,
        #[doc = "The current transport state can't handle this message."]
        #[err = "current state incompatible with message"]
        InvalidState = 82,
        #[doc = "Received an invalid attribute."]
        #[err = "invalid attribute in message"]
        InvalidAttribute = 83,
        // Reserved: 84-99 for future TransportError variants
    }
    #[doc = "Errors related to zenoh links."]
    enum LinkError: EndpointError {
        #[doc = "Could not get address info."]
        #[err = "could not get address info"]
        CouldNotGetAddrInfo = 100,
        #[doc = "Could not connect to the remote."]
        #[err = "could not connect to remote"]
        CouldNotConnect = 101,
        #[doc = "Link transmission failed."]
        #[err = "link transmission failed"]
        LinkTxFailed = 102,
        #[doc = "Link reception failed."]
        #[err = "link reception failed"]
        LinkRxFailed = 103,
        #[doc = "Could not listen for connection."]
        #[err = "could not listen"]
        CouldNotListen = 104,
        // Reserved: 105-119 for future LinkError variants
    }

    #[doc = "Errors related to zenoh transport links."]
    enum TransportLinkError: TransportError + LinkError {
        #[doc = "Couldn't connect in time."]
        #[err = "open timeout"]
        OpenTimeout = 120,
        #[doc = "Didn't receive any data from peer within lease period."]
        #[err = "rx lease expired"]
        RxLeaseExpired = 121,
        #[doc = "Didn't send any data to peer within lease period."]
        #[err = "tx lease expired"]
        TxLeaseExpired = 122,
        #[doc = "Transport has been closed."]
        #[err = "transport has been closed"]
        TransportClosed = 123,
        // Reserved: 124-139 for future TransportLinkError variants
    }

    #[doc = "Errors related to zenoh collections."]
    enum CollectionError {
        #[doc = "Key not found in collection."]
        #[err = "key not found in collection"]
        KeyNotFound = 140,
        #[doc = "Key already exists in collection."]
        #[err = "key already exists in collection"]
        KeyAlreadyExists = 141,
        #[doc = "Collection is full."]
        #[err = "collection is full"]
        CollectionIsFull = 142,
        #[doc = "Collection is empty."]
        #[err = "collection is empty"]
        CollectionIsEmpty = 143,
        #[doc = "Collection too small."]
        #[err = "collection is too small"]
        CollectionTooSmall = 144,
        // Reserved: 145-149 for future CollectionError variants
    }

    #[doc = "Errors related to zenoh session."]
    enum SessionError: TransportLinkError + CollectionError + KeyexprError {
        #[doc = "Request timed out."]
        #[err = "request timed out"]
        RequestTimedout = 150,
        // Reserved: 151-159 for future SessionError variants
    }

    #[doc = "Errors related to zenoh broker."]
    enum BrokerError: TransportLinkError + CollectionError + KeyexprError {
        // Reserved: 160-169 for future BrokerError variants
    }
}

#[derive(Debug)]
pub enum EitherError<A, B> {
    A(A),
    B(B),
}

impl<A> EitherError<A, A> {
    pub fn flatten(self) -> A {
        match self {
            Self::A(x) => x,
            Self::B(x) => x,
        }
    }
}

impl<A, B> EitherError<A, B> {
    pub fn flatten_map<C>(self) -> C
    where
        C: From<A> + From<B>,
    {
        match self {
            EitherError::A(x) => x.into(),
            EitherError::B(x) => x.into(),
        }
    }
}

impl<A, B> From<A> for EitherError<A, B> {
    fn from(value: A) -> Self {
        Self::A(value)
    }
}

#[macro_export]
macro_rules! zbail {
    ($err:expr) => {
        return Err($err.into())
    };

    (@log $err:expr) => {{
        $crate::error!("{}: {}", $err, $crate::zctx!());
        $crate::zbail!($err)
    }};

    ($err:expr, $($arg:tt)+) => {{
        $crate::error!("{}: {}", $err, $crate::zctx!());
        $crate::error!($($arg)+);
        $crate::zbail!($err);
    }};

    (@continue $err:expr) => {{
        $crate::error!("{}: {}", $err, $crate::zctx!());
        continue;
    }};

    (@None $err:expr) => {{
        $crate::error!("{}: {}", $err, $crate::zctx!());
        return None;
    }};

    (@ret $ret:expr, $err:expr) => {{
        $crate::error!("{}: {}", $err, $crate::zctx!());
        return $ret;
    }};
}

#[macro_export]
macro_rules! zctx {
    () => {
        concat!(core::file!(), ":", core::line!(), ":", core::column!(),)
    };
}
