use core::cell::RefCell;

use zenoh_proto::{Endpoint, LinkError};

pub trait ZLinkInfo {
    fn mtu(&self) -> u16;
    fn is_streamed(&self) -> bool;
}

pub trait ZLinkTx: ZLinkInfo {
    fn write_all(
        &mut self,
        buffer: &[u8],
    ) -> impl Future<Output = core::result::Result<(), zenoh_proto::LinkError>>;
}

pub trait ZLinkRx: ZLinkInfo {
    fn read(
        &mut self,
        buffer: &mut [u8],
    ) -> impl Future<Output = core::result::Result<usize, zenoh_proto::LinkError>>;

    fn read_exact(
        &mut self,
        buffer: &mut [u8],
    ) -> impl Future<Output = core::result::Result<(), zenoh_proto::LinkError>>;
}

pub trait ZLink: ZLinkInfo + ZLinkTx + ZLinkRx {
    type Tx<'link>: ZLinkTx + ZLinkInfo
    where
        Self: 'link;

    type Rx<'link>: ZLinkRx + ZLinkInfo
    where
        Self: 'link;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>);
}

pub trait ZLinkManager {
    type Link<'a>: ZLink
    where
        Self: 'a;

    fn connect(
        &self,
        endpoint: Endpoint<'_>,
    ) -> impl Future<Output = core::result::Result<Self::Link<'_>, LinkError>>;

    fn listen(
        &self,
        endpoint: Endpoint<'_>,
    ) -> impl Future<Output = core::result::Result<Self::Link<'_>, LinkError>>;
}

/// A one-shot client-side link manager for serial transports.
///
/// Serial devices are normally supplied by the board support crate rather than
/// opened from an endpoint string.  The endpoint is still checked so callers
/// cannot accidentally use this manager for a TCP or UDP session.  Taking the
/// link exactly once also makes ownership of an interrupt-driven UART explicit.
pub struct SerialLinkManager<Link> {
    link: RefCell<Option<Link>>,
}

impl<Link> SerialLinkManager<Link> {
    pub const fn new(link: Link) -> Self {
        Self {
            link: RefCell::new(Some(link)),
        }
    }
}

impl<Link> ZLinkManager for SerialLinkManager<Link>
where
    Link: ZLink,
{
    type Link<'a>
        = Link
    where
        Self: 'a;

    async fn connect(
        &self,
        endpoint: Endpoint<'_>,
    ) -> core::result::Result<Self::Link<'_>, LinkError> {
        if endpoint.protocol().as_str() != "serial" {
            return Err(LinkError::CouldNotParseProtocol);
        }

        self.link
            .borrow_mut()
            .take()
            .ok_or(LinkError::CouldNotConnect)
    }

    async fn listen(
        &self,
        _endpoint: Endpoint<'_>,
    ) -> core::result::Result<Self::Link<'_>, LinkError> {
        Err(LinkError::CouldNotListen)
    }
}

pub struct EmbeddedIOLink<Tx: embedded_io_async::Write, Rx: embedded_io_async::Read> {
    tx: Tx,
    rx: Rx,

    mtu: u16,
    is_streamed: bool,
}

impl<Tx: embedded_io_async::Write, Rx: embedded_io_async::Read> EmbeddedIOLink<Tx, Rx> {
    pub fn new(tx: Tx, rx: Rx, mtu: u16, is_streamed: bool) -> Self {
        Self {
            tx,
            rx,
            mtu,
            is_streamed,
        }
    }
}

pub struct EmbeddedIOLinkTx<'a, Tx: embedded_io_async::Write> {
    tx: &'a mut Tx,

    mtu: u16,
    is_streamed: bool,
}

pub struct EmbeddedIOLinkRx<'a, Rx: embedded_io_async::Read> {
    rx: &'a mut Rx,

    mtu: u16,
    is_streamed: bool,
}

impl<Tx: embedded_io_async::Write, Rx: embedded_io_async::Read> ZLinkInfo
    for EmbeddedIOLink<Tx, Rx>
{
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        self.is_streamed
    }
}

impl<Tx: embedded_io_async::Write> ZLinkInfo for EmbeddedIOLinkTx<'_, Tx> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        self.is_streamed
    }
}

impl<Rx: embedded_io_async::Read> ZLinkInfo for EmbeddedIOLinkRx<'_, Rx> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        self.is_streamed
    }
}

impl<Tx: embedded_io_async::Write, Rx: embedded_io_async::Read> ZLinkTx for EmbeddedIOLink<Tx, Rx> {
    async fn write_all(
        &mut self,
        buffer: &[u8],
    ) -> core::result::Result<(), zenoh_proto::LinkError> {
        self.tx
            .write_all(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl<Tx: embedded_io_async::Write> ZLinkTx for EmbeddedIOLinkTx<'_, Tx> {
    async fn write_all(
        &mut self,
        buffer: &[u8],
    ) -> core::result::Result<(), zenoh_proto::LinkError> {
        self.tx
            .write_all(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl<Tx: embedded_io_async::Write, Rx: embedded_io_async::Read> ZLinkRx for EmbeddedIOLink<Tx, Rx> {
    async fn read(
        &mut self,
        buffer: &mut [u8],
    ) -> core::result::Result<usize, zenoh_proto::LinkError> {
        self.rx
            .read(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }

    async fn read_exact(
        &mut self,
        buffer: &mut [u8],
    ) -> core::result::Result<(), zenoh_proto::LinkError> {
        self.rx
            .read_exact(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }
}

impl<Rx: embedded_io_async::Read> ZLinkRx for EmbeddedIOLinkRx<'_, Rx> {
    async fn read(
        &mut self,
        buffer: &mut [u8],
    ) -> core::result::Result<usize, zenoh_proto::LinkError> {
        self.rx
            .read(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }

    async fn read_exact(
        &mut self,
        buffer: &mut [u8],
    ) -> core::result::Result<(), zenoh_proto::LinkError> {
        self.rx
            .read_exact(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }
}

impl<Tx: embedded_io_async::Write, Rx: embedded_io_async::Read> ZLink for EmbeddedIOLink<Tx, Rx> {
    type Tx<'link>
        = EmbeddedIOLinkTx<'link, Tx>
    where
        Self: 'link;

    type Rx<'link>
        = EmbeddedIOLinkRx<'link, Rx>
    where
        Self: 'link;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
        let Self {
            tx,
            rx,
            mtu,
            is_streamed,
        } = self;

        (
            EmbeddedIOLinkTx {
                tx,
                mtu: *mtu,
                is_streamed: *is_streamed,
            },
            EmbeddedIOLinkRx {
                rx,
                mtu: *mtu,
                is_streamed: *is_streamed,
            },
        )
    }
}
