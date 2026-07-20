use core::cell::RefCell;

use embassy_net::udp::{UdpMetadata, UdpSocket};
use zenoh_nostd::platform::*;

use crate::BufferPoolDrop;

pub struct EmbassyUdpLink<'net> {
    socket: UdpSocket<'net>,
    addr: UdpMetadata,
    mtu: u16,

    idx1: usize,
    pool1: &'net RefCell<dyn BufferPoolDrop>,

    idx2: usize,
    pool2: &'net RefCell<dyn BufferPoolDrop>,
}

impl<'net> EmbassyUdpLink<'net> {
    pub(crate) fn new(
        socket: UdpSocket<'net>,
        metadata: UdpMetadata,
        mtu: u16,
        idx1: usize,
        pool1: &'net RefCell<dyn BufferPoolDrop>,
        idx2: usize,
        pool2: &'net RefCell<dyn BufferPoolDrop>,
    ) -> Self {
        Self {
            socket,
            addr: metadata,
            mtu,
            idx1,
            pool1,
            idx2,
            pool2,
        }
    }
}

impl Drop for EmbassyUdpLink<'_> {
    fn drop(&mut self) {
        self.pool1.borrow_mut().release(self.idx1);
        self.pool2.borrow_mut().release(self.idx2);
    }
}

pub struct EmbassyUdpLinkTx<'link> {
    socket: &'link UdpSocket<'link>,
    mtu: u16,
    addr: UdpMetadata,
}

pub struct EmbassyUdpLinkRx<'link> {
    socket: &'link UdpSocket<'link>,
    mtu: u16,
}

impl<'net> ZLinkInfo for EmbassyUdpLink<'net> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl<'link> ZLinkInfo for EmbassyUdpLinkTx<'link> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl<'link> ZLinkInfo for EmbassyUdpLinkRx<'link> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl<'net> ZLinkTx for EmbassyUdpLink<'net> {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .send_to(buffer, self.addr)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl<'link> ZLinkTx for EmbassyUdpLinkTx<'link> {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .send_to(buffer, self.addr)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl<'net> ZLinkRx for EmbassyUdpLink<'net> {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.socket
            .recv_from(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
            .map(|m| m.0)
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .recv_from(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
            .map(|_| ())
    }
}

impl<'link> ZLinkRx for EmbassyUdpLinkRx<'link> {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.socket
            .recv_from(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
            .map(|m| m.0)
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .recv_from(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
            .map(|_| ())
    }
}

impl<'net> ZLink for EmbassyUdpLink<'net> {
    type Tx<'link>
        = EmbassyUdpLinkTx<'link>
    where
        'net: 'link;

    type Rx<'link>
        = EmbassyUdpLinkRx<'link>
    where
        'net: 'link;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
        let tx = EmbassyUdpLinkTx {
            socket: &self.socket,
            mtu: self.mtu,
            addr: self.addr,
        };
        let rx = EmbassyUdpLinkRx {
            socket: &self.socket,
            mtu: self.mtu,
        };
        (tx, rx)
    }
}
