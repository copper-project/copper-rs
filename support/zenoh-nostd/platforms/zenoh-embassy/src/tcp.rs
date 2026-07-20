use core::cell::RefCell;

use embassy_net::tcp::{TcpReader, TcpSocket, TcpWriter};
use embedded_io_async::{Read, Write};
use zenoh_nostd::platform::*;

use crate::BufferPoolDrop;

pub struct EmbassyTcpLink<'net> {
    socket: TcpSocket<'net>,
    mtu: u16,

    idx: usize,
    pool: &'net RefCell<dyn BufferPoolDrop>,
}

impl<'net> EmbassyTcpLink<'net> {
    pub(crate) fn new(
        socket: TcpSocket<'net>,
        mtu: u16,
        idx: usize,
        pool: &'net RefCell<dyn BufferPoolDrop>,
    ) -> Self {
        Self {
            socket,
            mtu,
            idx,
            pool,
        }
    }
}

impl Drop for EmbassyTcpLink<'_> {
    fn drop(&mut self) {
        self.pool.borrow_mut().release(self.idx);
    }
}

pub struct EmbassyTcpLinkTx<'net> {
    socket: TcpWriter<'net>,
    mtu: u16,
}

pub struct EmbassyTcpLinkRx<'net> {
    socket: TcpReader<'net>,
    mtu: u16,
}

impl<'net> ZLinkInfo for EmbassyTcpLink<'net> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        true
    }
}

impl<'net> ZLinkInfo for EmbassyTcpLinkTx<'net> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        true
    }
}

impl<'net> ZLinkInfo for EmbassyTcpLinkRx<'net> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        true
    }
}

impl<'net> ZLinkTx for EmbassyTcpLink<'net> {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .write_all(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl<'net> ZLinkTx for EmbassyTcpLinkTx<'net> {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .write_all(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl<'net> ZLinkRx for EmbassyTcpLink<'net> {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.socket
            .read(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .read_exact(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }
}

impl<'net> ZLinkRx for EmbassyTcpLinkRx<'net> {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.socket
            .read(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .read_exact(buffer)
            .await
            .map_err(|_| LinkError::LinkRxFailed)
    }
}

impl<'net> ZLink for EmbassyTcpLink<'net> {
    type Tx<'b>
        = EmbassyTcpLinkTx<'b>
    where
        Self: 'b;

    type Rx<'b>
        = EmbassyTcpLinkRx<'b>
    where
        Self: 'b;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
        let (rx, tx) = self.socket.split();
        let tx = EmbassyTcpLinkTx {
            socket: tx,
            mtu: self.mtu,
        };
        let rx = EmbassyTcpLinkRx {
            socket: rx,
            mtu: self.mtu,
        };
        (tx, rx)
    }
}
