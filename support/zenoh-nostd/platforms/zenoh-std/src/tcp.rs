use futures_lite::{AsyncReadExt, AsyncWriteExt};

use zenoh_nostd::platform::*;

pub struct StdTcpLink {
    stream: async_net::TcpStream,
    mtu: u16,
}

impl StdTcpLink {
    pub fn new(stream: async_net::TcpStream, mtu: u16) -> Self {
        Self { stream, mtu }
    }
}

pub struct StdTcpLinkTx {
    stream: async_net::TcpStream,
    mtu: u16,
}

pub struct StdTcpLinkRx {
    stream: async_net::TcpStream,
    mtu: u16,
}

impl ZLinkInfo for StdTcpLink {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        true
    }
}

impl ZLinkInfo for StdTcpLinkTx {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        true
    }
}

impl ZLinkInfo for StdTcpLinkRx {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        true
    }
}

impl ZLinkTx for StdTcpLink {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.stream
            .write_all(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl ZLinkTx for StdTcpLinkTx {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.stream
            .write_all(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl ZLinkRx for StdTcpLink {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.stream
            .read(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.stream
            .read_exact(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl ZLinkRx for StdTcpLinkRx {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.stream
            .read(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.stream
            .read_exact(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl ZLink for StdTcpLink {
    type Tx<'a> = StdTcpLinkTx;
    type Rx<'a> = StdTcpLinkRx;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
        let tx = StdTcpLinkTx {
            stream: self.stream.clone(),
            mtu: self.mtu,
        };

        let rx = StdTcpLinkRx {
            stream: self.stream.clone(),
            mtu: self.mtu,
        };

        (tx, rx)
    }
}
