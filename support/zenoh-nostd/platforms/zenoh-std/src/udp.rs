use zenoh_nostd::platform::*;

pub struct StdUdpLink {
    socket: async_net::UdpSocket,
    mtu: u16,
}

impl StdUdpLink {
    pub fn new(socket: async_net::UdpSocket, mtu: u16) -> Self {
        Self { socket, mtu }
    }
}

pub struct StdUdpLinkTx {
    socket: async_net::UdpSocket,
    mtu: u16,
}

pub struct StdUdpLinkRx {
    socket: async_net::UdpSocket,
    mtu: u16,
}

impl ZLinkInfo for StdUdpLink {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkInfo for StdUdpLinkTx {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkInfo for StdUdpLinkRx {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkTx for StdUdpLink {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .send(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
            .map(|_| ())
    }
}

impl ZLinkTx for StdUdpLinkTx {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.socket
            .send(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
            .map(|_| ())
    }
}

impl ZLinkRx for StdUdpLink {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.socket
            .recv(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }

    async fn read_exact(&mut self, _: &mut [u8]) -> core::result::Result<(), LinkError> {
        unimplemented!()
    }
}

impl ZLinkRx for StdUdpLinkRx {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.socket
            .recv(buffer)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }

    async fn read_exact(&mut self, _: &mut [u8]) -> core::result::Result<(), LinkError> {
        unimplemented!()
    }
}

impl ZLink for StdUdpLink {
    type Tx<'a> = StdUdpLinkTx;
    type Rx<'a> = StdUdpLinkRx;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
        let tx = StdUdpLinkTx {
            socket: self.socket.clone(),
            mtu: self.mtu,
        };

        let rx = StdUdpLinkRx {
            socket: self.socket.clone(),
            mtu: self.mtu,
        };

        (tx, rx)
    }
}
