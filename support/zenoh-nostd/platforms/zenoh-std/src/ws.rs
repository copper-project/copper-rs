use {
    async_net::TcpStream,
    wtx::{
        collection::Vector,
        rng::Xorshift64,
        web_socket::{
            Frame, OpCode, WebSocketPayloadOrigin, WebSocketReaderOwned, WebSocketWriterOwned,
        },
    },
    zenoh_nostd::platform::*,
};

pub struct StdWsLink {
    stream: WebSocketReaderOwned<(), Xorshift64, TcpStream, true>,
    sink: WebSocketWriterOwned<(), Xorshift64, TcpStream, true>,
    read_buffer: Vector<u8>,
    write_buffer: Vector<u8>,
    mtu: u16,
}

impl StdWsLink {
    pub fn new(
        stream: WebSocketReaderOwned<(), Xorshift64, TcpStream, true>,
        sink: WebSocketWriterOwned<(), Xorshift64, TcpStream, true>,
        mtu: u16,
    ) -> Self {
        Self {
            stream,
            sink,
            read_buffer: Vector::new(),
            write_buffer: Vector::new(),
            mtu,
        }
    }
}

pub struct StdWsLinkTx<'a> {
    sink: &'a mut WebSocketWriterOwned<(), Xorshift64, TcpStream, true>,
    write_buffer: &'a mut Vector<u8>,
    mtu: u16,
}

pub struct StdWsLinkRx<'a> {
    stream: &'a mut WebSocketReaderOwned<(), Xorshift64, TcpStream, true>,
    read_buffer: &'a mut Vector<u8>,
    mtu: u16,
}

impl ZLinkInfo for StdWsLink {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkInfo for StdWsLinkTx<'_> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkInfo for StdWsLinkRx<'_> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkTx for StdWsLink {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.write_buffer.clear();
        self.write_buffer
            .extend_from_copyable_slice(buffer)
            .map_err(|e| {
                zenoh::error!("Failed to extend write buffer: {}", e);
                LinkError::LinkTxFailed
            })?;

        let payload = self.write_buffer.as_slice_mut();

        self.sink
            .write_frame(&mut Frame::new_fin(OpCode::Binary, payload))
            .await
            .map_err(|e| {
                zenoh::error!("Could not write frame: {}", e);
                LinkError::LinkTxFailed
            })
    }
}

impl ZLinkTx for StdWsLinkTx<'_> {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        self.write_buffer.clear();
        self.write_buffer
            .extend_from_copyable_slice(buffer)
            .map_err(|e| {
                zenoh::error!("Failed to extend write buffer: {}", e);
                LinkError::LinkTxFailed
            })?;

        let payload = self.write_buffer.as_slice_mut();

        self.sink
            .write_frame(&mut Frame::new_fin(OpCode::Binary, payload))
            .await
            .map_err(|e| {
                zenoh::error!("Could not write frame: {}", e);
                LinkError::LinkTxFailed
            })
    }
}

impl ZLinkRx for StdWsLink {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.read_buffer.clear();

        let frame = self
            .stream
            .read_frame(&mut self.read_buffer, WebSocketPayloadOrigin::Consistent)
            .await
            .map_err(|e| {
                zenoh::error!("Could not read frame: {}", e);
                LinkError::LinkRxFailed
            })?;

        match frame.op_code() {
            OpCode::Binary => {
                let len = frame.payload().len().min(buffer.len());
                buffer[..len].copy_from_slice(&frame.payload()[..len]);
                Ok(len)
            }
            code => {
                zenoh::error!(
                    "Could not read frame into buffer: unexpected OpCode {:?}",
                    code
                );
                zenoh::zbail!(LinkError::LinkRxFailed);
            }
        }
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.read(buffer).await.map(|_| ())
    }
}

impl ZLinkRx for StdWsLinkRx<'_> {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        self.read_buffer.clear();

        let frame = self
            .stream
            .read_frame(self.read_buffer, WebSocketPayloadOrigin::Consistent)
            .await
            .map_err(|e| {
                zenoh::error!("Could not read frame: {}", e);
                LinkError::LinkRxFailed
            })?;

        match frame.op_code() {
            OpCode::Binary => {
                let len = frame.payload().len().min(buffer.len());
                buffer[..len].copy_from_slice(&frame.payload()[..len]);
                Ok(len)
            }
            code => {
                zenoh::error!(
                    "Could not read frame into buffer: unexpected OpCode {:?}",
                    code
                );
                zenoh::zbail!(LinkError::LinkRxFailed);
            }
        }
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.read(buffer).await.map(|_| ())
    }
}

impl ZLink for StdWsLink {
    type Tx<'link>
        = StdWsLinkTx<'link>
    where
        Self: 'link;

    type Rx<'link>
        = StdWsLinkRx<'link>
    where
        Self: 'link;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
        let tx = StdWsLinkTx {
            sink: &mut self.sink,
            write_buffer: &mut self.write_buffer,
            mtu: self.mtu,
        };
        let rx = StdWsLinkRx {
            stream: &mut self.stream,
            read_buffer: &mut self.read_buffer,
            mtu: self.mtu,
        };
        (tx, rx)
    }
}
