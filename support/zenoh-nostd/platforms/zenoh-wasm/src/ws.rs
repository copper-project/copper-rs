use {
    futures_util::{
        SinkExt as _, StreamExt as _,
        stream::{SplitSink, SplitStream},
    },
    yawc::{
        WebSocket,
        frame::{FrameView, OpCode},
    },
    zenoh_nostd::platform::*,
};

pub struct WasmWsLink {
    sink: SplitSink<WebSocket, FrameView>,
    stream: SplitStream<WebSocket>,
    mtu: u16,
}

impl WasmWsLink {
    pub fn new(stream: WebSocket) -> Self {
        let (sink, stream) = stream.split();
        Self {
            sink,
            stream,
            mtu: u16::MAX,
        }
    }
}

pub struct WasmWsLinkTx<'a> {
    sink: &'a mut SplitSink<WebSocket, FrameView>,
    mtu: u16,
}

pub struct WasmWsLinkRx<'a> {
    stream: &'a mut SplitStream<WebSocket>,
    mtu: u16,
}

impl ZLinkInfo for WasmWsLink {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkInfo for WasmWsLinkTx<'_> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkInfo for WasmWsLinkRx<'_> {
    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn is_streamed(&self) -> bool {
        false
    }
}

impl ZLinkTx for WasmWsLink {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        let item = FrameView::binary(buffer.to_vec());
        self.sink
            .send(item)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl ZLinkTx for WasmWsLinkTx<'_> {
    async fn write_all(&mut self, buffer: &[u8]) -> core::result::Result<(), LinkError> {
        let item = FrameView::binary(buffer.to_vec());
        self.sink
            .send(item)
            .await
            .map_err(|_| LinkError::LinkTxFailed)
    }
}

impl ZLinkRx for WasmWsLink {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        #[cfg(target_arch = "wasm32")]
        let Some(Ok(frame)) = self.stream.next().await else {
            return Err(LinkError::LinkRxFailed);
        };
        #[cfg(not(target_arch = "wasm32"))]
        let Some(frame) = self.stream.next().await else {
            return Err(LinkError::LinkRxFailed);
        };
        match frame.opcode {
            OpCode::Binary => {
                let len = frame.payload.len().min(buffer.len());
                buffer[..len].copy_from_slice(&frame.payload[..len]);
                Ok(len)
            }
            _ => zenoh::zbail!(LinkError::LinkRxFailed),
        }
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.read(buffer).await.map(|_| ())
    }
}

impl ZLinkRx for WasmWsLinkRx<'_> {
    async fn read(&mut self, buffer: &mut [u8]) -> core::result::Result<usize, LinkError> {
        #[cfg(target_arch = "wasm32")]
        let Some(Ok(frame)) = self.stream.next().await else {
            return Err(LinkError::LinkRxFailed);
        };
        #[cfg(not(target_arch = "wasm32"))]
        let Some(frame) = self.stream.next().await else {
            return Err(LinkError::LinkRxFailed);
        };
        match frame.opcode {
            OpCode::Binary => {
                let len = frame.payload.len().min(buffer.len());
                buffer[..len].copy_from_slice(&frame.payload[..len]);
                Ok(len)
            }
            _ => zenoh::zbail!(LinkError::LinkRxFailed),
        }
    }

    async fn read_exact(&mut self, buffer: &mut [u8]) -> core::result::Result<(), LinkError> {
        self.read(buffer).await.map(|_| ())
    }
}

impl ZLink for WasmWsLink {
    type Tx<'link>
        = WasmWsLinkTx<'link>
    where
        Self: 'link;

    type Rx<'link>
        = WasmWsLinkRx<'link>
    where
        Self: 'link;

    fn split(&mut self) -> (Self::Tx<'_>, Self::Rx<'_>) {
        let tx = WasmWsLinkTx {
            sink: &mut self.sink,
            mtu: self.mtu,
        };
        let rx = WasmWsLinkRx {
            stream: &mut self.stream,
            mtu: self.mtu,
        };
        (tx, rx)
    }
}
