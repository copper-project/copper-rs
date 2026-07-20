use std::net::SocketAddr;

use yawc::WebSocket;
use zenoh_nostd::platform::*;

pub mod ws;

pub struct WasmLinkManager;

#[derive(ZLinkInfo, ZLinkTx, ZLinkRx, ZLink)]
#[zenoh(ZLink = (WasmLinkTx<'link>, WasmLinkRx<'link>))]
pub enum WasmLink {
    Ws(ws::WasmWsLink),
}

#[derive(ZLinkInfo, ZLinkTx)]
pub enum WasmLinkTx<'link> {
    Ws(ws::WasmWsLinkTx<'link>),
}

#[derive(ZLinkInfo, ZLinkRx)]
pub enum WasmLinkRx<'link> {
    Ws(ws::WasmWsLinkRx<'link>),
}

impl ZLinkManager for WasmLinkManager {
    type Link<'a>
        = WasmLink
    where
        Self: 'a;

    async fn connect(
        &self,
        endpoint: Endpoint<'_>,
    ) -> core::result::Result<Self::Link<'_>, LinkError> {
        let protocol = endpoint.protocol();
        let address = endpoint.address();

        match protocol.as_str() {
            "ws" => {
                let dst_addr = SocketAddr::try_from(address)?;
                let url = format!("ws://{}", dst_addr);
                let socket =
                    WebSocket::connect(url.parse().map_err(|_| LinkError::CouldNotConnect)?)
                        .await
                        .map_err(|_| LinkError::CouldNotConnect)?;

                Ok(Self::Link::Ws(ws::WasmWsLink::new(socket)))
            }
            _ => zenoh::zbail!(LinkError::CouldNotParseProtocol),
        }
    }

    async fn listen(&self, _: Endpoint<'_>) -> core::result::Result<Self::Link<'_>, LinkError> {
        zenoh::zbail!(LinkError::CouldNotListen)
    }
}
