use zenoh_proto::{
    TransportLinkError,
    msgs::{NetworkMessage, NetworkMessageRef},
};
use zenoh_sansio::{ZTransportRx, ZTransportTx};

use super::{ZLinkInfo, ZLinkRx, ZLinkTx};

pub trait ZTransportLinkTx {
    fn tx(&mut self) -> (&mut impl ZLinkTx, &mut impl ZTransportTx);

    fn send<'a>(
        &mut self,
        msgs: impl Iterator<Item = NetworkMessage<'a>>,
    ) -> impl Future<Output = core::result::Result<(), zenoh_proto::TransportLinkError>> {
        let (link, transport) = self.tx();
        transport.encode(msgs);

        async move {
            if let Some(bytes) = transport.flush(link.is_streamed()) {
                link.write_all(bytes).await.map_err(|e| e.into())
            } else {
                Ok(())
            }
        }
    }

    #[allow(dead_code)]
    fn send_optimized_ref<'a>(
        &mut self,
        msgs: impl Iterator<Item = (NetworkMessageRef<'a>, &'a [u8])>,
    ) -> impl Future<Output = core::result::Result<(), zenoh_proto::TransportLinkError>> {
        let (link, transport) = self.tx();
        transport.encode_optimized_ref(msgs);

        async move {
            if let Some(bytes) = transport.flush(link.is_streamed()) {
                link.write_all(bytes).await.map_err(|e| e.into())
            } else {
                Ok(())
            }
        }
    }

    fn keepalive(
        &mut self,
    ) -> impl Future<Output = core::result::Result<(), zenoh_proto::TransportLinkError>> {
        let (link, transport) = self.tx();
        transport.keepalive();

        async move {
            if let Some(bytes) = transport.flush(link.is_streamed()) {
                link.write_all(bytes).await.map_err(|e| e.into())
            } else {
                Ok(())
            }
        }
    }
}

pub trait ZTransportLinkRx {
    fn rx(&mut self) -> (&mut impl ZLinkRx, &mut impl ZTransportRx);

    fn recv(
        &mut self,
    ) -> impl core::future::Future<
        Output = core::result::Result<
            impl Iterator<Item = (NetworkMessage<'_>, &'_ [u8])>,
            zenoh_proto::TransportLinkError,
        >,
    > {
        let (link, transport) = self.rx();
        let streamed = link.is_streamed();

        async move {
            transport
                .decode_with_async(
                    async |bytes| {
                        if streamed {
                            link.read_exact(bytes).await.map(|_| bytes.len())
                        } else {
                            link.read(bytes).await
                        }
                    },
                    streamed,
                )
                .await
                .map_err(|e| e.flatten_map::<TransportLinkError>())?;

            Ok(transport.flush())
        }
    }
}
