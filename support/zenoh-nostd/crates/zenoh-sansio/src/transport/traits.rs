use core::fmt::Display;

use zenoh_proto::{
    EitherError, TransportError,
    msgs::{
        InitAck, InitSyn, NetworkMessage, NetworkMessageRef, OpenAck, OpenSyn, TransportMessage,
        TransportMessageRef,
    },
};

pub trait ZTransportRx {
    fn decode_prefixed(&mut self, read: &[u8]) -> core::result::Result<(), TransportError>;
    fn decode_raw(&mut self, read: &[u8]) -> core::result::Result<(), TransportError>;

    fn decode(&mut self, read: &[u8], prefixed: bool) -> core::result::Result<(), TransportError> {
        if prefixed {
            self.decode_prefixed(read)
        } else {
            self.decode_raw(read)
        }
    }

    fn decode_prefixed_with<E>(
        &mut self,
        read: impl FnMut(&mut [u8]) -> core::result::Result<usize, E>,
    ) -> core::result::Result<(), EitherError<TransportError, E>>
    where
        E: Display;

    fn decode_raw_with<E>(
        &mut self,
        read: impl FnMut(&mut [u8]) -> core::result::Result<usize, E>,
    ) -> core::result::Result<(), EitherError<TransportError, E>>
    where
        E: Display;

    fn decode_with<E>(
        &mut self,
        read: impl FnMut(&mut [u8]) -> core::result::Result<usize, E>,
        prefixed: bool,
    ) -> core::result::Result<(), EitherError<TransportError, E>>
    where
        E: Display,
    {
        if prefixed {
            self.decode_prefixed_with(read)
        } else {
            self.decode_raw_with(read)
        }
    }

    fn decode_prefixed_with_async<E>(
        &mut self,
        read: impl AsyncFnMut(&mut [u8]) -> core::result::Result<usize, E>,
    ) -> impl Future<Output = core::result::Result<(), EitherError<TransportError, E>>>
    where
        E: Display;

    fn decode_raw_with_async<E>(
        &mut self,
        read: impl AsyncFnMut(&mut [u8]) -> core::result::Result<usize, E>,
    ) -> impl Future<Output = core::result::Result<(), EitherError<TransportError, E>>>
    where
        E: Display;

    fn decode_with_async<E>(
        &mut self,
        read: impl AsyncFnMut(&mut [u8]) -> core::result::Result<usize, E>,
        prefixed: bool,
    ) -> impl Future<Output = core::result::Result<(), EitherError<TransportError, E>>>
    where
        E: Display,
    {
        async move {
            if prefixed {
                self.decode_prefixed_with_async(read).await
            } else {
                self.decode_raw_with_async(read).await
            }
        }
    }

    fn flush(&mut self) -> impl Iterator<Item = (NetworkMessage<'_>, &'_ [u8])>;
    fn clear(&mut self);
}

pub trait ZTransportTx {
    fn keepalive(&mut self);
    fn init_syn(&mut self, syn: &InitSyn);
    fn init_ack(&mut self, ack: &InitAck);
    fn open_syn(&mut self, syn: &OpenSyn);
    fn open_ack(&mut self, ack: &OpenAck);
    fn close(&mut self);

    fn transport(&mut self, msg: TransportMessage);
    fn transport_ref(&mut self, msg: TransportMessageRef);

    fn encode<'a>(&mut self, msgs: impl Iterator<Item = NetworkMessage<'a>>);
    fn encode_ref<'a>(&mut self, msgs: impl Iterator<Item = NetworkMessageRef<'a>>);
    fn encode_optimized<'a>(&mut self, msgs: impl Iterator<Item = (NetworkMessage<'a>, &'a [u8])>);
    fn encode_optimized_ref<'a>(
        &mut self,
        msgs: impl Iterator<Item = (NetworkMessageRef<'a>, &'a [u8])>,
    );

    fn flush_prefixed(&mut self) -> Option<&'_ [u8]>;
    fn flush_raw(&mut self) -> Option<&'_ [u8]>;

    fn flush(&mut self, prefixed: bool) -> Option<&'_ [u8]> {
        if prefixed {
            self.flush_prefixed()
        } else {
            self.flush_raw()
        }
    }
    fn clear(&mut self);
}
