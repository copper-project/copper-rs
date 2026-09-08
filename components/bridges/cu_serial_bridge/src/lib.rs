#![cfg_attr(not(feature = "std"), no_std)]

//! Raw byte chunks over an exclusively owned serial resource. Each RX chunk
//! contains the bytes returned by one serial read.
use cu_serial::SerialIo;
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;

pub const BYTE_CAPACITY: usize = 256;
pub type ByteChunk = cu29::payload::CuArrayVec<u8, BYTE_CAPACITY>;
cu29::rx_channels! { bytes_rx => ByteChunk }
cu29::tx_channels! { bytes_tx => ByteChunk }
cu29::resources!(for<S> where S: Send + Sync + 'static { serial => Owned<S> });

#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct SerialBridge<S> {
    #[reflect(ignore)]
    serial: S,
    pending: [u8; BYTE_CAPACITY],
    pending_len: usize,
    pending_offset: usize,
}
impl<S> SerialBridge<S> {
    pub fn from_serial(serial: S) -> Self {
        Self {
            serial,
            pending: [0; BYTE_CAPACITY],
            pending_len: 0,
            pending_offset: 0,
        }
    }
}
impl<S: SerialIo> SerialBridge<S> {
    /// Advance queued output with at most one nonblocking write.
    fn drain_pending(&mut self) -> CuResult<()> {
        if self.pending_len == 0 {
            return Ok(());
        }
        let written = self
            .serial
            .try_write(&self.pending[self.pending_offset..self.pending_len])
            .map_err(|_| CuError::from("Serial bridge write failed"))?;
        self.pending_offset += written;
        if self.pending_offset == self.pending_len {
            self.pending_len = 0;
            self.pending_offset = 0;
        }
        Ok(())
    }
}
impl<S: 'static> cu29::reflect::TypePath for SerialBridge<S> {
    fn type_path() -> &'static str {
        "cu_serial_bridge::SerialBridge"
    }
    fn short_type_path() -> &'static str {
        "SerialBridge"
    }
    fn type_ident() -> Option<&'static str> {
        Some("SerialBridge")
    }
    fn crate_name() -> Option<&'static str> {
        Some("cu_serial_bridge")
    }
    fn module_path() -> Option<&'static str> {
        Some("cu_serial_bridge")
    }
}
impl<S> Freezable for SerialBridge<S> {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        bincode::Encode::encode(
            &self.pending[self.pending_offset..self.pending_len],
            encoder,
        )
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        let len: usize = bincode::Decode::decode(decoder)?;
        if len > BYTE_CAPACITY {
            return Err(bincode::error::DecodeError::Other(
                "Serial bridge TX snapshot exceeds capacity",
            ));
        }
        let mut pending = [0; BYTE_CAPACITY];
        for byte in &mut pending[..len] {
            *byte = bincode::Decode::decode(decoder)?;
        }
        self.pending = pending;
        self.pending_len = len;
        self.pending_offset = 0;
        Ok(())
    }
}
impl<S: SerialIo + Send + Sync + 'static> CuBridge for SerialBridge<S> {
    type Tx = TxChannels;
    type Rx = RxChannels;
    type Resources<'r> = Resources<S>;
    fn new(
        _: Option<&ComponentConfig>,
        _: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self::from_serial(resources.serial.0))
    }
    fn preprocess(&mut self, _: &CuContext) -> CuResult<()> {
        self.drain_pending()
    }
    fn stop(&mut self, _: &CuContext) -> CuResult<()> {
        self.drain_pending()?;
        if self.pending_len != 0 {
            return Err(CuError::from("Serial bridge stopped with pending TX bytes"));
        }
        Ok(())
    }
    fn send<'a, Payload: CuMsgPayload + 'a>(
        &mut self,
        _: &CuContext,
        _: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()> {
        let msg: &CuMsg<ByteChunk> = msg.downcast_ref()?;
        if let Some(bytes) = msg.payload()
            && !bytes.0.is_empty()
        {
            if self.pending_len != 0 {
                return Err(CuError::from("Serial bridge TX buffer full"));
            }
            self.pending_len = bytes.0.len();
            self.pending[..self.pending_len].copy_from_slice(bytes.0.as_slice());
            // An initial write error accepted no bytes. Reject this message;
            // subsequent lifecycle errors retain an already accepted message.
            if let Err(error) = self.drain_pending() {
                self.pending_len = 0;
                return Err(error);
            }
        }
        Ok(())
    }
    fn receive<'a, Payload: CuMsgPayload + 'a>(
        &mut self,
        ctx: &CuContext,
        _: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()> {
        let msg: &mut CuMsg<ByteChunk> = msg.downcast_mut()?;
        let payload = msg.payload_mut().get_or_insert_with(ByteChunk::default);
        payload.0.clear();
        payload.0.extend(core::iter::repeat_n(0, BYTE_CAPACITY));
        let n = match self.serial.try_read(payload.0.as_mut_slice()) {
            Ok(n) => n,
            Err(_) => {
                msg.clear_payload();
                return Err(CuError::from("Serial bridge read failed"));
            }
        };
        payload.0.truncate(n);
        if n == 0 {
            msg.clear_payload();
        } else {
            msg.tov = Tov::Time(ctx.now());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::{collections::VecDeque, vec::Vec};
    struct Uart {
        received: VecDeque<u8>,
        written: Vec<u8>,
        limit: usize,
        fail: bool,
    }
    impl embedded_io::ErrorType for Uart {
        type Error = embedded_io::ErrorKind;
    }
    impl SerialIo for Uart {
        fn try_read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
            if self.fail {
                return Err(embedded_io::ErrorKind::Other);
            }
            let n = self.received.len().min(out.len()).min(self.limit);
            for byte in &mut out[..n] {
                *byte = self.received.pop_front().unwrap();
            }
            Ok(n)
        }
        fn try_write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
            if self.fail {
                return Err(embedded_io::ErrorKind::Other);
            }
            let n = bytes.len().min(self.limit);
            self.written.extend_from_slice(&bytes[..n]);
            Ok(n)
        }
    }
    fn bridge() -> SerialBridge<Uart> {
        SerialBridge::from_serial(Uart {
            received: [0, 1, 255].into(),
            written: Vec::new(),
            limit: 256,
            fail: false,
        })
    }
    #[test]
    fn raw_bytes_roundtrip_and_idle_clears_payload() {
        let mut bridge = bridge();
        let (ctx, _) = CuContext::new_mock_clock();
        let mut msg = CuMsg::new(None::<ByteChunk>);
        bridge
            .receive(&ctx, &RxChannels::BYTES_RX, &mut msg)
            .unwrap();
        assert_eq!(msg.payload().unwrap().0.as_slice(), &[0, 1, 255]);
        bridge.send(&ctx, &TxChannels::BYTES_TX, &msg).unwrap();
        assert_eq!(bridge.serial.written, [0, 1, 255]);
        bridge
            .receive(&ctx, &RxChannels::BYTES_RX, &mut msg)
            .unwrap();
        assert!(msg.payload().is_none());
        bridge.serial.fail = true;
        assert!(
            bridge
                .receive(&ctx, &RxChannels::BYTES_RX, &mut msg)
                .is_err()
        );
        assert!(msg.payload().is_none());
    }
    #[test]
    fn partial_writes_and_stalls_preserve_order_and_reject_overflow() {
        let mut bridge = bridge();
        let (ctx, _) = CuContext::new_mock_clock();
        let mut msg = CuMsg::new(None::<ByteChunk>);
        bridge
            .receive(&ctx, &RxChannels::BYTES_RX, &mut msg)
            .unwrap();
        bridge.serial.limit = 1;
        bridge.send(&ctx, &TxChannels::BYTES_TX, &msg).unwrap();
        assert_eq!(bridge.serial.written, [0]);
        assert!(bridge.send(&ctx, &TxChannels::BYTES_TX, &msg).is_err());
        bridge.serial.limit = 0;
        bridge.preprocess(&ctx).unwrap();
        assert_eq!(bridge.serial.written, [0]);
        bridge.serial.limit = 1;
        bridge.preprocess(&ctx).unwrap();
        bridge.preprocess(&ctx).unwrap();
        bridge.preprocess(&ctx).unwrap();
        assert_eq!(bridge.serial.written, [0, 1, 255]);
        bridge.send(&ctx, &TxChannels::BYTES_TX, &msg).unwrap();
        bridge.preprocess(&ctx).unwrap();
        bridge.preprocess(&ctx).unwrap();
        assert_eq!(bridge.serial.written, [0, 1, 255, 0, 1, 255]);
    }

    fn full_message() -> CuMsg<ByteChunk> {
        let mut payload = ByteChunk::default();
        payload.0.extend(0..=255);
        CuMsg::new(Some(payload))
    }

    #[test]
    fn write_errors_reject_new_messages_but_preserve_accepted_suffixes() {
        let mut bridge = bridge();
        let (ctx, _) = CuContext::new_mock_clock();
        let msg = full_message();
        bridge.serial.fail = true;
        assert!(bridge.send(&ctx, &TxChannels::BYTES_TX, &msg).is_err());
        bridge.serial.fail = false;
        bridge.serial.limit = 128;
        bridge.send(&ctx, &TxChannels::BYTES_TX, &msg).unwrap();
        bridge.serial.fail = true;
        assert!(bridge.preprocess(&ctx).is_err());
        bridge.serial.fail = false;
        bridge.preprocess(&ctx).unwrap();
        assert_eq!(bridge.serial.written, msg.payload().unwrap().0.as_slice());
    }

    #[test]
    fn stop_reports_undrained_bytes_and_restart_preserves_them() {
        let mut bridge = bridge();
        let (ctx, _) = CuContext::new_mock_clock();
        let msg = full_message();
        bridge.serial.limit = 0;
        bridge.send(&ctx, &TxChannels::BYTES_TX, &msg).unwrap();
        assert!(bridge.stop(&ctx).is_err());
        bridge.start(&ctx).unwrap();
        bridge.serial.limit = BYTE_CAPACITY;
        bridge.stop(&ctx).unwrap();
        assert_eq!(bridge.serial.written, msg.payload().unwrap().0.as_slice());
    }

    #[test]
    fn snapshots_restore_only_unsent_bytes_and_reject_oversized_state() {
        let mut original = bridge();
        let (ctx, _) = CuContext::new_mock_clock();
        original.serial.limit = 128;
        original
            .send(&ctx, &TxChannels::BYTES_TX, &full_message())
            .unwrap();
        let mut storage = [0; BYTE_CAPACITY + 16];
        let mut encoder = bincode::enc::EncoderImpl::new(
            bincode::enc::write::SliceWriter::new(&mut storage),
            bincode::config::standard(),
        );
        original.freeze(&mut encoder).unwrap();
        let encoded_len = encoder.into_writer().bytes_written();
        let encoded = &storage[..encoded_len];
        let mut restored = bridge();
        let mut decoder = bincode::de::DecoderImpl::new(
            bincode::de::read::SliceReader::new(encoded),
            bincode::config::standard(),
            (),
        );
        restored.thaw(&mut decoder).unwrap();
        restored.preprocess(&ctx).unwrap();
        assert_eq!(restored.serial.written, (128..=255).collect::<Vec<u8>>());
        let oversized =
            bincode::encode_to_vec(BYTE_CAPACITY + 1, bincode::config::standard()).unwrap();
        let mut decoder = bincode::de::DecoderImpl::new(
            bincode::de::read::SliceReader::new(&oversized),
            bincode::config::standard(),
            (),
        );
        assert!(restored.thaw(&mut decoder).is_err());
    }
}
