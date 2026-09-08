#![cfg_attr(not(feature = "std"), no_std)]

//! Serial packet framing: 0x7e delimiters, 0x7d escaping (byte XOR 0x20).
//! A framing CRC32C protects the complete packet. RX resynchronizes at the
//! next delimiter after corruption. TX owns exactly one pending encoded frame.
//! Encoding runs in the LogStream sender worker, outside the runtime hot path.
use core::{fmt, marker::PhantomData};
use crc::{CRC_32_ISCSI, Crc};
use cu_serial::SerialIo;
use cu29::prelude::*;
use cu29::resource::{
    BundleContext, NamedResourceBundleDecl, ResourceBindings, ResourceBundle, ResourceBundleDecl,
    ResourceManager,
};
use cu29_logstream::{CuStreamRx, CuStreamRxError, CuStreamTx, CuStreamTxError};

pub const DEFAULT_FRAME_CAPACITY: usize = 514;
const DELIMITER: u8 = 0x7e;
const ESCAPE: u8 = 0x7d;
const CHECKSUM_BYTES: usize = 4;
const CRC32C: Crc<u32> = Crc::<u32>::new(&CRC_32_ISCSI);

pub struct SerialLogStreamTx<S, const N: usize = DEFAULT_FRAME_CAPACITY> {
    serial: S,
    frame: [u8; N],
    len: usize,
    sent: usize,
}
impl<S, const N: usize> SerialLogStreamTx<S, N> {
    pub fn new(serial: S) -> Self {
        Self {
            serial,
            frame: [0; N],
            len: 0,
            sent: 0,
        }
    }
    pub const fn max_packet_bytes() -> usize {
        (N.saturating_sub(2) / 2).saturating_sub(CHECKSUM_BYTES)
    }
}
impl<S, const N: usize> fmt::Debug for SerialLogStreamTx<S, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SerialLogStreamTx")
            .field("pending_bytes", &(self.len - self.sent))
            .finish()
    }
}
impl<S: SerialIo + Send + Sync, const N: usize> CuStreamTx for SerialLogStreamTx<S, N> {
    fn try_send(&mut self, packet: &[u8]) -> Result<(), CuStreamTxError> {
        if N < 2 || packet.is_empty() || packet.len() > Self::max_packet_bytes() {
            return Err(CuStreamTxError::Failed(
                "Serial packet exceeds frame capacity",
            ));
        }
        if self.sent < self.len {
            return Err(CuStreamTxError::WouldBlock);
        }
        self.sent = 0;
        self.len = 1;
        self.frame[0] = DELIMITER;
        let checksum = CRC32C.checksum(packet).to_be_bytes();
        for &byte in packet.iter().chain(checksum.iter()) {
            if byte == DELIMITER || byte == ESCAPE {
                self.frame[self.len] = ESCAPE;
                self.len += 1;
                self.frame[self.len] = byte ^ 0x20;
            } else {
                self.frame[self.len] = byte;
            }
            self.len += 1;
        }
        self.frame[self.len] = DELIMITER;
        self.len += 1;
        Ok(())
    }
    fn poll_pending(&mut self) -> Result<bool, CuStreamTxError> {
        if self.sent < self.len {
            self.sent += self
                .serial
                .try_write(&self.frame[self.sent..self.len])
                .map_err(|_| CuStreamTxError::Failed("Serial write failed"))?;
        }
        Ok(self.sent < self.len)
    }
}

pub struct SerialLogStreamRx<S, const N: usize = DEFAULT_FRAME_CAPACITY> {
    serial: S,
    frame: [u8; N],
    len: usize,
    ready: bool,
    active: bool,
    escaped: bool,
    input: [u8; 64],
    input_len: usize,
    input_pos: usize,
}
impl<S, const N: usize> SerialLogStreamRx<S, N> {
    pub fn new(serial: S) -> Self {
        Self {
            serial,
            frame: [0; N],
            len: 0,
            ready: false,
            active: false,
            escaped: false,
            input: [0; 64],
            input_len: 0,
            input_pos: 0,
        }
    }
    fn deliver(&mut self, out: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
        if out.len() < self.len {
            return Err(CuStreamRxError::BufferTooSmall { needed: self.len });
        }
        let len = self.len;
        out[..len].copy_from_slice(&self.frame[..len]);
        self.ready = false;
        self.len = 0;
        Ok(Some(len))
    }
}
impl<S, const N: usize> fmt::Debug for SerialLogStreamRx<S, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("SerialLogStreamRx")
    }
}
impl<S: SerialIo + Send + Sync, const N: usize> CuStreamRx for SerialLogStreamRx<S, N> {
    fn try_recv(&mut self, out: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
        if self.ready {
            return self.deliver(out);
        }
        // At most one UART read and 64 parser steps per call.
        if self.input_pos == self.input_len {
            self.input_len = self
                .serial
                .try_read(&mut self.input)
                .map_err(|_| CuStreamRxError::Failed("Serial read failed"))?;
            self.input_pos = 0;
        }
        while self.input_pos < self.input_len {
            let byte = self.input[self.input_pos];
            self.input_pos += 1;
            if byte == DELIMITER {
                let valid = self.active
                    && !self.escaped
                    && self.len > CHECKSUM_BYTES
                    && CRC32C
                        .checksum(&self.frame[..self.len - CHECKSUM_BYTES])
                        .to_be_bytes()
                        == self.frame[self.len - CHECKSUM_BYTES..self.len];
                self.active = true;
                self.escaped = false;
                if valid {
                    self.len -= CHECKSUM_BYTES;
                    self.ready = true;
                    return self.deliver(out);
                }
                self.len = 0;
            } else if self.active {
                let decoded = if self.escaped {
                    self.escaped = false;
                    if byte != (DELIMITER ^ 0x20) && byte != (ESCAPE ^ 0x20) {
                        self.active = false;
                        self.len = 0;
                        continue;
                    }
                    byte ^ 0x20
                } else if byte == ESCAPE {
                    self.escaped = true;
                    continue;
                } else {
                    byte
                };
                if self.len >= N.saturating_sub(2) / 2 {
                    self.active = false;
                    self.len = 0;
                } else {
                    self.frame[self.len] = decoded;
                    self.len += 1;
                }
            }
        }
        Ok(None)
    }
}

mod inputs {
    cu29::resources!(for<S> where S: Send + Sync + 'static { serial => Owned<S> });
}
#[allow(dead_code)]
struct TxSlots;
cu29::bundle_resources!(TxSlots: Tx);
pub use TxSlotsId as SerialLogStreamTxId;
#[allow(dead_code)]
struct RxSlots;
cu29::bundle_resources!(RxSlots: Rx);
pub use RxSlotsId as SerialLogStreamRxId;
pub struct SerialLogStreamTxResources<S, const N: usize = DEFAULT_FRAME_CAPACITY>(
    PhantomData<fn() -> S>,
);
pub struct SerialLogStreamRxResources<S, const N: usize = DEFAULT_FRAME_CAPACITY>(
    PhantomData<fn() -> S>,
);
macro_rules! provider {
    ($provider:ident, $endpoint:ident, $id:ident, $slot:ident, $name:literal) => {
        impl<S, const N: usize> ResourceBundleDecl for $provider<S, N> {
            type Id = $id;
        }
        impl<S, const N: usize> NamedResourceBundleDecl for $provider<S, N> {
            const NAMES: &'static [&'static str] = &[$name];
        }
        impl<S: SerialIo + Send + Sync + 'static, const N: usize> ResourceBundle
            for $provider<S, N>
        {
            const INPUT_NAMES: &'static [&'static str] =
                <inputs::Resources<S> as ResourceBindings>::NAMES;
            fn build(
                bundle: BundleContext<Self>,
                _: Option<&ComponentConfig>,
                manager: &mut ResourceManager,
            ) -> CuResult<()> {
                if N < 2 * (cu29_logstream::PACKET_HEADER_LEN + 1 + CHECKSUM_BYTES) + 2 {
                    return Err(CuError::from(
                        "Serial frame capacity does not fit a LogStream packet",
                    ));
                }
                let inputs = bundle.inputs::<inputs::Resources<S>>(manager)?;
                manager.add_owned(
                    bundle.key($id::$slot),
                    $endpoint::<S, N>::new(inputs.serial.0),
                )
            }
        }
    };
}
provider!(
    SerialLogStreamTxResources,
    SerialLogStreamTx,
    SerialLogStreamTxId,
    Tx,
    "tx"
);
provider!(
    SerialLogStreamRxResources,
    SerialLogStreamRx,
    SerialLogStreamRxId,
    Rx,
    "rx"
);

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::convert::Infallible;
    use cu29_logstream::{
        FecScheme, FecSymbolKind, Lane, RecordKind, WireHeader, encode_packet_into,
    };
    use std::{collections::VecDeque, vec, vec::Vec};
    #[derive(Default)]
    struct Uart {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
        step: usize,
    }
    impl embedded_io::ErrorType for Uart {
        type Error = Infallible;
    }
    impl SerialIo for Uart {
        fn try_read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
            let n = out.len().min(self.rx.len()).min(self.step);
            for byte in &mut out[..n] {
                *byte = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
        fn try_write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
            let n = bytes.len().min(self.step);
            self.tx.extend_from_slice(&bytes[..n]);
            Ok(n)
        }
    }
    fn packet() -> Vec<u8> {
        let mut bytes = vec![0; 256];
        let n = encode_packet_into(
            WireHeader {
                lane: Lane::ReplayCritical,
                record_kind: RecordKind::CopperList,
                fec_scheme: FecScheme::RlcGf256,
                symbol_kind: FecSymbolKind::Source,
                session_id: [0; 16],
                sender_id: 1,
                object_id: 2,
                fec_metadata: [0; 12],
                fragment_count: 1,
            },
            &[0, 0x7e, 0x7d, 255],
            &mut bytes,
        )
        .unwrap();
        bytes.truncate(n);
        bytes
    }
    fn encoded(packet: &[u8]) -> Vec<u8> {
        let mut tx = SerialLogStreamTx::<_, 514>::new(Uart {
            step: 3,
            ..Uart::default()
        });
        tx.try_send(packet).unwrap();
        assert_eq!(tx.try_send(packet), Err(CuStreamTxError::WouldBlock));
        while tx.poll_pending().unwrap() {}
        tx.serial.tx
    }
    fn received(wire: Vec<u8>) -> Vec<Vec<u8>> {
        let polls = wire.len() + 1;
        let mut rx = SerialLogStreamRx::<_, 514>::new(Uart {
            rx: wire.into(),
            step: 1,
            ..Uart::default()
        });
        let mut out = [0; 256];
        let mut packets = Vec::new();
        for _ in 0..polls {
            if let Some(n) = rx.try_recv(&mut out).unwrap() {
                packets.push(out[..n].to_vec());
            }
        }
        packets
    }

    #[test]
    fn framing_crc_is_independent_of_logstream_headers() {
        let packet = b"123456789";
        let wire = encoded(packet);
        let mut expected = vec![DELIMITER];
        expected.extend_from_slice(packet);
        // Published CRC32C check value, serialized big endian.
        expected.extend_from_slice(&[0xe3, 0x06, 0x92, 0x83, DELIMITER]);
        assert_eq!(wire, expected);
        assert_eq!(received(wire), vec![packet.to_vec()]);
    }

    #[test]
    fn damaged_frames_are_dropped_before_delivery_and_resynchronize() {
        // No inner LogStream CRC: only the adapter can detect this corruption.
        let packet = [0, DELIMITER, ESCAPE, 255, 1, 2, 3];
        let good = encoded(&packet);
        for offset in 0..good.len() {
            for bit in 0..8 {
                let mut damaged = good.clone();
                damaged[offset] ^= 1 << bit;
                damaged.extend_from_slice(&good);
                assert_eq!(received(damaged), vec![packet.to_vec()], "{offset}:{bit}");
            }
        }
        for end in 0..good.len() - 1 {
            let mut truncated = good[..end].to_vec();
            truncated.extend_from_slice(&good);
            assert_eq!(received(truncated), vec![packet.to_vec()], "{end}");
        }
    }

    #[test]
    fn checksum_bytes_are_escaped_and_capacity_includes_them() {
        assert_eq!(SerialLogStreamTx::<Uart>::max_packet_bytes(), 252);
        assert_eq!(SerialLogStreamTx::<Uart, 0>::max_packet_bytes(), 0);
        assert_eq!(SerialLogStreamTx::<Uart, 9>::max_packet_bytes(), 0);
        let mut checksum_escape_seen = false;
        for byte in 0..=255 {
            let packet = vec![byte; 252];
            checksum_escape_seen |= CRC32C
                .checksum(&packet)
                .to_be_bytes()
                .iter()
                .any(|byte| *byte == DELIMITER || *byte == ESCAPE);
            let wire = encoded(&packet);
            assert!(wire.len() <= DEFAULT_FRAME_CAPACITY);
            assert_eq!(received(wire), vec![packet]);
        }
        assert!(checksum_escape_seen);
        let mut tx = SerialLogStreamTx::<_, 514>::new(Uart::default());
        assert!(tx.try_send(&[0; 253]).is_err());
        assert_eq!(tx.len, 0);
        let mut tiny = SerialLogStreamTx::<_, 12>::new(Uart {
            step: 12,
            ..Uart::default()
        });
        tiny.try_send(&[DELIMITER]).unwrap();
        assert!(!tiny.poll_pending().unwrap());
        assert_eq!(received(tiny.serial.tx), vec![vec![DELIMITER]]);
    }

    #[test]
    fn partial_writes_deliver_last_frame_without_another_send() {
        let packet = packet();
        let wire = encoded(&packet);
        let mut rx = SerialLogStreamRx::<_, 514>::new(Uart {
            rx: wire.into(),
            step: 1,
            ..Uart::default()
        });
        let mut out = [0; 256];
        let mut found = None;
        for _ in 0..514 {
            if let Some(n) = rx.try_recv(&mut out).unwrap() {
                found = Some(n);
                break;
            }
        }
        assert_eq!(&out[..found.unwrap()], packet);
        assert_eq!(rx.try_recv(&mut out).unwrap(), None);
    }
    #[test]
    fn corruption_oversize_and_truncation_resynchronize() {
        let packet = packet();
        let good = encoded(&packet);
        for prefix in [
            vec![0x7e; 3],
            vec![0x7e, 1, 0x7d],
            vec![1; 600],
            {
                let mut bad = good.clone();
                bad[20] ^= 1;
                bad
            },
            {
                let mut bad = vec![0x7e];
                bad.extend(vec![1; 600]);
                bad
            },
        ] {
            let mut wire = prefix;
            wire.extend_from_slice(&good);
            wire.extend_from_slice(&good);
            let mut rx = SerialLogStreamRx::<_, 514>::new(Uart {
                rx: wire.into(),
                step: 64,
                ..Uart::default()
            });
            let mut out = [0; 256];
            let mut received = 0;
            for _ in 0..40 {
                if let Some(n) = rx.try_recv(&mut out).unwrap() {
                    assert_eq!(&out[..n], packet);
                    received += 1;
                }
            }
            assert_eq!(received, 2);
        }
    }
    #[test]
    fn small_destination_retains_the_packet_and_backpressure_is_atomic() {
        let packet = packet();
        let wire = encoded(&packet);
        let mut rx = SerialLogStreamRx::<_, 514>::new(Uart {
            rx: wire.into(),
            step: 64,
            ..Uart::default()
        });
        let mut small = [0; 1];
        loop {
            match rx.try_recv(&mut small) {
                Ok(None) => {}
                Err(CuStreamRxError::BufferTooSmall { needed }) => {
                    assert_eq!(needed, packet.len());
                    break;
                }
                other => panic!("{other:?}"),
            }
        }
        let mut out = [0; 256];
        assert_eq!(rx.try_recv(&mut out).unwrap(), Some(packet.len()));
        let mut tx = SerialLogStreamTx::<_, 514>::new(Uart::default());
        tx.try_send(&packet).unwrap();
        assert!(tx.poll_pending().unwrap());
        assert!(tx.serial.tx.is_empty());
        assert_eq!(tx.try_send(&packet), Err(CuStreamTxError::WouldBlock));
        assert!(tx.try_send(&[0; 257]).is_err());
    }
}
