use core::mem::MaybeUninit;

use bincode::{Encode, enc::write::Writer, error::EncodeError};
use rtrb::{Consumer, Producer, RingBuffer};

pub const POOL_BYTES: usize = 256 * 1024;
const DESCRIPTORS: usize = 1024;

#[derive(Clone, Copy)]
pub(super) enum Record {
    Entry { stream: usize, size: usize },
    Flush { stream: usize },
}
pub struct Input {
    bytes: Producer<u8>,
    pub(super) records: Producer<Record>,
    pub(super) dropped: usize,
}
pub struct Output {
    pub(super) bytes: Consumer<u8>,
    pub(super) records: Consumer<Record>,
}

pub fn queue() -> (Input, Output) {
    let (bytes_tx, bytes_rx) = RingBuffer::new(POOL_BYTES);
    let (records_tx, records_rx) = RingBuffer::new(DESCRIPTORS);
    (
        Input {
            bytes: bytes_tx,
            records: records_tx,
            dropped: 0,
        },
        Output {
            bytes: bytes_rx,
            records: records_rx,
        },
    )
}

struct RingWriter<'a> {
    first: &'a mut [MaybeUninit<u8>],
    second: &'a mut [MaybeUninit<u8>],
    used: usize,
    full: bool,
}
impl Writer for RingWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        if bytes.len() > self.first.len() + self.second.len() {
            self.full = true;
            return Err(EncodeError::UnexpectedEnd);
        }
        let count = bytes.len().min(self.first.len());
        let (dst, rest) = core::mem::take(&mut self.first).split_at_mut(count);
        dst.write_copy_of_slice(&bytes[..count]);
        self.first = rest;
        if count < bytes.len() {
            let (dst, rest) = core::mem::take(&mut self.second).split_at_mut(bytes.len() - count);
            dst.write_copy_of_slice(&bytes[count..]);
            self.second = rest;
        }
        self.used += bytes.len();
        Ok(())
    }
}
impl Input {
    pub(super) fn append<E: Encode>(
        &mut self,
        stream: usize,
        entry: &E,
    ) -> Result<usize, EncodeError> {
        // Keep descriptors available for closing the flight streams under saturation.
        if self.records.slots() <= 8 {
            self.dropped += 1;
            return Ok(0);
        }
        let free = self.bytes.slots();
        let mut chunk = self
            .bytes
            .write_chunk_uninit(free)
            .expect("single producer owns the free space");
        let (first, second) = chunk.as_mut_slices();
        let mut writer = RingWriter {
            first,
            second,
            used: 0,
            full: false,
        };
        match bincode::encode_into_writer(entry, &mut writer, bincode::config::standard()) {
            Ok(()) => {}
            Err(EncodeError::UnexpectedEnd) if writer.full => {
                self.dropped += 1;
                return Ok(0);
            }
            Err(error) => return Err(error),
        }
        let size = writer.used;
        // SAFETY: RingWriter initialized exactly the first size bytes, including across wrap.
        unsafe {
            chunk.commit(size);
        }
        self.records
            .push(Record::Entry { stream, size })
            .map_err(|_| EncodeError::Other("lost descriptor reservation"))?;
        Ok(size)
    }
}
