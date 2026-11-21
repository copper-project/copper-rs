#[path = "../src/deferred_sd/queue.rs"]
mod queue;

use bincode::{
    Encode,
    enc::{Encoder, write::Writer},
    error::EncodeError,
};
use queue::{POOL_BYTES, Record};

struct Raw<'a>(&'a [u8]);
impl Encode for Raw<'_> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        encoder.writer().write(self.0)
    }
}

#[test]
fn partial_entry_is_never_published_and_wrap_preserves_bytes() {
    let (mut input, mut output) = queue::queue();
    let first = vec![0xaa; POOL_BYTES - 2];
    input.append(4, &Raw(&first)).unwrap();
    assert_eq!(input.append(5, &(0x11u8, Raw(&[2, 3, 4]))).unwrap(), 0);
    assert_eq!(input.dropped, 1);
    assert!(
        matches!(output.records.pop(), Ok(Record::Entry { stream: 4, size }) if size == first.len())
    );
    let data = output.bytes.read_chunk(first.len()).unwrap();
    assert_eq!(data.as_slices().0, first);
    data.commit_all();
    assert!(output.records.is_empty());
    assert!(output.bytes.is_empty());

    let next = [5, 6, 7, 8, 9];
    input.append(6, &Raw(&next)).unwrap();
    assert!(matches!(
        output.records.pop(),
        Ok(Record::Entry { stream: 6, size: 5 })
    ));
    let bytes = output.bytes.read_chunk(5).unwrap();
    let (a, b) = bytes.as_slices();
    assert_eq!(a, &next[..2]);
    assert_eq!(b, &next[2..]);
    bytes.commit_all();
    assert!(output.bytes.is_empty());
}

#[test]
fn encoding_error_releases_reservation_and_flush_keeps_order() {
    struct Failed;
    impl Encode for Failed {
        fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            encoder.writer().write(&[42])?;
            Err(EncodeError::UnexpectedEnd)
        }
    }
    let (mut input, mut output) = queue::queue();
    assert!(input.append(0, &Failed).is_err());
    assert_eq!(input.dropped, 0);
    assert!(output.bytes.is_empty());
    assert!(output.records.is_empty());
    input.append(2, &Raw(&[0x99])).unwrap();
    assert!(input.records.push(Record::Flush { stream: 2 }).is_ok());
    assert!(matches!(
        output.records.pop(),
        Ok(Record::Entry { stream: 2, size: 1 })
    ));
    assert_eq!(output.bytes.pop().unwrap(), 0x99);
    assert!(matches!(
        output.records.pop(),
        Ok(Record::Flush { stream: 2 })
    ));
}
