//! Statically composed fan-out used by generated runtimes.

use bincode::Encode;
use cu29_traits::{CuResult, WriteStream};

/// Fans one borrowed semantic record out to two concrete sinks.
///
/// Both sinks are called, in field order, even when the first one fails. If
/// both fail, the first error is returned. Nesting this type builds a static
/// fan-out tree without a subscriber collection, allocation, or additional
/// type erasure.
///
/// [`WriteStream::last_log_bytes`] is forwarded from the first sink. Generated
/// runtimes place the local unified-log sink first so existing local-log byte
/// accounting keeps its current meaning.
#[derive(Debug)]
pub struct StaticFanoutSink<First, Second> {
    first: First,
    second: Second,
}

impl<First, Second> StaticFanoutSink<First, Second> {
    #[inline]
    pub const fn new(first: First, second: Second) -> Self {
        Self { first, second }
    }

    #[inline]
    pub const fn first(&self) -> &First {
        &self.first
    }

    #[inline]
    pub const fn second(&self) -> &Second {
        &self.second
    }

    #[inline]
    pub fn first_mut(&mut self) -> &mut First {
        &mut self.first
    }

    #[inline]
    pub fn second_mut(&mut self) -> &mut Second {
        &mut self.second
    }

    #[inline]
    pub fn into_parts(self) -> (First, Second) {
        (self.first, self.second)
    }
}

impl<E, First, Second> WriteStream<E> for StaticFanoutSink<First, Second>
where
    E: Encode,
    First: WriteStream<E>,
    Second: WriteStream<E>,
{
    #[inline]
    fn log(&mut self, record: &E) -> CuResult<()> {
        let first_result = self.first.log(record);
        let second_result = self.second.log(record);
        first_result.and(second_result)
    }

    #[inline]
    fn flush(&mut self) -> CuResult<()> {
        let first_result = self.first.flush();
        let second_result = self.second.flush();
        first_result.and(second_result)
    }

    #[inline]
    fn last_log_bytes(&self) -> Option<usize> {
        self.first.last_log_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Mutex;

    #[derive(Debug)]
    struct ProbeSink {
        id: usize,
        calls: Arc<Mutex<Vec<(usize, usize)>>>,
        flushes: Arc<AtomicUsize>,
        bytes: Option<usize>,
        fail_log: bool,
        fail_flush: bool,
    }

    impl ProbeSink {
        fn new(id: usize, calls: Arc<Mutex<Vec<(usize, usize)>>>) -> Self {
            Self {
                id,
                calls,
                flushes: Arc::new(AtomicUsize::new(0)),
                bytes: None,
                fail_log: false,
                fail_flush: false,
            }
        }
    }

    impl WriteStream<u64> for ProbeSink {
        fn log(&mut self, record: &u64) -> CuResult<()> {
            self.calls
                .lock()
                .unwrap()
                .push((self.id, core::ptr::from_ref(record).addr()));
            if self.fail_log {
                Err(format!("sink {} log failed", self.id).into())
            } else {
                Ok(())
            }
        }

        fn flush(&mut self) -> CuResult<()> {
            self.flushes.fetch_add(1, Ordering::Relaxed);
            if self.fail_flush {
                Err(format!("sink {} flush failed", self.id).into())
            } else {
                Ok(())
            }
        }

        fn last_log_bytes(&self) -> Option<usize> {
            self.bytes
        }
    }

    #[test]
    fn fans_out_the_same_borrow_in_order() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let mut fanout = StaticFanoutSink::new(
            ProbeSink::new(1, calls.clone()),
            ProbeSink::new(2, calls.clone()),
        );
        let record = 42_u64;

        fanout.log(&record).unwrap();

        let record_address = core::ptr::from_ref(&record).addr();
        assert_eq!(
            *calls.lock().unwrap(),
            vec![(1, record_address), (2, record_address)]
        );
    }

    #[test]
    fn invokes_every_sink_and_returns_the_first_error() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let mut first = ProbeSink::new(1, calls.clone());
        first.fail_log = true;
        let mut second = ProbeSink::new(2, calls.clone());
        second.fail_log = true;
        let mut fanout = StaticFanoutSink::new(first, second);
        let record = 42_u64;

        let error = fanout.log(&record).unwrap_err();

        assert!(error.to_string().contains("sink 1 log failed"));
        assert_eq!(calls.lock().unwrap().len(), 2);
    }

    #[test]
    fn flushes_every_sink_and_returns_the_first_error() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let mut first = ProbeSink::new(1, calls.clone());
        first.fail_flush = true;
        let first_flushes = first.flushes.clone();
        let mut second = ProbeSink::new(2, calls);
        second.fail_flush = true;
        let second_flushes = second.flushes.clone();
        let mut fanout = StaticFanoutSink::new(first, second);

        let error = fanout.flush().unwrap_err();

        assert!(error.to_string().contains("sink 1 flush failed"));
        assert_eq!(first_flushes.load(Ordering::Relaxed), 1);
        assert_eq!(second_flushes.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn preserves_first_sink_byte_accounting() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let mut first = ProbeSink::new(1, calls.clone());
        first.bytes = Some(23);
        let mut second = ProbeSink::new(2, calls);
        second.bytes = Some(91);
        let fanout = StaticFanoutSink::new(first, second);

        assert_eq!(fanout.last_log_bytes(), Some(23));
    }

    #[test]
    fn nesting_builds_a_static_three_way_fanout() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let rest = StaticFanoutSink::new(
            ProbeSink::new(2, calls.clone()),
            ProbeSink::new(3, calls.clone()),
        );
        let mut fanout = StaticFanoutSink::new(ProbeSink::new(1, calls.clone()), rest);

        fanout.log(&42).unwrap();

        let ids: Vec<_> = calls.lock().unwrap().iter().map(|(id, _)| *id).collect();
        assert_eq!(ids, vec![1, 2, 3]);
    }
}
