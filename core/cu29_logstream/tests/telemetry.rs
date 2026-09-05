#![cfg(feature = "std")]

use cu29_logstream::telemetry::telemetry_channel;
use std::{
    future::Future,
    pin::pin,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    task::{Context, Poll, Wake, Waker},
    thread,
    time::Duration,
};

#[test]
fn overwrite_reports_local_loss_and_preserves_the_oldest_retained_frame() {
    let (mut publisher, mut reader) = telemetry_channel(3.try_into().unwrap(), 0);
    // Source IDs have a gap and restart; they are not the delivery sequence.
    for source_id in [100, 105, 0, 1, 2] {
        publisher.publish(source_id);
    }
    let read = reader.try_read().unwrap();
    assert_eq!((*read.frame, read.missed), (0, 2));
    assert_eq!(*reader.try_read().unwrap().frame, 1);
    assert_eq!(*reader.try_read().unwrap().frame, 2);
    assert!(reader.try_read().is_none());
    assert_eq!(reader.overwritten(), 2);
}

#[test]
fn borrowed_frame_survives_producer_overwrite_on_another_thread() {
    let (mut publisher, mut reader) = telemetry_channel(1.try_into().unwrap(), 0);
    publisher.publish(String::from("borrowed"));
    let read = reader.try_read().unwrap();
    thread::spawn(move || {
        for _ in 0..1000 {
            publisher.publish(String::from("new"));
        }
        publisher.set_status(7);
    })
    .join()
    .unwrap();
    assert_eq!(read.frame, "borrowed");
    let read = reader.try_read().unwrap();
    assert_eq!(read.frame, "new");
    assert_eq!(read.missed, 999);
    assert_eq!(reader.status(), 7);
    assert!(reader.is_closed());
}

#[test]
fn paused_reader_can_observe_status_without_consuming_frames() {
    let (mut publisher, mut reader) = telemetry_channel(2.try_into().unwrap(), 0);
    reader.status();
    assert!(!reader.wait_timeout(Duration::ZERO));
    for i in 0..8 {
        publisher.publish(i);
        publisher.set_status(i);
    }
    assert_eq!(reader.status(), 7);
    assert_eq!(reader.try_read().unwrap().missed, 6);
    assert_eq!(reader.status(), 7);
}

#[derive(Default)]
struct WakeCount(AtomicUsize);
impl Wake for WakeCount {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
    fn wake_by_ref(self: &Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn readiness_handles_preexisting_data_status_closure_and_cancelled_waits() {
    let (mut publisher, mut reader) = telemetry_channel(2.try_into().unwrap(), 0);
    let count = Arc::new(WakeCount::default());
    let waker = Waker::from(count.clone());
    let mut cx = Context::from_waker(&waker);
    reader.status();
    {
        let mut ready = pin!(reader.ready());
        assert_eq!(ready.as_mut().poll(&mut cx), Poll::Pending);
        publisher.set_status(1);
        assert!(count.0.load(Ordering::Relaxed) > 0);
        assert_eq!(ready.as_mut().poll(&mut cx), Poll::Ready(()));
    }
    assert_eq!(reader.status(), 1);
    {
        let mut cancelled = pin!(reader.ready());
        assert_eq!(cancelled.as_mut().poll(&mut cx), Poll::Pending);
    }
    publisher.publish(42);
    assert_eq!(pin!(reader.ready()).as_mut().poll(&mut cx), Poll::Ready(()));
    assert_eq!(*reader.try_read().unwrap().frame, 42);
    drop(publisher);
    assert!(reader.wait_timeout(Duration::ZERO));
    assert!(reader.is_closed());
    assert!(reader.try_read().is_none());
}

#[test]
fn no_wakeup_is_lost_when_registration_races_publication() {
    let (mut publisher, mut reader) = telemetry_channel(1.try_into().unwrap(), 0);
    reader.status();
    let producer = thread::spawn(move || {
        for i in 0..100_000 {
            publisher.publish(i);
        }
    });
    let mut total = 0;
    loop {
        assert!(reader.wait_timeout(Duration::from_secs(5)));
        while let Some(read) = reader.try_read() {
            total += read.missed + 1;
        }
        if reader.is_closed() {
            // Closure can race the last empty read. Drain again after Acquire.
            while let Some(read) = reader.try_read() {
                total += read.missed + 1;
            }
            break;
        }
    }
    producer.join().unwrap();
    assert_eq!(total, 100_000);
}

#[test]
fn disconnect_discards_without_growing_or_blocking() {
    let (mut publisher, reader) = telemetry_channel(2.try_into().unwrap(), 0);
    publisher.publish(1);
    drop(reader);
    assert!(!publisher.is_connected());
    for i in 0..100_000 {
        publisher.publish(i);
    }
    assert_eq!(publisher.overwritten(), 0);
}
