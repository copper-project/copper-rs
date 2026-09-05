use super::*;
use cu29::resource::BundleIndex;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

fn component(config: &str) -> ComponentConfig {
    cu29::config::CuConfig::deserialize_ron(&format!(
        "(resources: [(id: \"network\", provider: \"cu29_logstream_udp::CuUdpLogStreamResources\", config: {config})], tasks: [], cnx: [])"
    ))
    .unwrap()
    .resources.remove(0).config.unwrap()
}

fn receiver() -> CuUdpLogStreamRx {
    CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap())
        .open()
        .unwrap()
        .1
}

fn sender(remote: SocketAddr) -> CuUdpLogStreamTx {
    let mut config = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    config.remote_addr = Some(remote);
    config.open().unwrap().0.unwrap()
}

fn receive(rx: &mut CuUdpLogStreamRx, packet: &mut [u8]) -> Result<usize, CuStreamRxError> {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        match rx.try_recv(packet)? {
            Some(len) => return Ok(len),
            None => {
                assert!(Instant::now() < deadline, "UDP packet did not arrive");
                std::thread::yield_now();
            }
        }
    }
}

#[test]
fn packets_are_atomic_and_receive_clones_share_one_queue() {
    let mut rx = receiver();
    let mut tx = sender(rx.local_addr().unwrap());
    let mut rx_clone = rx.clone();
    let mut tx_clone = tx.clone();
    assert_eq!(tx.local_addr().unwrap(), tx_clone.local_addr().unwrap());
    let mut packet = [0; 32];
    assert_eq!(rx.try_recv(&mut packet), Ok(None));
    tx.try_send(b"first").unwrap();
    tx_clone.try_send(b"second packet").unwrap();
    let first = receive(&mut rx, &mut packet).unwrap();
    assert_eq!(&packet[..first], b"first");
    let second = receive(&mut rx_clone, &mut packet).unwrap();
    assert_eq!(&packet[..second], b"second packet");
    assert_eq!(rx.try_recv(&mut packet), Ok(None));
    assert_eq!(rx_clone.try_recv(&mut packet), Ok(None));
}

#[test]
fn truncation_consumes_one_packet_and_never_returns_partial_success() {
    let mut rx = receiver();
    let mut tx = sender(rx.local_addr().unwrap());
    let mut packet = [0; 8];
    tx.try_send(&[7; 128]).unwrap();
    tx.try_send(b"next").unwrap();
    let error = receive(&mut rx, &mut packet).unwrap_err();
    #[cfg(any(target_os = "linux", target_os = "android"))]
    assert_eq!(error, CuStreamRxError::BufferTooSmall { needed: 128 });
    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    assert_eq!(
        error,
        CuStreamRxError::BufferTooSmall {
            needed: UDP_DATAGRAM_CAPACITY
        }
    );
    let len = receive(&mut rx, &mut packet).unwrap();
    assert_eq!(&packet[..len], b"next");
    assert_eq!(rx.try_recv(&mut packet), Ok(None));
}

#[test]
fn empty_packets_and_exact_capacity_are_distinct_from_no_data() {
    let mut rx = receiver();
    let mut tx = sender(rx.local_addr().unwrap());
    tx.try_send(b"").unwrap();
    assert_eq!(receive(&mut rx, &mut []), Ok(0));
    assert_eq!(rx.try_recv(&mut []), Ok(None));
    tx.try_send(b"x").unwrap();
    assert!(matches!(
        receive(&mut rx, &mut []),
        Err(CuStreamRxError::BufferTooSmall { .. })
    ));
    tx.try_send(b"12345678").unwrap();
    let mut packet = [0; 8];
    assert_eq!(receive(&mut rx, &mut packet), Ok(8));
    assert_eq!(&packet, b"12345678");
}

#[test]
fn socket_options_are_applied_and_slots_share_the_bound_socket() {
    let config = component(
        r#"{
        "bind_addr": "127.0.0.1:0", "remote_addr": "127.0.0.1:7447",
        "send_buffer_bytes": 32768, "recv_buffer_bytes": 32768,
        "ttl": 7
    }"#,
    );
    #[cfg(not(windows))]
    let config = {
        let mut config = config;
        config.set("dscp", 46u8);
        config
    };
    let mut manager = ResourceManager::new(&[2]);
    CuUdpLogStreamResources::build(
        BundleContext::new(BundleIndex::new(0), "network"),
        Some(&config),
        &mut manager,
    )
    .unwrap();
    let context = BundleContext::<CuUdpLogStreamResources>::new(BundleIndex::new(0), "network");
    let tx: CuUdpLogStreamTx = manager
        .take(context.key(CuUdpLogStreamResourcesId::Tx))
        .unwrap()
        .0;
    let mut rx: CuUdpLogStreamRx = manager
        .take(context.key(CuUdpLogStreamResourcesId::Rx))
        .unwrap()
        .0;
    assert_eq!(tx.local_addr().unwrap(), rx.local_addr().unwrap());
    assert_eq!(tx.socket.ttl_v4().unwrap(), 7);
    #[cfg(not(windows))]
    assert_eq!(tx.socket.tos_v4().unwrap(), 46 << 2);
    assert!(tx.socket.send_buffer_size().unwrap() >= 32768);
    assert!(rx.socket.recv_buffer_size().unwrap() >= 32768);
    assert_eq!(rx.try_recv(&mut [0; 8]), Ok(None));
}

#[test]
fn receiver_only_bundle_does_not_register_a_transmitter() {
    let config = component(r#"{"bind_addr": "127.0.0.1:0"}"#);
    let mut manager = ResourceManager::new(&[2]);
    CuUdpLogStreamResources::build(
        BundleContext::new(BundleIndex::new(0), "listen"),
        Some(&config),
        &mut manager,
    )
    .unwrap();
    let context = BundleContext::<CuUdpLogStreamResources>::new(BundleIndex::new(0), "listen");
    assert!(
        manager
            .take::<CuUdpLogStreamTx>(context.key(CuUdpLogStreamResourcesId::Tx))
            .is_err()
    );
    assert!(
        manager
            .take::<CuUdpLogStreamRx>(context.key(CuUdpLogStreamResourcesId::Rx))
            .is_ok()
    );
}

#[test]
fn invalid_configuration_is_rejected_before_socket_setup() {
    for config in [
        r#"{}"#,
        r#"{"bind_addr": "localhost:1234"}"#,
        r#"{"bind_addr": 1234}"#,
        r#"{"bind_addr": "127.0.0.1:0", "mtu_bytes": 1200}"#,
        r#"{"bind_addr": "127.0.0.1:0", "remote_addr": "[::1]:1234"}"#,
        r#"{"bind_addr": "127.0.0.1:0", "remote_addr": "127.0.0.1:0"}"#,
        r#"{"bind_addr": "127.0.0.1:0", "send_buffer_bytes": 0}"#,
        r#"{"bind_addr": "127.0.0.1:0", "send_buffer_bytes": 4294967296}"#,
        r#"{"bind_addr": "127.0.0.1:0", "recv_buffer_bytes": -1}"#,
        r#"{"bind_addr": "127.0.0.1:0", "ttl": 0}"#,
        r#"{"bind_addr": "127.0.0.1:0", "ttl": 256}"#,
        r#"{"bind_addr": "127.0.0.1:0", "ttl": 4294967297}"#,
        r#"{"bind_addr": "127.0.0.1:0", "dscp": 64}"#,
        r#"{"bind_addr": "127.0.0.1:0", "dscp": 256}"#,
        r#"{"bind_addr": "127.0.0.1:0", "dscp": "46"}"#,
    ] {
        let component = component(config);
        assert!(
            CuUdpLogStreamConfig::from_component(&component).is_err(),
            "accepted {config}"
        );
    }
    let mut manager = ResourceManager::new(&[2]);
    assert!(
        CuUdpLogStreamResources::build(
            BundleContext::new(BundleIndex::new(0), "missing"),
            None,
            &mut manager,
        )
        .is_err()
    );
    let mut typed = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    typed.dscp = Some(64);
    assert!(typed.open().is_err());
}

#[test]
fn bind_errors_preserve_the_socket_setup_cause() {
    let held = UdpSocket::bind("127.0.0.1:0").unwrap();
    let error = CuUdpLogStreamConfig::new(held.local_addr().unwrap())
        .open()
        .unwrap_err();
    assert!(
        error
            .to_string()
            .contains("Failed to configure UDP stream socket")
    );
}

#[cfg(windows)]
#[test]
fn windows_rejects_explicit_dscp_instead_of_claiming_it_was_applied() {
    let mut config = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    config.dscp = Some(46);
    let error = config.open().unwrap_err();
    assert!(
        error
            .to_string()
            .contains("DSCP configuration is unsupported on Windows")
    );
}

#[test]
fn backpressure_and_interruption_are_returned_after_one_submission() {
    for (kind, expected) in [
        (io::ErrorKind::WouldBlock, CuStreamTxError::WouldBlock),
        (
            io::ErrorKind::Interrupted,
            CuStreamTxError::Failed("UDP datagram submission failed"),
        ),
        (
            io::ErrorKind::PermissionDenied,
            CuStreamTxError::Failed("UDP datagram submission failed"),
        ),
    ] {
        let mut calls = 0;
        let result = submit_once(b"packet", |packet| {
            calls += 1;
            assert_eq!(packet, b"packet");
            Err(kind.into())
        });
        assert_eq!(result, Err(expected));
        assert_eq!(calls, 1);
    }
    assert!(matches!(
        submit_once(b"packet", |_| Ok(2)),
        Err(CuStreamTxError::Failed(_))
    ));
}

#[test]
fn oversized_udp_send_fails_without_splitting_or_retaining_the_packet() {
    let mut rx = receiver();
    let mut tx = sender(rx.local_addr().unwrap());
    assert!(matches!(
        tx.try_send(&vec![0; 65536]),
        Err(CuStreamTxError::Failed(_))
    ));
    assert_eq!(rx.try_recv(&mut [0; 8]), Ok(None));
    tx.try_send(b"ok").unwrap();
    let mut packet = [0; 8];
    assert_eq!(receive(&mut rx, &mut packet), Ok(2));
    assert_eq!(&packet[..2], b"ok");
}

#[test]
fn full_receive_queue_drops_packets_without_replay_or_partial_packets() {
    let mut config = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    config.recv_buffer_bytes = Some(8192);
    let mut rx = config.open().unwrap().1;
    let mut tx = sender(rx.local_addr().unwrap());
    const COUNT: usize = 4096;
    let mut sent = 0;
    for id in 0..COUNT {
        let mut packet = [0xa5; 1024];
        packet[..8].copy_from_slice(&(id as u64).to_le_bytes());
        match tx.try_send(&packet) {
            Ok(()) => sent += 1,
            Err(CuStreamTxError::WouldBlock) => {}
            other => panic!("unexpected send result: {other:?}"),
        }
    }
    let mut last = None;
    let mut received = 0;
    let mut packet = [0; 1024];
    while let Some(len) = rx.try_recv(&mut packet).unwrap() {
        assert_eq!(len, packet.len());
        assert!(packet[8..].iter().all(|&byte| byte == 0xa5));
        let id = u64::from_le_bytes(packet[..8].try_into().unwrap());
        assert!(last.is_none_or(|previous| id > previous));
        last = Some(id);
        received += 1;
        assert!(received <= COUNT);
    }
    assert!(received > 0);
    assert!(
        received < sent,
        "small unread receive queue should shed UDP traffic"
    );
    tx.try_send(b"after pressure").unwrap();
    let len = receive(&mut rx, &mut packet).unwrap();
    assert_eq!(&packet[..len], b"after pressure");
    assert_eq!(rx.try_recv(&mut packet), Ok(None));
}

#[test]
fn ipv6_loopback_uses_hop_limit_and_preserves_packets() {
    let mut config = CuUdpLogStreamConfig::new("[::1]:0".parse().unwrap());
    config.ttl = Some(9);
    #[cfg(any(target_os = "linux", target_os = "macos", target_os = "android"))]
    {
        config.dscp = Some(12);
    }
    let mut rx = config.open().unwrap().1;
    config.remote_addr = Some(rx.local_addr().unwrap());
    let mut tx = config.open().unwrap().0.unwrap();
    assert_eq!(tx.socket.unicast_hops_v6().unwrap(), 9);
    #[cfg(any(target_os = "linux", target_os = "macos", target_os = "android"))]
    assert_eq!(tx.socket.tclass_v6().unwrap(), 12 << 2);
    tx.try_send(b"ipv6").unwrap();
    let mut packet = [0; 4];
    assert_eq!(receive(&mut rx, &mut packet), Ok(4));
    assert_eq!(&packet, b"ipv6");
}
