You can use this `sansio` implementation (for now, only the transport) with any `io` as long as you can send bytes and receive bytes

An async version :
```rust
async fn handle_client(
    mut stream: smol::net::TcpStream,
    mut transport: Transport<[u8; BATCH_SIZE]>,
) {
    // The idea is that the sansio transport is not aware of the global time. You must provide yourself a representation of the time through
    // a `core::time::Duration`. The time machine is only updated when you do a `transport.{?/tx/rx}.sync(now: Duration)`. If you don't use your `tx` within the `tx` lease, the
    // time state machine will close your transport (and the remote peer may do as well). If your `rx` doesn't get any data within the `rx` lease, the time state machine will close the transport.

    println!("Opening a connection with {:?}...", stream.peer_addr());
    let start = Instant::now();

    loop {
        transport.sync(start.elapsed());

        if transport.closed() {
            println!("Transport requested a close");
            break;
        }

        // After this timeout you should probably send a keepalive message. You can chek that doing `transport.tx.should_send_keepalive()`.  You may also check if you need to close the transport first
        let write_lease = smol::Timer::at(start + transport.tx.next_timeout());
        // After this timeout you should probably close your transport. You can chek that doing `transport.rx.should_close()`
        let read_lease = smol::Timer::at(start + transport.rx.next_timeout());

        match select3(
            write_lease,
            read_lease,
            transport.rx.decode_prefixed_with_async(async |bytes| {
                stream.read_exact(bytes).await.map(|_| bytes.len())
            }),
        )
        .await
        {
            Either3::First(_) => {
                if transport.tx.should_close(start.elapsed()) {
                    eprintln!("Transport didn't send keepalive in time");
                    break;
                }

                if transport.tx.should_send_keepalive(start.elapsed()) {
                    transport.tx.keepalive();
                    if let Some(bytes) = transport.tx.flush_prefixed() {
                        println!("Sending KeepAlive");
                        if stream.write_all(bytes).await.is_err() {
                            break;
                        }
                    }
                }
            }
            Either3::Second(_) => {
                if transport.rx.should_close(start.elapsed()) {
                    eprintln!("Transport didn't receive keepalive in time");
                    break;
                }
            }
            Either3::Third(res) => {
                if res.is_err() {
                    break;
                }
                println!("Received msgs");
            }
        }
    }

    eprintln!("Connection closed!");
}
```

A busy sync way of doing this:
```rust
fn handle_client(mut stream: std::net::TcpStream, mut transport: Transport<[u8; BATCH_SIZE]>) {
    println!("Reading indefinitely from {:?}...", stream.peer_addr());
    let start = Instant::now();

    stream.set_nonblocking(true).unwrap();
    loop {
        transport.sync(start.elapsed());
        if transport.closed() {
            break;
        }

        let write_lease = start + transport.tx.next_timeout();
        let read_lease = start + transport.rx.next_timeout();

        if Instant::now() > write_lease {
            if transport.tx.should_close(start.elapsed()) {
                eprintln!("Transport didn't send keepalive in time");
                break;
            }

            if transport.tx.should_send_keepalive(start.elapsed()) {
                transport.tx.keepalive();
                if let Some(bytes) = transport.tx.flush_prefixed() {
                    println!("Sending KeepAlive");
                    if stream.write_all(bytes).is_err() {
                        break;
                    }
                }
            }
        }

        if Instant::now() > read_lease {
            if transport.rx.should_close(start.elapsed()) {
                eprintln!("Transport didn't receive keepalive in time");
                break;
            }
        }

        match transport
            .rx
            .decode_prefixed_with(|bytes| stream.read_exact(bytes).map(|_| bytes.len()))
        {
            Ok(_) => {}
            Err(e) => match e {
                EitherError::Other(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                _ => break,
            },
        };

        // We just clear, because `flush` would clear and create an iterator. Here we don't want this overhead
        transport.rx.clear();
    }
}
```
