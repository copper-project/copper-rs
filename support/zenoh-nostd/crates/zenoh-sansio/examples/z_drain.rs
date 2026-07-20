use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
};

use zenoh_sansio::{Transport, ZTransportRx, ZTransportTx};

use zenoh_proto::{
    exts::QoS,
    fields::{Reliability, WireExpr},
    keyexpr,
    msgs::*,
};
const BATCH_SIZE: usize = u16::MAX as usize;

fn open_listen(stream: &mut std::net::TcpStream) -> Transport<[u8; BATCH_SIZE]> {
    Transport::builder([0u8; BATCH_SIZE])
        .listen(
            stream,
            |stream, bytes| stream.read_exact(bytes).map(|_| bytes.len()),
            |stream, bytes| stream.write_all(bytes),
        )
        .prefixed()
        .finish()
        .expect("Error doing handshake")
}

fn open_connect(stream: &mut std::net::TcpStream) -> Transport<[u8; BATCH_SIZE]> {
    Transport::builder([0u8; BATCH_SIZE])
        .connect(
            stream,
            |stream, bytes| stream.read_exact(bytes).map(|_| bytes.len()),
            |stream, bytes| stream.write_all(bytes),
        )
        .prefixed()
        .finish()
        .expect("Error doing handshake")
}

fn handle_client(mut stream: std::net::TcpStream, mut transport: Transport<[u8; BATCH_SIZE]>) {
    let declare = NetworkMessage {
        reliability: Reliability::default(),
        qos: QoS::default(),
        body: NetworkBody::Declare(Declare {
            body: DeclareBody::DeclareSubscriber(DeclareSubscriber {
                id: 0,
                wire_expr: WireExpr::from(keyexpr::from_str_unchecked("test/thr/**")),
            }),
            ..Default::default()
        }),
    };

    transport.tx.encode(core::iter::once(declare));
    let bytes = transport.tx.flush_prefixed().unwrap();
    stream.write_all(bytes).unwrap();

    println!("Reading indefinitely from {:?}...", stream.peer_addr());
    loop {
        if transport
            .rx
            .decode_prefixed_with(|bytes| stream.read_exact(bytes).map(|_| bytes.len()))
            .is_err()
        {
            break;
        }

        // We just clear, because `flush` would clear and create an iterator. Here we don't want this overhead
        transport.rx.clear();
    }
}

fn main() {
    match std::env::args().nth(1) {
        None => {
            let listener = TcpListener::bind("127.0.0.1:7447").expect("Could not bind");
            for stream in listener.incoming() {
                match stream {
                    Ok(mut stream) => {
                        let transport = open_listen(&mut stream);
                        handle_client(stream, transport)
                    }
                    Err(e) => {
                        panic!("Error accepting connection: {}", e);
                    }
                }
            }
        }
        Some(str) => match str.as_str() {
            "--listen" => {
                let listener = TcpListener::bind("127.0.0.1:7447").expect("Could not bind");
                for stream in listener.incoming() {
                    match stream {
                        Ok(mut stream) => {
                            let transport = open_listen(&mut stream);
                            handle_client(stream, transport)
                        }
                        Err(e) => {
                            panic!("Error accepting connection: {}", e);
                        }
                    }
                }
            }
            "--connect" => {
                let mut stream = TcpStream::connect("127.0.0.1:7447").expect("Couldn't connect");
                let transport = open_connect(&mut stream);
                handle_client(stream, transport)
            }
            _ => {
                panic!("Invalid argument")
            }
        },
    }
}
