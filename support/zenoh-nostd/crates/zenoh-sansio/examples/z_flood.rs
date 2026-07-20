use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
};

use zenoh_proto::{
    exts::QoS,
    fields::{Reliability, WireExpr},
    keyexpr,
    msgs::*,
};
use zenoh_sansio::{Transport, ZTransportTx};

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
    let put = NetworkMessage {
        reliability: Reliability::default(),
        qos: QoS::default(),
        body: NetworkBody::Push(Push {
            wire_expr: WireExpr::from(keyexpr::from_str_unchecked("test/thr")),
            payload: PushBody::Put(Put {
                payload: &[0, 1, 2, 3, 4, 5, 6, 7],
                ..Default::default()
            }),
            ..Default::default()
        }),
    };

    transport
        .tx
        .encode_ref(core::iter::repeat_n(put.as_ref(), 200));
    let bytes = transport.tx.flush_prefixed().unwrap();

    println!("Sending indefinitely to {:?}...", stream.peer_addr());
    loop {
        if stream.write_all(bytes).is_err() {
            break;
        }
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
