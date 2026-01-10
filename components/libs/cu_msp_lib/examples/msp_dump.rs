use std::env;
use std::io::Read;
use std::time::Duration;

use cu_msp_lib::structs::{MspRequest, MspResponse};
use cu_msp_lib::{MspPacketDirection, MspParser};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args().skip(1);
    let device = args.next().unwrap_or_else(|| "/dev/ttyACM0".to_string());
    let baudrate = args
        .next()
        .and_then(|value| value.parse::<u32>().ok())
        .unwrap_or(115_200);

    let mut port = serialport::new(&device, baudrate)
        .timeout(Duration::from_millis(50))
        .open()?;

    let mut parser = MspParser::new();
    let mut buf = [0u8; 256];

    loop {
        match port.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                for &byte in &buf[..n] {
                    match parser.parse(byte) {
                        Ok(Some(packet)) => {
                            if packet.direction == MspPacketDirection::ToFlightController {
                                if let Some(request) = MspRequest::from_packet(&packet) {
                                    println!("request: {request:?}");
                                } else {
                                    println!(
                                        "request: cmd=0x{:04X} data={:?}",
                                        packet.cmd, packet.data
                                    );
                                }
                            } else {
                                println!("response: {:?}", MspResponse::from(packet));
                            }
                        }
                        Ok(None) => {}
                        Err(err) => {
                            eprintln!("parse error: {err:?}");
                        }
                    }
                }
            }
            Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {}
            Err(err) => return Err(err.into()),
        }
    }
}
