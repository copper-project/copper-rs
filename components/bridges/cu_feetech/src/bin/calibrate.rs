//! Standalone calibration tool for Feetech STS/SCS servos.
//!
//! ```sh
//! cargo run --bin feetech-calibrate -- /dev/ttyACM0 1 2 3 4 5 6
//! cargo run --bin feetech-calibrate -- /dev/ttyACM0 1 2 3 4 5 6 calibration_leader.json
//! cargo run --bin feetech-calibrate -- /dev/ttyACM1 1 2 3 4 5 6 calibration_follower.json
//! ```
//!
//! Move every servo through its full range of motion.  The tool
//! continuously reads positions and tracks each servo's min and max.
//! Press Enter when done.  Output file defaults to `calibration.json`;
//! pass a path as the last argument to override.

use cu_feetech::calibration::{CalibrationData, ServoCalibration};
use cu_linux_resources::LinuxSerialPort;
use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

// -- Minimal Feetech protocol (just enough to read positions) ---------------

const HEADER: [u8; 2] = [0xFF, 0xFF];
const INSTR_READ: u8 = 0x02;
const PRESENT_POSITION: u8 = 56;

fn checksum(data: &[u8]) -> u8 {
    let mut s: u8 = 0;
    for &b in data {
        s = s.wrapping_add(b);
    }
    !s
}

fn send_packet(port: &mut LinuxSerialPort, id: u8, instr: u8, params: &[u8]) -> io::Result<()> {
    let length = (params.len() + 2) as u8;
    let mut pkt = Vec::with_capacity(6 + params.len());
    pkt.extend_from_slice(&HEADER);
    pkt.push(id);
    pkt.push(length);
    pkt.push(instr);
    pkt.extend_from_slice(params);
    pkt.push(checksum(&pkt[2..]));
    port.write_all(&pkt)?;
    port.flush()
}

fn read_response(port: &mut LinuxSerialPort) -> io::Result<Vec<u8>> {
    let mut hdr = [0u8; 4];
    port.read_exact(&mut hdr)?;
    if hdr[0] != 0xFF || hdr[1] != 0xFF {
        return Err(io::Error::other("bad header"));
    }
    let len = hdr[3] as usize;
    let mut rest = vec![0u8; len];
    port.read_exact(&mut rest)?;
    Ok(rest[1..rest.len() - 1].to_vec())
}

fn read_position(port: &mut LinuxSerialPort, id: u8) -> io::Result<u16> {
    send_packet(port, id, INSTR_READ, &[PRESENT_POSITION, 2])?;
    let data = read_response(port)?;
    if data.len() < 2 {
        return Err(io::Error::other("short response"));
    }
    Ok(u16::from_le_bytes([data[0], data[1]]))
}

// -- Entry point ------------------------------------------------------------

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: feetech-calibrate <device> <servo_id> [servo_id …] [output.json]");
        eprintln!("  e.g. feetech-calibrate /dev/ttyACM0 1 2 3 4 5 6");
        eprintln!("  e.g. feetech-calibrate /dev/ttyACM0 1 2 3 4 5 6 calibration_leader.json");
        std::process::exit(1);
    }

    let dev = &args[1];
    let (id_args, output_path): (&[String], &str) =
        if args.len() > 3 && args.last().map(|s| s.ends_with(".json")).unwrap_or(false) {
            let out = args.last().unwrap();
            (&args[2..args.len() - 1], out.as_str())
        } else {
            (&args[2..], "calibration.json")
        };
    let ids: Vec<u8> = id_args
        .iter()
        .map(|s| s.parse().expect("servo IDs must be numbers"))
        .collect();
    if ids.is_empty() {
        eprintln!("At least one servo ID required.");
        std::process::exit(1);
    }
    let n = ids.len();

    let mut port = LinuxSerialPort::open(dev, 1_000_000, 10).expect("Failed to open serial port");

    // Track min/max per servo.
    let mut mins = vec![u16::MAX; n];
    let mut maxs = vec![u16::MIN; n];

    println!("Calibrating {} servos on {dev}", n);
    println!("Move all servos through their full range of motion.");
    println!("Press Enter when done.\n");

    // Background thread waits for Enter so the main loop stays non-blocking.
    let done = Arc::new(AtomicBool::new(false));
    let done2 = done.clone();
    std::thread::spawn(move || {
        let mut buf = [0u8; 1];
        let _ = io::stdin().read(&mut buf);
        done2.store(true, Ordering::Relaxed);
    });

    let mut cycles = 0u64;
    while !done.load(Ordering::Relaxed) {
        for (i, &id) in ids.iter().enumerate() {
            if let Ok(pos) = read_position(&mut port, id) {
                mins[i] = mins[i].min(pos);
                maxs[i] = maxs[i].max(pos);
            }
        }
        cycles += 1;

        // Print live update every ~30 reads.
        if cycles.is_multiple_of(30) {
            print!("\r");
            for (i, &id) in ids.iter().enumerate() {
                print!("  s{id}:[{:>4}–{:>4}]", mins[i], maxs[i]);
            }
            io::stdout().flush().ok();
        }
    }

    println!("\n");

    let calibrations: Vec<ServoCalibration> = ids
        .iter()
        .enumerate()
        .map(|(i, &id)| ServoCalibration {
            id,
            min: mins[i],
            max: maxs[i],
        })
        .collect();

    let data = CalibrationData {
        servos: calibrations,
    };
    let path = std::path::Path::new(output_path);
    data.save(path).expect("Failed to save calibration");

    println!("Saved to {}:", output_path);
    for s in &data.servos {
        println!(
            "  servo {:>2}: min={:>4}  max={:>4}  center={:>4}  range={:>4}",
            s.id,
            s.min,
            s.max,
            s.center(),
            s.range()
        );
    }
}
