//! Bounded over-the-air smoke test for two HC-12s connected to this host.
#[cfg(unix)]
mod test_pair {
    use cu_hc12::{Hc12, host::StartupDelay};
    use cu_linux_resources::{
        DEFAULT_SERIAL_PARITY, DEFAULT_SERIAL_STOPBITS, LinuxNonblockingSerialPort,
        LinuxSerialRtsPin, SerialSlotConfig,
    };
    use cu_serial::SerialIo;
    use cu29::prelude::*;
    use std::time::{Duration, Instant};

    type Radio = Hc12<LinuxNonblockingSerialPort, LinuxSerialRtsPin>;
    const TIMEOUT: Duration = Duration::from_secs(5);

    fn open(device: String, baudrate: u32, channel: u8) -> CuResult<Radio> {
        let config = SerialSlotConfig {
            dev: device.clone(),
            baudrate,
            parity: DEFAULT_SERIAL_PARITY,
            stop_bits: DEFAULT_SERIAL_STOPBITS,
            timeout_ms: 50,
        };
        let (serial, set) = LinuxNonblockingSerialPort::open_with_rts(&config)
            .map_err(|e| CuError::new_with_cause(&format!("Open {device} with RTS"), e))?;
        let radio = Hc12::configure(serial, set, &mut StartupDelay, channel)
            .map_err(|e| CuError::new_with_cause(&format!("Configure {device}"), e))?;
        println!("{device}: AT configuration passed (FU3, channel {channel}, {baudrate} baud)");
        Ok(radio)
    }

    fn transfer(tx: &mut Radio, rx: &mut Radio, bytes: &[u8]) -> CuResult<()> {
        let deadline = Instant::now() + TIMEOUT;
        let mut sent = 0;
        let mut received = 0;
        let mut buffer = [0; 256];
        while received < bytes.len() {
            if Instant::now() >= deadline {
                return Err(CuError::from(format!(
                    "Radio transfer timed out: accepted {sent}, received {received}, expected {} bytes",
                    bytes.len()
                )));
            }
            if sent < bytes.len() {
                sent += tx
                    .try_write(&bytes[sent..])
                    .map_err(|e| CuError::new_with_cause("Radio TX", e))?;
            }
            let n = rx
                .try_read(&mut buffer)
                .map_err(|e| CuError::new_with_cause("Radio RX", e))?;
            if received + n > bytes.len() || buffer[..n] != bytes[received..received + n] {
                return Err(CuError::from("Radio payload mismatch or unexpected bytes"));
            }
            received += n;
            std::thread::sleep(Duration::from_millis(1));
        }
        // Allow the half-duplex radio to finish before reversing direction.
        std::thread::sleep(Duration::from_millis(100));
        if rx
            .try_read(&mut buffer)
            .map_err(|e| CuError::new_with_cause("Check trailing radio data", e))?
            != 0
        {
            return Err(CuError::from("Unexpected trailing radio bytes"));
        }
        Ok(())
    }

    pub fn run() -> CuResult<()> {
        let args: Vec<_> = std::env::args().skip(1).collect();
        if args.len() != 2 || args[0] == args[1] {
            return Err(CuError::from(
                "Usage: just pair /dev/ttyACM0 /dev/ttyACM1 (distinct ports)",
            ));
        }
        // Reuse the example's RON channel and UART speed rather than hidden tuning knobs.
        let config = cu29::config::read_configuration(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/examples/echo.ron"
        ))?;
        let board = config
            .resources
            .iter()
            .find(|r| r.id == "board")
            .and_then(|r| r.config.as_ref())
            .ok_or_else(|| CuError::from("Missing board config"))?;
        let radio = config
            .resources
            .iter()
            .find(|r| r.id == "radio")
            .and_then(|r| r.config.as_ref())
            .ok_or_else(|| CuError::from("Missing radio config"))?;
        let baud = board.get::<u32>("serial0_baudrate")?.unwrap_or(9600);
        let channel = radio
            .get::<u8>("channel")?
            .ok_or_else(|| CuError::from("Missing radio channel"))?;
        let mut first = open(args[0].clone(), baud, channel)?;
        let mut second = open(args[1].clone(), baud, channel)?;
        let mut payload = [0; 256];
        for (index, byte) in payload.iter_mut().enumerate() {
            *byte = index as u8;
        }
        for len in [1, 31, 128, 256] {
            transfer(&mut first, &mut second, &payload[..len])?;
            println!("{} -> {}: {len} bytes verified", args[0], args[1]);
            payload.reverse();
            transfer(&mut second, &mut first, &payload[..len])?;
            println!("{} -> {}: {len} bytes verified", args[1], args[0]);
            payload.reverse();
        }
        println!("PASS: bidirectional radio data, including all 256 byte values");
        Ok(())
    }
}

#[cfg(unix)]
fn main() -> cu29::CuResult<()> {
    test_pair::run()
}

#[cfg(not(unix))]
fn main() {
    eprintln!("The pair test requires Unix serial RTS resources.");
}
