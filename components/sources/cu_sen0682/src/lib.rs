#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod protocol;

use alloc::format;
use core::fmt;

#[cfg(feature = "std")]
use cu_linux_resources::LinuxSerialPort;
use cu_sensor_payloads::PointCloudSoa;
use cu29::prelude::*;
use cu29::resource::{Owned, ResourceBindingMap, ResourceBindings, ResourceManager};
use embedded_io::{ErrorKind, ErrorType, Read, Write};

pub use protocol::{MAX_FRAME_BYTES, MAX_POINTS};

const SERIAL_BUFFER_BYTES: usize = protocol::MAX_FRAME_BYTES * 2;
const DEFAULT_MIN_RANGE_M: f32 = 0.05;
const DEFAULT_ROW_ID: u8 = 0;
const DEFAULT_START_COLUMN: u8 = 1;
const DEFAULT_END_COLUMN: u8 = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sen0682ReadoutConfig {
    pub configure_device: bool,
    pub row_id: u8,
    pub start_column: u8,
    pub end_column: u8,
}

impl Sen0682ReadoutConfig {
    fn from_component_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let configure_device = cfg_bool(config, "configure_device", true)?;
        let row_id = cfg_u8(config, "row_id", DEFAULT_ROW_ID)?;
        let start_column = cfg_u8(config, "start_column", DEFAULT_START_COLUMN)?;
        let end_column = cfg_u8(config, "end_column", DEFAULT_END_COLUMN)?;

        if row_id > 8 {
            return Err(CuError::from(format!(
                "sen0682 row_id must be between 0 and 8, got {row_id}"
            )));
        }
        if !(1..=64).contains(&start_column) {
            return Err(CuError::from(format!(
                "sen0682 start_column must be between 1 and 64, got {start_column}"
            )));
        }
        if !(1..=64).contains(&end_column) {
            return Err(CuError::from(format!(
                "sen0682 end_column must be between 1 and 64, got {end_column}"
            )));
        }
        if start_column > end_column {
            return Err(CuError::from(format!(
                "sen0682 start_column ({start_column}) must be <= end_column ({end_column})"
            )));
        }

        Ok(Self {
            configure_device,
            row_id,
            start_column,
            end_column,
        })
    }
}

fn cfg_bool(config: Option<&ComponentConfig>, key: &str, default: bool) -> CuResult<bool> {
    Ok(match config {
        Some(cfg) => cfg.get::<bool>(key)?.unwrap_or(default),
        None => default,
    })
}

fn cfg_u8(config: Option<&ComponentConfig>, key: &str, default: u8) -> CuResult<u8> {
    let raw = match config {
        Some(cfg) => cfg.get::<u64>(key)?.unwrap_or(default as u64),
        None => default as u64,
    };
    u8::try_from(raw).map_err(|_| {
        CuError::from(format!(
            "sen0682 config key `{key}` must fit in u8, got {raw}"
        ))
    })
}

fn cfg_f32(config: Option<&ComponentConfig>, key: &str, default: f32) -> CuResult<f32> {
    let raw = match config {
        Some(cfg) => cfg.get::<f64>(key)?.unwrap_or(default as f64),
        None => default as f64,
    };
    Ok(raw as f32)
}

trait FrameTransport {
    fn start(&mut self, readout: &Sen0682ReadoutConfig) -> CuResult<()>;
    fn read_frame(&mut self, out: &mut [u8]) -> CuResult<Option<usize>>;
    fn stop(&mut self) -> CuResult<()> {
        Ok(())
    }
}

struct Sen0682SourceCore<T> {
    transport: T,
    // The sensor publishes a small, fixed-size frame, so keeping the scratch
    // buffers inline is simpler and more deterministic than pulling a pool into
    // a 15 Hz source.
    frame_buffer: [u8; protocol::MAX_FRAME_BYTES],
    min_range_m: f32,
}

impl<T> Sen0682SourceCore<T>
where
    T: FrameTransport,
{
    fn new(transport: T, min_range_m: f32) -> Self {
        Self {
            transport,
            frame_buffer: [0u8; protocol::MAX_FRAME_BYTES],
            min_range_m,
        }
    }

    fn start(&mut self, readout: &Sen0682ReadoutConfig) -> CuResult<()> {
        self.transport.start(readout)
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        output: &mut CuMsg<PointCloudSoa<MAX_POINTS>>,
    ) -> CuResult<()> {
        let Some(frame_len) = self.transport.read_frame(&mut self.frame_buffer)? else {
            output.metadata.set_status("waiting");
            output.clear_payload();
            return Ok(());
        };

        // Reusing the message-owned storage keeps the hot path from re-zeroing a
        // 512-point SoA every cycle just because the transport delivered another frame.
        let payload = output
            .payload_mut()
            .get_or_insert_with(PointCloudSoa::<MAX_POINTS>::default);

        let stats = protocol::decode_frame_into(
            &self.frame_buffer[..frame_len],
            ctx.now(),
            self.min_range_m,
            payload,
        )
        .map_err(|err| CuError::from(format!("sen0682 frame decode failed: {err}")))?;

        if payload.len == 0 {
            output.metadata.set_status("filtered");
            output.clear_payload();
            return Ok(());
        }

        output.metadata.set_status("streaming");
        output.tov = Tov::Time(ctx.now());

        if stats.frame_idx == 0 {
            debug!(
                "sen0682: first frame width={} height={} points={} device_index={}",
                stats.width, stats.height, stats.valid_points, stats.device_index
            );
        }

        Ok(())
    }

    fn stop(&mut self) -> CuResult<()> {
        self.transport.stop()
    }
}

struct SerialTransport<S> {
    serial: S,
    buffer: [u8; SERIAL_BUFFER_BYTES],
    buffered: usize,
    configured_by_driver: bool,
}

impl<S> SerialTransport<S> {
    fn new(serial: S) -> Self {
        Self {
            serial,
            buffer: [0u8; SERIAL_BUFFER_BYTES],
            buffered: 0,
            configured_by_driver: false,
        }
    }

    fn discard_prefix(&mut self, count: usize) {
        if count >= self.buffered {
            self.buffered = 0;
            return;
        }
        self.buffer.copy_within(count..self.buffered, 0);
        self.buffered -= count;
    }

    fn keep_tail(&mut self, count: usize) {
        if count >= self.buffered {
            return;
        }
        let start = self.buffered - count;
        self.buffer.copy_within(start..self.buffered, 0);
        self.buffered = count;
    }

    fn try_extract_frame(&mut self, out: &mut [u8]) -> CuResult<Option<usize>> {
        loop {
            if self.buffered < protocol::TAG.len() {
                return Ok(None);
            }

            let Some(tag_pos) = protocol::find_tag(&self.buffer[..self.buffered]) else {
                self.keep_tail(self.buffered.min(protocol::TAG.len() - 1));
                return Ok(None);
            };

            if tag_pos > 0 {
                self.discard_prefix(tag_pos);
            }

            if self.buffered < protocol::HEADER_BYTES {
                return Ok(None);
            }

            match protocol::frame_total_bytes_from_prefix(&self.buffer[..protocol::HEADER_BYTES]) {
                Ok(Some(total_bytes)) => {
                    if total_bytes > out.len() {
                        return Err(CuError::from(format!(
                            "sen0682 frame length {total_bytes} exceeds parser buffer {}",
                            out.len()
                        )));
                    }
                    if self.buffered < total_bytes {
                        return Ok(None);
                    }
                    out[..total_bytes].copy_from_slice(&self.buffer[..total_bytes]);
                    self.discard_prefix(total_bytes);
                    return Ok(Some(total_bytes));
                }
                Ok(None) => return Ok(None),
                Err(_) => {
                    self.discard_prefix(1);
                }
            }
        }
    }
}

impl<S> SerialTransport<S>
where
    S: Read + Write + ErrorType,
    <S as ErrorType>::Error: embedded_io::Error + fmt::Debug,
{
    fn send_command(&mut self, command: &str) -> CuResult<()> {
        write_all(&mut self.serial, command.as_bytes())?;
        self.serial
            .flush()
            .map_err(|err| CuError::from(format!("sen0682 command flush failed: {err:?}")))?;

        let mut saw_error = false;
        let mut scratch = [0u8; 256];
        loop {
            match self.serial.read(&mut scratch) {
                Ok(0) => break,
                Ok(n) => {
                    saw_error |= contains_ascii_token(&scratch[..n], b"ERROR");
                }
                Err(err) if is_idle_io_error(&err) => break,
                Err(err) => {
                    return Err(CuError::from(format!(
                        "sen0682 response read failed after `{}`: {err:?}",
                        command.trim()
                    )));
                }
            }
        }

        if saw_error {
            return Err(CuError::from(format!(
                "sen0682 rejected command `{}`",
                command.trim()
            )));
        }

        Ok(())
    }

    fn configure_streaming(&mut self, readout: &Sen0682ReadoutConfig) -> CuResult<()> {
        if !readout.configure_device {
            self.configured_by_driver = false;
            return Ok(());
        }

        self.send_command("AT+STREAM_CONTROL=0\n")?;
        self.send_command("AT+STREAM_DATA_TYPE=3\n")?;
        self.send_command("AT+SPAD_FRAME_MODE=0\n")?;
        self.send_command(
            format!(
                "AT+SPAD_OUTPUT_LINE_DATA={},{},{}\n",
                readout.row_id, readout.start_column, readout.end_column
            )
            .as_str(),
        )?;
        // A Copper source should not rewrite bench hardware state as a startup
        // side effect, so we intentionally stay away from AT+SAVE_CONFIG here.
        self.send_command("AT+STREAM_CONTROL=1\n")?;
        self.buffered = 0;
        self.configured_by_driver = true;
        Ok(())
    }
}

impl<S> FrameTransport for SerialTransport<S>
where
    S: Read + Write + ErrorType + Send + Sync + 'static,
    <S as ErrorType>::Error: embedded_io::Error + fmt::Debug + 'static,
{
    fn start(&mut self, readout: &Sen0682ReadoutConfig) -> CuResult<()> {
        self.configure_streaming(readout)
    }

    fn read_frame(&mut self, out: &mut [u8]) -> CuResult<Option<usize>> {
        if let Some(frame) = self.try_extract_frame(out)? {
            return Ok(Some(frame));
        }

        if self.buffered == self.buffer.len() {
            return Err(CuError::from(
                "sen0682 serial framing buffer saturated before a valid frame was found",
            ));
        }

        match self.serial.read(&mut self.buffer[self.buffered..]) {
            Ok(0) => Ok(None),
            Ok(n) => {
                self.buffered += n;
                self.try_extract_frame(out)
            }
            Err(err) if is_idle_io_error(&err) => Ok(None),
            Err(err) => Err(CuError::from(format!(
                "sen0682 serial read failed: {err:?}"
            ))),
        }
    }

    fn stop(&mut self) -> CuResult<()> {
        if self.configured_by_driver {
            let _ = self.send_command("AT+STREAM_CONTROL=0\n");
            self.configured_by_driver = false;
        }
        Ok(())
    }
}

fn write_all<S>(serial: &mut S, bytes: &[u8]) -> CuResult<()>
where
    S: Write + ErrorType,
    <S as ErrorType>::Error: fmt::Debug,
{
    let mut written = 0;
    while written < bytes.len() {
        let n = serial
            .write(&bytes[written..])
            .map_err(|err| CuError::from(format!("sen0682 command write failed: {err:?}")))?;
        if n == 0 {
            return Err(CuError::from(
                "sen0682 command write returned zero bytes before completion",
            ));
        }
        written += n;
    }
    Ok(())
}

fn contains_ascii_token(haystack: &[u8], token: &[u8]) -> bool {
    haystack.windows(token.len()).any(|window| window == token)
}

fn is_idle_io_error<E>(err: &E) -> bool
where
    E: embedded_io::Error,
{
    matches!(err.kind(), ErrorKind::TimedOut | ErrorKind::Interrupted)
}

/// Address-scoped I2C integration point for board bundles.
///
/// The driver expects the resource layer to own addressing and any burst-read
/// quirks so the task config stays about lidar behavior instead of hardware
/// wiring details.
pub trait Sen0682I2cBus: Send + Sync + 'static {
    type Error: fmt::Debug + Send + 'static;

    fn configure_stream(&mut self, readout: &Sen0682ReadoutConfig) -> Result<(), Self::Error>;
    fn read_frame(&mut self, out: &mut [u8]) -> Result<Option<usize>, Self::Error>;
    fn stop_stream(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

struct I2cTransport<B> {
    bus: B,
}

impl<B> I2cTransport<B> {
    fn new(bus: B) -> Self {
        Self { bus }
    }
}

impl<B> FrameTransport for I2cTransport<B>
where
    B: Sen0682I2cBus,
{
    fn start(&mut self, readout: &Sen0682ReadoutConfig) -> CuResult<()> {
        self.bus
            .configure_stream(readout)
            .map_err(|err| CuError::from(format!("sen0682 i2c configure failed: {err:?}")))
    }

    fn read_frame(&mut self, out: &mut [u8]) -> CuResult<Option<usize>> {
        self.bus
            .read_frame(out)
            .map_err(|err| CuError::from(format!("sen0682 i2c read failed: {err:?}")))
    }

    fn stop(&mut self) -> CuResult<()> {
        self.bus
            .stop_stream()
            .map_err(|err| CuError::from(format!("sen0682 i2c stop failed: {err:?}")))
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SerialBinding {
    Serial,
}

pub struct Sen0682SerialResourcesT<S> {
    pub serial: Owned<S>,
}

#[cfg(feature = "std")]
pub type Sen0682SerialResources = Sen0682SerialResourcesT<LinuxSerialPort>;

impl<'r, S: 'static + Send + Sync> ResourceBindings<'r> for Sen0682SerialResourcesT<S> {
    type Binding = SerialBinding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| {
            CuError::from("Sen0682SerialSourceTask requires a `serial` resource mapping")
        })?;
        let path = mapping.get(SerialBinding::Serial).ok_or_else(|| {
            CuError::from(
                "Sen0682SerialSourceTask resources must include `serial: <bundle.resource>`",
            )
        })?;
        let serial = manager
            .take::<S>(path.typed())
            .map_err(|e| e.add_cause("Failed to fetch SEN0682 serial resource"))?;
        Ok(Self { serial })
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum I2cBinding {
    I2c,
}

pub struct Sen0682I2cResourcesT<B> {
    pub i2c: Owned<B>,
}

impl<'r, B: 'static + Send + Sync> ResourceBindings<'r> for Sen0682I2cResourcesT<B> {
    type Binding = I2cBinding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| {
            CuError::from("Sen0682I2cSourceTask requires an `i2c` resource mapping")
        })?;
        let path = mapping.get(I2cBinding::I2c).ok_or_else(|| {
            CuError::from("Sen0682I2cSourceTask resources must include `i2c: <bundle.resource>`")
        })?;
        let i2c = manager
            .take::<B>(path.typed())
            .map_err(|e| e.add_cause("Failed to fetch SEN0682 I2C resource"))?;
        Ok(Self { i2c })
    }
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct Sen0682SerialSourceTask<S> {
    #[reflect(ignore)]
    core: Sen0682SourceCore<SerialTransport<S>>,
    configure_device: bool,
    row_id: u8,
    start_column: u8,
    end_column: u8,
    min_range_m: f32,
}

#[cfg(feature = "std")]
pub type Sen0682SerialSource = Sen0682SerialSourceTask<LinuxSerialPort>;

impl<S: 'static> TypePath for Sen0682SerialSourceTask<S> {
    fn type_path() -> &'static str {
        "cu_sen0682::Sen0682SerialSourceTask"
    }

    fn short_type_path() -> &'static str {
        "Sen0682SerialSourceTask"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Sen0682SerialSourceTask")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_sen0682")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_sen0682")
    }
}

impl<S> fmt::Debug for Sen0682SerialSourceTask<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Sen0682SerialSourceTask")
            .field("configure_device", &self.configure_device)
            .field("row_id", &self.row_id)
            .field("start_column", &self.start_column)
            .field("end_column", &self.end_column)
            .field("min_range_m", &self.min_range_m)
            .finish()
    }
}

impl<S> Freezable for Sen0682SerialSourceTask<S> {}

impl<S> Sen0682SerialSourceTask<S> {
    fn readout_config(&self) -> Sen0682ReadoutConfig {
        Sen0682ReadoutConfig {
            configure_device: self.configure_device,
            row_id: self.row_id,
            start_column: self.start_column,
            end_column: self.end_column,
        }
    }
}

impl<S> CuSrcTask for Sen0682SerialSourceTask<S>
where
    S: Read + Write + ErrorType + Send + Sync + 'static,
    <S as ErrorType>::Error: embedded_io::Error + fmt::Debug + 'static,
{
    type Resources<'r> = Sen0682SerialResourcesT<S>;
    type Output<'m> = output_msg!(PointCloudSoa<MAX_POINTS>);

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let readout = Sen0682ReadoutConfig::from_component_config(config)?;
        let min_range_m = cfg_f32(config, "min_range_m", DEFAULT_MIN_RANGE_M)?;

        Ok(Self {
            core: Sen0682SourceCore::new(SerialTransport::new(resources.serial.0), min_range_m),
            configure_device: readout.configure_device,
            row_id: readout.row_id,
            start_column: readout.start_column,
            end_column: readout.end_column,
            min_range_m,
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.core.start(&self.readout_config())
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.core.process(ctx, output)
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.core.stop()
    }
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct Sen0682I2cSourceTask<B> {
    #[reflect(ignore)]
    core: Sen0682SourceCore<I2cTransport<B>>,
    configure_device: bool,
    row_id: u8,
    start_column: u8,
    end_column: u8,
    min_range_m: f32,
}

impl<B: 'static> TypePath for Sen0682I2cSourceTask<B> {
    fn type_path() -> &'static str {
        "cu_sen0682::Sen0682I2cSourceTask"
    }

    fn short_type_path() -> &'static str {
        "Sen0682I2cSourceTask"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Sen0682I2cSourceTask")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_sen0682")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_sen0682")
    }
}

impl<B> fmt::Debug for Sen0682I2cSourceTask<B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Sen0682I2cSourceTask")
            .field("configure_device", &self.configure_device)
            .field("row_id", &self.row_id)
            .field("start_column", &self.start_column)
            .field("end_column", &self.end_column)
            .field("min_range_m", &self.min_range_m)
            .finish()
    }
}

impl<B> Freezable for Sen0682I2cSourceTask<B> {}

impl<B> Sen0682I2cSourceTask<B> {
    fn readout_config(&self) -> Sen0682ReadoutConfig {
        Sen0682ReadoutConfig {
            configure_device: self.configure_device,
            row_id: self.row_id,
            start_column: self.start_column,
            end_column: self.end_column,
        }
    }
}

impl<B> CuSrcTask for Sen0682I2cSourceTask<B>
where
    B: Sen0682I2cBus,
{
    type Resources<'r> = Sen0682I2cResourcesT<B>;
    type Output<'m> = output_msg!(PointCloudSoa<MAX_POINTS>);

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let readout = Sen0682ReadoutConfig::from_component_config(config)?;
        let min_range_m = cfg_f32(config, "min_range_m", DEFAULT_MIN_RANGE_M)?;

        Ok(Self {
            core: Sen0682SourceCore::new(I2cTransport::new(resources.i2c.0), min_range_m),
            configure_device: readout.configure_device,
            row_id: readout.row_id,
            start_column: readout.start_column,
            end_column: readout.end_column,
            min_range_m,
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.core.start(&self.readout_config())
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.core.process(ctx, output)
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.core.stop()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::collections::VecDeque;
    use alloc::vec::Vec;

    fn build_test_frame() -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"wyld");
        bytes.extend_from_slice(&2u16.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes());
        bytes.extend_from_slice(&16u32.to_le_bytes());
        bytes.extend_from_slice(&16u16.to_le_bytes());
        bytes.extend_from_slice(&3u16.to_le_bytes());
        bytes.extend_from_slice(&7u32.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(&1000i16.to_le_bytes());
        bytes.extend_from_slice(&0i16.to_le_bytes());
        bytes.extend_from_slice(&2000i16.to_le_bytes());
        bytes.extend_from_slice(&123u16.to_le_bytes());
        bytes.extend_from_slice(&1500i16.to_le_bytes());
        bytes.extend_from_slice(&0i16.to_le_bytes());
        bytes.extend_from_slice(&2500i16.to_le_bytes());
        bytes.extend_from_slice(&456u16.to_le_bytes());
        bytes
    }

    #[derive(Default)]
    struct FakeSerial {
        reads: VecDeque<Result<Vec<u8>, std::io::Error>>,
        writes: Vec<Vec<u8>>,
    }

    impl FakeSerial {
        fn with_reads(reads: impl IntoIterator<Item = Result<Vec<u8>, std::io::Error>>) -> Self {
            Self {
                reads: reads.into_iter().collect(),
                writes: Vec::new(),
            }
        }
    }

    impl ErrorType for FakeSerial {
        type Error = std::io::Error;
    }

    impl Read for FakeSerial {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let Some(next) = self.reads.pop_front() else {
                return Err(std::io::Error::from(std::io::ErrorKind::TimedOut));
            };
            match next {
                Ok(chunk) => {
                    let len = chunk.len().min(buf.len());
                    buf[..len].copy_from_slice(&chunk[..len]);
                    Ok(len)
                }
                Err(err) => Err(err),
            }
        }
    }

    impl Write for FakeSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.writes.push(buf.to_vec());
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn serial_transport_resynchronizes_after_text_noise() {
        let frame = build_test_frame();
        let serial = FakeSerial::with_reads([
            Ok(b"OK\r\n".to_vec()),
            Ok(frame[..11].to_vec()),
            Ok(frame[11..].to_vec()),
        ]);
        let mut transport = SerialTransport::new(serial);
        let mut out = [0u8; protocol::MAX_FRAME_BYTES];

        let mut extracted = None;
        for _ in 0..3 {
            extracted = transport.read_frame(&mut out).expect("read should succeed");
            if extracted.is_some() {
                break;
            }
        }

        assert!(extracted.is_some());
        assert_eq!(extracted.unwrap(), frame.len());
        assert_eq!(&out[..frame.len()], frame.as_slice());
    }
}
