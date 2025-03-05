use cu29::prelude::*;
use cu_msp_lib::structs::MspResponse;
use cu_msp_lib::MspParser;
use serialport::{SerialPort, TTYPort};
use smallvec::SmallVec;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};
use std::io::Read;

const MAX_MSG_SIZE: usize = 16;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MspResponseBatch(pub SmallVec<[MspResponse; MAX_MSG_SIZE]>);

impl Encode for MspResponseBatch {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.as_slice(), encoder)
    }
}

impl Decode for MspResponseBatch {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        // allocations are ok in decode
        let v = <Vec<MspResponse> as Decode>::decode(decoder)?;
        Ok(Self(v.into()))
    }
}

impl MspResponseBatch {
    pub fn new() -> Self {
        Self(SmallVec::new())
    }

    pub fn push(&mut self, resp: MspResponse) {
        let MspResponseBatch(ref mut vec) = self;
        vec.push(resp);
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

pub struct MSPSrc {
    serial: TTYPort,
    parser: MspParser,
    buffer: SmallVec<[u8; 512]>,
}

impl Freezable for MSPSrc {}

impl<'cl> CuSrcTask<'cl> for MSPSrc {
    type Output = output_msg!('cl, MspResponseBatch);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        if config.is_none() {
            return Err("No config provided".into());
        }
        let port: String = config
            .and_then(|config| config.get::<String>("device"))
            .unwrap_or("/dev/ttyUSB0".to_string());
        let baudrate = config
            .and_then(|config| config.get::<u32>("baudrate"))
            .unwrap_or(115200);

        let builder = serialport::new(port, baudrate);
        let mut serial = TTYPort::open(&builder).unwrap();
        serial.set_exclusive(false).unwrap();
        serial
            .set_timeout(std::time::Duration::from_millis(100))
            .unwrap();

        let parser = MspParser::new();
        Ok(Self {
            serial,
            parser,
            buffer: SmallVec::new(),
        })
    }

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        self.buffer.clear();
        self.buffer.resize(512, 0);
        let n = self.serial.read(&mut self.buffer);
        if let Err(e) = &n {
            debug!("read error: {}", e.to_string());
            return Ok(());
        }
        let n = n.unwrap();
        self.buffer.truncate(n);

        let mut batch = MspResponseBatch::default();
        if n > 0 {
            for &b in &self.buffer {
                let maybe_packet = self.parser.parse(b);
                if let Ok(Some(packet)) = maybe_packet {
                    let response = MspResponse::from(packet);
                    debug!("Response: {}", &response);
                    batch.push(response);
                    if batch.len() >= MAX_MSG_SIZE {
                        debug!("batch full, sending");
                        break;
                    }
                }
            }
        }
        output.set_payload(batch);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    #[ignore]
    fn test_concurrent_r_w_serial() {
        let port = "/dev/ttyS4";
        let baudrate = 115200;
        let builder = serialport::new(port, baudrate);
        let mut r = TTYPort::open(&builder).unwrap();
        r.set_exclusive(false).unwrap();
        r.set_timeout(std::time::Duration::from_millis(10)).unwrap();

        let mut w = TTYPort::open(&builder).unwrap();
        w.set_exclusive(false).unwrap();

        // test loopback
        let data = [0x01, 0x02, 0x03, 0x04];
        let nb = w.write(&data).unwrap();
        assert_eq!(nb, data.len());
        w.flush().unwrap();

        let mut buf = [0; 4];
        let nb = r.read(&mut buf).unwrap();
        assert_eq!(nb, data.len());
        assert_eq!(data, buf);
    }
}
