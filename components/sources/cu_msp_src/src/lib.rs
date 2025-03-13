use cu29::prelude::*;
use cu_msp_lib::structs::MspResponse;
#[cfg(unix)]
use cu_msp_lib::MspParser;
#[cfg(unix)]
use serialport::SerialPort;
#[cfg(unix)]
use serialport::TTYPort;
use smallvec::SmallVec;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};
#[cfg(unix)]
use std::io::Read;

const MAX_MSG_SIZE: usize = 16;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MspResponseBatch(pub SmallVec<[MspResponse; MAX_MSG_SIZE]>);

impl Encode for MspResponseBatch {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.as_slice(), encoder)
    }
}

impl Decode<()> for MspResponseBatch {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        // allocations are ok in decode
        let v = <Vec<MspResponse> as Decode<()>>::decode(decoder)?;
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
    #[cfg(unix)]
    serial: TTYPort,
    #[cfg(unix)]
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
        #[cfg(unix)]
        let port: String = config
            .and_then(|config| config.get::<String>("device"))
            .unwrap_or("/dev/ttyUSB0".to_string());
        #[cfg(unix)]
        let baudrate = config
            .and_then(|config| config.get::<u32>("baudrate"))
            .unwrap_or(115200);

        #[cfg(unix)]
        let builder = serialport::new(port, baudrate);
        #[cfg(unix)]
        let mut serial = TTYPort::open(&builder).unwrap();
        #[cfg(unix)]
        serial.set_exclusive(false).unwrap();
        #[cfg(unix)]
        serial
            .set_timeout(std::time::Duration::from_millis(100))
            .unwrap();

        #[cfg(unix)]
        let parser = MspParser::new();
        Ok(Self {
            #[cfg(unix)]
            serial,
            #[cfg(unix)]
            parser,
            buffer: SmallVec::new(),
        })
    }

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        self.buffer.clear();
        self.buffer.resize(512, 0);
        #[cfg(unix)]
        let n = self.serial.read(&mut self.buffer);
        #[cfg(unix)]
        if let Err(e) = &n {
            debug!("read error: {}", e.to_string());
            return Ok(());
        }
        #[cfg(unix)]
        let n = n.unwrap();
        #[cfg(unix)]
        self.buffer.truncate(n);

        #[cfg(unix)]
        let mut batch = MspResponseBatch::default();
        #[cfg(unix)]
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
        #[cfg(unix)]
        output.set_payload(batch);

        #[cfg(windows)]
        output.set_payload(MspResponseBatch::default());
        Ok(())
    }
}

#[cfg(unix)]
#[cfg(test)]
mod tests {
    use serialport::SerialPort;
    use serialport::TTYPort;
    use std::io::Read;
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
