use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu_msp_lib::structs::MspRequest;
use cu_msp_lib::MspPacket;
use serde::{Deserialize, Serialize};
#[cfg(unix)]
use serialport::SerialPort;
#[cfg(unix)]
use serialport::TTYPort;
#[cfg(unix)]
use std::io::Write;

use smallvec::SmallVec;

const MAX_MSG_SIZE: usize = 8;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MspRequestBatch(pub SmallVec<[MspRequest; MAX_MSG_SIZE]>);

impl MspRequestBatch {
    pub fn new() -> Self {
        Self(SmallVec::new())
    }

    pub fn push(&mut self, req: MspRequest) {
        let MspRequestBatch(ref mut vec) = self;
        vec.push(req);
    }
}

impl Encode for MspRequestBatch {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.as_slice(), encoder)
    }
}

impl Decode<()> for MspRequestBatch {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let v = <Vec<MspRequest> as Decode<()>>::decode(decoder)?;
        Ok(Self(v.into()))
    }
}

pub struct MSPSink {
    #[cfg(unix)]
    serial: TTYPort,
}

impl Freezable for MSPSink {}

impl<'cl> CuSinkTask<'cl> for MSPSink {
    type Input = input_msg!('cl, MspRequestBatch);

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
            .set_timeout(std::time::Duration::from_millis(10))
            .unwrap();

        Ok(Self {
            #[cfg(unix)]
            serial,
        })
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        if let Some(batch) = input.payload() {
            let MspRequestBatch(ref batch) = batch;
            for req in batch {
                debug!("Sending request {}", req);
                let msp_packet: MspPacket = req.into();
                let size = msp_packet.packet_size_bytes();
                let mut buffer = SmallVec::<[u8; 256]>::new();
                buffer.resize(size, 0);
                msp_packet.serialize(&mut buffer).map_err(|e| {
                    debug!("Error Serializing packet {}", e.to_string());
                    CuError::new_with_cause("failed to serialize msp packet", e)
                })?;
                #[cfg(unix)]
                self.serial.write_all(buffer.as_slice()).map_err(|e| {
                    debug!("Error writing to serial port {}", e.to_string());
                    CuError::new_with_cause("failed to write to serial port", e)
                })?;
            }
        }
        Ok(())
    }
}
