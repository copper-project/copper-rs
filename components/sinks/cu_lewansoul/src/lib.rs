use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSinkTask, Freezable};
use cu29::{input_msg, CuError, CuResult};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{self, Read, Write};
use std::time::Duration;
use uom::si::angle::{degree, radian};
use uom::si::f32::Angle;

#[allow(dead_code)]
mod servo {
    // From "lx-16a LewanSoul Bus Servo Communication Protocol.pdf"
    pub const SERVO_MOVE_TIME_WRITE: u8 = 1; // 7 bytes
    pub const SERVO_MOVE_TIME_READ: u8 = 2; // 3 bytes
    pub const SERVO_MOVE_TIME_WAIT_WRITE: u8 = 7; // 7 bytes
    pub const SERVO_MOVE_TIME_WAIT_READ: u8 = 8; // 3 bytes
    pub const SERVO_MOVE_START: u8 = 11; // 3 bytes
    pub const SERVO_MOVE_STOP: u8 = 12; // 3 bytes
    pub const SERVO_ID_WRITE: u8 = 13; // 4 bytes
    pub const SERVO_ID_READ: u8 = 14; // 3 bytes
    pub const SERVO_ANGLE_OFFSET_ADJUST: u8 = 17; // 4 bytes
    pub const SERVO_ANGLE_OFFSET_WRITE: u8 = 18; // 3 bytes
    pub const SERVO_ANGLE_OFFSET_READ: u8 = 19; // 3 bytes
    pub const SERVO_ANGLE_LIMIT_WRITE: u8 = 20; // 7 bytes
    pub const SERVO_ANGLE_LIMIT_READ: u8 = 21; // 3 bytes
    pub const SERVO_VIN_LIMIT_WRITE: u8 = 22; // 7 bytes
    pub const SERVO_VIN_LIMIT_READ: u8 = 23; // 3 bytes
    pub const SERVO_TEMP_MAX_LIMIT_WRITE: u8 = 24; // 4 bytes
    pub const SERVO_TEMP_MAX_LIMIT_READ: u8 = 25; // 3 bytes
    pub const SERVO_TEMP_READ: u8 = 0x1A; // 26 -> 3 bytes
    pub const SERVO_VIN_READ: u8 = 0x1B; // 27 -> 3 bytes
    pub const SERVO_POS_READ: u8 = 28; // 3 bytes
    pub const SERVO_OR_MOTOR_MODE_WRITE: u8 = 29; // 7 bytes
    pub const SERVO_OR_MOTOR_MODE_READ: u8 = 30; // 3 bytes
    pub const SERVO_LOAD_OR_UNLOAD_WRITE: u8 = 31; // 4 bytes
    pub const SERVO_LOAD_OR_UNLOAD_READ: u8 = 32; // 3 bytes
    pub const SERVO_LED_CTRL_WRITE: u8 = 33; // 4 bytes
    pub const SERVO_LED_CTRL_READ: u8 = 34; // 3 bytes
    pub const SERVO_LED_ERROR_WRITE: u8 = 35; // 4 bytes
    pub const SERVO_LED_ERROR_READ: u8 = 36; // 3 bytes
}

const SERIAL_SPEED: u32 = 115200; // only this speed is supported by the servos
const TIMEOUT: Duration = Duration::from_secs(1); // TODO: add that as a parameter in the config

const MAX_SERVOS: usize = 8; // in theory it could be higher. Revisit if needed.

/// Compute the checksum for the given data.
/// The spec is "Checksum:The calculation method is as follows:
// Checksum=~(ID+ Length+Cmd+ Prm1+...PrmN)If the numbers in the
// brackets are calculated and exceeded 255,Then take the lowest one byte, "~"
// means Negation."
#[inline]
fn compute_checksum(data: impl Iterator<Item = u8>) -> u8 {
    let mut checksum: u8 = 0;
    for byte in data {
        checksum = checksum.wrapping_add(byte);
    }
    !checksum
}

// angle in degrees, returns position in 0.24 degrees
#[inline]
#[allow(dead_code)]
fn angle_to_position(angle: Angle) -> i16 {
    let angle = angle.get::<degree>();
    (angle * 1000.0 / 240.0) as i16
}

/// This is a driver for the LewanSoul LX-16A, LX-225 etc.  Serial Bus Servos.
pub struct Lewansoul {
    port: Box<dyn SerialPort>,
    #[allow(dead_code)]
    ids: [u8; 8], // TODO: WIP
}

impl Lewansoul {
    fn send_packet(&mut self, id: u8, command: u8, data: &[u8]) -> io::Result<()> {
        let mut packet = vec![0x55, 0x55, id, data.len() as u8 + 3, command];
        packet.extend(data.iter());
        let checksum = compute_checksum(packet[2..].iter().cloned());
        packet.push(checksum);

        // println!("Packet: {:02x?}", packet);
        self.port.write_all(&packet)?;
        Ok(())
    }

    /// This should only be use at HW setup. I leave it there for you to make a tool around it if needed.
    /// See the unit test how you can reuse independently those functions
    #[allow(dead_code)]
    fn reassign_servo_id(&mut self, id: u8, new_id: u8) -> io::Result<()> {
        self.send_packet(id, servo::SERVO_ID_WRITE, &[new_id])?;
        self.read_response()?;
        Ok(())
    }

    #[allow(dead_code)]
    fn read_current_position(&mut self, id: u8) -> io::Result<f32> {
        self.send_packet(id, servo::SERVO_POS_READ, &[])?;
        let response = self.read_response()?;
        Ok((i16::from_le_bytes([response.2[0], response.2[1]])) as f32 * 240.0 / 1000.0)
    }

    #[allow(dead_code)]
    fn read_present_voltage(&mut self, id: u8) -> io::Result<f32> {
        self.send_packet(id, servo::SERVO_VIN_READ, &[])?;
        let response = self.read_response()?;
        Ok(u16::from_le_bytes([response.2[0], response.2[1]]) as f32 / 1000.0)
    }

    #[allow(dead_code)]
    fn read_temperature(&mut self, id: u8) -> io::Result<u8> {
        self.send_packet(id, servo::SERVO_TEMP_READ, &[])?;
        let response = self.read_response()?;
        Ok(response.2[0])
    }

    #[allow(dead_code)]
    fn read_angle_limits(&mut self, id: u8) -> io::Result<(f32, f32)> {
        self.send_packet(id, servo::SERVO_ANGLE_LIMIT_READ, &[])?;
        let response = self.read_response()?;
        Ok((
            (i16::from_le_bytes([response.2[0], response.2[1]]) as f32) * 240.0 / 1000.0,
            (i16::from_le_bytes([response.2[2], response.2[3]]) as f32) * 240.0 / 1000.0,
        ))
    }

    #[allow(dead_code)]
    fn ping(&mut self, id: u8) -> CuResult<()> {
        self.send_packet(id, servo::SERVO_ID_READ, &[])
            .map_err(|e| CuError::new_with_cause("IO Error trying to write to the SBUS", &e))?;
        let response = self.read_response().map_err(|e| {
            CuError::new_with_cause("IO Error trying to read the ping response from SBUS", &e)
        })?;

        if response.2[0] == id {
            Ok(())
        } else {
            Err(format!(
                "The servo ID {} did not respond to ping got {} as ID instead.",
                id, response.2[0]
            )
            .into())
        }
    }

    fn read_response(&mut self) -> io::Result<(u8, u8, Vec<u8>)> {
        let mut header = [0; 5];
        self.port.read_exact(&mut header)?;
        if header[0] != 0x55 || header[1] != 0x55 {
            return Err(io::Error::other("Invalid header"));
        }
        let id = header[2];
        let length = header[3];
        let command = header[4];
        let mut remaining = vec![0; length as usize - 2]; // -2 for length itself already read + command already read
        self.port.read_exact(&mut remaining)?;
        let checksum = compute_checksum(
            &mut header[2..]
                .iter()
                .chain(remaining[..remaining.len() - 1].iter())
                .cloned(),
        );
        if checksum != *remaining.last().unwrap() {
            return Err(io::Error::other("Invalid checksum"));
        }
        Ok((id, command, remaining[..remaining.len() - 1].to_vec()))
    }
}

impl Freezable for Lewansoul {
    // This driver is stateless as the IDs are recreate at new time, we keep the default implementation.
}

#[derive(Debug, Clone, Default)]
pub struct ServoPositionsPayload {
    pub positions: [Angle; MAX_SERVOS],
}

impl Encode for ServoPositionsPayload {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let angles: [f32; MAX_SERVOS] = self.positions.map(|a| a.value);
        angles.encode(encoder)
    }
}

impl Decode<()> for ServoPositionsPayload {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let angles: [f32; 8] = Decode::decode(decoder)?;
        let positions: [Angle; 8] = angles.map(Angle::new::<radian>);
        Ok(ServoPositionsPayload { positions })
    }
}

impl<'cl> CuSinkTask<'cl> for Lewansoul {
    type Input = input_msg!('cl, ServoPositionsPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let ComponentConfig(kv) =
            config.ok_or("RPGpio needs a config, None was passed as ComponentConfig")?;

        let serial_dev: String = kv
            .get("serial_dev")
            .expect(
                "Lewansoul expects a serial_dev config entry pointing to the serial device to use.",
            )
            .clone()
            .into();

        let mut ids = [0u8; 8];
        for (i, id) in ids.iter_mut().enumerate() {
            let servo = kv.get(format!("servo{i}").as_str());
            if servo.is_none() {
                if i == 0 {
                    return Err(
                        "You need to specify at least one servo ID to address (as \"servo0\")"
                            .into(),
                    );
                }
                break;
            }
            *id = servo.unwrap().clone().into();
        }

        let port = serialport::new(serial_dev.as_str(), SERIAL_SPEED)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(TIMEOUT)
            .open()
            .map_err(|e| format!("Error opening serial port: {e:?}"))?;

        Ok(Lewansoul { port, ids })
    }

    fn process(&mut self, _clock: &RobotClock, _input: Self::Input) -> CuResult<()> {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore]
    fn end2end_2_servos() {
        let mut config = ComponentConfig::default();
        config
            .0
            .insert("serial_dev".to_string(), "/dev/ttyACM0".to_string().into());

        config.0.insert("servo0".to_string(), 1.into());
        config.0.insert("servo1".to_string(), 2.into());

        let mut lewansoul = Lewansoul::new(Some(&config)).unwrap();
        let _position = lewansoul.read_current_position(1).unwrap();

        let _angle_limits = lewansoul.read_angle_limits(1).unwrap();
    }
}
