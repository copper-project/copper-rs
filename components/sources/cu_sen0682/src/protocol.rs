use core::fmt;

use cu_sensor_payloads::PointCloudSoa;
use cu29::clock::CuTime;
use cu29::units::si::f32::{Length, Ratio};
use cu29::units::si::length::meter;
use cu29::units::si::ratio::ratio;

pub const TAG: [u8; 4] = *b"wyld";
pub const HEADER_BYTES: usize = 32;
pub const BYTES_PER_POINT: usize = 8;
pub const MAX_WIDTH: usize = 64;
pub const MAX_HEIGHT: usize = 8;
pub const MAX_POINTS: usize = MAX_WIDTH * MAX_HEIGHT;
pub const MAX_FRAME_BYTES: usize = HEADER_BYTES + MAX_POINTS * BYTES_PER_POINT;
pub const POINT_CLOUD_DATA_TYPE: u16 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameHeader {
    pub width: u16,
    pub height: u16,
    pub frame_size: u32,
    pub line_size: u16,
    pub data_type: u16,
    pub frame_idx: u32,
    pub device_index: u8,
}

impl FrameHeader {
    pub fn point_count(self) -> usize {
        self.width as usize * self.height as usize
    }

    pub fn total_bytes(self) -> usize {
        HEADER_BYTES + self.frame_size as usize
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameStats {
    pub frame_idx: u32,
    pub width: u16,
    pub height: u16,
    pub device_index: u8,
    pub raw_points: usize,
    pub valid_points: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    MissingTag,
    ShortHeader,
    IncompleteFrame { expected: usize, actual: usize },
    InvalidWidth(u16),
    InvalidHeight(u16),
    TooManyPoints(usize),
    UnexpectedDataType(u16),
    InvalidFrameSize { expected: usize, actual: u32 },
    InvalidLineSize { expected: usize, actual: u16 },
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingTag => write!(f, "frame tag is not \"wyld\""),
            Self::ShortHeader => write!(f, "frame header is truncated"),
            Self::IncompleteFrame { expected, actual } => {
                write!(
                    f,
                    "frame truncated: expected {expected} bytes, got {actual}"
                )
            }
            Self::InvalidWidth(width) => write!(f, "invalid frame width {width}"),
            Self::InvalidHeight(height) => write!(f, "invalid frame height {height}"),
            Self::TooManyPoints(points) => write!(f, "frame has {points} points, exceeds limit"),
            Self::UnexpectedDataType(data_type) => {
                write!(
                    f,
                    "unexpected frame data type {data_type}, expected point cloud"
                )
            }
            Self::InvalidFrameSize { expected, actual } => {
                write!(
                    f,
                    "invalid frame payload size {actual}, expected {expected} bytes"
                )
            }
            Self::InvalidLineSize { expected, actual } => {
                write!(f, "invalid line size {actual}, expected {expected} bytes")
            }
        }
    }
}

impl core::error::Error for FrameError {}

pub fn find_tag(bytes: &[u8]) -> Option<usize> {
    bytes.windows(TAG.len()).position(|window| window == TAG)
}

pub fn frame_total_bytes_from_prefix(prefix: &[u8]) -> Result<Option<usize>, FrameError> {
    if prefix.len() < HEADER_BYTES {
        return Ok(None);
    }

    let header = parse_frame_header(prefix)?;
    Ok(Some(header.total_bytes()))
}

pub fn decode_frame_into<const N: usize>(
    frame: &[u8],
    tov: CuTime,
    min_range_m: f32,
    out: &mut PointCloudSoa<N>,
) -> Result<FrameStats, FrameError> {
    let header = parse_frame(frame)?;
    let raw_points = header.point_count();
    if raw_points > N {
        return Err(FrameError::TooManyPoints(raw_points));
    }

    out.len = 0;
    let bounded_min_range = min_range_m.max(0.0);
    let min_range_sq = bounded_min_range * bounded_min_range;
    let payload = &frame[HEADER_BYTES..header.total_bytes()];

    for idx in 0..raw_points {
        let base = idx * BYTES_PER_POINT;
        let x_m = millimeters_to_meters(read_i16_le(&payload[base..base + 2]));
        let y_m = millimeters_to_meters(read_i16_le(&payload[base + 2..base + 4]));
        let z_m = millimeters_to_meters(read_i16_le(&payload[base + 4..base + 6]));
        let intensity = read_u16_le(&payload[base + 6..base + 8]) as f32 / u16::MAX as f32;

        let range_sq = x_m * x_m + y_m * y_m + z_m * z_m;
        if range_sq < min_range_sq {
            continue;
        }

        let write_idx = out.len;
        out.tov[write_idx] = tov;
        out.x[write_idx] = Length::new::<meter>(x_m);
        out.y[write_idx] = Length::new::<meter>(y_m);
        out.z[write_idx] = Length::new::<meter>(z_m);
        out.i[write_idx] = Ratio::new::<ratio>(intensity);
        out.return_order[write_idx] = 0;
        out.len += 1;
    }

    Ok(FrameStats {
        frame_idx: header.frame_idx,
        width: header.width,
        height: header.height,
        device_index: header.device_index,
        raw_points,
        valid_points: out.len,
    })
}

fn parse_frame(frame: &[u8]) -> Result<FrameHeader, FrameError> {
    let header = parse_frame_header(frame)?;
    let expected = header.total_bytes();
    if frame.len() < expected {
        return Err(FrameError::IncompleteFrame {
            expected,
            actual: frame.len(),
        });
    }
    Ok(header)
}

fn parse_frame_header(bytes: &[u8]) -> Result<FrameHeader, FrameError> {
    if bytes.len() < HEADER_BYTES {
        return Err(FrameError::ShortHeader);
    }
    if bytes[..TAG.len()] != TAG {
        return Err(FrameError::MissingTag);
    }

    let width = read_u16_le(&bytes[4..6]);
    let height = read_u16_le(&bytes[6..8]);
    let frame_size = read_u32_le(&bytes[8..12]);
    let line_size = read_u16_le(&bytes[12..14]);
    let data_type = read_u16_le(&bytes[14..16]);
    let frame_idx = read_u32_le(&bytes[16..20]);
    let device_index = bytes[20];

    if width == 0 || width as usize > MAX_WIDTH {
        return Err(FrameError::InvalidWidth(width));
    }
    if height == 0 || height as usize > MAX_HEIGHT {
        return Err(FrameError::InvalidHeight(height));
    }
    if data_type != POINT_CLOUD_DATA_TYPE {
        return Err(FrameError::UnexpectedDataType(data_type));
    }

    let point_count = width as usize * height as usize;
    if point_count > MAX_POINTS {
        return Err(FrameError::TooManyPoints(point_count));
    }

    let expected_frame_size = point_count * BYTES_PER_POINT;
    if frame_size as usize != expected_frame_size {
        return Err(FrameError::InvalidFrameSize {
            expected: expected_frame_size,
            actual: frame_size,
        });
    }

    let expected_line_size = width as usize * BYTES_PER_POINT;
    if line_size as usize != expected_line_size {
        return Err(FrameError::InvalidLineSize {
            expected: expected_line_size,
            actual: line_size,
        });
    }

    Ok(FrameHeader {
        width,
        height,
        frame_size,
        line_size,
        data_type,
        frame_idx,
        device_index,
    })
}

fn read_u16_le(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}

fn read_i16_le(bytes: &[u8]) -> i16 {
    i16::from_le_bytes([bytes[0], bytes[1]])
}

fn read_u32_le(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

fn millimeters_to_meters(value_mm: i16) -> f32 {
    value_mm as f32 * 0.001
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_frame(points: &[(i16, i16, i16, u16)], width: u16, height: u16) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(HEADER_BYTES + points.len() * BYTES_PER_POINT);
        bytes.extend_from_slice(&TAG);
        bytes.extend_from_slice(&width.to_le_bytes());
        bytes.extend_from_slice(&height.to_le_bytes());
        bytes.extend_from_slice(&((points.len() * BYTES_PER_POINT) as u32).to_le_bytes());
        bytes.extend_from_slice(&((width as usize * BYTES_PER_POINT) as u16).to_le_bytes());
        bytes.extend_from_slice(&POINT_CLOUD_DATA_TYPE.to_le_bytes());
        bytes.extend_from_slice(&42u32.to_le_bytes());
        bytes.extend_from_slice(&0xA5A5_0001_u32.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());

        for &(x, y, z, i) in points {
            bytes.extend_from_slice(&x.to_le_bytes());
            bytes.extend_from_slice(&y.to_le_bytes());
            bytes.extend_from_slice(&z.to_le_bytes());
            bytes.extend_from_slice(&i.to_le_bytes());
        }

        bytes
    }

    #[test]
    fn decodes_full_frame_header() {
        let frame = build_frame(&[(1000, 0, 2000, 100)], 1, 1);
        let header = parse_frame(&frame).expect("header should parse");
        assert_eq!(header.width, 1);
        assert_eq!(header.height, 1);
        assert_eq!(header.frame_size, 8);
        assert_eq!(header.total_bytes(), HEADER_BYTES + 8);
    }

    #[test]
    fn decodes_points_and_filters_near_origin() {
        let frame = build_frame(&[(1, 0, 1, 10), (1000, -250, 2200, 5000)], 2, 1);
        let mut out = PointCloudSoa::<MAX_POINTS>::default();

        let stats = decode_frame_into(&frame, CuTime::from(123u64), 0.05, &mut out)
            .expect("frame should decode");

        assert_eq!(stats.raw_points, 2);
        assert_eq!(stats.valid_points, 1);
        assert_eq!(out.len, 1);
        assert!((out.x[0].value - 1.0).abs() < 1e-6);
        assert!((out.y[0].value + 0.25).abs() < 1e-6);
        assert!((out.z[0].value - 2.2).abs() < 1e-6);
        assert_eq!(out.tov[0], CuTime::from(123u64));
    }

    #[test]
    fn rejects_unexpected_data_type() {
        let mut frame = build_frame(&[(1000, 0, 2000, 100)], 1, 1);
        frame[14..16].copy_from_slice(&0u16.to_le_bytes());

        let err = parse_frame(&frame).expect_err("non-point frames must be rejected");
        assert!(matches!(err, FrameError::UnexpectedDataType(0)));
    }
}
