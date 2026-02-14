use compact_str::CompactString;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat, PointCloud, PointCloudSoa};
use cu29::prelude::CuHandle;
use cu29::units::si::length::meter;
use cu29::units::si::ratio::percent;
use serde::{Deserialize, Serialize};

use crate::{RosMsgAdapter, builtin::Header};

const DATATYPE_UINT32: u8 = 6;
const DATATYPE_FLOAT32: u8 = 7;

// sensor_msgs/PointField
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PointField {
    pub name: CompactString,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

// sensor_msgs/PointCloud2 like struct
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub pointfields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

// sensor_msgs/Image like struct
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Image {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: Vec<u8>,
}

impl<const N: usize> RosMsgAdapter<'static> for PointCloudSoa<N> {
    type Output = PointCloud2;

    fn namespace() -> &'static str {
        "sensor_msgs"
    }

    fn type_name() -> &'static str {
        "PointCloud2"
    }

    fn type_hash() -> &'static str {
        "RIHS01_9198cabf7da3796ae6fe19c4cb3bdd3525492988c70522628af5daa124bae2b5"
    }
}

impl RosMsgAdapter<'static> for CuImage<Vec<u8>> {
    type Output = Image;

    fn namespace() -> &'static str {
        "sensor_msgs"
    }

    fn type_name() -> &'static str {
        "Image"
    }

    fn type_hash() -> &'static str {
        "RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47"
    }
}

impl<const N: usize> From<&PointCloudSoa<N>> for PointCloud2 {
    fn from(pointcloud: &PointCloudSoa<N>) -> Self {
        let len = pointcloud.len;

        let pointfields = vec![
            PointField {
                name: "x".into(),
                offset: 0,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "y".into(),
                offset: 4,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "z".into(),
                offset: 8,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "intensity".into(),
                offset: 12,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "tov_sec".into(),
                offset: 16,
                datatype: DATATYPE_UINT32,
                count: 1,
            },
            PointField {
                name: "tov_nsec".into(),
                offset: 20,
                datatype: DATATYPE_UINT32,
                count: 1,
            },
        ];

        let point_step = 24u32;
        let width = len as u32;
        let row_step = point_step * width;
        let mut data = Vec::with_capacity((row_step * 1) as usize);

        for idx in 0..len {
            let x = pointcloud.x[idx].get::<meter>();
            let y = pointcloud.y[idx].get::<meter>();
            let z = pointcloud.z[idx].get::<meter>();
            let intensity = pointcloud.i[idx].get::<percent>();
            let tov_nanos = pointcloud.tov[idx].as_nanos();
            let tov_sec = (tov_nanos / 1_000_000_000) as u32;
            let tov_nsec = (tov_nanos % 1_000_000_000) as u32;

            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());
            data.extend_from_slice(&intensity.to_le_bytes());
            data.extend_from_slice(&tov_sec.to_le_bytes());
            data.extend_from_slice(&tov_nsec.to_le_bytes());
        }

        PointCloud2 {
            header: Header {
                stamp: crate::builtin::Time { sec: 0, nanosec: 0 },
                frame_id: "".into(),
            },
            height: 1,
            width,
            pointfields,
            is_bigendian: false,
            point_step,
            row_step,
            data,
            is_dense: true,
        }
    }
}

impl From<&CuImage<Vec<u8>>> for Image {
    fn from(image: &CuImage<Vec<u8>>) -> Self {
        let data = image.buffer_handle.with_inner(|inner| inner.to_vec());
        Self {
            header: Header {
                stamp: crate::builtin::Time { sec: 0, nanosec: 0 },
                frame_id: "".into(),
            },
            height: image.format.height,
            width: image.format.width,
            encoding: pixel_format_to_encoding(image.format.pixel_format),
            is_bigendian: 0,
            step: image.format.stride,
            data,
        }
    }
}

impl<const N: usize> TryFrom<PointCloud2> for PointCloudSoa<N> {
    type Error = String;

    fn try_from(value: PointCloud2) -> Result<Self, Self::Error> {
        let count = (value.width as usize)
            .checked_mul(value.height as usize)
            .ok_or_else(|| "PointCloud2: width*height overflow".to_string())?;
        if count > N {
            return Err(format!(
                "PointCloud2: {} points exceed PointCloudSoa capacity {}",
                count, N
            ));
        }

        let point_step = value.point_step as usize;
        if point_step < 24 {
            return Err("PointCloud2: point_step too small for x/y/z/intensity/tov".to_string());
        }

        let offsets = field_offsets(&value.pointfields)?;

        let required_bytes = count
            .checked_mul(point_step)
            .ok_or_else(|| "PointCloud2: point_step overflow".to_string())?;
        if value.data.len() < required_bytes {
            return Err(format!(
                "PointCloud2: data length {} < expected {}",
                value.data.len(),
                required_bytes
            ));
        }

        let mut out = PointCloudSoa::<N>::default();
        for idx in 0..count {
            let base = idx * point_step;
            let x = read_f32(&value.data, base + offsets.x)?;
            let y = read_f32(&value.data, base + offsets.y)?;
            let z = read_f32(&value.data, base + offsets.z)?;
            let intensity = read_f32(&value.data, base + offsets.intensity)?;
            let tov_sec = read_u32(&value.data, base + offsets.tov_sec)? as u64;
            let tov_nsec = read_u32(&value.data, base + offsets.tov_nsec)? as u64;

            let tov = tov_sec
                .checked_mul(1_000_000_000)
                .and_then(|s| s.checked_add(tov_nsec))
                .ok_or_else(|| "PointCloud2: timestamp overflow".to_string())?;
            let point = PointCloud::new(tov.into(), x, y, z, intensity, None);
            out.push(point);
        }

        Ok(out)
    }
}

impl TryFrom<Image> for CuImage<Vec<u8>> {
    type Error = String;

    fn try_from(value: Image) -> Result<Self, Self::Error> {
        let required = (value.step as usize)
            .checked_mul(value.height as usize)
            .ok_or_else(|| "Image: step*height overflow".to_string())?;
        if value.data.len() < required {
            return Err(format!(
                "Image: data length {} < expected {}",
                value.data.len(),
                required
            ));
        }

        let format = CuImageBufferFormat {
            width: value.width,
            height: value.height,
            stride: value.step,
            pixel_format: encoding_to_pixel_format(&value.encoding),
        };
        Ok(CuImage::new(format, CuHandle::new_detached(value.data)))
    }
}

#[derive(Clone, Copy)]
struct PointOffsets {
    x: usize,
    y: usize,
    z: usize,
    intensity: usize,
    tov_sec: usize,
    tov_nsec: usize,
}

fn field_offsets(fields: &[PointField]) -> Result<PointOffsets, String> {
    let find = |name: &str, datatype: u8| -> Result<usize, String> {
        fields
            .iter()
            .find(|f| f.name.as_str() == name)
            .ok_or_else(|| format!("PointCloud2: missing field '{name}'"))
            .and_then(|field| {
                if field.datatype != datatype {
                    Err(format!(
                        "PointCloud2: field '{}' has datatype {}, expected {}",
                        name, field.datatype, datatype
                    ))
                } else {
                    Ok(field.offset as usize)
                }
            })
    };

    let intensity_offset = fields
        .iter()
        .find(|f| f.name.as_str() == "intensity" || f.name.as_str() == "i")
        .ok_or_else(|| "PointCloud2: missing field 'intensity' (or 'i')".to_string())
        .and_then(|field| {
            if field.datatype != DATATYPE_FLOAT32 {
                Err(format!(
                    "PointCloud2: field '{}' has datatype {}, expected {}",
                    field.name, field.datatype, DATATYPE_FLOAT32
                ))
            } else {
                Ok(field.offset as usize)
            }
        })?;

    Ok(PointOffsets {
        x: find("x", DATATYPE_FLOAT32)?,
        y: find("y", DATATYPE_FLOAT32)?,
        z: find("z", DATATYPE_FLOAT32)?,
        intensity: intensity_offset,
        tov_sec: find("tov_sec", DATATYPE_UINT32)?,
        tov_nsec: find("tov_nsec", DATATYPE_UINT32)?,
    })
}

fn read_f32(data: &[u8], offset: usize) -> Result<f32, String> {
    let bytes = data
        .get(offset..offset + 4)
        .ok_or_else(|| format!("PointCloud2: missing f32 at offset {offset}"))?;
    Ok(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

fn read_u32(data: &[u8], offset: usize) -> Result<u32, String> {
    let bytes = data
        .get(offset..offset + 4)
        .ok_or_else(|| format!("PointCloud2: missing u32 at offset {offset}"))?;
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

fn pixel_format_to_encoding(pixel_format: [u8; 4]) -> String {
    match &pixel_format {
        b"GRAY" => "mono8".to_string(),
        b"RGB3" => "rgb8".to_string(),
        b"BGR3" => "bgr8".to_string(),
        b"RGBA" => "rgba8".to_string(),
        b"BGRA" => "bgra8".to_string(),
        _ => {
            let end = pixel_format
                .iter()
                .position(|byte| *byte == 0)
                .unwrap_or(pixel_format.len());
            String::from_utf8_lossy(&pixel_format[..end]).to_string()
        }
    }
}

fn encoding_to_pixel_format(encoding: &str) -> [u8; 4] {
    match encoding {
        "mono8" => *b"GRAY",
        "rgb8" => *b"RGB3",
        "bgr8" => *b"BGR3",
        "rgba8" => *b"RGBA",
        "bgra8" => *b"BGRA",
        _ => {
            let mut out = [0u8; 4];
            let bytes = encoding.as_bytes();
            let copy_len = bytes.len().min(out.len());
            out[..copy_len].copy_from_slice(&bytes[..copy_len]);
            out
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RosBridgeAdapter;

    #[test]
    fn pointcloud_soa_roundtrip() {
        let mut cloud = PointCloudSoa::<4>::default();
        cloud.push(PointCloud::new(
            1_500_000_111u64.into(),
            1.0,
            2.0,
            3.0,
            4.0,
            None,
        ));
        cloud.push(PointCloud::new(
            2_500_000_222u64.into(),
            5.0,
            6.0,
            7.0,
            8.0,
            None,
        ));

        let ros_value = cloud.to_ros_message();
        let bytes = cdr::serialize::<_, _, cdr::CdrBe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <PointCloudSoa<4> as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(bytes.as_slice()).expect("cdr decode should succeed");
        let recovered =
            PointCloudSoa::<4>::from_ros_message(decoded_ros).expect("adapter decode should work");

        assert_eq!(cloud.len, recovered.len);
        for i in 0..cloud.len {
            assert_eq!(cloud.tov[i].as_nanos(), recovered.tov[i].as_nanos());
            assert!((cloud.x[i].value - recovered.x[i].value).abs() < 1e-6);
            assert!((cloud.y[i].value - recovered.y[i].value).abs() < 1e-6);
            assert!((cloud.z[i].value - recovered.z[i].value).abs() < 1e-6);
            assert!((cloud.i[i].value - recovered.i[i].value).abs() < 1e-6);
        }
    }

    #[test]
    fn image_roundtrip() {
        let format = CuImageBufferFormat {
            width: 4,
            height: 2,
            stride: 8,
            pixel_format: *b"GRAY",
        };
        let bytes = (0u8..16).collect::<Vec<_>>();
        let mut image = CuImage::new(format, CuHandle::new_detached(bytes.clone()));
        image.seq = 0;

        let ros_value = image.to_ros_message();
        let encoded = cdr::serialize::<_, _, cdr::CdrBe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <CuImage<Vec<u8>> as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(encoded.as_slice()).expect("cdr decode should succeed");
        let recovered =
            CuImage::<Vec<u8>>::from_ros_message(decoded_ros).expect("adapter decode should work");

        assert_eq!(recovered.format.width, format.width);
        assert_eq!(recovered.format.height, format.height);
        assert_eq!(recovered.format.stride, format.stride);
        assert_eq!(recovered.format.pixel_format, format.pixel_format);
        let recovered_bytes = recovered.buffer_handle.with_inner(|inner| inner.to_vec());
        assert_eq!(recovered_bytes, bytes);
    }
}
