use compact_str::CompactString;
use cu_sensor_payloads::PointCloudSoa;
use serde::{Deserialize, Serialize};

use crate::{builtin::Header, RosMsgAdapter};

// sensor_msgs/PointField
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PointField {
    pub name: CompactString,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

// sensor_msgs/PointCloud2 like struct
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32, //
    pub width: u32,  // Length of the point cloud
    pub pointfields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32, // Length of a point in bytes
    pub row_step: u32,   // Length of a row in bytes
    pub data: Vec<u8>,   // size is (row_step*height)
    pub is_dense: bool,  // True if there are no invalid points
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

impl<const N: usize> From<&PointCloudSoa<N>> for PointCloud2 {
    #[allow(unreachable_code)]
    fn from(_: &PointCloudSoa<N>) -> Self {
        PointCloud2 {
            header: todo!(),
            height: 1,
            width: todo!(),
            pointfields: todo!(),
            is_bigendian: true,
            point_step: todo!(),
            row_step: todo!(),
            data: todo!(),
            is_dense: todo!(),
        }
    }
}
