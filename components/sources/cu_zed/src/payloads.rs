use bincode::de::Decoder;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use core::fmt::Debug;
use cu_transform::FrameTransform;
use cu29::prelude::*;
use serde::{Deserialize, Serialize, Serializer};

#[derive(Default, Debug, Encode, Decode, Clone, Copy, Serialize, Deserialize, Reflect)]
pub struct ZedRasterFormat {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
}

impl ZedRasterFormat {
    pub fn len_elements(&self) -> usize {
        self.stride as usize * self.height as usize
    }
}

macro_rules! impl_f32_raster_payload {
    ($name:ident, $type_path:literal, $ident:literal) => {
        #[derive(Debug, Default, Clone, Encode, Reflect)]
        #[reflect(from_reflect = false, no_field_bounds, type_path = false)]
        pub struct $name<A>
        where
            A: ArrayLike<Element = f32> + Send + Sync + 'static,
        {
            pub seq: u64,
            pub format: ZedRasterFormat,
            #[reflect(ignore)]
            pub buffer_handle: CuHandle<A>,
        }

        impl<A> TypePath for $name<A>
        where
            A: ArrayLike<Element = f32> + Send + Sync + 'static,
        {
            fn type_path() -> &'static str {
                $type_path
            }

            fn short_type_path() -> &'static str {
                $ident
            }

            fn type_ident() -> Option<&'static str> {
                Some($ident)
            }

            fn crate_name() -> Option<&'static str> {
                Some("cu_zed")
            }

            fn module_path() -> Option<&'static str> {
                Some("cu_zed")
            }
        }

        impl<A> Decode<()> for $name<A>
        where
            A: ArrayLike<Element = f32> + Send + Sync + 'static,
            CuHandle<A>: Decode<()>,
        {
            fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                let seq: u64 = Decode::decode(decoder)?;
                let format: ZedRasterFormat = Decode::decode(decoder)?;
                let buffer_handle: CuHandle<A> = Decode::decode(decoder)?;

                Ok(Self {
                    seq,
                    format,
                    buffer_handle,
                })
            }
        }

        impl<'de, A> Deserialize<'de> for $name<A>
        where
            A: ArrayLike<Element = f32> + Send + Sync + 'static,
            CuHandle<A>: Deserialize<'de>,
        {
            fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
            where
                D: serde::Deserializer<'de>,
            {
                #[derive(Deserialize)]
                struct Wire<H> {
                    seq: u64,
                    format: ZedRasterFormat,
                    handle: H,
                }

                let wire = Wire::<CuHandle<A>>::deserialize(deserializer)?;
                Ok(Self {
                    seq: wire.seq,
                    format: wire.format,
                    buffer_handle: wire.handle,
                })
            }
        }

        impl<A> Serialize for $name<A>
        where
            A: ArrayLike<Element = f32> + Send + Sync + 'static,
            CuHandle<A>: Serialize,
        {
            fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
            where
                S: Serializer,
            {
                use serde::ser::SerializeStruct;
                let mut struct_ = serializer.serialize_struct($ident, 3)?;
                struct_.serialize_field("seq", &self.seq)?;
                struct_.serialize_field("format", &self.format)?;
                struct_.serialize_field("handle", &self.buffer_handle)?;
                struct_.end()
            }
        }

        impl<A> $name<A>
        where
            A: ArrayLike<Element = f32> + Send + Sync + 'static,
        {
            pub fn new(format: ZedRasterFormat, buffer_handle: CuHandle<A>) -> Self {
                assert!(
                    format.len_elements() <= buffer_handle.with_inner(|i| i.len()),
                    "Buffer size must at least match the format."
                );
                Self {
                    seq: 0,
                    format,
                    buffer_handle,
                }
            }
        }
    };
}

impl_f32_raster_payload!(ZedDepthMap, "cu_zed::ZedDepthMap", "ZedDepthMap");
impl_f32_raster_payload!(
    ZedConfidenceMap,
    "cu_zed::ZedConfidenceMap",
    "ZedConfidenceMap"
);

#[derive(
    Default, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(from_reflect = false)]
pub enum ZedCoordinateSystem {
    Image,
    #[default]
    LeftHandedYUp,
    RightHandedYUp,
    RightHandedZUp,
    LeftHandedZUp,
    RightHandedZUpXForward,
}

#[derive(
    Default, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(from_reflect = false)]
pub enum ZedCoordinateUnit {
    Millimeter,
    Centimeter,
    #[default]
    Meter,
    Inch,
    Foot,
}

#[derive(Default, Clone, Debug, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(from_reflect = false)]
pub struct ZedCameraIntrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub disto: [f64; 12],
    pub v_fov: f32,
    pub h_fov: f32,
    pub d_fov: f32,
    pub width: u32,
    pub height: u32,
    pub focal_length_metric: f32,
}

#[derive(Default, Clone, Debug, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(from_reflect = false)]
pub struct ZedCalibrationBundle {
    pub serial_number: u32,
    pub width: u32,
    pub height: u32,
    pub fps: f32,
    pub coordinate_system: ZedCoordinateSystem,
    pub coordinate_unit: ZedCoordinateUnit,
    pub left: ZedCameraIntrinsics,
    pub right: ZedCameraIntrinsics,
    pub stereo_rotation_rodrigues: [f32; 3],
    pub stereo_translation_m: [f32; 3],
    pub camera_to_imu_translation_m: Option<[f32; 3]>,
    pub camera_to_imu_quaternion_xyzw: Option<[f32; 4]>,
}

#[derive(Default, Clone, Debug, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(from_reflect = false)]
pub struct ZedFrameMeta {
    pub seq: u64,
    pub image_timestamp_ns: u64,
    pub current_timestamp_ns: u64,
    pub current_fps: f32,
    pub camera_moving_state: Option<i32>,
    pub image_sync_trigger: Option<i32>,
    pub imu_temp_c: Option<f32>,
    pub barometer_temp_c: Option<f32>,
    pub onboard_left_temp_c: Option<f32>,
    pub onboard_right_temp_c: Option<f32>,
}

#[derive(Default, Clone, Debug, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(from_reflect = false)]
pub struct ZedNamedTransform {
    pub matrix: [[f32; 4]; 4],
    pub parent_frame: String,
    pub child_frame: String,
}

impl ZedNamedTransform {
    pub fn from_frame_transform(transform: &FrameTransform<f32>) -> Self {
        Self {
            matrix: transform.transform.to_matrix(),
            parent_frame: transform.parent_frame.as_str().to_string(),
            child_frame: transform.child_frame.as_str().to_string(),
        }
    }

    pub fn to_transform3d(&self) -> cu_transform::Transform3D<f32> {
        cu_transform::Transform3D::from_matrix(self.matrix)
    }
}

#[derive(Default, Clone, Debug, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(from_reflect = false)]
pub struct ZedRigTransforms {
    pub left_to_right: ZedNamedTransform,
    pub camera_to_imu: ZedNamedTransform,
    pub has_camera_to_imu: bool,
}
