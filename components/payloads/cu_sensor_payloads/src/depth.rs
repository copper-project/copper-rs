use bincode::de::Decoder;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use core::fmt::Debug;
use cu29::prelude::*;
use serde::{Deserialize, Serialize, Serializer};

/// Raster layout for a depth map whose samples are `f32` distances in meters.
///
/// `stride` is expressed in elements, not bytes, and may be greater than `width`
/// when rows contain padding.
#[derive(
    Default, Debug, Encode, Decode, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Reflect,
)]
pub struct CuDepthMapFormat {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
}

impl CuDepthMapFormat {
    pub fn is_valid(&self) -> bool {
        self.width <= self.stride
    }

    pub fn required_elements(&self) -> usize {
        self.stride as usize * self.height as usize
    }
}

/// Standard depth-map payload backed by a Copper handle.
///
/// Every sample is an `f32` distance in meters. Non-finite values are preserved
/// and may be used by producers to mark invalid measurements.
#[derive(Debug, Default, Clone, Encode, Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct CuDepthMap<A>
where
    A: ArrayLike<Element = f32> + Send + Sync + 'static,
{
    pub seq: u64,
    pub format: CuDepthMapFormat,
    #[reflect(ignore)]
    pub buffer_handle: CuHandle<A>,
}

impl<A> TypePath for CuDepthMap<A>
where
    A: ArrayLike<Element = f32> + Send + Sync + 'static,
{
    fn type_path() -> &'static str {
        "cu_sensor_payloads::CuDepthMap"
    }

    fn short_type_path() -> &'static str {
        "CuDepthMap"
    }

    fn type_ident() -> Option<&'static str> {
        Some("CuDepthMap")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_sensor_payloads")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_sensor_payloads")
    }
}

impl<A> Decode<()> for CuDepthMap<A>
where
    A: ArrayLike<Element = f32> + Send + Sync + 'static,
    CuHandle<A>: Decode<()>,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            seq: Decode::decode(decoder)?,
            format: Decode::decode(decoder)?,
            buffer_handle: Decode::decode(decoder)?,
        })
    }
}

impl<'de, A> Deserialize<'de> for CuDepthMap<A>
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
            format: CuDepthMapFormat,
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

impl<A> Serialize for CuDepthMap<A>
where
    A: ArrayLike<Element = f32> + Send + Sync + 'static,
    CuHandle<A>: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("CuDepthMap", 3)?;
        state.serialize_field("seq", &self.seq)?;
        state.serialize_field("format", &self.format)?;
        state.serialize_field("handle", &self.buffer_handle)?;
        state.end()
    }
}

impl<A> CuDepthMap<A>
where
    A: ArrayLike<Element = f32> + Send + Sync + 'static,
{
    pub fn new(format: CuDepthMapFormat, buffer_handle: CuHandle<A>) -> Self {
        assert!(
            format.is_valid(),
            "Depth-map stride must be at least its width."
        );
        assert!(
            format.required_elements() <= buffer_handle.with_inner(|inner| inner.len()),
            "Buffer size must at least match the depth-map format."
        );
        Self {
            seq: 0,
            format,
            buffer_handle,
        }
    }

    /// Returns the sample at `(x, y)` in meters, preserving non-finite values.
    pub fn get_meters(&self, x: u32, y: u32) -> Option<f32> {
        if x >= self.format.width || y >= self.format.height {
            return None;
        }
        let index = y as usize * self.format.stride as usize + x as usize;
        self.buffer_handle
            .with_inner(|inner| inner.as_ref().get(index).copied())
    }

    pub fn payload_should_log(&self) -> bool {
        self.buffer_handle.payload_should_log()
    }

    pub fn apply_handle_content_policy(&self, mode: cu29::pool::HandleContent) {
        self.buffer_handle.apply_handle_content_policy(mode);
    }

    pub fn mark_touched(&self) {
        self.buffer_handle.mark_touched();
    }
}

impl<A> cu29::pool::HandleContentAware for CuDepthMap<A> where
    A: ArrayLike<Element = f32> + Send + Sync + 'static
{
}

#[cfg(test)]
mod tests {
    use super::*;

    const FORMAT: CuDepthMapFormat = CuDepthMapFormat {
        width: 3,
        height: 2,
        stride: 4,
    };

    #[test]
    fn padded_depth_map_indexes_distances_in_meters() {
        let depth = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(vec![1.0, 1.5, f32::NAN, 99.0, 2.0, 2.5, 3.0, 99.0]),
        );

        assert_eq!(depth.get_meters(0, 0), Some(1.0));
        assert_eq!(depth.get_meters(1, 1), Some(2.5));
        assert!(depth.get_meters(2, 0).expect("in bounds").is_nan());
        assert_eq!(depth.get_meters(3, 0), None);
        assert_eq!(depth.get_meters(0, 2), None);
    }

    #[test]
    fn format_rejects_short_stride() {
        assert!(
            !CuDepthMapFormat {
                width: 4,
                height: 2,
                stride: 3,
            }
            .is_valid()
        );
    }

    #[test]
    fn serde_wire_uses_handle_field() {
        let depth = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(vec![1.0, 1.5, 2.0, 0.0, 2.5, 3.0, 3.5, 0.0]),
        );
        let value = serde_json::to_value(depth).expect("serialize depth");
        assert_eq!(value["format"]["stride"], 4);
        assert!(value.get("handle").is_some());
        assert!(value.get("buffer_handle").is_none());
    }
}
