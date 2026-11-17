//! Depth images for local Rerun logging and Copper debug inspection.
use crate::{CuDepthEncoding, CuDepthMap};
use alloc::vec::Vec;
use cu29::prelude::ArrayLike;
use cu29::units::si::length::meter;
use rerun::{AsComponents, ChannelDatatype, DepthImage, SerializedComponentBatch};

impl<A, E> AsComponents for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample>,
{
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let Some(depth) = depth_meters(self) else {
            return rerun::Clear::flat().as_serialized_batches();
        };
        DepthImage::from_data_type_and_bytes(
            depth
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect::<Vec<_>>(),
            [self.format.width, self.format.height],
            ChannelDatatype::F32,
        )
        .with_meter(1.0)
        .as_serialized_batches()
    }
}

fn depth_meters<A, E>(depth: &CuDepthMap<A, E>) -> Option<Vec<f32>>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample>,
{
    let format = depth.format;
    if !format.is_valid() || format.width == 0 || format.height == 0 {
        return None;
    }
    depth.buffer_handle.with_inner(|buffer| {
        let mut values = Vec::new();
        for row in 0..format.height as usize {
            let start = row.checked_mul(format.stride as usize)?;
            let end = start.checked_add(format.width as usize)?;
            for sample in buffer.get(start..end)? {
                let value = E::decode(*sample).map(|length| length.get::<meter>());
                values.push(value.filter(|v| v.is_finite() && *v > 0.0).unwrap_or(0.0));
            }
        }
        Some(values)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CuDepthInteger, CuDepthMapFormat, CuDepthMillimeter};
    use cu29::prelude::CuHandle;
    use cu29::units::si::f32::Length;

    #[test]
    fn compact_depth_scales_and_removes_padding() {
        let depth = CuDepthMap::<_, CuDepthInteger<u16, CuDepthMillimeter>>::from_integer(
            CuDepthMapFormat {
                width: 2,
                height: 2,
                stride: 3,
            },
            CuHandle::new_detached(vec![1000, 2000, 9999, 0, 3500, 9999]),
        );
        assert_eq!(depth_meters(&depth), Some(vec![1.0, 2.0, 0.0, 3.5]));
        assert!(!depth.as_serialized_batches().is_empty());
    }

    #[test]
    fn length_depth_masks_invalid_and_nonpositive_samples() {
        let depth = CuDepthMap::new(
            CuDepthMapFormat {
                width: 5,
                height: 1,
                stride: 5,
            },
            CuHandle::new_detached(
                [f32::NAN, f32::INFINITY, -1.0, 0.0, 2.0]
                    .map(Length::new::<meter>)
                    .to_vec(),
            ),
        );
        assert_eq!(depth_meters(&depth), Some(vec![0.0, 0.0, 0.0, 0.0, 2.0]));
    }

    #[test]
    fn malformed_depth_clears_previous_image() {
        let mut depth = CuDepthMap::new(
            CuDepthMapFormat {
                width: 1,
                height: 1,
                stride: 1,
            },
            CuHandle::new_detached(vec![Length::new::<meter>(1.0)]),
        );
        depth.format.height = 2;
        assert_eq!(
            depth.as_serialized_batches()[0].descriptor,
            rerun::Clear::descriptor_is_recursive()
        );
    }
}
