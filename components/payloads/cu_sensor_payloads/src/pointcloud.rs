use crate::{CuHandlePayload, CuHandlePayloadInit, CuHandlePayloadMeta};
use alloc::alloc::{Layout, alloc_zeroed, handle_alloc_error};
use alloc::boxed::Box;
use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::{Length, Ratio};
use cu29::units::si::length::meter;
use cu29::units::si::ratio::percent;
use cu29_clock::CuTime;
use cu29_soa_derive::Soa;
use serde::{Deserialize, Serialize};

pub type Distance = Length;
pub type Reflectivity = Ratio;

/// Standardized PointCloud.
/// note: the derive(Soa) will generate a PointCloudSoa struct that will store the data in a SoA format.
/// The Soa format is appropriate for early pipeline operations like changing their frame of reference.
/// important: The ToV of the points are not assumed to be sorted.
#[derive(
    Default, Clone, PartialEq, Debug, Soa, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(from_reflect = false)]
pub struct PointCloud {
    pub tov: CuTime, // Time of Validity, not sorted.
    pub x: Distance,
    pub y: Distance,
    pub z: Distance,
    pub i: Reflectivity,
    pub return_order: u8, // 0 for first return, 1 for second return, etc.
}

impl PointCloud {
    pub fn new(tov: CuTime, x: f32, y: f32, z: f32, i: f32, return_order: Option<u8>) -> Self {
        Self {
            tov,
            x: Distance::new::<meter>(x),
            y: Distance::new::<meter>(y),
            z: Distance::new::<meter>(z),
            i: Reflectivity::new::<percent>(i),
            return_order: return_order.unwrap_or(0),
        }
    }

    pub fn new_units(
        tov: CuTime,
        x: Length,
        y: Length,
        z: Length,
        i: Ratio,
        return_order: Option<u8>,
    ) -> Self {
        Self {
            tov,
            x,
            y,
            z,
            i,
            return_order: return_order.unwrap_or(0),
        }
    }
}

pub struct PointCloudSoaHandleMeta;

impl CuHandlePayloadMeta for PointCloudSoaHandleMeta {
    const TYPE_PATH: &'static str = "cu_sensor_payloads::PointCloudSoaHandle";
    const SHORT_TYPE_PATH: &'static str = "PointCloudSoaHandle";
    const TYPE_IDENT: Option<&'static str> = Some("PointCloudSoaHandle");
    const CRATE_NAME: Option<&'static str> = Some("cu_sensor_payloads");
    const MODULE_PATH: Option<&'static str> = Some("cu_sensor_payloads");
}

/// Copper point-cloud payload stored behind a `CuHandle` so large clouds do not live inline on
/// the thread stack.
pub type PointCloudSoaHandle<const N: usize> =
    CuHandlePayload<PointCloudSoa<N>, PointCloudSoaHandleMeta>;

impl<const N: usize> PointCloudSoa<N> {
    /// Allocate a zeroed point cloud directly on the heap.
    ///
    /// SAFETY: `PointCloudSoa<N>` is generated from `PointCloud`, whose fields are `usize`, `u8`,
    /// `u64` (`CuTime`), and `uom::Quantity<f32>` wrappers. The quantity type is
    /// `#[repr(transparent)]` over its scalar plus `PhantomData`, so an all-zero bit pattern is a
    /// valid semantic zero for every field in the generated SoA storage.
    pub fn boxed_zeroed() -> Box<Self> {
        let layout = Layout::new::<Self>();
        let ptr = unsafe { alloc_zeroed(layout) };
        if ptr.is_null() {
            handle_alloc_error(layout);
        }
        unsafe { Box::from_raw(ptr.cast()) }
    }

    /// Sort in place the point cloud so it can be ready for merge sorts for example
    pub fn sort(&mut self) {
        self.quicksort(0, N - 1);
    }

    /// Implementation of the sort
    fn quicksort(&mut self, low: usize, high: usize) {
        if low < high {
            let pivot_index = self.partition(low, high);
            if pivot_index > 0 {
                self.quicksort(low, pivot_index - 1);
            }
            self.quicksort(pivot_index + 1, high);
        }
    }

    /// Used by quicksort.
    fn partition(&mut self, low: usize, high: usize) -> usize {
        let pivot = self.tov[high];
        let mut i = low;
        for j in low..high {
            if self.tov[j] <= pivot {
                self.swap(i, j);
                i += 1;
            }
        }
        self.swap(i, high);
        i
    }

    /// swap the elements at index a and b
    pub fn swap(&mut self, a: usize, b: usize) {
        self.tov.swap(a, b);
        self.x.swap(a, b);
        self.y.swap(a, b);
        self.z.swap(a, b);
        self.i.swap(a, b);
    }
}

impl<const N: usize> CuHandlePayloadInit for PointCloudSoa<N> {
    fn boxed_init() -> Box<Self> {
        Self::boxed_zeroed()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_clock::CuTime;

    #[test]
    fn test_point_payload() {
        let payload = PointCloud::new(CuTime::from_nanos(1), 1.0, 2.0, 3.0, 0.0, None);
        assert_eq!(payload.x.value, 1.0);
        assert_eq!(payload.y.value, 2.0);
        assert_eq!(payload.z.value, 3.0);
    }

    #[test]
    fn test_length_add_sub() {
        let a = Distance::new::<meter>(1.0);
        let b = Distance::new::<meter>(2.0);
        let c = a + b;
        assert_eq!(c.value, 3.0);
        let d = c - a;
        assert_eq!(d.value, 2.0);
    }

    #[test]
    fn test_encoding_length() {
        let a = Distance::new::<meter>(1.0);
        let mut encoded = vec![0u8; 1024]; // Reserve a buffer with sufficient capacity

        let length =
            bincode::encode_into_slice(a, &mut encoded, bincode::config::standard()).unwrap();
        assert_eq!(length, 4);
    }
}
