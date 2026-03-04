use bincode::de::{BorrowDecoder, Decoder};
use bincode::error::DecodeError;
use bincode::{BorrowDecode, Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

/// Joint state for up to `N` axes: positions, velocities, and efforts.
///
/// All fields use plain `f32` rather than typed units because the unit is a
/// bridge-level concern: the same wire format is used whether the bridge
/// outputs radians, degrees, raw ticks, or normalised values.
///
/// # Type parameter
///
/// `N` is the maximum number of joints stored in the fixed-capacity arrays.
/// Choose it to match your robot; `8` covers most 6-DOF arms with a spare.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Reflect)]
#[reflect(from_reflect = false, no_field_bounds)]
pub struct JointState<const N: usize> {
    /// Joint positions (radians for revolute joints, metres for prismatic).
    pub positions: CuArray<f32, N>,
    /// Joint velocities (rad/s or m/s).
    pub velocities: CuArray<f32, N>,
    /// Joint efforts (N·m for revolute joints, N for prismatic).
    pub efforts: CuArray<f32, N>,
}

impl<const N: usize> JointState<N> {
    pub fn new() -> Self {
        Self::default()
    }
}

impl<const N: usize> std::fmt::Display for JointState<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "pos:{:?} vel:{:?} eff:{:?}",
            self.positions.as_slice(),
            self.velocities.as_slice(),
            self.efforts.as_slice()
        )
    }
}

// CuArray only implements Decode<()> (unit context), not the generic
// Decode<__Context> that the derive macro would require.  Mirror the same
// manual pattern used by CuArray itself.
impl<const N: usize> Decode<()> for JointState<N> {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            positions: Decode::decode(decoder)?,
            velocities: Decode::decode(decoder)?,
            efforts: Decode::decode(decoder)?,
        })
    }
}

// JointState contains no borrowed data, so BorrowDecode just delegates to Decode.
impl<'de, const N: usize> BorrowDecode<'de, ()> for JointState<N> {
    fn borrow_decode<D: BorrowDecoder<'de, Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, DecodeError> {
        Decode::decode(decoder)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config;

    #[test]
    fn round_trip_encode_decode() {
        let mut js = JointState::<4>::default();
        js.positions.fill_from_iter([0.1_f32, -0.2]);
        js.velocities.fill_from_iter([1.0_f32, 2.0]);
        js.efforts.fill_from_iter([0.5_f32, 0.6]);

        let cfg = config::standard();
        let mut buf = [0u8; 256];
        let len = bincode::encode_into_slice(&js, &mut buf, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<JointState<4>, _>(&buf[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(
            decoded.positions.as_slice()[..2],
            js.positions.as_slice()[..2]
        );
        assert_eq!(
            decoded.velocities.as_slice()[..2],
            js.velocities.as_slice()[..2]
        );
        assert_eq!(decoded.efforts.as_slice()[..2], js.efforts.as_slice()[..2]);
    }

    #[test]
    fn default_is_all_empty() {
        let js = JointState::<8>::default();
        assert!(js.positions.is_empty());
        assert!(js.velocities.is_empty());
        assert!(js.efforts.is_empty());
    }
}
