use cu_transform::{FrameId, TypedTransform3D};
use cu29::bincode::{
    Decode, Encode,
    de::Decoder,
    enc::Encoder,
    error::{DecodeError, EncodeError},
};
use cu29::prelude::*;
use cu29::units::si::f64::{Angle, Length};
use serde::{Deserialize, Serialize};

pub const FULL_TURN: u16 = 36_000;
const STEP: u64 = 30; // Hundredths of a degree per 10 ms tick: one turn in 12 seconds.
pub const UPPER_ARM_M: f64 = 1.0;
pub const FOREARM_M: f64 = 0.65;

/// Encoder readings in hundredths of a degree; elbow is relative to the upper arm.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[bincode(crate = "cu29::bincode")]
pub struct JointAngles {
    pub shoulder: u16,
    pub elbow: u16,
}

impl JointAngles {
    pub fn at_tick(tick: u64) -> Self {
        let shoulder = ((tick % (u64::from(FULL_TURN) / STEP)) * STEP) as u16;
        let elbow = ((u32::from(FULL_TURN) - (u32::from(shoulder) * 4) % u32::from(FULL_TURN))
            % u32::from(FULL_TURN)) as u16;
        Self { shoulder, elbow }
    }
}

/// Positions in the base frame, in meters. This payload never crosses the link.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[bincode(crate = "cu29::bincode")]
pub struct ArmPose {
    pub elbow: [f64; 2],
    pub tip: [f64; 2],
}

#[derive(Default, Reflect)]
pub struct Encoders {
    tick: u64,
}
impl Freezable for Encoders {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.tick, encoder)
    }
    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.tick = Decode::decode(decoder)?;
        Ok(())
    }
}
impl CuSrcTask for Encoders {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(JointAngles);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self::default())
    }
    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(JointAngles::at_tick(self.tick));
        output.tov = Tov::Time(ctx.now());
        self.tick = self.tick.wrapping_add(1);
        Ok(())
    }
}

macro_rules! frame {
    ($name:ident, $id:expr) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        struct $name;
        impl FrameId for $name {
            const ID: u32 = $id;
            const NAME: &'static str = stringify!($name);
        }
    };
}
frame!(Base, 0);
frame!(Shoulder, 1);
frame!(Elbow, 2);
frame!(Forearm, 3);
frame!(Tip, 4);

const ZERO_LENGTH: Length = Length { value: 0.0 };
const ZERO_ANGLE: Angle = Angle { value: 0.0 };
const UPPER_ARM: TypedTransform3D<f64, Shoulder, Elbow> =
    TypedTransform3D::<f64, Shoulder, Elbow>::from_translation_euler_xyz(
        [Length { value: UPPER_ARM_M }, ZERO_LENGTH, ZERO_LENGTH],
        [ZERO_ANGLE; 3],
    );
const FOREARM: TypedTransform3D<f64, Forearm, Tip> =
    TypedTransform3D::<f64, Forearm, Tip>::from_translation_euler_xyz(
        [Length { value: FOREARM_M }, ZERO_LENGTH, ZERO_LENGTH],
        [ZERO_ANGLE; 3],
    );

const fn rotation<P: FrameId, C: FrameId>(centidegrees: u16) -> TypedTransform3D<f64, P, C> {
    TypedTransform3D::<f64, P, C>::from_translation_euler_xyz(
        [ZERO_LENGTH; 3],
        [
            ZERO_ANGLE,
            ZERO_ANGLE,
            Angle {
                value: centidegrees as f64 * (core::f64::consts::PI / 18_000.0),
            },
        ],
    )
}

pub fn forward_kinematics(angles: JointAngles) -> CuResult<ArmPose> {
    if angles.shoulder >= FULL_TURN || angles.elbow >= FULL_TURN {
        return Err(CuError::from("Encoder angle outside [0, 360) degrees"));
    }
    let elbow = rotation::<Base, Shoulder>(angles.shoulder).then(UPPER_ARM);
    let tip = elbow
        .then(rotation::<Elbow, Forearm>(angles.elbow))
        .then(FOREARM);
    let elbow = elbow.transform().translation();
    let tip = tip.transform().translation();
    Ok(ArmPose {
        elbow: [elbow[0].value, elbow[1].value],
        tip: [tip[0].value, tip[1].value],
    })
}

/// Same task on the robot and in generated replay; the UI never computes kinematics.
#[derive(Default, Reflect)]
pub struct Kinematics;
impl Freezable for Kinematics {}
impl CuCrossPlatformDeterministic for Kinematics {
    // Bounded integer inputs, scalar f64 const-transform arithmetic, no libm,
    // SIMD multiplication, wall clock, allocation, or external side effects.
    const REPLAY_ABI: u32 = 1;
}
impl CuTask for Kinematics {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(JointAngles);
    type Output<'m> = output_msg!(ArmPose);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let angles = *input
            .payload()
            .ok_or_else(|| CuError::from("Encoders produced no angles"))?;
        output.set_payload(forward_kinematics(angles)?);
        output.tov = input.tov;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_geometry_matches_known_joint_positions() {
        for (angles, elbow, tip) in [
            (
                JointAngles {
                    shoulder: 0,
                    elbow: 0,
                },
                [1.0, 0.0],
                [1.65, 0.0],
            ),
            (
                JointAngles {
                    shoulder: 9000,
                    elbow: 0,
                },
                [0.0, 1.0],
                [0.0, 1.65],
            ),
            (
                JointAngles {
                    shoulder: 0,
                    elbow: 9000,
                },
                [1.0, 0.0],
                [1.0, 0.65],
            ),
            (
                JointAngles {
                    shoulder: 9000,
                    elbow: 9000,
                },
                [0.0, 1.0],
                [-0.65, 1.0],
            ),
        ] {
            let pose = forward_kinematics(angles).unwrap();
            for (actual, expected) in pose
                .elbow
                .into_iter()
                .chain(pose.tip)
                .zip(elbow.into_iter().chain(tip))
            {
                assert!((actual - expected).abs() < 1e-12, "{angles:?}: {pose:?}");
            }
        }
        assert!(
            forward_kinematics(JointAngles {
                shoulder: FULL_TURN,
                elbow: 0
            })
            .is_err()
        );
    }

    #[test]
    fn runtime_composition_matches_const_evaluation_bit_for_bit() {
        const EXPECTED: TypedTransform3D<f64, Base, Tip> = rotation::<Base, Shoulder>(12345)
            .then(UPPER_ARM)
            .then(rotation::<Elbow, Forearm>(23456))
            .then(FOREARM);
        let actual = forward_kinematics(std::hint::black_box(JointAngles {
            shoulder: 12345,
            elbow: 23456,
        }))
        .unwrap();
        let expected = EXPECTED.transform().translation();
        assert_eq!(
            actual.tip.map(f64::to_bits),
            [expected[0].value.to_bits(), expected[1].value.to_bits()]
        );
    }
}
