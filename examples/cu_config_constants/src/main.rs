use cu_transform::{CameraFrame, RobotFrame, TypedTransform3D, WorldFrame};
use cu29::prelude::*;

#[derive(Reflect)]
struct ConstantsDemo;

impl Freezable for ConstantsDemo {}

impl CuSrcTask for ConstantsDemo {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(());

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(());
        Ok(())
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const _: usize = constants::MAX_DETECTIONS;
const _: [cu29::units::si::f32::Length; 3] = frames::camera::CAMERA_TRANSLATION;
const _: [cu29::units::si::f32::Angle; 3] = frames::camera::CAMERA_ROTATION;
const _: [cu29::units::si::f32::Length; 3] = frames::camera::CAMERA_OFFSET;
const _: cu29::units::si::f64::Time = control::CONTROL_PERIOD;
const _: cu29::units::si::f64::Angle = control::YAW_LIMIT;

const WORLD_TO_ROBOT: TypedTransform3D<f32, WorldFrame, RobotFrame> =
    TypedTransform3D::<f32, WorldFrame, RobotFrame>::from_translation_euler_xyz(
        frames::robot::ROBOT_TRANSLATION,
        frames::robot::ROBOT_ROTATION,
    );
const ROBOT_TO_CAMERA: TypedTransform3D<f32, RobotFrame, CameraFrame> =
    TypedTransform3D::<f32, RobotFrame, CameraFrame>::from_translation_euler_xyz(
        frames::camera::CAMERA_TRANSLATION,
        frames::camera::CAMERA_ROTATION,
    );
const WORLD_TO_CAMERA: TypedTransform3D<f32, WorldFrame, CameraFrame> =
    WORLD_TO_ROBOT.then(ROBOT_TO_CAMERA);

fn main() {
    assert_eq!(constants::MAX_DETECTIONS, 64);
    assert_eq!(
        frames::camera::CAMERA_OFFSET[0].raw(),
        frames::camera::CAMERA_TRANSLATION[0].raw()
    );

    // Construction and frame-checked composition happened at compile time. Only the timestamp is
    // runtime data.
    let world_to_camera = WORLD_TO_CAMERA.at(CuTime::from_nanos(1_000));
    assert_eq!(world_to_camera.parent_name(), "world");
    assert_eq!(world_to_camera.child_name(), "camera");
}
