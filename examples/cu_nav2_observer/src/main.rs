use bincode::{Decode, Encode};
use cu_ros2_bridge::register_ros2_payload;
use cu_ros2_payloads::RosBridgeAdapter;
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use rerun::{LineStrips2D, Points2D, RecordingStream, RecordingStreamBuilder, Scalars, Vec2D};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::time::Duration;

pub mod payloads {
    use super::*;

    const PATH_HASH: &str =
        "RIHS01_1957a5bb3cee5da65c4e52e52b65a93df227efce4c20f8458b36e73066ca334b";
    const ODOMETRY_HASH: &str =
        "RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0";

    #[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
    pub struct Pose2D {
        pub x: f32,
        pub y: f32,
        pub yaw_rad: f32,
    }

    #[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
    pub struct NavPathPayload {
        pub frame_id: String,
        pub poses: Vec<Pose2D>,
    }

    #[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
    pub struct Odometry2DPayload {
        pub frame_id: String,
        pub child_frame_id: String,
        pub pose: Pose2D,
        pub linear_velocity_mps: f32,
        pub angular_velocity_rps: f32,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Time {
        pub sec: i32,
        pub nanosec: u32,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Header {
        pub stamp: Time,
        pub frame_id: String,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Pose {
        pub position: Point,
        pub orientation: Quaternion,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct PoseStamped {
        pub header: Header,
        pub pose: Pose,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct PathMsg {
        pub header: Header,
        pub poses: Vec<PoseStamped>,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct PoseWithCovariance {
        pub pose: Pose,
        pub covariance: [[f64; 6]; 6],
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct TwistWithCovariance {
        pub twist: Twist,
        pub covariance: [[f64; 6]; 6],
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct OdometryMsg {
        pub header: Header,
        pub child_frame_id: String,
        pub pose: PoseWithCovariance,
        pub twist: TwistWithCovariance,
    }

    fn zero_time() -> Time {
        Time { sec: 0, nanosec: 0 }
    }

    fn header(frame_id: &str) -> Header {
        Header {
            stamp: zero_time(),
            frame_id: frame_id.to_string(),
        }
    }

    fn quaternion_from_yaw(yaw_rad: f32) -> Quaternion {
        let half = (yaw_rad as f64) * 0.5;
        Quaternion {
            x: 0.0,
            y: 0.0,
            z: half.sin(),
            w: half.cos(),
        }
    }

    fn yaw_from_quaternion(q: &Quaternion) -> f32 {
        let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        siny_cosp.atan2(cosy_cosp) as f32
    }

    fn pose_to_ros(pose: &Pose2D) -> Pose {
        Pose {
            position: Point {
                x: pose.x as f64,
                y: pose.y as f64,
                z: 0.0,
            },
            orientation: quaternion_from_yaw(pose.yaw_rad),
        }
    }

    fn pose_from_ros(pose: Pose) -> Pose2D {
        Pose2D {
            x: pose.position.x as f32,
            y: pose.position.y as f32,
            yaw_rad: yaw_from_quaternion(&pose.orientation),
        }
    }

    impl RosBridgeAdapter for NavPathPayload {
        type RosMessage = PathMsg;

        fn namespace() -> &'static str {
            "nav_msgs"
        }

        fn type_name() -> &'static str {
            "Path"
        }

        fn type_hash() -> &'static str {
            PATH_HASH
        }

        fn to_ros_message(&self) -> Self::RosMessage {
            let path_header = header(&self.frame_id);
            let poses = self
                .poses
                .iter()
                .map(|pose| PoseStamped {
                    header: header(&self.frame_id),
                    pose: pose_to_ros(pose),
                })
                .collect();
            PathMsg {
                header: path_header,
                poses,
            }
        }

        fn from_ros_message(msg: Self::RosMessage) -> Result<Self, String> {
            Ok(Self {
                frame_id: msg.header.frame_id,
                poses: msg
                    .poses
                    .into_iter()
                    .map(|pose| pose_from_ros(pose.pose))
                    .collect(),
            })
        }
    }

    impl RosBridgeAdapter for Odometry2DPayload {
        type RosMessage = OdometryMsg;

        fn namespace() -> &'static str {
            "nav_msgs"
        }

        fn type_name() -> &'static str {
            "Odometry"
        }

        fn type_hash() -> &'static str {
            ODOMETRY_HASH
        }

        fn to_ros_message(&self) -> Self::RosMessage {
            OdometryMsg {
                header: header(&self.frame_id),
                child_frame_id: self.child_frame_id.clone(),
                pose: PoseWithCovariance {
                    pose: pose_to_ros(&self.pose),
                    covariance: [[0.0; 6]; 6],
                },
                twist: TwistWithCovariance {
                    twist: Twist {
                        linear: Vector3 {
                            x: self.linear_velocity_mps as f64,
                            y: 0.0,
                            z: 0.0,
                        },
                        angular: Vector3 {
                            x: 0.0,
                            y: 0.0,
                            z: self.angular_velocity_rps as f64,
                        },
                    },
                    covariance: [[0.0; 6]; 6],
                },
            }
        }

        fn from_ros_message(msg: Self::RosMessage) -> Result<Self, String> {
            Ok(Self {
                frame_id: msg.header.frame_id,
                child_frame_id: msg.child_frame_id,
                pose: pose_from_ros(msg.pose.pose),
                linear_velocity_mps: msg.twist.twist.linear.x as f32,
                angular_velocity_rps: msg.twist.twist.angular.z as f32,
            })
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn path_adapter_roundtrips() {
            let payload = NavPathPayload {
                frame_id: "map".to_string(),
                poses: vec![
                    Pose2D {
                        x: 1.0,
                        y: 2.0,
                        yaw_rad: 0.25,
                    },
                    Pose2D {
                        x: 3.5,
                        y: -1.25,
                        yaw_rad: -0.5,
                    },
                ],
            };

            let ros = payload.to_ros_message();
            let recovered = NavPathPayload::from_ros_message(ros).expect("path decode");
            assert_eq!(recovered.frame_id, "map");
            assert_eq!(recovered.poses.len(), 2);
            assert!((recovered.poses[0].yaw_rad - 0.25).abs() < 1e-5);
            assert!((recovered.poses[1].yaw_rad + 0.5).abs() < 1e-5);
        }

        #[test]
        fn odometry_adapter_roundtrips() {
            let payload = Odometry2DPayload {
                frame_id: "odom".to_string(),
                child_frame_id: "base_link".to_string(),
                pose: Pose2D {
                    x: -0.75,
                    y: 4.0,
                    yaw_rad: 1.2,
                },
                linear_velocity_mps: 0.8,
                angular_velocity_rps: -0.2,
            };

            let ros = payload.to_ros_message();
            let recovered = Odometry2DPayload::from_ros_message(ros).expect("odom decode");
            assert_eq!(recovered.frame_id, "odom");
            assert_eq!(recovered.child_frame_id, "base_link");
            assert!((recovered.pose.yaw_rad - 1.2).abs() < 1e-5);
            assert!((recovered.linear_velocity_mps - 0.8).abs() < 1e-5);
            assert!((recovered.angular_velocity_rps + 0.2).abs() < 1e-5);
        }
    }
}

pub mod bridges {
    use super::payloads::{NavPathPayload, Odometry2DPayload};
    use cu_ros2_bridge::Ros2Bridge;
    use cu29::cubridge::{BridgeChannelInfo, BridgeChannelSet};
    use cu29::prelude::*;

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum Nav2TxId {}

    pub struct Nav2TxChannels;

    impl BridgeChannelSet for Nav2TxChannels {
        type Id = Nav2TxId;

        const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
    }

    rx_channels! {
        pub struct Nav2RxChannels : Nav2RxId {
            global_plan => NavPathPayload = "/plan",
            received_global_plan => NavPathPayload = "/received_global_plan",
            smoothed_plan => NavPathPayload = "/plan_smoothed",
            local_plan => NavPathPayload = "/local_plan",
            odom => Odometry2DPayload = "/odom",
        }
    }

    pub type Nav2ObserverBridge = Ros2Bridge<Nav2TxChannels, Nav2RxChannels>;
}

pub mod tasks {
    use super::payloads::{NavPathPayload, Odometry2DPayload};
    use super::*;

    const PLAN_ENTITY: &str = "map/nav2/plan";
    const PLAN_POINTS_ENTITY: &str = "map/nav2/plan_points";
    const RECEIVED_PLAN_ENTITY: &str = "map/nav2/received_global_plan";
    const RECEIVED_PLAN_POINTS_ENTITY: &str = "map/nav2/received_global_plan_points";
    const SMOOTHED_PLAN_ENTITY: &str = "map/nav2/plan_smoothed";
    const SMOOTHED_PLAN_POINTS_ENTITY: &str = "map/nav2/plan_smoothed_points";
    const LOCAL_PLAN_ENTITY: &str = "map/nav2/local_plan";
    const LOCAL_PLAN_POINTS_ENTITY: &str = "map/nav2/local_plan_points";
    const ROBOT_ENTITY: &str = "map/base_link";
    const TRAIL_ENTITY: &str = "map/base_link/trail";
    const HEADING_ENTITY: &str = "map/base_link/heading";
    const POSITION_X_ENTITY: &str = "plots/base_link/x_m";
    const POSITION_Y_ENTITY: &str = "plots/base_link/y_m";
    const YAW_ENTITY: &str = "plots/base_link/yaw_rad";
    const LINEAR_SPEED_ENTITY: &str = "plots/base_link/linear_velocity_mps";
    const ANGULAR_SPEED_ENTITY: &str = "plots/base_link/angular_velocity_rps";
    const TRAIL_CAPACITY: usize = 2048;
    const HEADING_ARROW_LEN_M: f32 = 0.35;

    fn plan_color() -> rerun::Color {
        rerun::Color::from_rgb(255, 215, 0)
    }

    fn received_plan_color() -> rerun::Color {
        rerun::Color::from_rgb(0, 220, 120)
    }

    fn smoothed_plan_color() -> rerun::Color {
        rerun::Color::from_rgb(80, 200, 255)
    }

    fn local_plan_color() -> rerun::Color {
        rerun::Color::from_rgb(255, 120, 40)
    }

    #[derive(Reflect)]
    #[reflect(from_reflect = false)]
    pub struct RerunNav2Viz {
        #[reflect(ignore)]
        rec: RecordingStream,
        #[reflect(ignore)]
        trail: VecDeque<[f32; 2]>,
        seen_plan: bool,
        seen_received_global_plan: bool,
        seen_smoothed_plan: bool,
        seen_local_plan: bool,
        seen_odom: bool,
    }

    impl Freezable for RerunNav2Viz {}

    impl CuSinkTask for RerunNav2Viz {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(
            'm,
            NavPathPayload,
            NavPathPayload,
            NavPathPayload,
            NavPathPayload,
            Odometry2DPayload
        );

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let rec = RecordingStreamBuilder::new("Copper Nav2 Observer")
                .spawn()
                .map_err(|e| CuError::new_with_cause("Failed to spawn Rerun stream", e))?;

            Ok(Self {
                rec,
                trail: VecDeque::with_capacity(TRAIL_CAPACITY),
                seen_plan: false,
                seen_received_global_plan: false,
                seen_smoothed_plan: false,
                seen_local_plan: false,
                seen_odom: false,
            })
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let (plan_msg, received_plan_msg, smoothed_plan_msg, local_plan_msg, odom_msg) = *input;

            if let Some(path) = plan_msg.payload() {
                if !self.seen_plan {
                    info!(
                        "received first Nav2 path on /plan ({} poses)",
                        path.poses.len()
                    );
                    self.seen_plan = true;
                }
                self.log_path(path, PLAN_ENTITY, PLAN_POINTS_ENTITY, plan_color())?;
            }

            if let Some(path) = received_plan_msg.payload() {
                if !self.seen_received_global_plan {
                    info!(
                        "received first Nav2 path on /received_global_plan ({} poses)",
                        path.poses.len()
                    );
                    self.seen_received_global_plan = true;
                }
                self.log_path(
                    path,
                    RECEIVED_PLAN_ENTITY,
                    RECEIVED_PLAN_POINTS_ENTITY,
                    received_plan_color(),
                )?;
            }

            if let Some(path) = smoothed_plan_msg.payload() {
                if !self.seen_smoothed_plan {
                    info!(
                        "received first Nav2 path on /plan_smoothed ({} poses)",
                        path.poses.len()
                    );
                    self.seen_smoothed_plan = true;
                }
                self.log_path(
                    path,
                    SMOOTHED_PLAN_ENTITY,
                    SMOOTHED_PLAN_POINTS_ENTITY,
                    smoothed_plan_color(),
                )?;
            }

            if let Some(path) = local_plan_msg.payload() {
                if !self.seen_local_plan {
                    info!(
                        "received first Nav2 path on /local_plan ({} poses)",
                        path.poses.len()
                    );
                    self.seen_local_plan = true;
                }
                self.log_path(
                    path,
                    LOCAL_PLAN_ENTITY,
                    LOCAL_PLAN_POINTS_ENTITY,
                    local_plan_color(),
                )?;
            }

            if let Some(odom) = odom_msg.payload() {
                if !self.seen_odom {
                    info!(
                        "received first Nav2 odometry on /odom ({:.3}, {:.3})",
                        odom.pose.x, odom.pose.y
                    );
                    self.seen_odom = true;
                }
                self.log_odometry(odom)?;
            }

            Ok(())
        }
    }

    impl RerunNav2Viz {
        fn log_path(
            &self,
            path: &NavPathPayload,
            line_entity: &str,
            points_entity: &str,
            color: rerun::Color,
        ) -> CuResult<()> {
            let points: Vec<Vec2D> = path
                .poses
                .iter()
                .map(|pose| Vec2D::new(pose.x, pose.y))
                .collect();

            if points.is_empty() {
                return Ok(());
            }

            self.rec
                .log(
                    points_entity,
                    &Points2D::new(points.clone()).with_colors(vec![color; points.len()]),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log Nav2 path points", e))?;

            self.rec
                .log(
                    line_entity,
                    &LineStrips2D::new(vec![points]).with_colors(vec![color]),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log Nav2 path strip", e))?;

            Ok(())
        }

        fn log_odometry(&mut self, odom: &Odometry2DPayload) -> CuResult<()> {
            if self.trail.len() == TRAIL_CAPACITY {
                self.trail.pop_front();
            }
            self.trail.push_back([odom.pose.x, odom.pose.y]);

            let pose_point = Vec2D::new(odom.pose.x, odom.pose.y);
            let heading_point = Vec2D::new(
                odom.pose.x + odom.pose.yaw_rad.cos() * HEADING_ARROW_LEN_M,
                odom.pose.y + odom.pose.yaw_rad.sin() * HEADING_ARROW_LEN_M,
            );
            let trail_points: Vec<Vec2D> = self
                .trail
                .iter()
                .map(|point| Vec2D::new(point[0], point[1]))
                .collect();

            self.rec
                .log(ROBOT_ENTITY, &Points2D::new(vec![pose_point]))
                .map_err(|e| CuError::new_with_cause("Failed to log robot pose", e))?;

            if trail_points.len() >= 2 {
                self.rec
                    .log(TRAIL_ENTITY, &LineStrips2D::new(vec![trail_points]))
                    .map_err(|e| CuError::new_with_cause("Failed to log odom trail", e))?;
            }

            self.rec
                .log(
                    HEADING_ENTITY,
                    &LineStrips2D::new(vec![vec![pose_point, heading_point]]),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log robot heading", e))?;

            self.rec
                .log(POSITION_X_ENTITY, &Scalars::new([odom.pose.x as f64]))
                .map_err(|e| CuError::new_with_cause("Failed to log x position scalar", e))?;
            self.rec
                .log(POSITION_Y_ENTITY, &Scalars::new([odom.pose.y as f64]))
                .map_err(|e| CuError::new_with_cause("Failed to log y position scalar", e))?;
            self.rec
                .log(YAW_ENTITY, &Scalars::new([odom.pose.yaw_rad as f64]))
                .map_err(|e| CuError::new_with_cause("Failed to log yaw scalar", e))?;
            self.rec
                .log(
                    LINEAR_SPEED_ENTITY,
                    &Scalars::new([odom.linear_velocity_mps as f64]),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log linear speed scalar", e))?;
            self.rec
                .log(
                    ANGULAR_SPEED_ENTITY,
                    &Scalars::new([odom.angular_velocity_rps as f64]),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log angular speed scalar", e))?;

            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const LOG_PATH: &str = "logs/nav2_observer.copper";
const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

fn main() {
    if let Err(error) = drive() {
        eprintln!("cu-nav2-observer failed: {error}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    register_ros2_payload::<payloads::NavPathPayload>();
    register_ros2_payload::<payloads::Odometry2DPayload>();

    ensure_log_dir(Path::new(LOG_PATH))?;

    let copper_ctx = basic_copper_setup(&PathBuf::from(LOG_PATH), SLAB_SIZE, true, None)?;
    let mut app = App::new(
        copper_ctx.clock.clone(),
        copper_ctx.unified_logger.clone(),
        None,
    )?;
    app.start_all_tasks()?;

    println!("observing Nav2 topics via ROS2 bridge; logs -> {LOG_PATH}; press Ctrl-C to stop");

    loop {
        app.run_one_iteration()?;
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn ensure_log_dir(log_path: &Path) -> CuResult<()> {
    if let Some(parent) = log_path.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|e| CuError::new_with_cause("Failed to create log directory", e))?;
    }
    Ok(())
}
