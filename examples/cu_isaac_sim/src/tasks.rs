//! Ordinary image processing and stop control; simulated hardware is injected by callbacks.

use cu_sensor_payloads::CuImage;
use cu_sensor_payloads::PointCloudSoaHandle;
use cu29::bincode::Decode;
use cu29::bincode::Encode;
use cu29::pool::CuSharedMemoryBuffer;
use cu29::prelude::*;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f32::AngularVelocity;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

pub const MAX_POINTS: usize = 524_288;
pub type CameraFrame = CuImage<CuSharedMemoryBuffer<u8>>;
pub type LidarFrame = PointCloudSoaHandle<MAX_POINTS>;

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct ImageStats {
    pub mean_rgb: [f32; 3],
}

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct Clearance {
    pub distance: Option<Length>,
}

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct MotorCommand {
    pub left: AngularVelocity,
    pub right: AngularVelocity,
}

#[derive(Reflect)]
pub struct ImageStatistics;
impl Freezable for ImageStatistics {}
impl CuTask for ImageStatistics {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CameraFrame);
    type Output<'m> = output_msg!(ImageStats);

    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        let Some(image) = input.payload() else {
            output.clear_payload();
            output.metadata.set_status("No image");
            return Ok(());
        };
        if image.format.pixel_format != *b"RGB3"
            || !image.format.is_valid()
            || image.format.width == 0
            || image.format.height == 0
        {
            return Err("Expected packed RGB3 image".into());
        }
        let mean_rgb = image.with_plane_bytes(0, |bytes, plane| {
            let mut sums = [0u64; 3];
            for row in bytes.chunks_exact(plane.stride_bytes as usize) {
                for pixel in row[..plane.row_bytes as usize].as_chunks::<3>().0 {
                    for channel in 0..3 {
                        sums[channel] += u64::from(pixel[channel]);
                    }
                }
            }
            let count = (image.format.width * image.format.height) as f32;
            sums.map(|sum| sum as f32 / count)
        })?;
        output.metadata.set_status(format_args!(
            "RGB {:.0}/{:.0}/{:.0}",
            mean_rgb[0], mean_rgb[1], mean_rgb[2]
        ));
        output.set_payload(ImageStats { mean_rgb });
        Ok(())
    }
}

#[derive(Reflect)]
pub struct ForwardClearance;
impl Freezable for ForwardClearance {}
impl CuTask for ForwardClearance {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(LidarFrame);
    type Output<'m> = output_msg!(Clearance);

    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        // The adapter transforms world returns into the lidar frame: +X forward, +Z up.
        let distance = input.payload().and_then(|points| {
            points.with_inner(|cloud| {
                let mut nearest = f32::INFINITY;
                for index in 0..cloud.len {
                    let x = cloud.x[index].get::<meter>();
                    let y = cloud.y[index].get::<meter>();
                    let z = cloud.z[index].get::<meter>();
                    if x.is_finite()
                        && y.is_finite()
                        && z.is_finite()
                        && x > 0.1
                        && y.abs() < 0.08
                        && z.abs() < 0.10
                    {
                        nearest = nearest.min(x);
                    }
                }
                nearest.is_finite().then(|| Length::new::<meter>(nearest))
            })
        });
        if let Some(distance) = distance {
            let meters = distance.get::<meter>();
            // Keep status formatting within the metadata string's inline capacity.
            if meters < 1000.0 {
                output.metadata.set_status(format_args!("{meters:.2} m"));
            } else {
                output.metadata.set_status(format_args!("{meters:.2e} m"));
            }
        } else {
            output.metadata.set_status("No forward return");
        }
        output.set_payload(Clearance { distance });
        Ok(())
    }
}

#[derive(Reflect)]
pub struct StopController {
    stop_distance: Length,
    wheel_speed: AngularVelocity,
}
impl Freezable for StopController {}
impl CuTask for StopController {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(Clearance);
    type Output<'m> = output_msg!(MotorCommand);

    fn new(config: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        let config = config.ok_or(CuError::from("Missing controller config"))?;
        let distance = config
            .get::<f32>("stop_distance_m")?
            .ok_or(CuError::from("Missing stop distance"))?;
        let speed = config
            .get::<f32>("wheel_speed_rad_s")?
            .ok_or(CuError::from("Missing wheel speed"))?;
        if !distance.is_finite()
            || distance <= 0.0
            || !speed.is_finite()
            || !(0.0..=10.0).contains(&speed)
        {
            return Err("Invalid stop controller configuration".into());
        }
        Ok(Self {
            stop_distance: Length::new::<meter>(distance),
            wheel_speed: AngularVelocity::new::<radian_per_second>(speed),
        })
    }

    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        let distance = input.payload().and_then(|reading| reading.distance);
        let clear = distance.is_some_and(|distance| {
            distance.get::<meter>().is_finite() && distance > self.stop_distance
        });
        output.metadata.set_status(if clear {
            "DRIVE"
        } else if distance.is_some_and(|distance| distance.get::<meter>().is_finite()) {
            "STOP obstacle"
        } else {
            "STOP no clearance"
        });
        let speed = if clear {
            self.wheel_speed
        } else {
            AngularVelocity::new::<radian_per_second>(0.0)
        };
        output.set_payload(MotorCommand {
            left: speed,
            right: speed,
        });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_sensor_payloads::PointCloud;
    use cu_sensor_payloads::PointCloudSoa;

    #[test]
    fn test_clearance_status_updates_without_heap_storage() {
        let mut task = ForwardClearance::new(None, ()).unwrap();
        let (ctx, _) = CuContext::new_mock_clock();
        let mut cloud = LidarFrame::from_box(PointCloudSoa::boxed_zeroed());
        let mut output = CuMsg::default();
        for (distance, status) in [
            (Some(2.25), "2.25 m"),
            (None, "No forward return"),
            (Some(1.0e30), "1.00e30 m"),
        ] {
            cloud.with_inner_mut(|cloud| {
                cloud.len = 0;
                if let Some(x) = distance {
                    cloud.push(PointCloud::new(CuTime::default(), x, 0.0, 0.0, 1.0, None));
                }
            });
            let mut input = CuMsg::default();
            input.set_payload(cloud.clone());
            task.process(&ctx, &input, &mut output).unwrap();
            assert_eq!(output.metadata.status_txt.0.as_str(), status);
            assert!(!output.metadata.status_txt.0.is_heap_allocated());
            assert_eq!(
                output.payload().unwrap().distance.is_some(),
                distance.is_some()
            );
        }
    }

    #[test]
    fn test_controller_stops_for_missing_near_and_invalid_clearance() {
        let mut config = ComponentConfig::default();
        config.set("stop_distance_m", 0.65f32);
        config.set("wheel_speed_rad_s", 5.0f32);
        let mut task = StopController::new(Some(&config), ()).unwrap();
        let (ctx, _) = CuContext::new_mock_clock();
        for (distance, expected, status) in [
            (None, 0.0, "STOP no clearance"),
            (Some(0.5), 0.0, "STOP obstacle"),
            (Some(0.65), 0.0, "STOP obstacle"),
            (Some(f32::NAN), 0.0, "STOP no clearance"),
            (Some(f32::INFINITY), 0.0, "STOP no clearance"),
            (Some(2.0), 5.0, "DRIVE"),
        ] {
            let mut input = CuMsg::default();
            input.tov = Tov::Time(CuTime::from_nanos(123));
            input.set_payload(Clearance {
                distance: distance.map(Length::new::<meter>),
            });
            let mut output = CuMsg::default();
            task.process(&ctx, &input, &mut output).unwrap();
            assert_eq!(
                output.payload().unwrap().left.get::<radian_per_second>(),
                expected
            );
            assert_eq!(output.tov, input.tov);
            assert_eq!(output.metadata.status_txt.0.as_str(), status);
            assert!(!output.metadata.status_txt.0.is_heap_allocated());
        }
    }
}
