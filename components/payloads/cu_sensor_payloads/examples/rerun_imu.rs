//! Record sensor-frame directions for inspection in Rerun.
use cu_rerun::rerun::RecordingStreamBuilder;
use cu_sensor_payloads::{ImuPayload, ImuWithMagPayload, MagnetometerPayload};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "imu.rrd".into());
    let rec = RecordingStreamBuilder::new("Copper IMU").save(path)?;
    let imu = ImuPayload::from_raw([3.0, 1.0, 9.3], [0.1, 0.3, 0.2], 25.0);
    let mag = MagnetometerPayload::from_raw([20.0, 5.0, -40.0]);
    rec.log("combined", &ImuWithMagPayload::new(imu, Some(mag)))?;
    rec.flush_blocking()?;
    Ok(())
}
