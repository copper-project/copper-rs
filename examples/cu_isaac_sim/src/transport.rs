//! Bounded metadata exchange and exclusive shared-buffer leases outside the Copper iteration.

use crate::tasks::CameraFrame;
use crate::tasks::LidarFrame;
use crate::tasks::MAX_POINTS;
use cu_sensor_payloads::CuImageBufferFormat;
use cu_sensor_payloads::PointCloud;
use cu_sensor_payloads::PointCloudSoa;
use cu29::clock::CuTimeRange;
use cu29::pool::CuHandle;
use cu29::pool::CuPool;
use cu29::pool::CuSharedMemoryBuffer;
use cu29::pool::CuSharedMemoryHandleDescriptor;
use cu29::pool::CuSharedMemoryPool;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;
use std::io::BufRead;
use std::io::BufReader;
use std::io::Read;
use std::io::Write;
use std::os::unix::net::UnixStream;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

pub const WIDTH: usize = 320;
pub const HEIGHT: usize = 240;
pub const POINT_BYTES: usize = 20;
const SLOTS: usize = 8;
const MAX_METADATA: u64 = 65_536;

pub struct Channel {
    reader: BufReader<UnixStream>,
    writer: UnixStream,
}

impl Channel {
    pub fn new(stream: UnixStream) -> CuResult<Self> {
        // Accepted sockets can inherit the listener's nonblocking mode on macOS.
        // Metadata exchange waits for the simulator, bounded by the timeouts below.
        stream.set_nonblocking(false).map_err(io_error)?;
        stream
            .set_read_timeout(Some(Duration::from_secs(60)))
            .map_err(io_error)?;
        stream
            .set_write_timeout(Some(Duration::from_secs(60)))
            .map_err(io_error)?;
        Ok(Self {
            reader: BufReader::new(stream.try_clone().map_err(io_error)?),
            writer: stream,
        })
    }

    pub fn receive<T: serde::de::DeserializeOwned>(&mut self) -> CuResult<T> {
        let mut line = String::new();
        (&mut self.reader)
            .take(MAX_METADATA + 1)
            .read_line(&mut line)
            .map_err(io_error)?;
        if line.len() as u64 > MAX_METADATA || !line.ends_with('\n') {
            return Err("Metadata overflow or simulator disconnected".into());
        }
        serde_json::from_str(&line)
            .map_err(|e| CuError::new_with_cause("Invalid simulator metadata", e))
    }

    pub fn send(&mut self, message: &impl Serialize) -> CuResult<()> {
        serde_json::to_writer(&mut self.writer, message)
            .map_err(|e| CuError::new_with_cause("Send simulator metadata", e))?;
        self.writer.write_all(b"\n").map_err(io_error)
    }
}

pub fn io_error(error: std::io::Error) -> CuError {
    CuError::new_with_cause("Isaac transport", error)
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Request {
    Hello {
        version: u32,
        observations: u32,
        scenario: serde_json::Value,
    },
    Frame(Frame),
    Finish,
}

#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub struct Frame {
    pub sequence: u64,
    pub simulation_time_ns: u64,
    pub camera_time_ns: u64,
    pub scan_start_ns: u64,
    pub scan_end_ns: u64,
    pub points: usize,
    pub origin_world_m: [f32; 3],
    // Rows are the lidar's +X, +Y, +Z unit vectors in world coordinates.
    pub axes_world: [[f32; 3]; 3],
}

impl Frame {
    pub fn validate(&self, sequence: u64, previous: Option<&Frame>) -> CuResult<()> {
        if self.sequence != sequence
            || self.points == 0
            || self.points > MAX_POINTS
            || self.scan_start_ns > self.scan_end_ns
            || self.scan_end_ns > self.simulation_time_ns
            || self.camera_time_ns > self.simulation_time_ns
            || self.simulation_time_ns - self.camera_time_ns > 100_000_001
            || self.scan_end_ns - self.scan_start_ns > 100_000_001
            || self.simulation_time_ns - self.scan_end_ns > 100_000_001
        {
            return Err(CuError::from(format!(
                "Invalid observation sequence, bounds, or acquisition times: {self:?}"
            )));
        }
        if let Some(previous) = previous {
            if self.simulation_time_ns <= previous.simulation_time_ns
                || self.scan_start_ns <= previous.scan_start_ns
                || self.camera_time_ns <= previous.camera_time_ns
            {
                return Err("Repeated or reversed sensor observation".into());
            }
            if !(80_000_000..=120_000_000)
                .contains(&(self.simulation_time_ns - previous.simulation_time_ns))
                || !(80_000_000..=120_000_000)
                    .contains(&(self.scan_start_ns - previous.scan_start_ns))
            {
                return Err("Missed the 10 Hz observation boundary".into());
            }
        }
        if !self
            .origin_world_m
            .iter()
            .chain(self.axes_world.iter().flatten())
            .all(|v| v.is_finite())
        {
            return Err("Nonfinite lidar pose".into());
        }
        for row in 0..3 {
            for col in 0..3 {
                let dot: f32 = self.axes_world[row]
                    .iter()
                    .zip(self.axes_world[col])
                    .map(|(a, b)| a * b)
                    .sum();
                if (dot - if row == col { 1.0 } else { 0.0 }).abs() > 0.001 {
                    return Err("Lidar axes are not orthonormal".into());
                }
            }
        }
        Ok(())
    }
}

#[derive(Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Response<'a> {
    Lease {
        sequence: u64,
        camera: &'a CuSharedMemoryHandleDescriptor,
        lidar: &'a CuSharedMemoryHandleDescriptor,
        max_points: usize,
        point_bytes: usize,
    },
    Command {
        sequence: u64,
        simulation_time_ns: u64,
        left_rad_s: f32,
        right_rad_s: f32,
        clearance_m: Option<f32>,
        mean_rgb: [f32; 3],
    },
    Finished {
        observations: u32,
        log: &'a Path,
    },
}

pub struct Buffers {
    camera: Arc<CuSharedMemoryPool<u8>>,
    lidar: Arc<CuSharedMemoryPool<u8>>,
    clouds: Vec<LidarFrame>,
}

pub struct Lease {
    pub camera: CuHandle<CuSharedMemoryBuffer<u8>>,
    pub lidar: CuHandle<CuSharedMemoryBuffer<u8>>,
    pub camera_descriptor: CuSharedMemoryHandleDescriptor,
    pub lidar_descriptor: CuSharedMemoryHandleDescriptor,
}

impl Buffers {
    pub fn new(session: &Path) -> CuResult<Self> {
        let camera = CuSharedMemoryPool::new("isaac-camera", SLOTS, WIDTH * HEIGHT * 3)?;
        let lidar = CuSharedMemoryPool::new("isaac-lidar", 2, MAX_POINTS * POINT_BYTES)?;
        let clouds = (0..SLOTS)
            .map(|_| LidarFrame::from_box(PointCloudSoa::boxed_zeroed()))
            .collect();
        let buffers = Self {
            camera,
            lidar,
            clouds,
        };
        // Pool files honor TMPDIR. Reject an incorrectly launched server before giving out paths
        // that would be invisible inside Docker's same-path session mount.
        let lease = buffers.acquire()?;
        if !Path::new(&lease.camera_descriptor.path).starts_with(session)
            || !Path::new(&lease.lidar_descriptor.path).starts_with(session)
        {
            return Err("Launch with TMPDIR pointing to the mounted session directory".into());
        }
        Ok(buffers)
    }

    pub fn acquire(&self) -> CuResult<Lease> {
        let camera = self.camera.acquire().ok_or(CuError::from(
            "Camera leases exhausted; retained Copper handles prevent reuse",
        ))?;
        let lidar = self
            .lidar
            .acquire()
            .ok_or(CuError::from("Lidar leases exhausted"))?;
        let camera_descriptor = camera
            .with_inner(|buffer| buffer.as_ref().descriptor())
            .ok_or(CuError::from("Missing camera descriptor"))?;
        let lidar_descriptor = lidar
            .with_inner(|buffer| buffer.as_ref().descriptor())
            .ok_or(CuError::from("Missing lidar descriptor"))?;
        Ok(Lease {
            camera,
            lidar,
            camera_descriptor,
            lidar_descriptor,
        })
    }

    pub fn observation(
        &mut self,
        lease: &Lease,
        frame: &Frame,
    ) -> CuResult<(CameraFrame, LidarFrame)> {
        let cloud = self
            .clouds
            .iter_mut()
            .find(|cloud| cloud.handle.is_unique())
            .ok_or(CuError::from(
                "Point-cloud slots exhausted; retained Copper handles prevent reuse",
            ))?;
        // Publication on the socket transfers exclusive write ownership to Rust. Python cannot
        // write these descriptors again until a future lease grants them anew.
        lease.lidar.with_inner(|bytes| -> CuResult<()> {
            cloud.with_inner_mut(|cloud| -> CuResult<()> {
                cloud.len = 0;
                for record in bytes[..frame.points * POINT_BYTES]
                    .as_chunks::<POINT_BYTES>()
                    .0
                {
                    let scalar = |offset| {
                        f32::from_le_bytes(
                            record[offset..offset + 4]
                                .try_into()
                                .expect("fixed record width"),
                        )
                    };
                    let world = [scalar(0), scalar(4), scalar(8)];
                    let intensity = scalar(12);
                    let offset =
                        u32::from_le_bytes(record[16..20].try_into().expect("fixed record width"));
                    let timestamp = frame
                        .scan_start_ns
                        .checked_add(u64::from(offset))
                        .ok_or(CuError::from("Point timestamp overflow"))?;
                    if !world.iter().all(|v| v.is_finite())
                        || !intensity.is_finite()
                        || timestamp > frame.scan_end_ns
                    {
                        return Err(
                            "Invalid point coordinates, intensity, or acquisition time".into()
                        );
                    }
                    let delta = std::array::from_fn::<_, 3, _>(|axis| {
                        world[axis] - frame.origin_world_m[axis]
                    });
                    let local = frame
                        .axes_world
                        .map(|axis| axis.iter().zip(delta).map(|(a, b)| a * b).sum::<f32>());
                    let point = PointCloud::new(
                        CuTime::from_nanos(timestamp),
                        local[0],
                        local[1],
                        local[2],
                        intensity * 100.0,
                        Some(0),
                    );
                    cloud.push(point);
                }
                Ok(())
            })
        })?;
        let mut image = CameraFrame::new(
            CuImageBufferFormat {
                width: WIDTH as u32,
                height: HEIGHT as u32,
                stride: (WIDTH * 3) as u32,
                pixel_format: *b"RGB3",
            },
            lease.camera.clone(),
        );
        image.seq = frame.sequence;
        Ok((image, cloud.clone()))
    }
}

pub fn lidar_tov(frame: &Frame) -> Tov {
    Tov::Range(CuTimeRange {
        start: CuTime::from_nanos(frame.scan_start_ns),
        end: CuTime::from_nanos(frame.scan_end_ns),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame() -> Frame {
        Frame {
            sequence: 0,
            simulation_time_ns: 200_000_000,
            camera_time_ns: 200_000_000,
            scan_start_ns: 100_000_000,
            scan_end_ns: 199_999_999,
            points: 1,
            origin_world_m: [0.0; 3],
            axes_world: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    #[test]
    fn test_rejects_repeated_time_sequence_overflow_and_bad_pose() {
        let good = frame();
        good.validate(0, None).unwrap();
        assert!(good.validate(1, None).is_err());
        assert!(good.validate(0, Some(&good)).is_err());
        let mut bad = frame();
        bad.points = MAX_POINTS + 1;
        assert!(bad.validate(0, None).is_err());
        bad = frame();
        bad.axes_world[0][0] = 2.0;
        assert!(bad.validate(0, None).is_err());
        bad = frame();
        bad.scan_end_ns = bad.simulation_time_ns + 1;
        assert!(bad.validate(0, None).is_err());
    }

    #[test]
    fn test_camera_slot_is_not_reused_while_a_logger_handle_is_retained() {
        let pool = CuSharedMemoryPool::<u8>::new("isaac-lease-test", 1, 32).unwrap();
        let lease = pool.acquire().unwrap();
        let retained = lease.clone();
        drop(lease); // Sending a command/acknowledgement does not release this slot.
        assert!(pool.acquire().is_none());
        drop(retained);
        assert!(pool.acquire().is_some());
    }

    #[test]
    fn test_nonblocking_stream_waits_for_complete_metadata() {
        let (server, mut client) = UnixStream::pair().unwrap();
        server.set_nonblocking(true).unwrap();
        let mut channel = Channel::new(server).unwrap();
        let peer = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            client.write_all(b"{\"type\":").unwrap();
            std::thread::sleep(Duration::from_millis(50));
            client.write_all(b"\"finish\"}\n").unwrap();
            let mut reply = String::new();
            BufReader::new(client).read_line(&mut reply).unwrap();
            assert_eq!(reply, "{\"ok\":true}\n");
        });
        assert!(matches!(channel.receive().unwrap(), Request::Finish));
        channel.send(&serde_json::json!({ "ok": true })).unwrap();
        peer.join().unwrap();
    }

    #[test]
    fn test_disconnect_and_metadata_limit() {
        let (server, mut client) = UnixStream::pair().unwrap();
        client
            .write_all(&vec![b'x'; MAX_METADATA as usize + 1])
            .unwrap();
        assert!(Channel::new(server).unwrap().receive::<Request>().is_err());
        let (server, client) = UnixStream::pair().unwrap();
        drop(client);
        assert!(Channel::new(server).unwrap().receive::<Request>().is_err());
    }
}
