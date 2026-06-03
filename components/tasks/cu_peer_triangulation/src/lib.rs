#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(not(feature = "std"))]
use alloc::{string::String, vec::Vec};

use bincode::{Decode, Encode};
use cu_sensor_payloads::{PeerRangeSample, PeerRangeSnapshot, RangePeerId};
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize};

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct LocalPosition3d {
    pub x: Length,
    pub y: Length,
    pub z: Length,
    pub rms_residual: Length,
}

impl LocalPosition3d {
    pub fn from_meters(x_m: f32, y_m: f32, z_m: f32, rms_residual_m: f32) -> Self {
        Self {
            x: Length::new::<meter>(x_m),
            y: Length::new::<meter>(y_m),
            z: Length::new::<meter>(z_m),
            rms_residual: Length::new::<meter>(rms_residual_m),
        }
    }
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct PeerAnchor3d {
    pub peer_id: RangePeerId,
    pub x: Length,
    pub y: Length,
    pub z: Length,
}

impl PeerAnchor3d {
    pub fn from_meters(peer_id: RangePeerId, x_m: f32, y_m: f32, z_m: f32) -> Self {
        Self {
            peer_id,
            x: Length::new::<meter>(x_m),
            y: Length::new::<meter>(y_m),
            z: Length::new::<meter>(z_m),
        }
    }

    fn x_m(self) -> f32 {
        self.x.get::<meter>()
    }

    fn y_m(self) -> f32 {
        self.y.get::<meter>()
    }

    fn z_m(self) -> f32 {
        self.z.get::<meter>()
    }
}

#[derive(Debug, Deserialize)]
struct AnchorConfig {
    peer_id: String,
    x_m: f32,
    y_m: f32,
    z_m: f32,
}

#[derive(Reflect)]
pub struct PeerTriangulation3dTask<const SNAPSHOT_N: usize, const ANCHOR_N: usize> {
    anchors: [Option<PeerAnchor3d>; ANCHOR_N],
    len: usize,
    max_rms_residual: Option<Length>,
}

impl<const SNAPSHOT_N: usize, const ANCHOR_N: usize> Freezable
    for PeerTriangulation3dTask<SNAPSHOT_N, ANCHOR_N>
{
}

impl<const SNAPSHOT_N: usize, const ANCHOR_N: usize> PeerTriangulation3dTask<SNAPSHOT_N, ANCHOR_N> {
    pub fn new_with_anchors(
        anchors: &[PeerAnchor3d],
        max_rms_residual: Option<Length>,
    ) -> Result<Self, PeerTriangulationError> {
        if anchors.len() > ANCHOR_N {
            return Err(PeerTriangulationError::TooManyAnchors {
                len: anchors.len(),
                capacity: ANCHOR_N,
            });
        }

        let mut task = Self {
            anchors: [None; ANCHOR_N],
            len: anchors.len(),
            max_rms_residual,
        };
        for (slot, anchor) in task.anchors.iter_mut().zip(anchors) {
            *slot = Some(*anchor);
        }

        Ok(task)
    }

    pub fn estimate(
        &self,
        snapshot: &PeerRangeSnapshot<SNAPSHOT_N>,
    ) -> Result<Option<LocalPosition3d>, PeerTriangulationError> {
        let mut matched = [None; SNAPSHOT_N];
        let mut matched_len = 0;

        for sample in snapshot.samples() {
            if let Some(anchor) = self.anchor_for(sample) {
                matched[matched_len] = Some((*sample, anchor));
                matched_len += 1;
            }
        }

        if matched_len < 4 {
            return Ok(None);
        }

        let Some((reference_sample, reference_anchor)) = matched[0] else {
            return Ok(None);
        };
        let x0 = reference_anchor.x_m();
        let y0 = reference_anchor.y_m();
        let z0 = reference_anchor.z_m();
        let r0 = reference_sample.observation.distance.get::<meter>();

        let mut ata = [[0.0_f32; 3]; 3];
        let mut atb = [0.0_f32; 3];

        for matched in matched[1..matched_len].iter().flatten() {
            let (sample, anchor) = *matched;
            let xi = anchor.x_m();
            let yi = anchor.y_m();
            let zi = anchor.z_m();
            let ri = sample.observation.distance.get::<meter>();
            let row = [2.0 * (xi - x0), 2.0 * (yi - y0), 2.0 * (zi - z0)];
            let b =
                (r0 * r0 - ri * ri) - (x0 * x0 + y0 * y0 + z0 * z0) + (xi * xi + yi * yi + zi * zi);

            for i in 0..3 {
                atb[i] += row[i] * b;
                for j in 0..3 {
                    ata[i][j] += row[i] * row[j];
                }
            }
        }

        let Some([x, y, z]) = solve_3x3(ata, atb) else {
            return Err(PeerTriangulationError::DegenerateGeometry);
        };
        let rms_residual = self.rms_residual_m(&matched[..matched_len], x, y, z);

        if let Some(max_rms_residual) = self.max_rms_residual
            && rms_residual > max_rms_residual.get::<meter>()
        {
            return Ok(None);
        }

        Ok(Some(LocalPosition3d::from_meters(x, y, z, rms_residual)))
    }

    fn anchor_for(&self, sample: &PeerRangeSample) -> Option<PeerAnchor3d> {
        self.anchors[..self.len]
            .iter()
            .flatten()
            .copied()
            .find(|anchor| anchor.peer_id == sample.observation.peer_id)
    }

    fn rms_residual_m(
        &self,
        matched: &[Option<(PeerRangeSample, PeerAnchor3d)>],
        x: f32,
        y: f32,
        z: f32,
    ) -> f32 {
        let mut sum_sq = 0.0_f32;
        let mut count = 0_u32;

        for (sample, anchor) in matched.iter().flatten() {
            let dx = x - anchor.x_m();
            let dy = y - anchor.y_m();
            let dz = z - anchor.z_m();
            let expected = libm::sqrtf(dx * dx + dy * dy + dz * dz);
            let residual = expected - sample.observation.distance.get::<meter>();
            sum_sq += residual * residual;
            count += 1;
        }

        libm::sqrtf(sum_sq / count as f32)
    }
}

impl<const SNAPSHOT_N: usize, const ANCHOR_N: usize> CuTask
    for PeerTriangulation3dTask<SNAPSHOT_N, ANCHOR_N>
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!(PeerRangeSnapshot<SNAPSHOT_N>);
    type Output<'m> = output_msg!(LocalPosition3d);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or("peer triangulation task requires config")?;
        let anchors = config
            .get_value::<Vec<AnchorConfig>>("anchors")?
            .ok_or("peer triangulation task requires anchors")?;
        let mut parsed = [PeerAnchor3d::default(); ANCHOR_N];
        let anchor_len = anchors.len();
        if anchor_len > ANCHOR_N {
            return Err(CuError::from(
                "peer triangulation config has too many anchors",
            ));
        }
        for (slot, anchor) in parsed.iter_mut().zip(anchors) {
            *slot = PeerAnchor3d::from_meters(
                RangePeerId::new(anchor.peer_id.as_str())
                    .map_err(|_| CuError::from("peer triangulation config has invalid peer id"))?,
                anchor.x_m,
                anchor.y_m,
                anchor.z_m,
            );
        }

        let max_rms_residual = config
            .get::<f64>("max_rms_residual_m")?
            .map(|value| Length::new::<meter>(value as f32));

        Self::new_with_anchors(&parsed[..anchor_len], max_rms_residual)
            .map_err(|_| CuError::from("peer triangulation config has too many anchors"))
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(snapshot) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        match self
            .estimate(snapshot)
            .map_err(|_| CuError::from("peer triangulation geometry is degenerate"))?
        {
            Some(position) => {
                output.tov = input.tov;
                output.set_payload(position);
            }
            None => output.clear_payload(),
        }

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PeerTriangulationError {
    TooManyAnchors { len: usize, capacity: usize },
    DegenerateGeometry,
}

impl core::fmt::Display for PeerTriangulationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooManyAnchors { len, capacity } => {
                write!(f, "{len} anchors exceed capacity {capacity}")
            }
            Self::DegenerateGeometry => write!(f, "peer range geometry is degenerate"),
        }
    }
}

impl core::error::Error for PeerTriangulationError {}

fn solve_3x3(a: [[f32; 3]; 3], b: [f32; 3]) -> Option<[f32; 3]> {
    let det = determinant_3x3(a);
    if det.abs() <= f32::EPSILON {
        return None;
    }

    let mut ax = a;
    ax[0][0] = b[0];
    ax[1][0] = b[1];
    ax[2][0] = b[2];

    let mut ay = a;
    ay[0][1] = b[0];
    ay[1][1] = b[1];
    ay[2][1] = b[2];

    let mut az = a;
    az[0][2] = b[0];
    az[1][2] = b[1];
    az[2][2] = b[2];

    Some([
        determinant_3x3(ax) / det,
        determinant_3x3(ay) / det,
        determinant_3x3(az) / det,
    ])
}

fn determinant_3x3(a: [[f32; 3]; 3]) -> f32 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_sensor_payloads::{PeerRangeObservation, RangePeerId};
    use cu29::units::si::length::meter;

    fn anchor(peer_id: &str, x: f32, y: f32, z: f32) -> PeerAnchor3d {
        PeerAnchor3d::from_meters(RangePeerId::new(peer_id).unwrap(), x, y, z)
    }

    fn sample(peer_id: &str, meters: f32) -> PeerRangeSample {
        PeerRangeSample::new(
            CuTime::from_nanos(1),
            PeerRangeObservation::from_meters(RangePeerId::new(peer_id).unwrap(), meters, None),
        )
    }

    #[test]
    fn estimates_exact_position_from_four_ranges() {
        let task = PeerTriangulation3dTask::<5, 5>::new_with_anchors(
            &[
                anchor("A", 0.0, 0.0, 0.0),
                anchor("B", 4.0, 0.0, 0.0),
                anchor("C", 0.0, 3.0, 0.0),
                anchor("D", 0.0, 0.0, 4.0),
            ],
            None,
        )
        .unwrap();
        let mut snapshot = PeerRangeSnapshot::<5>::new();
        snapshot.push(sample("A", 2.449_489_8)).unwrap();
        snapshot.push(sample("B", 2.449_489_8)).unwrap();
        snapshot.push(sample("C", 3.0)).unwrap();
        snapshot.push(sample("D", 3.741_657_5)).unwrap();

        let position = task.estimate(&snapshot).unwrap().unwrap();

        assert!((position.x.get::<meter>() - 2.0).abs() < 0.001);
        assert!((position.y.get::<meter>() - 1.0).abs() < 0.001);
        assert!((position.z.get::<meter>() - 1.0).abs() < 0.001);
        assert!(position.rms_residual.get::<meter>() < 0.001);
    }

    #[test]
    fn returns_none_when_too_few_ranges_match_anchors() {
        let task = PeerTriangulation3dTask::<5, 5>::new_with_anchors(
            &[
                anchor("A", 0.0, 0.0, 0.0),
                anchor("B", 4.0, 0.0, 0.0),
                anchor("C", 0.0, 3.0, 0.0),
                anchor("D", 0.0, 0.0, 4.0),
            ],
            None,
        )
        .unwrap();
        let mut snapshot = PeerRangeSnapshot::<5>::new();
        snapshot.push(sample("A", 2.0)).unwrap();
        snapshot.push(sample("B", 2.0)).unwrap();
        snapshot.push(sample("C", 2.0)).unwrap();

        assert!(task.estimate(&snapshot).unwrap().is_none());
    }

    #[test]
    fn rejects_coplanar_anchor_geometry() {
        let task = PeerTriangulation3dTask::<5, 5>::new_with_anchors(
            &[
                anchor("A", 0.0, 0.0, 0.0),
                anchor("B", 1.0, 0.0, 0.0),
                anchor("C", 0.0, 1.0, 0.0),
                anchor("D", 1.0, 1.0, 0.0),
            ],
            None,
        )
        .unwrap();
        let mut snapshot = PeerRangeSnapshot::<5>::new();
        snapshot.push(sample("A", 1.0)).unwrap();
        snapshot.push(sample("B", 1.0)).unwrap();
        snapshot.push(sample("C", 1.0)).unwrap();
        snapshot.push(sample("D", 1.0)).unwrap();

        assert_eq!(
            task.estimate(&snapshot),
            Err(PeerTriangulationError::DegenerateGeometry)
        );
    }

    #[test]
    fn residual_gate_suppresses_noisy_estimate() {
        let task = PeerTriangulation3dTask::<5, 5>::new_with_anchors(
            &[
                anchor("A", 0.0, 0.0, 0.0),
                anchor("B", 4.0, 0.0, 0.0),
                anchor("C", 0.0, 3.0, 0.0),
                anchor("D", 0.0, 0.0, 4.0),
            ],
            Some(Length::new::<meter>(0.01)),
        )
        .unwrap();
        let mut snapshot = PeerRangeSnapshot::<5>::new();
        snapshot.push(sample("A", 2.449_489_8)).unwrap();
        snapshot.push(sample("B", 2.449_489_8)).unwrap();
        snapshot.push(sample("C", 3.0)).unwrap();
        snapshot.push(sample("D", 4.5)).unwrap();

        assert!(task.estimate(&snapshot).unwrap().is_none());
    }
}
