use cu29::prelude::*;
use cu_sensor_payloads::PointCloudSoa;

pub fn fuse_lidar<const LS: usize, const RS: usize, const OS: usize>(
    left: &PointCloudSoa<LS>,
    right: &PointCloudSoa<RS>,
    result: &mut PointCloudSoa<OS>,
) -> CuResult<()> {
    left.iter().chain(right.iter()).for_each(|l| result.push(l));
    Ok(())
}

pub fn fuse_lidar_fast<const LS: usize, const RS: usize, const OS: usize>(
    left: &PointCloudSoa<LS>,
    right: &PointCloudSoa<RS>,
    result: &mut PointCloudSoa<OS>,
) -> CuResult<()> {
    let left_size = left.len();
    let right_size = right.len();

    // favor reading linearity for better prefetching
    result.tov[0..left_size].copy_from_slice(&left.tov[..left_size]);
    result.x[0..left_size].copy_from_slice(&left.x[..left_size]);
    result.y[0..left_size].copy_from_slice(&left.y[..left_size]);
    result.z[0..left_size].copy_from_slice(&left.z[..left_size]);
    result.i[0..left_size].copy_from_slice(&left.i[..left_size]);

    result.tov[left_size..].copy_from_slice(&right.tov[..right_size]);
    result.x[left_size..].copy_from_slice(&right.x[..right_size]);
    result.y[left_size..].copy_from_slice(&right.y[..right_size]);
    result.z[left_size..].copy_from_slice(&right.z[..right_size]);
    result.i[left_size..].copy_from_slice(&right.i[..right_size]);

    result.len = left_size + right_size;

    Ok(())
}

// Both need to be sorted by tov, the result will be sorted by tov
pub fn mergesort_lidar<const LS: usize, const RS: usize, const OS: usize>(
    left: &PointCloudSoa<LS>,
    right: &PointCloudSoa<RS>,
    result: &mut PointCloudSoa<OS>,
) {
    let mut left_index = 0;
    let mut right_index = 0;
    let mut result_index = 0;

    while left_index < left.len() && right_index < right.len() {
        if left.tov[left_index] < right.tov[right_index] {
            result.push(left.get(left_index));
            left_index += 1;
        } else {
            result.push(right.get(right_index));
            right_index += 1;
        }
        result_index += 1;
    }

    while left_index < left.len() {
        result.push(left.get(left_index));
        left_index += 1;
        result_index += 1;
    }

    while right_index < right.len() {
        result.push(right.get(right_index));
        right_index += 1;
        result_index += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_sensor_payloads::PointCloud;
    type Cube = PointCloudSoa<72>;
    type Sphere = PointCloudSoa<561>;
    type Result = PointCloudSoa<{ 72 + 561 }>;
    use rerun::Position3D;

    fn generate_cube() -> Cube {
        let mut cube = Cube::default();
        let steps = 5;

        let edges = [
            // Far face
            [(-1.0f32, -1.0f32, -1.0f32), (1.0f32, -1.0f32, -1.0f32)], // Bottom
            [(-1.0f32, -1.0f32, -1.0f32), (-1.0f32, 1.0f32, -1.0f32)], // Left
            [(1.0f32, -1.0f32, -1.0f32), (1.0f32, 1.0f32, -1.0f32)],   // Right
            [(-1.0f32, 1.0f32, -1.0f32), (1.0f32, 1.0f32, -1.0f32)],   // Top
            // Near face
            [(-1.0f32, -1.0f32, 1.0f32), (1.0f32, -1.0f32, 1.0f32)], // Bottom
            [(-1.0f32, -1.0f32, 1.0f32), (-1.0f32, 1.0f32, 1.0f32)], // Left
            [(1.0f32, -1.0f32, 1.0f32), (1.0f32, 1.0f32, 1.0f32)],   // Right
            [(-1.0f32, 1.0f32, 1.0f32), (1.0f32, 1.0f32, 1.0f32)],   // Top
            // Connecting edges
            [(-1.0f32, -1.0f32, -1.0f32), (-1.0f32, -1.0f32, 1.0f32)],
            [(1.0f32, -1.0f32, -1.0f32), (1.0f32, -1.0f32, 1.0f32)],
            [(-1.0f32, 1.0f32, -1.0f32), (-1.0f32, 1.0f32, 1.0f32)],
            [(1.0f32, 1.0f32, -1.0f32), (1.0f32, 1.0f32, 1.0f32)],
        ];

        let mut ts: CuTime = (72 * 2).into();

        for &[(x1, y1, z1), (x2, y2, z2)] in &edges {
            for step in 0..=steps {
                let t = step as f32 / steps as f32;
                let x = x1 + t * (x2 - x1);
                let y = y1 + t * (y2 - y1);
                let z = z1 + t * (z2 - z1);
                ts -= 2u64.into(); // put it in reverse in time to test the ordering
                cube.push(PointCloud::new(ts, x, y, z, 1.0f32, None));
            }
        }

        cube
    }

    fn generate_sphere() -> Sphere {
        let mut sphere = Sphere::default();
        let num_latitudes = 10;
        let num_longitudes = 50;
        let mut ts: CuTime = 0.into();

        for lat_step in 0..=num_latitudes {
            let theta = std::f32::consts::PI * (lat_step as f32 / num_latitudes as f32); // Latitude angle
            let sin_theta = theta.sin();
            let cos_theta = theta.cos();

            for lon_step in 0..=num_longitudes {
                let phi = 2.0 * std::f32::consts::PI * (lon_step as f32 / num_longitudes as f32); // Longitude angle
                let sin_phi = phi.sin();
                let cos_phi = phi.cos();

                let x = sin_theta * cos_phi;
                let y = sin_theta * sin_phi;
                let z = cos_theta;
                ts += 1u64.into();
                sphere.push(PointCloud::new(ts, x, y, z, 1.0f32, None));
            }
        }

        sphere
    }

    #[test]
    fn test_simple_fusion() {
        let l = generate_cube();
        let r = generate_sphere();
        let mut res = Result::default();
        let rec = rerun::RecordingStreamBuilder::new("Ply Visualizer")
            .spawn()
            .map_err(|e| CuError::new_with_cause("Failed to spawn rerun stream", e))
            .unwrap();
        fuse_lidar(&l, &r, &mut res).unwrap();

        // push res to rerun
        let points: Vec<Position3D> = res
            .iter()
            .map(|p| Position3D::new(p.x.0.value, p.y.0.value, p.z.0.value))
            .collect();

        rec.log("points", &rerun::Points3D::new(points)).unwrap();

        assert_eq!(res.len(), l.len() + r.len());
    }

    #[test]
    fn test_ordering() {
        let mut l = generate_cube();
        l.sort();
        // assert that all tov are in order
        for i in 1..l.len() {
            assert!(l.tov[i] > l.tov[i - 1]);
        }

        let mut s = generate_sphere();
        s.sort();
        // assert that all tov are in order
        for i in 1..s.len() {
            assert!(s.tov[i] > s.tov[i - 1]);
        }
    }
    #[test]
    fn test_mergesort() {
        let mut l = generate_cube();
        l.sort();
        let mut s = generate_sphere();
        s.sort();

        let mut res = Result::default();
        mergesort_lidar(&l, &s, &mut res);
        // asset that all tov are in order
        for i in 1..res.len() {
            assert!(res.tov[i] > res.tov[i - 1]);
        }
    }
}
