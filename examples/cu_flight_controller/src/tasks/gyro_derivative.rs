//! First-order low-pass on the measured roll/pitch gyro derivative.
pub(super) struct GyroDerivative {
    tau: f32,
    gain: f32,
    pub(super) previous: Option<(u64, [f32; 2])>,
    pub(super) output: [f32; 2],
}

impl GyroDerivative {
    pub(super) fn new(cutoff_hz: f32, gain: f32) -> Self {
        Self {
            tau: 1.0 / (core::f32::consts::TAU * cutoff_hz),
            gain,
            previous: None,
            output: [0.0; 2],
        }
    }

    pub(super) fn update(&mut self, timestamp_ns: u64, gyro: [f32; 2]) -> [f32; 2] {
        if let Some((last_ns, last)) = self.previous {
            if timestamp_ns <= last_ns {
                return self.output;
            }
            let dt = (timestamp_ns - last_ns) as f32 * 1e-9;
            // Backward Euler: D = (tau * D_prev - kd * delta_gyro) / (tau + dt).
            let inverse = 1.0 / (self.tau + dt);
            for axis in 0..2 {
                self.output[axis] = (self.tau * self.output[axis]
                    - self.gain * (gyro[axis] - last[axis]))
                    * inverse;
            }
        }
        self.previous = Some((timestamp_ns, gyro));
        self.output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn damping_tracks_a_ramp_with_irregular_sample_times() {
        let mut filter = GyroDerivative::new(50.0, 0.00054);
        let mut time = 1_000_000_000;
        let mut gyro = [0.3, -0.4];
        assert_eq!(filter.update(time, gyro), [0.0; 2]);
        for dt in [700_000, 1_100_000, 5_000_000].into_iter().cycle().take(90) {
            time += dt;
            gyro[0] += 100.0 * dt as f32 * 1e-9;
            gyro[1] -= 50.0 * dt as f32 * 1e-9;
            filter.update(time, gyro);
        }
        let result = filter.output;
        assert!((result[0] + 0.054).abs() < 1e-5);
        assert!((result[1] - 0.027).abs() < 1e-5);
        assert_eq!(filter.update(time, [99.0; 2]), result);
        assert_eq!(filter.update(time - 1, [99.0; 2]), result);
        assert_eq!(filter.previous, Some((time, gyro)));
    }
}
