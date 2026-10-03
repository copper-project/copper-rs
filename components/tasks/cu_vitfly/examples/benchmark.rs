use std::time::Instant;

use candle_core::{Device, Tensor};
use cu_vitfly::VitFly;

fn main() -> candle_core::Result<()> {
    let use_cuda = std::env::args().any(|argument| argument == "--cuda");
    let device = if use_cuda {
        Device::new_cuda(0)?
    } else {
        Device::Cpu
    };
    let model = VitFly::load(&device)?;
    let depth = Tensor::from_vec(vec![0.4_f32; 60 * 90], (1, 1, 60, 90), &device)?;
    let desired_velocity = Tensor::new(&[[4.0_f32]], &device)?;
    let attitude = Tensor::new(&[[1.0_f32, 0.0, 0.0, 0.0]], &device)?;
    let mut state = None;

    for _ in 0..10 {
        let (_, next_state) =
            model.forward(&depth, &desired_velocity, &attitude, state.as_ref())?;
        state = Some(next_state);
        device.synchronize()?;
    }

    let mut measurements = Vec::with_capacity(100);
    for _ in 0..100 {
        let started = Instant::now();
        let (_, next_state) =
            model.forward(&depth, &desired_velocity, &attitude, state.as_ref())?;
        device.synchronize()?;
        measurements.push(started.elapsed().as_secs_f64() * 1_000.0);
        state = Some(next_state);
    }
    measurements.sort_by(f64::total_cmp);
    let mean = measurements.iter().sum::<f64>() / measurements.len() as f64;
    println!(
        "backend={} frames={} mean_ms={mean:.3} median_ms={:.3} p95_ms={:.3}",
        if use_cuda { "cuda" } else { "cpu" },
        measurements.len(),
        measurements[measurements.len() / 2],
        measurements[measurements.len() * 95 / 100],
    );
    Ok(())
}
