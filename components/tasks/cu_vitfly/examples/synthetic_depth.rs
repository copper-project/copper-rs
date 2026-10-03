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

    let mut depth = vec![0.8_f32; 60 * 90];
    for row in 20..40 {
        for column in 35..55 {
            depth[row * 90 + column] = 0.16;
        }
    }
    let depth = Tensor::from_vec(depth, (1, 1, 60, 90), &device)?;
    let desired_velocity = Tensor::new(&[[4.0_f32]], &device)?;
    let attitude = Tensor::new(&[[1.0_f32, 0.0, 0.0, 0.0]], &device)?;

    let (prediction, state) = model.forward(&depth, &desired_velocity, &attitude, None)?;
    println!("prediction: {:?}", prediction.to_vec2::<f32>()?);

    let (next_prediction, _) = model.forward(&depth, &desired_velocity, &attitude, Some(&state))?;
    println!("next prediction: {:?}", next_prediction.to_vec2::<f32>()?);
    Ok(())
}
