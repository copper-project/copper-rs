//! Tensor operations needed by AnyNet that Candle does not provide directly.

use candle_core::{DType, IndexOp, Result, Tensor};
use candle_nn::{Conv2d, Conv2dConfig, Module};

/// A 3×3×3 convolution with unit stride and one-cell zero padding.
///
/// Candle 0.11 has no native 3D convolution. Each kernel-depth slice is a
/// regular 2D convolution and the three neighboring results are accumulated.
#[derive(Debug)]
pub(crate) struct Conv3d {
    depth_slices: [Conv2d; 3],
}

impl Conv3d {
    pub(crate) fn new(weight: Tensor) -> Result<Self> {
        let config = Conv2dConfig {
            padding: 1,
            ..Default::default()
        };
        let depth_slices = [
            Conv2d::new(weight.i((.., .., 0, .., ..))?, None, config),
            Conv2d::new(weight.i((.., .., 1, .., ..))?, None, config),
            Conv2d::new(weight.i((.., .., 2, .., ..))?, None, config),
        ];
        Ok(Self { depth_slices })
    }

    pub(crate) fn forward(&self, input: &Tensor) -> Result<Tensor> {
        let (_batch, _channels, depth, _height, _width) = input.dims5()?;
        let mut planes = Vec::with_capacity(depth);
        for output_depth in 0..depth {
            let mut output = None;
            for (kernel_depth, conv) in self.depth_slices.iter().enumerate() {
                let input_depth = output_depth as isize + kernel_depth as isize - 1;
                if !(0..depth as isize).contains(&input_depth) {
                    continue;
                }
                let plane = input.i((.., .., input_depth as usize, .., ..))?;
                let contribution = conv.forward(&plane)?;
                output = Some(match output {
                    None => contribution,
                    Some(output) => (output + contribution)?,
                });
            }
            let Some(output) = output else {
                candle_core::bail!("conv3d cannot operate on an empty depth dimension");
            };
            planes.push(output);
        }
        Tensor::stack(&planes, 2)
    }
}

/// Samples `input` at the horizontal pixel coordinate `x - disparity`.
///
/// This is the horizontal-only equivalent of PyTorch `grid_sample` with
/// bilinear interpolation, zero padding, and the `align_corners=true`
/// coordinate convention used by the AnyNet reference implementation.
pub(crate) fn warp_horizontal(input: &Tensor, disparity: &Tensor) -> Result<Tensor> {
    let input = input.contiguous()?;
    let (batch, channels, height, width) = input.dims4()?;
    let (disp_batch, disp_channels, disp_height, disp_width) = disparity.dims4()?;
    if (disp_batch, disp_channels, disp_height, disp_width) != (batch, 1, height, width) {
        candle_core::bail!(
            "warp disparity shape {:?} does not match input shape {:?}",
            disparity.shape(),
            input.shape()
        );
    }

    let x = Tensor::arange(0f32, width as f32, input.device())?
        .reshape((1, 1, 1, width))?
        .broadcast_as((batch, 1, height, width))?;
    let source = (&x - disparity)?;
    let lower = source.floor()?;
    let fraction = (&source - &lower)?;
    let upper = (&lower + 1.0)?;

    let sample = |coordinate: &Tensor| -> Result<Tensor> {
        let valid = coordinate
            .ge(0.0)?
            .broadcast_mul(&coordinate.le((width - 1) as f64)?)?
            .to_dtype(input.dtype())?
            .broadcast_as((batch, channels, height, width))?;
        let indices = coordinate
            .clamp(0.0, (width - 1) as f64)?
            .to_dtype(DType::I64)?
            .broadcast_as((batch, channels, height, width))?;
        input
            .gather(&indices.contiguous()?, 3)?
            .broadcast_mul(&valid)
    };

    let lower_value = sample(&lower)?;
    let upper_value = sample(&upper)?;
    let lower_weight = fraction
        .affine(-1.0, 1.0)?
        .broadcast_as((batch, channels, height, width))?;
    let upper_weight = fraction.broadcast_as((batch, channels, height, width))?;
    lower_value.broadcast_mul(&lower_weight)? + upper_value.broadcast_mul(&upper_weight)?
}

#[cfg(test)]
mod tests {
    use super::*;
    use candle_core::Device;

    #[test]
    fn conv3d_matches_a_naive_zero_padded_convolution() -> Result<()> {
        let device = Device::Cpu;
        let input_values = (0..18).map(|value| value as f32).collect::<Vec<_>>();
        let input = Tensor::from_vec(input_values.clone(), (1, 1, 2, 3, 3), &device)?;
        let weight = Tensor::ones((1, 1, 3, 3, 3), DType::F32, &device)?;
        let actual = Conv3d::new(weight)?
            .forward(&input)?
            .flatten_all()?
            .to_vec1::<f32>()?;

        let mut expected = vec![0.0; input_values.len()];
        for z in 0..2isize {
            for y in 0..3isize {
                for x in 0..3isize {
                    let mut sum = 0.0;
                    for dz in -1..=1 {
                        for dy in -1..=1 {
                            for dx in -1..=1 {
                                let (input_z, input_y, input_x) = (z + dz, y + dy, x + dx);
                                if (0..2).contains(&input_z)
                                    && (0..3).contains(&input_y)
                                    && (0..3).contains(&input_x)
                                {
                                    sum += input_values
                                        [(input_z * 9 + input_y * 3 + input_x) as usize];
                                }
                            }
                        }
                    }
                    expected[(z * 9 + y * 3 + x) as usize] = sum;
                }
            }
        }
        assert_eq!(actual, expected);
        Ok(())
    }

    #[test]
    fn warp_horizontal_interpolates_and_zero_pads_each_neighbor() -> Result<()> {
        let device = Device::Cpu;
        let input = Tensor::new(&[[[[10f32, 20.0, 30.0, 40.0]]]], &device)?;
        let disparity = Tensor::new(&[[[[0.25f32, 0.5, -0.5, -0.25]]]], &device)?;
        let actual = warp_horizontal(&input, &disparity)?
            .flatten_all()?
            .to_vec1::<f32>()?;
        let expected = [7.5, 15.0, 35.0, 30.0];
        for (actual, expected) in actual.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-6, "{actual} != {expected}");
        }
        Ok(())
    }
}
