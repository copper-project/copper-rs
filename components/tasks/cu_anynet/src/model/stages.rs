//! AnyNet cost volumes, 3D regularization, and disparity regression stages.

use super::ops::{Conv3d, warp_horizontal};
use super::unet::FeatureExtractor;
use candle_core::{DType, Result, Tensor};
use candle_nn::{BatchNorm, ModuleT, VarBuilder, batch_norm};

const INITIAL_CHANNELS: usize = 1;
const FEATURE_BLOCKS: usize = 2;
const VOLUME_LAYERS: usize = 4;
const VOLUME_CHANNELS: usize = 4;
const GROWTH_RATES: [usize; 3] = [4, 1, 1];

#[derive(Debug)]
struct PreConv3d {
    batch_norm: BatchNorm,
    conv: Conv3d,
}

impl PreConv3d {
    fn load(vb: VarBuilder, input_channels: usize, output_channels: usize) -> Result<Self> {
        let batch_norm = batch_norm(input_channels, 1e-5, vb.pp("0"))?;
        let weight = vb
            .pp("2")
            .get((output_channels, input_channels, 3, 3, 3), "weight")?;
        Ok(Self {
            batch_norm,
            conv: Conv3d::new(weight)?,
        })
    }

    fn forward(&self, input: &Tensor) -> Result<Tensor> {
        let input = self.batch_norm.forward_t(input, false)?.relu()?;
        self.conv.forward(&input)
    }
}

#[derive(Debug)]
struct VolumePostprocess {
    layers: Vec<PreConv3d>,
}

impl VolumePostprocess {
    fn load(vb: VarBuilder, channels: usize) -> Result<Self> {
        let total_layers = VOLUME_LAYERS + 2;
        let mut layers = Vec::with_capacity(total_layers);
        for index in 0..total_layers {
            let input_channels = if index == 0 { 1 } else { channels };
            let output_channels = if index + 1 == total_layers {
                1
            } else {
                channels
            };
            layers.push(PreConv3d::load(
                vb.pp(index),
                input_channels,
                output_channels,
            )?);
        }
        Ok(Self { layers })
    }

    fn forward(&self, cost: &Tensor) -> Result<Tensor> {
        let mut cost = cost.unsqueeze(1)?;
        for layer in &self.layers {
            cost = layer.forward(&cost)?;
        }
        cost.squeeze(1)
    }
}

/// The first three inference stages of AnyNet.
#[derive(Debug)]
pub struct AnyNet {
    feature_extractor: FeatureExtractor,
    volume_postprocess: [VolumePostprocess; 3],
    max_disparities: [usize; 3],
}

impl AnyNet {
    pub fn load(vb: VarBuilder, max_disparity: usize) -> Result<Self> {
        if max_disparity == 0 || !max_disparity.is_multiple_of(16) {
            candle_core::bail!("AnyNet max disparity must be a non-zero multiple of 16");
        }
        let feature_extractor = FeatureExtractor::load(
            vb.pp("feature_extraction"),
            INITIAL_CHANNELS,
            FEATURE_BLOCKS,
        )?;
        let volume_postprocess = [
            VolumePostprocess::load(
                vb.pp("volume_postprocess.0"),
                VOLUME_CHANNELS * GROWTH_RATES[0],
            )?,
            VolumePostprocess::load(
                vb.pp("volume_postprocess.1"),
                VOLUME_CHANNELS * GROWTH_RATES[1],
            )?,
            VolumePostprocess::load(
                vb.pp("volume_postprocess.2"),
                VOLUME_CHANNELS * GROWTH_RATES[2],
            )?,
        ];
        Ok(Self {
            feature_extractor,
            volume_postprocess,
            max_disparities: [max_disparity / 16, 3, 3],
        })
    }

    pub fn features(&self, image: &Tensor) -> Result<[Tensor; 3]> {
        self.feature_extractor.forward(image)
    }

    pub fn base(
        &self,
        left: &[Tensor; 3],
        right: &[Tensor; 3],
        image_height: usize,
        image_width: usize,
    ) -> Result<Tensor> {
        let cost = initial_cost_volume(&left[0], &right[0], self.max_disparities[0])?;
        let cost = self.volume_postprocess[0].forward(&cost)?;
        let probability = candle_nn::ops::softmax(&(&cost * -1.0)?, 1)?;
        let low_resolution = disparity_regression(&probability, 0, self.max_disparities[0] as i64)?;
        scale_to_full_resolution(&low_resolution, image_height, image_width)
    }

    pub fn refine(
        &self,
        scale: usize,
        left: &[Tensor; 3],
        right: &[Tensor; 3],
        current: &Tensor,
        image_height: usize,
        image_width: usize,
    ) -> Result<Tensor> {
        if !(1..=2).contains(&scale) {
            candle_core::bail!("AnyNet refinement scale must be 1 or 2");
        }
        let (_batch, _channels, feature_height, feature_width) = left[scale].dims4()?;
        let scaled_disparity =
            (current.upsample_bilinear2d(feature_height, feature_width, false)?
                * (feature_height as f64 / image_height as f64))?;
        let cost = residual_cost_volume(
            &left[scale],
            &right[scale],
            &scaled_disparity,
            self.max_disparities[scale],
        )?;
        let cost = self.volume_postprocess[scale].forward(&cost)?;
        let probability = candle_nn::ops::softmax(&(&cost * -1.0)?, 1)?;
        let radius = self.max_disparities[scale] as i64 - 1;
        let residual = disparity_regression(&probability, -radius, radius + 1)?;
        let residual = scale_to_full_resolution(&residual, image_height, image_width)?;
        current + residual
    }
}

fn initial_cost_volume(left: &Tensor, right: &Tensor, max_disparity: usize) -> Result<Tensor> {
    let (batch, _channels, height, width) = left.dims4()?;
    if right.dims4()? != left.dims4()? {
        candle_core::bail!("AnyNet left and right feature shapes differ");
    }
    let mut costs = Vec::with_capacity(max_disparity);
    for disparity in 0..max_disparity {
        let cost = if disparity == 0 {
            (left - right)?.abs()?.sum(1)?
        } else if disparity >= width {
            left.abs()?.sum(1)?
        } else {
            let prefix = left.narrow(3, 0, disparity)?.abs()?.sum(1)?;
            let remaining_width = width - disparity;
            let shifted = (left.narrow(3, disparity, remaining_width)?
                - right.narrow(3, 0, remaining_width)?)?
            .abs()?
            .sum(1)?;
            Tensor::cat(&[&prefix, &shifted], 2)?
        };
        costs.push(cost.reshape((batch, height, width))?);
    }
    Tensor::stack(&costs, 1)
}

fn residual_cost_volume(
    left: &Tensor,
    right: &Tensor,
    disparity: &Tensor,
    max_disparity: usize,
) -> Result<Tensor> {
    let radius = max_disparity as i64 - 1;
    let mut costs = Vec::with_capacity(max_disparity * 2 - 1);
    for offset in -radius..=radius {
        let sample_disparity = (disparity - offset as f64)?;
        let warped = warp_horizontal(right, &sample_disparity)?;
        costs.push((left - warped)?.abs()?.sum(1)?);
    }
    Tensor::stack(&costs, 1)
}

fn disparity_regression(probability: &Tensor, start: i64, end: i64) -> Result<Tensor> {
    let (batch, disparities, height, width) = probability.dims4()?;
    if end - start != disparities as i64 {
        candle_core::bail!("disparity range does not match cost-volume depth");
    }
    let values = Tensor::arange(start as f32, end as f32, probability.device())?
        .to_dtype(DType::F32)?
        .reshape((1, disparities, 1, 1))?
        .broadcast_as((batch, disparities, height, width))?;
    probability.broadcast_mul(&values)?.sum_keepdim(1)
}

fn scale_to_full_resolution(
    disparity: &Tensor,
    image_height: usize,
    image_width: usize,
) -> Result<Tensor> {
    let (_batch, _channels, low_height, _low_width) = disparity.dims4()?;
    let disparity = (disparity * (image_height as f64 / low_height as f64))?;
    disparity.upsample_bilinear2d(image_height, image_width, false)
}

#[cfg(test)]
mod tests {
    use super::*;
    use candle_core::Device;
    use std::path::PathBuf;

    #[test]
    fn initial_cost_volume_matches_shifted_l1_definition() -> Result<()> {
        let device = Device::Cpu;
        let left = Tensor::new(&[[[[1f32, 2.0, 3.0, 4.0]]]], &device)?;
        let right = Tensor::new(&[[[[1f32, 1.0, 2.0, 2.0]]]], &device)?;
        let actual = initial_cost_volume(&left, &right, 3)?
            .get(0)?
            .to_vec3::<f32>()?;
        assert_eq!(actual[0][0], [0.0, 1.0, 1.0, 2.0]);
        assert_eq!(actual[1][0], [1.0, 1.0, 2.0, 2.0]);
        assert_eq!(actual[2][0], [1.0, 2.0, 2.0, 3.0]);
        Ok(())
    }

    #[test]
    fn disparity_regression_returns_probability_weighted_offset() -> Result<()> {
        let device = Device::Cpu;
        let probability = Tensor::new(&[[[[0.25f32]], [[0.5]], [[0.25]]]], &device)?;
        let disparity = disparity_regression(&probability, -1, 2)?
            .flatten_all()?
            .to_vec1::<f32>()?[0];
        assert_eq!(disparity, 0.0);
        Ok(())
    }

    #[test]
    fn three_stages_match_the_pytorch_golden_fixture() -> Result<()> {
        let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
        let weights = fixtures.join("weights.safetensors");
        let golden =
            candle_core::safetensors::load(fixtures.join("golden.safetensors"), &Device::Cpu)?;
        // SAFETY: The test fixture is a checked-in, read-only safetensors file.
        let builder =
            unsafe { VarBuilder::from_mmaped_safetensors(&[weights], DType::F32, &Device::Cpu)? };
        let model = AnyNet::load(builder, 192)?;
        let left = golden.get("left").expect("golden left tensor");
        let right = golden.get("right").expect("golden right tensor");
        let left_features = model.features(left)?;
        let right_features = model.features(right)?;
        let stage1 = model.base(&left_features, &right_features, 64, 128)?;
        let scaled = (stage1.upsample_bilinear2d(8, 16, false)? * (8.0 / 64.0))?;
        let stage2_cost = residual_cost_volume(&left_features[1], &right_features[1], &scaled, 3)?;
        for (name, actual) in [
            ("stage2_left_features", left_features[1].clone()),
            ("stage2_right_features", right_features[1].clone()),
            ("stage2_scaled_disparity", scaled),
            ("stage2_cost", stage2_cost),
        ] {
            let expected = golden.get(name).expect("golden intermediate tensor");
            let error = (&actual - expected)?.abs()?.max_all()?.to_scalar::<f32>()?;
            assert!(error < 1e-4, "{name} maximum error was {error}");
        }
        let stage2 = model.refine(1, &left_features, &right_features, &stage1, 64, 128)?;
        let stage3 = model.refine(2, &left_features, &right_features, &stage2, 64, 128)?;

        for (name, actual) in [("stage1", stage1), ("stage2", stage2), ("stage3", stage3)] {
            let expected = golden.get(name).expect("golden stage tensor");
            let error = (&actual - expected)?.abs()?.max_all()?.to_scalar::<f32>()?;
            assert!(error < 1e-4, "{name} maximum error was {error}");
        }
        Ok(())
    }
}
