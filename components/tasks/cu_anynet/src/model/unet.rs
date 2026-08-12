//! Shared U-Net feature extractor used for the left and right images.

use candle_core::{Result, Tensor};
use candle_nn::{
    BatchNorm, Conv2d, Conv2dConfig, Module, ModuleT, VarBuilder, batch_norm, conv2d,
    conv2d_no_bias,
};

#[derive(Debug)]
struct PreConv2d {
    batch_norm: BatchNorm,
    conv: Conv2d,
}

impl PreConv2d {
    fn load(
        vb: VarBuilder,
        input_channels: usize,
        output_channels: usize,
        stride: usize,
    ) -> Result<Self> {
        let batch_norm = batch_norm(input_channels, 1e-5, vb.pp("0"))?;
        let conv = conv2d_no_bias(
            input_channels,
            output_channels,
            3,
            Conv2dConfig {
                padding: 1,
                stride,
                ..Default::default()
            },
            vb.pp("2"),
        )?;
        Ok(Self { batch_norm, conv })
    }

    fn forward(&self, input: &Tensor) -> Result<Tensor> {
        let input = self.batch_norm.forward_t(input, false)?.relu()?;
        self.conv.forward(&input)
    }
}

#[derive(Debug)]
struct DownBlock {
    convs: Vec<PreConv2d>,
}

impl DownBlock {
    fn load(
        vb: VarBuilder,
        input_channels: usize,
        output_channels: usize,
        blocks: usize,
    ) -> Result<Self> {
        let mut convs = Vec::with_capacity(blocks);
        for index in 0..blocks {
            let channels = if index == 0 {
                input_channels
            } else {
                output_channels
            };
            convs.push(PreConv2d::load(
                vb.pp(index + 1),
                channels,
                output_channels,
                1,
            )?);
        }
        Ok(Self { convs })
    }

    fn forward(&self, input: &Tensor) -> Result<Tensor> {
        let mut output = input.max_pool2d_with_stride(2, 2)?;
        for conv in &self.convs {
            output = conv.forward(&output)?;
        }
        Ok(output)
    }
}

#[derive(Debug)]
struct UpBlock {
    convs: [PreConv2d; 2],
}

impl UpBlock {
    fn load(vb: VarBuilder, deep_channels: usize, output_channels: usize) -> Result<Self> {
        // The bilinear branch concatenates the full deep tensor with a skip
        // tensor having half as many channels: 1.5 × deep_channels.
        let merged_channels = deep_channels * 3 / 2;
        let convs = [
            PreConv2d::load(vb.pp("conv.0"), merged_channels, output_channels, 1)?,
            PreConv2d::load(vb.pp("conv.1"), output_channels, output_channels, 1)?,
        ];
        Ok(Self { convs })
    }

    fn forward(&self, skip: &Tensor, deep: &Tensor) -> Result<Tensor> {
        let (_batch, _channels, skip_height, skip_width) = skip.dims4()?;
        // `nn.UpsamplingBilinear2d` in the reference model uses the legacy
        // corner-aligned interpolation convention.
        let mut upsampled = deep.upsample_bilinear2d_with_scale(2.0, 2.0, true)?;
        let (_batch, _channels, up_height, up_width) = upsampled.dims4()?;
        if up_height != skip_height {
            upsampled = upsampled.narrow(2, 0, skip_height)?;
        }
        if up_width != skip_width {
            upsampled = upsampled.narrow(3, 0, skip_width)?;
        }
        let mut output = Tensor::cat(&[skip, &upsampled], 1)?;
        for conv in &self.convs {
            output = conv.forward(&output)?;
        }
        Ok(output)
    }
}

/// AnyNet's three-scale feature pyramid, ordered 1/16, 1/8, 1/4.
#[derive(Debug)]
pub(crate) struct FeatureExtractor {
    initial: Conv2d,
    downsample: PreConv2d,
    block0: DownBlock,
    blocks: [DownBlock; 2],
    upblocks: [UpBlock; 2],
}

impl FeatureExtractor {
    pub(crate) fn load(vb: VarBuilder, initial_channels: usize, blocks: usize) -> Result<Self> {
        let initial = conv2d(
            3,
            initial_channels,
            3,
            Conv2dConfig {
                padding: 1,
                ..Default::default()
            },
            vb.pp("block0.0.0"),
        )?;
        let downsample =
            PreConv2d::load(vb.pp("block0.0.1"), initial_channels, initial_channels, 2)?;
        let block0 = DownBlock::load(
            vb.pp("block0.1"),
            initial_channels,
            initial_channels * 2,
            blocks,
        )?;
        let blocks = [
            DownBlock::load(
                vb.pp("blocks.0"),
                initial_channels * 2,
                initial_channels * 4,
                blocks,
            )?,
            DownBlock::load(
                vb.pp("blocks.1"),
                initial_channels * 4,
                initial_channels * 8,
                blocks,
            )?,
        ];
        let upblocks = [
            UpBlock::load(
                vb.pp("upblocks.0"),
                initial_channels * 8,
                initial_channels * 4,
            )?,
            UpBlock::load(
                vb.pp("upblocks.1"),
                initial_channels * 4,
                initial_channels * 2,
            )?,
        ];
        Ok(Self {
            initial,
            downsample,
            block0,
            blocks,
            upblocks,
        })
    }

    pub(crate) fn forward(&self, input: &Tensor) -> Result<[Tensor; 3]> {
        let output = self.initial.forward(input)?;
        let output = self.downsample.forward(&output)?;
        let scale_4 = self.block0.forward(&output)?;
        let scale_8 = self.blocks[0].forward(&scale_4)?;
        let scale_16 = self.blocks[1].forward(&scale_8)?;
        let scale_8 = self.upblocks[0].forward(&scale_8, &scale_16)?;
        let scale_4 = self.upblocks[1].forward(&scale_4, &scale_8)?;
        Ok([scale_16, scale_8, scale_4])
    }
}
