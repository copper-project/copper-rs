use cached_path::{Cache, Error as CacheError, Options, ProgressBar};
use candle_core::{D, DType, Device, IndexOp, Module, Result, Shape, Tensor};
use candle_nn::rnn::LSTMState;
use candle_nn::{
    Conv2d, Conv2dConfig, LSTM, LSTMConfig, LayerNorm, Linear, RNN, VarBuilder, conv2d, layer_norm,
    linear,
};
use std::{
    fs, io,
    path::{Path, PathBuf},
};

pub const INPUT_HEIGHT: usize = 60;
pub const INPUT_WIDTH: usize = 90;
pub const DEPTH_SHAPE: [usize; 4] = [1, 1, INPUT_HEIGHT, INPUT_WIDTH];
pub const DESIRED_VELOCITY_SHAPE: [usize; 2] = [1, 1];
pub const ATTITUDE_SHAPE: [usize; 2] = [1, 4];
pub const OUTPUT_SHAPE: [usize; 2] = [1, 3];
pub const RECURRENT_SHAPE: [usize; 2] = [3, 128];

const MODEL_NAME: &str = "vitfly-vitlstm-f32.safetensors";
const MODEL_URL: &str = "https://cdn.copper-robotics.com/models/vitfly-vitlstm-f32.safetensors";
const MODEL_CACHE_DIR: &str = ".download-cache";
const MODEL_SIZE_BYTES: u64 = 14_264_732;

fn model_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("weights")
}

fn link_or_copy_cached_model(src: &Path, dst: &Path) -> io::Result<()> {
    if fs::symlink_metadata(dst).is_ok() {
        fs::remove_file(dst)?;
    }

    #[cfg(unix)]
    {
        match std::os::unix::fs::symlink(src, dst) {
            Ok(()) => Ok(()),
            Err(symlink_err) => fs::copy(src, dst).map(|_| ()).map_err(|copy_err| {
                io::Error::new(
                    copy_err.kind(),
                    format!("failed to symlink ({symlink_err}) or copy ({copy_err})"),
                )
            }),
        }
    }

    #[cfg(windows)]
    {
        match std::os::windows::fs::symlink_file(src, dst) {
            Ok(()) => Ok(()),
            Err(symlink_err) => fs::copy(src, dst).map(|_| ()).map_err(|copy_err| {
                io::Error::new(
                    copy_err.kind(),
                    format!("failed to symlink ({symlink_err}) or copy ({copy_err})"),
                )
            }),
        }
    }

    #[cfg(not(any(unix, windows)))]
    {
        fs::copy(src, dst).map(|_| ())
    }
}

fn cached_model_path(
    online_cache: &Cache,
    offline_cache: &Cache,
) -> std::result::Result<PathBuf, CacheError> {
    match offline_cache.cached_path(MODEL_URL) {
        Ok(path) if model_file_is_valid(&path) => Ok(path),
        Ok(_) => {
            eprintln!("{MODEL_NAME}: cached file is incomplete; downloading it again");
            let path =
                online_cache.cached_path_with_options(MODEL_URL, &Options::default().force())?;
            validate_model_file(path)
        }
        Err(err) => {
            if matches!(
                err,
                CacheError::NoCachedVersions(_) | CacheError::CacheCorrupted(_)
            ) {
                eprintln!("{MODEL_NAME}: cache miss; downloading from {MODEL_URL}");
                let path = online_cache.cached_path(MODEL_URL)?;
                validate_model_file(path)
            } else {
                Err(err)
            }
        }
    }
}

fn model_file_is_valid(path: &Path) -> bool {
    fs::metadata(path)
        .map(|metadata| metadata.len() == MODEL_SIZE_BYTES)
        .unwrap_or(false)
}

fn validate_model_file(path: PathBuf) -> std::result::Result<PathBuf, CacheError> {
    if model_file_is_valid(&path) {
        Ok(path)
    } else {
        Err(CacheError::CacheCorrupted(format!(
            "{MODEL_NAME} must be {MODEL_SIZE_BYTES} bytes"
        )))
    }
}

fn prepare_model_weights() -> Result<PathBuf> {
    let weights_root = model_root();
    let model_path = weights_root.join(MODEL_NAME);
    if model_file_is_valid(&model_path) {
        return Ok(model_path);
    }

    fs::create_dir_all(&weights_root).map_err(candle_core::Error::msg)?;
    if fs::symlink_metadata(&model_path).is_ok() {
        fs::remove_file(&model_path).map_err(candle_core::Error::msg)?;
    }

    let cache_root = weights_root.join(MODEL_CACHE_DIR);
    let online_cache = Cache::builder()
        .dir(cache_root.clone())
        .progress_bar(Some(ProgressBar::Full))
        .build()
        .map_err(candle_core::Error::msg)?;
    let offline_cache = Cache::builder()
        .dir(cache_root)
        .offline(true)
        .progress_bar(None)
        .build()
        .map_err(candle_core::Error::msg)?;
    let cached_path =
        cached_model_path(&online_cache, &offline_cache).map_err(candle_core::Error::msg)?;
    link_or_copy_cached_model(&cached_path, &model_path).map_err(candle_core::Error::msg)?;
    Ok(model_path)
}

#[derive(Debug, Clone)]
struct PatchMerge {
    conv: Conv2d,
    norm: LayerNorm,
}

impl PatchMerge {
    fn load(
        in_channels: usize,
        out_channels: usize,
        kernel_size: usize,
        stride: usize,
        padding: usize,
        vb: VarBuilder<'_>,
    ) -> Result<Self> {
        let conv = conv2d(
            in_channels,
            out_channels,
            kernel_size,
            Conv2dConfig {
                stride,
                padding,
                ..Default::default()
            },
            vb.pp("cn1"),
        )?;
        let norm = layer_norm(out_channels, 1e-5, vb.pp("layerNorm"))?;
        Ok(Self { conv, norm })
    }

    fn forward(&self, input: &Tensor) -> Result<(Tensor, usize, usize)> {
        let output = self.conv.forward(input)?;
        let (_, _, height, width) = output.dims4()?;
        let output = output.flatten_from(2)?.transpose(1, 2)?.contiguous()?;
        let output = self.norm.forward(&output)?;
        Ok((output, height, width))
    }
}

#[derive(Debug, Clone)]
struct EfficientSelfAttention {
    reduction: Conv2d,
    reduction_norm: LayerNorm,
    key_value: Linear,
    query: Linear,
    output: Linear,
    heads: usize,
}

impl EfficientSelfAttention {
    fn load(
        channels: usize,
        reduction_ratio: usize,
        heads: usize,
        vb: VarBuilder<'_>,
    ) -> Result<Self> {
        let reduction = conv2d(
            channels,
            channels,
            reduction_ratio,
            Conv2dConfig {
                stride: reduction_ratio,
                ..Default::default()
            },
            vb.pp("cn1"),
        )?;
        let reduction_norm = layer_norm(channels, 1e-5, vb.pp("ln1"))?;
        let key_value = linear(channels, channels * 2, vb.pp("keyValueExtractor"))?;
        let query = linear(channels, channels, vb.pp("query"))?;
        let output = linear(channels, channels, vb.pp("finalLayer"))?;
        Ok(Self {
            reduction,
            reduction_norm,
            key_value,
            query,
            output,
            heads,
        })
    }

    fn forward(&self, input: &Tensor, height: usize, width: usize) -> Result<Tensor> {
        let (batch, tokens, channels) = input.dims3()?;
        let reduced = input
            .transpose(1, 2)?
            .contiguous()?
            .reshape((batch, channels, height, width))?;
        let reduced = self.reduction.forward(&reduced)?;
        let reduced = reduced.flatten_from(2)?.transpose(1, 2)?.contiguous()?;
        let reduced = self.reduction_norm.forward(&reduced)?;

        let key_value = self.key_value.forward(&reduced)?;
        let reduced_tokens = key_value.dim(1)?;
        let head_dim = channels / self.heads;
        let key_value = key_value
            .reshape((batch, reduced_tokens, 2, self.heads, head_dim))?
            .permute((2, 0, 3, 1, 4))?
            .contiguous()?;
        let key = key_value.i(0)?;
        let value = key_value.i(1)?;
        let query = self
            .query
            .forward(input)?
            .reshape((batch, tokens, self.heads, head_dim))?
            .permute((0, 2, 1, 3))?
            .contiguous()?;

        let scores = (query.matmul(&key.transpose(2, 3)?)? / (head_dim as f64).sqrt())?;
        let attention = candle_nn::ops::softmax(&scores, D::Minus1)?;
        let output = attention
            .matmul(&value)?
            .transpose(1, 2)?
            .contiguous()?
            .reshape((batch, tokens, channels))?;
        self.output.forward(&output)
    }
}

#[derive(Debug, Clone)]
struct MixFfn {
    expand: Linear,
    depthwise: Conv2d,
    project: Linear,
}

impl MixFfn {
    fn load(channels: usize, expansion_factor: usize, vb: VarBuilder<'_>) -> Result<Self> {
        let expanded_channels = channels * expansion_factor;
        let expand = linear(channels, expanded_channels, vb.pp("mlp1"))?;
        // This deliberately matches the original model's groups=channels,
        // whose checkpoint shape is [expanded_channels, expansion_factor, 3, 3].
        let depthwise = conv2d(
            expanded_channels,
            expanded_channels,
            3,
            Conv2dConfig {
                padding: 1,
                groups: channels,
                ..Default::default()
            },
            vb.pp("depthwise"),
        )?;
        let project = linear(expanded_channels, channels, vb.pp("mlp2"))?;
        Ok(Self {
            expand,
            depthwise,
            project,
        })
    }

    fn forward(&self, input: &Tensor, height: usize, width: usize) -> Result<Tensor> {
        let expanded = self.expand.forward(input)?;
        let (batch, _, channels) = expanded.dims3()?;
        let expanded = expanded
            .transpose(1, 2)?
            .contiguous()?
            .reshape((batch, channels, height, width))?;
        let expanded = self.depthwise.forward(&expanded)?.gelu_erf()?;
        let expanded = expanded.flatten_from(2)?.transpose(1, 2)?.contiguous()?;
        self.project.forward(&expanded)
    }
}

#[derive(Debug, Clone)]
struct EncoderStage {
    patch_merge: PatchMerge,
    attentions: Vec<EfficientSelfAttention>,
    ffns: Vec<MixFfn>,
    norms: Vec<LayerNorm>,
}

impl EncoderStage {
    #[allow(clippy::too_many_arguments)]
    fn load(
        in_channels: usize,
        out_channels: usize,
        patch_size: usize,
        stride: usize,
        padding: usize,
        layers: usize,
        reduction_ratio: usize,
        heads: usize,
        expansion_factor: usize,
        vb: VarBuilder<'_>,
    ) -> Result<Self> {
        let patch_merge = PatchMerge::load(
            in_channels,
            out_channels,
            patch_size,
            stride,
            padding,
            vb.pp("patchMerge"),
        )?;
        let mut attentions = Vec::with_capacity(layers);
        let mut ffns = Vec::with_capacity(layers);
        let mut norms = Vec::with_capacity(layers);
        for index in 0..layers {
            attentions.push(EfficientSelfAttention::load(
                out_channels,
                reduction_ratio,
                heads,
                vb.pp(format!("_attn.{index}")),
            )?);
            ffns.push(MixFfn::load(
                out_channels,
                expansion_factor,
                vb.pp(format!("_ffn.{index}")),
            )?);
            norms.push(layer_norm(
                out_channels,
                1e-5,
                vb.pp(format!("_lNorm.{index}")),
            )?);
        }
        Ok(Self {
            patch_merge,
            attentions,
            ffns,
            norms,
        })
    }

    fn forward(
        &self,
        input: &Tensor,
        trace: &mut Option<&mut ForwardTrace>,
        prefix: &str,
    ) -> Result<Tensor> {
        let (mut output, height, width) = self.patch_merge.forward(input)?;
        record(trace, format!("{prefix}.patch"), &output);
        for index in 0..self.attentions.len() {
            let attention = self.attentions[index].forward(&output, height, width)?;
            output = (&output + attention)?;
            record(trace, format!("{prefix}.attention{index}"), &output);
            let ffn = self.ffns[index].forward(&output, height, width)?;
            output = (&output + ffn)?;
            record(trace, format!("{prefix}.ffn{index}"), &output);
            output = self.norms[index].forward(&output)?;
            record(trace, format!("{prefix}.norm{index}"), &output);
        }
        let batch = output.dim(0)?;
        let channels = output.dim(2)?;
        let output = output
            .reshape((batch, height, width, channels))?
            .permute((0, 3, 1, 2))?
            .contiguous()?;
        record(trace, format!("{prefix}.output"), &output);
        Ok(output)
    }
}

/// Recurrent hidden and cell state for the policy's three LSTM layers.
#[derive(Debug, Clone)]
pub struct VitFlyState {
    layers: Vec<LSTMState>,
}

impl VitFlyState {
    /// Builds a state from the original PyTorch `(h, c)` tensor shapes.
    pub fn from_tensors(hidden: &Tensor, cell: &Tensor) -> Result<Self> {
        validate_shape("hidden", hidden.shape(), &RECURRENT_SHAPE)?;
        validate_shape("cell", cell.shape(), &RECURRENT_SHAPE)?;
        let mut layers = Vec::with_capacity(RECURRENT_SHAPE[0]);
        for index in 0..RECURRENT_SHAPE[0] {
            let hidden = hidden.narrow(0, index, 1)?.contiguous()?;
            let cell = cell.narrow(0, index, 1)?.contiguous()?;
            layers.push(LSTMState::new(hidden, cell));
        }
        Ok(Self { layers })
    }

    /// Returns the hidden state in the original PyTorch `[3, 128]` shape.
    pub fn hidden_tensor(&self) -> Result<Tensor> {
        let layers = self
            .layers
            .iter()
            .map(|state| state.h())
            .collect::<Vec<_>>();
        Tensor::cat(&layers, 0)
    }

    /// Returns the cell state in the original PyTorch `[3, 128]` shape.
    pub fn cell_tensor(&self) -> Result<Tensor> {
        let layers = self
            .layers
            .iter()
            .map(|state| state.c())
            .collect::<Vec<_>>();
        Tensor::cat(&layers, 0)
    }

    pub fn layer(&self, index: usize) -> Option<&LSTMState> {
        self.layers.get(index)
    }

    pub fn len(&self) -> usize {
        self.layers.len()
    }

    pub fn is_empty(&self) -> bool {
        self.layers.is_empty()
    }
}

/// Named intermediate tensors used by the PyTorch/Candle parity tests.
#[derive(Debug, Default)]
pub struct ForwardTrace {
    tensors: Vec<(String, Tensor)>,
}

impl ForwardTrace {
    pub fn get(&self, name: &str) -> Option<&Tensor> {
        self.tensors
            .iter()
            .find_map(|(candidate, tensor)| (candidate == name).then_some(tensor))
    }

    pub fn iter(&self) -> impl Iterator<Item = (&str, &Tensor)> {
        self.tensors
            .iter()
            .map(|(name, tensor)| (name.as_str(), tensor))
    }
}

fn record(trace: &mut Option<&mut ForwardTrace>, name: String, tensor: &Tensor) {
    if let Some(trace) = trace.as_deref_mut() {
        trace.tensors.push((name, tensor.clone()));
    }
}

/// Candle implementation of the pretrained ViTFly ViT+LSTM policy.
#[derive(Debug, Clone)]
pub struct VitFly {
    encoder_stages: [EncoderStage; 2],
    down_sample: Conv2d,
    decoder: Linear,
    lstm_layers: [LSTM; 3],
    output: Linear,
}

impl VitFly {
    /// Loads the model from the locally cached F32 Safetensors checkpoint.
    ///
    /// The first call downloads the checkpoint from Copper's CDN into the
    /// crate's `weights` directory. Later calls reuse that local file.
    pub fn load(device: &Device) -> Result<Self> {
        let weights_path = prepare_model_weights()?;
        // SAFETY: the stable local model path is created before mapping and is
        // never modified while the returned model can retain the mapping.
        let vb =
            unsafe { VarBuilder::from_mmaped_safetensors(&[weights_path], DType::F32, device)? };
        Self::load_from_var_builder(vb)
    }

    fn load_from_var_builder(vb: VarBuilder<'_>) -> Result<Self> {
        let encoder0 = EncoderStage::load(1, 32, 7, 4, 3, 2, 8, 1, 8, vb.pp("encoder_blocks.0"))?;
        let encoder1 = EncoderStage::load(32, 64, 3, 2, 1, 2, 4, 2, 8, vb.pp("encoder_blocks.1"))?;
        let down_sample = conv2d(
            48,
            12,
            3,
            Conv2dConfig {
                padding: 1,
                ..Default::default()
            },
            vb.pp("down_sample"),
        )?;
        let decoder = linear(4608, 512, vb.pp("decoder"))?;
        let load_lstm = |index, input_size| {
            candle_nn::lstm(
                input_size,
                128,
                LSTMConfig {
                    layer_idx: index,
                    ..Default::default()
                },
                vb.pp("lstm"),
            )
        };
        let lstm_layers = [load_lstm(0, 517)?, load_lstm(1, 128)?, load_lstm(2, 128)?];
        let output = linear(128, 3, vb.pp("nn_fc2"))?;
        Ok(Self {
            encoder_stages: [encoder0, encoder1],
            down_sample,
            decoder,
            lstm_layers,
            output,
        })
    }

    /// Creates a zero recurrent state on the model's device.
    pub fn zero_state(&self) -> Result<VitFlyState> {
        let layers = self
            .lstm_layers
            .iter()
            .map(|layer| layer.zero_state(1))
            .collect::<Result<Vec<_>>>()?;
        Ok(VitFlyState { layers })
    }

    /// Runs one frame with the exact low-level shapes used by the original model.
    pub fn forward(
        &self,
        depth: &Tensor,
        desired_velocity: &Tensor,
        attitude: &Tensor,
        state: Option<&VitFlyState>,
    ) -> Result<(Tensor, VitFlyState)> {
        self.forward_impl(depth, desired_velocity, attitude, state, None)
    }

    /// Runs one frame using the original model's optional `[3, 128]` hidden
    /// and cell tensors, and returns the next tensors in that same shape.
    pub fn forward_tensors(
        &self,
        depth: &Tensor,
        desired_velocity: &Tensor,
        attitude: &Tensor,
        recurrent: Option<(&Tensor, &Tensor)>,
    ) -> Result<(Tensor, Tensor, Tensor)> {
        let state = recurrent
            .map(|(hidden, cell)| VitFlyState::from_tensors(hidden, cell))
            .transpose()?;
        let (output, state) = self.forward(depth, desired_velocity, attitude, state.as_ref())?;
        Ok((output, state.hidden_tensor()?, state.cell_tensor()?))
    }

    /// Runs one frame and retains named intermediate tensors for parity validation.
    pub fn forward_traced(
        &self,
        depth: &Tensor,
        desired_velocity: &Tensor,
        attitude: &Tensor,
        state: Option<&VitFlyState>,
    ) -> Result<(Tensor, VitFlyState, ForwardTrace)> {
        let mut trace = ForwardTrace::default();
        let (output, state) =
            self.forward_impl(depth, desired_velocity, attitude, state, Some(&mut trace))?;
        Ok((output, state, trace))
    }

    fn forward_impl(
        &self,
        depth: &Tensor,
        desired_velocity: &Tensor,
        attitude: &Tensor,
        state: Option<&VitFlyState>,
        mut trace: Option<&mut ForwardTrace>,
    ) -> Result<(Tensor, VitFlyState)> {
        validate_shape("depth", depth.shape(), &DEPTH_SHAPE)?;
        validate_shape(
            "desired_velocity",
            desired_velocity.shape(),
            &DESIRED_VELOCITY_SHAPE,
        )?;
        validate_shape("attitude", attitude.shape(), &ATTITUDE_SHAPE)?;
        if let Some(state) = state
            && state.layers.len() != self.lstm_layers.len()
        {
            candle_core::bail!(
                "state has {} layers, expected {}",
                state.layers.len(),
                self.lstm_layers.len()
            )
        }

        record(&mut trace, "input.depth".into(), depth);
        let encoder0 = self.encoder_stages[0].forward(depth, &mut trace, "encoder0")?;
        let encoder1 = self.encoder_stages[1].forward(&encoder0, &mut trace, "encoder1")?;

        let shuffled = candle_nn::ops::pixel_shuffle(&encoder1, 2)?;
        record(&mut trace, "decoder.pixel_shuffle".into(), &shuffled);
        let upsampled = encoder0.upsample_bilinear2d(16, 24, true)?;
        record(&mut trace, "decoder.upsample".into(), &upsampled);
        let decoded = Tensor::cat(&[&shuffled, &upsampled], 1)?;
        record(&mut trace, "decoder.concat".into(), &decoded);
        let decoded = self.down_sample.forward(&decoded)?;
        record(&mut trace, "decoder.down_sample".into(), &decoded);
        let decoded = self.decoder.forward(&decoded.flatten_from(1)?)?;
        record(&mut trace, "decoder.linear".into(), &decoded);

        let scaled_velocity = (desired_velocity / 10.0)?;
        let mut recurrent = Tensor::cat(&[&decoded, &scaled_velocity, attitude], 1)?;
        record(&mut trace, "metadata.concat".into(), &recurrent);

        let mut next_layers = Vec::with_capacity(self.lstm_layers.len());
        for (index, layer) in self.lstm_layers.iter().enumerate() {
            let current = match state {
                Some(state) => state.layers[index].clone(),
                None => layer.zero_state(1)?,
            };
            let next = layer.step(&recurrent, &current)?;
            recurrent = next.h().clone();
            record(&mut trace, format!("lstm{index}.h"), next.h());
            record(&mut trace, format!("lstm{index}.c"), next.c());
            next_layers.push(next);
        }
        let output = self.output.forward(&recurrent)?;
        record(&mut trace, "output".into(), &output);
        Ok((
            output,
            VitFlyState {
                layers: next_layers,
            },
        ))
    }
}

fn validate_shape(name: &str, actual: &Shape, expected: &[usize]) -> Result<()> {
    if actual.dims() != expected {
        candle_core::bail!(
            "{name} shape {:?} does not match required shape {expected:?}",
            actual.dims()
        )
    }
    Ok(())
}
