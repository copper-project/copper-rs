use anyhow::{Context, Result, ensure};
use candle_core::{DType, Device, Tensor};
use candle_nn::VarBuilder;
use cu_vitfly::{RECURRENT_SHAPE, VitFly};

const FIXTURES: &[u8] = include_bytes!("../fixtures/pytorch-parity.safetensors");

fn expected(vb: &VarBuilder<'_>, name: &str, actual: &Tensor) -> Result<Tensor> {
    vb.get(actual.shape().clone(), name)
        .with_context(|| format!("loading fixture {name}"))
}

fn assert_close(name: &str, actual: &Tensor, expected: &Tensor, tolerance: f32) -> Result<()> {
    let maximum = (actual - expected)?.abs()?.max_all()?.to_scalar::<f32>()?;
    println!("{name:<36} max_abs={maximum:.8}");
    ensure!(
        maximum <= tolerance,
        "{name}: max absolute difference {maximum:.8} exceeds {tolerance:.8}"
    );
    Ok(())
}

fn run_parity(device: &Device, tolerance: f32) -> Result<()> {
    let fixtures = VarBuilder::from_slice_safetensors(FIXTURES, DType::F32, device)?;
    let model = VitFly::load(device)?;
    let mut state = None;

    for sample in 0..2 {
        let prefix = format!("sample{sample}");
        let depth = fixtures.get((1, 1, 60, 90), &format!("{prefix}.input.depth"))?;
        let desired_velocity = fixtures.get((1, 1), &format!("{prefix}.desired_velocity"))?;
        let attitude = fixtures.get((1, 4), &format!("{prefix}.attitude"))?;
        let (output, next_state, trace) =
            model.forward_traced(&depth, &desired_velocity, &attitude, state.as_ref())?;

        for (name, actual) in trace.iter() {
            let fixture_name = format!("{prefix}.{name}");
            let wanted = expected(&fixtures, &fixture_name, actual)?;
            assert_close(&fixture_name, actual, &wanted, tolerance)?;
        }
        let traced_output = trace.get("output").context("trace is missing output")?;
        assert_close("returned output", &output, traced_output, 0.0)?;
        state = Some(next_state);
    }
    Ok(())
}

#[test]
fn pytorch_layer_by_layer_cpu_parity() -> Result<()> {
    run_parity(&Device::Cpu, 2e-5)
}

#[test]
fn exact_low_level_shape_contract_is_enforced() -> Result<()> {
    let device = Device::Cpu;
    let model = VitFly::load(&device)?;
    let wrong_depth = Tensor::zeros((1, 1, 61, 90), DType::F32, &device)?;
    let desired_velocity = Tensor::zeros((1, 1), DType::F32, &device)?;
    let attitude = Tensor::zeros((1, 4), DType::F32, &device)?;
    let error = model
        .forward(&wrong_depth, &desired_velocity, &attitude, None)
        .expect_err("the core must not silently resize an input");
    ensure!(error.to_string().contains("[1, 1, 60, 90]"));
    Ok(())
}

#[test]
fn pytorch_recurrent_tensor_shape_round_trip() -> Result<()> {
    let device = Device::Cpu;
    let fixtures = VarBuilder::from_slice_safetensors(FIXTURES, DType::F32, &device)?;
    let model = VitFly::load(&device)?;
    let depth = fixtures.get((1, 1, 60, 90), "sample0.input.depth")?;
    let desired_velocity = fixtures.get((1, 1), "sample0.desired_velocity")?;
    let attitude = fixtures.get((1, 4), "sample0.attitude")?;
    let (_, hidden, cell) = model.forward_tensors(&depth, &desired_velocity, &attitude, None)?;
    ensure!(hidden.dims() == RECURRENT_SHAPE);
    ensure!(cell.dims() == RECURRENT_SHAPE);

    let (output, _, _) =
        model.forward_tensors(&depth, &desired_velocity, &attitude, Some((&hidden, &cell)))?;
    ensure!(output.dims() == [1, 3]);
    Ok(())
}

#[cfg(feature = "cuda")]
#[test]
fn pytorch_layer_by_layer_cuda_parity() -> Result<()> {
    run_parity(&Device::new_cuda(0)?, 3e-5)
}
