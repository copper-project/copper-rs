//! Python-backed Copper tasks.
//!
//! This crate exists for rapid algorithm prototyping, not for production realtime
//! robotics.
//!
//! A [`PyTask`] lets Copper keep ownership of scheduling, logging, replay, and task
//! lifecycle while delegating one task's algorithm body to a Python function:
//!
//! ```python
//! def process(input, state, output):
//!     ...
//! ```
//!
//! Two execution modes are available:
//!
//! - [`PyTaskMode::Process`]: spawn a separate interpreter and exchange
//!   length-prefixed CBOR frames over stdin/stdout. This avoids putting the GIL
//!   inside the Copper process, but adds another serialization layer, extra
//!   copying, allocations, IPC overhead, and scheduler jitter.
//! - [`PyTaskMode::Embedded`]: call Python in-process through PyO3. This avoids
//!   the external CBOR transport, but executes under the GIL inside the Copper
//!   process and still allocates and converts values on every call.
//!
//! Both modes are fundamentally at odds with Copper's design center: predictable
//! low-latency execution with minimal allocation on the realtime path. Using
//! Python here will increase latency, jitter, and allocation pressure enough to
//! ruin the realtime characteristics of the stack. Compared to a native Rust
//! Copper task, the performance is abysmal.
//!
//! The intended workflow is narrow:
//!
//! - prototype one task quickly in Python
//! - stabilize the algorithm
//! - rewrite it in Rust, optionally using an LLM-assisted translation as a first
//!   draft
//!
//! Do not treat Python tasks as a normal production integration path.

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29_value::{Value as CuValue, py_to_value, to_value, value_to_py};
use minicbor::data::{IanaTag, Int as CborInt, Type as CborType};
use minicbor::{Decoder as CborDecoder, Encoder as CborEncoder};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::ffi::CString;
use std::io::{BufReader, Read, Write};
use std::marker::PhantomData;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::OnceLock;
use std::thread;
use std::time::{Duration, Instant};

#[cfg(not(target_os = "macos"))]
use pyo3::prelude::*;
#[cfg(not(target_os = "macos"))]
use pyo3::types::{PyAny, PyModule};

const DEFAULT_SCRIPT_PATH: &str = "python/task.py";
const MAX_CBOR_FRAME_BYTES: usize = 16 * 1024 * 1024;
const PROCESS_START_TIMEOUT: Duration = Duration::from_secs(5);
const PROCESS_STOP_TIMEOUT: Duration = Duration::from_secs(2);
const PROCESS_POLL_INTERVAL: Duration = Duration::from_millis(10);
const PYTHON_BOOTSTRAP: &str = include_str!("bootstrap.py");
const PYTHON_COMMAND_CANDIDATES: &[&str] = &["python3", "python"];

/// State carried across invocations of a [`PyTask`].
///
/// The state must be serializable because both backends turn it into an owned,
/// mutable Python value on every `process(...)` call and then deserialize the
/// result back into Rust.
pub trait PyTaskState:
    Default
    + Clone
    + core::fmt::Debug
    + Serialize
    + DeserializeOwned
    + Reflect
    + TypePath
    + GetTypeRegistration
    + cu29::bevy_reflect::Typed
{
}

impl<T> PyTaskState for T where
    T: Default
        + Clone
        + core::fmt::Debug
        + Serialize
        + DeserializeOwned
        + Reflect
        + TypePath
        + GetTypeRegistration
        + cu29::bevy_reflect::Typed
{
}

/// Describes how a Python-backed task receives its Copper inputs.
///
/// Python cannot safely borrow Copper messages directly, so every call converts
/// the runtime input into an owned representation first.
pub trait PyInputSpec {
    type Input<'m>: CuMsgPack
    where
        Self: 'm;
    type Owned: Clone + Serialize + DeserializeOwned;

    fn to_owned(input: &Self::Input<'_>) -> Self::Owned;
}

/// Describes how a Python-backed task exposes Copper outputs.
///
/// Outputs are converted into mutable owned values before calling Python, then
/// written back into Copper message slots after the Python function returns.
pub trait PyOutputSpec {
    type Output<'m>: CuMsgPayload
    where
        Self: 'm;
    type Owned: Clone + Serialize + DeserializeOwned;

    fn to_owned(output: &Self::Output<'_>) -> Self::Owned;
    fn replace_output(output: &mut Self::Output<'_>, owned: Self::Owned);
}

impl PyInputSpec for () {
    type Input<'m>
        = ()
    where
        Self: 'm;
    type Owned = ();

    fn to_owned(_input: &Self::Input<'_>) -> Self::Owned {}
}

impl<T> PyInputSpec for (T,)
where
    T: CuMsgPayload,
{
    type Input<'m>
        = input_msg!(T)
    where
        Self: 'm;
    type Owned = CuMsg<T>;

    fn to_owned(input: &Self::Input<'_>) -> Self::Owned {
        input.clone()
    }
}

macro_rules! impl_py_input_spec_tuple {
    ($first_ty:ident => $first_var:ident, $second_ty:ident => $second_var:ident $(, $ty:ident => $var:ident)* $(,)?) => {
        impl<$first_ty, $second_ty $(, $ty)*> PyInputSpec for ($first_ty, $second_ty $(, $ty)*)
        where
            $first_ty: CuMsgPayload,
            $second_ty: CuMsgPayload,
            $($ty: CuMsgPayload),*
        {
            type Input<'m>
                = input_msg!('m, $first_ty, $second_ty $(, $ty)*)
            where
                Self: 'm;
            type Owned = (CuMsg<$first_ty>, CuMsg<$second_ty> $(, CuMsg<$ty>)*);

            fn to_owned(input: &Self::Input<'_>) -> Self::Owned {
                #[allow(non_snake_case)]
                let ($first_var, $second_var $(, $var)*) = *input;
                ($first_var.clone(), $second_var.clone() $(, $var.clone())*)
            }
        }
    };
}

impl_py_input_spec_tuple!(T1 => v1, T2 => v2);
impl_py_input_spec_tuple!(T1 => v1, T2 => v2, T3 => v3);
impl_py_input_spec_tuple!(T1 => v1, T2 => v2, T3 => v3, T4 => v4);
impl_py_input_spec_tuple!(T1 => v1, T2 => v2, T3 => v3, T4 => v4, T5 => v5);
impl_py_input_spec_tuple!(T1 => v1, T2 => v2, T3 => v3, T4 => v4, T5 => v5, T6 => v6);
impl_py_input_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7
);
impl_py_input_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8
);
impl_py_input_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9
);
impl_py_input_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9,
    T10 => v10
);
impl_py_input_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9,
    T10 => v10,
    T11 => v11
);
impl_py_input_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9,
    T10 => v10,
    T11 => v11,
    T12 => v12
);

/// Convenience alias for the common one-input, one-output case.
pub type PyUnaryTask<In, State, Out> = PyTask<(In,), State, (Out,)>;

impl PyOutputSpec for () {
    type Output<'m>
        = ()
    where
        Self: 'm;
    type Owned = ();

    fn to_owned(_output: &Self::Output<'_>) -> Self::Owned {}

    fn replace_output(_output: &mut Self::Output<'_>, _owned: Self::Owned) {}
}

impl<T> PyOutputSpec for (T,)
where
    T: CuMsgPayload,
{
    type Output<'m>
        = output_msg!(T)
    where
        Self: 'm;
    type Owned = PyCuMsg<T>;

    fn to_owned(output: &Self::Output<'_>) -> Self::Owned {
        PyCuMsg::from_output(output)
    }

    fn replace_output(output: &mut Self::Output<'_>, owned: Self::Owned) {
        *output = owned.into_output();
    }
}

macro_rules! impl_py_output_spec_tuple {
    ($first_ty:ident => $first_var:ident, $second_ty:ident => $second_var:ident $(, $ty:ident => $var:ident)* $(,)?) => {
        impl<$first_ty, $second_ty $(, $ty)*> PyOutputSpec for ($first_ty, $second_ty $(, $ty)*)
        where
            $first_ty: CuMsgPayload,
            $second_ty: CuMsgPayload,
            $($ty: CuMsgPayload),*
        {
            type Output<'m>
                = output_msg!($first_ty, $second_ty $(, $ty)*)
            where
                Self: 'm;
            type Owned = (PyCuMsg<$first_ty>, PyCuMsg<$second_ty> $(, PyCuMsg<$ty>)*);

            fn to_owned(output: &Self::Output<'_>) -> Self::Owned {
                #[allow(non_snake_case)]
                let ($first_var, $second_var $(, $var)*) = output;
                (
                    PyCuMsg::from_output($first_var),
                    PyCuMsg::from_output($second_var)
                    $(, PyCuMsg::from_output($var))*
                )
            }

            fn replace_output(output: &mut Self::Output<'_>, owned: Self::Owned) {
                #[allow(non_snake_case)]
                let ($first_var, $second_var $(, $var)*) = owned;
                *output = (
                    $first_var.into_output(),
                    $second_var.into_output()
                    $(, $var.into_output())*
                );
            }
        }
    };
}

impl_py_output_spec_tuple!(T1 => v1, T2 => v2);
impl_py_output_spec_tuple!(T1 => v1, T2 => v2, T3 => v3);
impl_py_output_spec_tuple!(T1 => v1, T2 => v2, T3 => v3, T4 => v4);
impl_py_output_spec_tuple!(T1 => v1, T2 => v2, T3 => v3, T4 => v4, T5 => v5);
impl_py_output_spec_tuple!(T1 => v1, T2 => v2, T3 => v3, T4 => v4, T5 => v5, T6 => v6);
impl_py_output_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7
);
impl_py_output_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8
);
impl_py_output_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9
);
impl_py_output_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9,
    T10 => v10
);
impl_py_output_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9,
    T10 => v10,
    T11 => v11
);
impl_py_output_spec_tuple!(
    T1 => v1,
    T2 => v2,
    T3 => v3,
    T4 => v4,
    T5 => v5,
    T6 => v6,
    T7 => v7,
    T8 => v8,
    T9 => v9,
    T10 => v10,
    T11 => v11,
    T12 => v12
);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Reflect)]
pub enum PyTaskMode {
    /// Run the task in a separate Python interpreter process.
    ///
    /// This keeps the GIL out of the Copper process, but every cycle pays for
    /// CBOR serialization, copies, allocations, IPC, and process scheduling.
    Process,
    /// Run the task inside the Copper process through PyO3.
    ///
    /// This removes the external transport layer, but now the GIL and Python
    /// runtime are inside the same process as Copper. This mode is unsupported
    /// on macOS in this workspace.
    Embedded,
}

impl PyTaskMode {
    fn parse(value: &str) -> Option<Self> {
        match value.trim().to_ascii_lowercase().as_str() {
            "process" => Some(Self::Process),
            "embedded" => Some(Self::Embedded),
            _ => None,
        }
    }
}

#[derive(Serialize)]
struct ProcessRequest<I, S, O> {
    input: I,
    state: S,
    output: O,
}

impl<I, S, O> ProcessRequest<I, S, O> {
    fn as_child_request(&self) -> ChildRequest<'_, I, S, O> {
        ChildRequest::Process {
            input: &self.input,
            state: &self.state,
            output: &self.output,
        }
    }
}

#[derive(Serialize, Deserialize)]
struct ProcessResult<S, O> {
    state: S,
    output: O,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound(serialize = "T: Serialize", deserialize = "T: DeserializeOwned"))]
#[doc(hidden)]
pub struct PyCuMsg<T>
where
    T: CuMsgPayload,
{
    payload: Option<T>,
    tov: Tov,
    metadata: CuMsgMetadata,
    #[serde(default, rename = "__cu_payload_present__")]
    payload_present: bool,
    #[serde(
        default,
        rename = "__cu_payload_template__",
        skip_serializing_if = "Option::is_none"
    )]
    payload_template: Option<T>,
}

impl<T> PyCuMsg<T>
where
    T: CuMsgPayload,
{
    fn from_output(output: &CuMsg<T>) -> Self {
        let payload = output.payload().cloned();
        let payload_present = payload.is_some();
        Self {
            payload,
            tov: output.tov,
            metadata: output.metadata.clone(),
            payload_present,
            payload_template: (!payload_present).then(T::default),
        }
    }

    fn into_output(self) -> CuMsg<T> {
        let mut output = CuMsg::new(self.payload);
        output.tov = self.tov;
        output.metadata = self.metadata;
        output
    }
}

#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum ChildRequest<'a, I, S, O> {
    Process {
        input: &'a I,
        state: &'a S,
        output: &'a O,
    },
    Shutdown,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum ChildResponse<S, O> {
    Ready {
        #[serde(default)]
        cbor2_accelerated: bool,
    },
    Result {
        state: S,
        output: O,
    },
    Error {
        message: String,
    },
}

/// A Copper task whose `process(...)` implementation is provided by a Python
/// script.
///
/// The configured script must define:
///
/// ```python
/// def process(input, state, output):
///     ...
/// ```
///
/// Configuration keys:
///
/// - `script`: path to the Python file, default `python/task.py`
/// - `mode`: `"process"` or `"embedded"`, default `"process"`
///
/// `input`, `state`, and `output` are all converted into owned values before the
/// Python call. That makes the API flexible, but it also means allocations and
/// copies happen on every call. This is the core reason the type is suitable for
/// prototyping only and strongly discouraged on a realtime path.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct PyTask<I, S, O>
where
    I: PyInputSpec + 'static,
    S: PyTaskState + 'static,
    O: PyOutputSpec + 'static,
{
    mode: PyTaskMode,
    script: String,
    state: S,
    #[reflect(ignore)]
    backend: Option<PythonBackend>,
    #[reflect(ignore)]
    marker: PhantomData<fn() -> (I, O)>,
}

impl<I, S, O> TypePath for PyTask<I, S, O>
where
    I: PyInputSpec + 'static,
    S: PyTaskState + 'static,
    O: PyOutputSpec + 'static,
{
    fn type_path() -> &'static str {
        "cu_python_task::PyTask"
    }

    fn short_type_path() -> &'static str {
        "PyTask"
    }

    fn type_ident() -> Option<&'static str> {
        Some("PyTask")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_python_task")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_python_task")
    }
}

impl<I, S, O> Freezable for PyTask<I, S, O>
where
    I: PyInputSpec + 'static,
    S: PyTaskState + 'static,
    O: PyOutputSpec + 'static,
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let bytes = minicbor_serde::to_vec(&self.state)
            .map_err(|e| EncodeError::OtherString(e.to_string()))?;
        Encode::encode(&bytes, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let bytes = <Vec<u8> as Decode<D::Context>>::decode(decoder)?;
        self.state = minicbor_serde::from_slice(&bytes)
            .map_err(|e| DecodeError::OtherString(e.to_string()))?;
        Ok(())
    }
}

impl<I, S, O> CuTask for PyTask<I, S, O>
where
    I: PyInputSpec + 'static,
    S: PyTaskState + 'static,
    O: PyOutputSpec + 'static,
{
    type Resources<'r> = ();
    type Input<'m> = I::Input<'m>;
    type Output<'m> = O::Output<'m>;

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let mode = parse_mode(config)?;
        let script = resolve_script_path(config)?;
        if !script.is_file() {
            return Err(CuError::from(format!(
                "Python task script '{}' does not exist",
                script.display()
            )));
        }
        Ok(Self {
            mode,
            script: script.display().to_string(),
            state: S::default(),
            backend: None,
            marker: PhantomData,
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        if self.backend.is_none() {
            self.backend = Some(PythonBackend::start(self.mode, Path::new(&self.script))?);
        }
        Ok(())
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if self.backend.is_none() {
            self.start(ctx)?;
        }

        let request = ProcessRequest {
            input: I::to_owned(input),
            state: self.state.clone(),
            output: O::to_owned(output),
        };

        let backend = self
            .backend
            .as_mut()
            .expect("backend initialized immediately above");
        let ProcessResult {
            state,
            output: new_output,
        } = backend.process(&request)?;
        self.state = state;
        O::replace_output(output, new_output);
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        if let Some(mut backend) = self.backend.take() {
            backend.stop()?;
        }
        Ok(())
    }
}

fn parse_mode(config: Option<&ComponentConfig>) -> CuResult<PyTaskMode> {
    let Some(config) = config else {
        return Ok(PyTaskMode::Process);
    };
    let Some(raw) = config.get::<String>("mode")? else {
        return Ok(PyTaskMode::Process);
    };
    PyTaskMode::parse(&raw).ok_or_else(|| {
        CuError::from(format!(
            "Unsupported Python task mode '{raw}', expected 'process' or 'embedded'"
        ))
    })
}

fn resolve_script_path(config: Option<&ComponentConfig>) -> CuResult<PathBuf> {
    let raw = if let Some(config) = config {
        config.get::<String>("script")?
    } else {
        None
    };

    let path = raw
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(DEFAULT_SCRIPT_PATH));
    absolutize_path(&path)
}

fn absolutize_path(path: &Path) -> CuResult<PathBuf> {
    if path.is_absolute() {
        return Ok(path.to_path_buf());
    }
    let cwd = std::env::current_dir()
        .map_err(|e| CuError::new_with_cause("Failed to read current working directory", e))?;
    Ok(cwd.join(path))
}

enum PythonBackend {
    Process(ProcessBackend),
    Embedded(EmbeddedBackend),
}

impl PythonBackend {
    fn start(mode: PyTaskMode, script: &Path) -> CuResult<Self> {
        match mode {
            PyTaskMode::Process => Ok(Self::Process(ProcessBackend::start(script)?)),
            PyTaskMode::Embedded => Ok(Self::Embedded(EmbeddedBackend::start(script)?)),
        }
    }

    fn process<I, S, O>(
        &mut self,
        request: &ProcessRequest<I, S, O>,
    ) -> CuResult<ProcessResult<S, O>>
    where
        I: Serialize,
        S: Serialize + DeserializeOwned,
        O: Serialize + DeserializeOwned,
    {
        match self {
            Self::Process(backend) => backend.process(request),
            Self::Embedded(backend) => backend.process(request),
        }
    }

    fn stop(&mut self) -> CuResult<()> {
        match self {
            Self::Process(backend) => backend.stop(),
            Self::Embedded(backend) => backend.stop(),
        }
    }
}

struct ProcessBackend {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
}

impl ProcessBackend {
    fn start(script: &Path) -> CuResult<Self> {
        let python = python_command()?;
        let mut child = Command::new(python)
            .arg("-u")
            .arg("-c")
            .arg(PYTHON_BOOTSTRAP)
            .arg(script)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|e| CuError::new_with_cause("Failed to spawn Python task process", e))?;

        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| CuError::from("Python task process did not expose stdin"))?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| CuError::from("Python task process did not expose stdout"))?;

        let mut backend = Self {
            child,
            stdin,
            stdout: BufReader::new(stdout),
        };
        backend.wait_ready()?;
        Ok(backend)
    }

    fn wait_ready(&mut self) -> CuResult<()> {
        let start = Instant::now();
        loop {
            if start.elapsed() > PROCESS_START_TIMEOUT {
                return Err(CuError::from(
                    "Timed out while waiting for Python task process to start",
                ));
            }

            if let Some(status) = self
                .child
                .try_wait()
                .map_err(|e| CuError::new_with_cause("Failed to poll Python task process", e))?
            {
                return Err(CuError::from(format!(
                    "Python task process exited during startup with status {status}"
                )));
            }

            match read_cbor_frame::<_, ChildResponse<(), ()>>(&mut self.stdout) {
                Ok(ChildResponse::Ready { cbor2_accelerated }) => {
                    if !cbor2_accelerated {
                        warning!(
                            "cu_python_task process mode is using pure-Python cbor2; install the C extension backend for better performance"
                        );
                    }
                    return Ok(());
                }
                Ok(ChildResponse::Error { message }) => {
                    return Err(CuError::from(format!(
                        "Python task process failed during startup:\n{message}"
                    )));
                }
                Ok(ChildResponse::Result { .. }) => {
                    return Err(CuError::from(
                        "Unexpected result frame while waiting for Python task process startup",
                    ));
                }
                Err(error) => {
                    if start.elapsed() > PROCESS_START_TIMEOUT {
                        return Err(error);
                    }
                    thread::sleep(PROCESS_POLL_INTERVAL);
                }
            }
        }
    }

    fn process<I, S, O>(
        &mut self,
        request: &ProcessRequest<I, S, O>,
    ) -> CuResult<ProcessResult<S, O>>
    where
        I: Serialize,
        S: Serialize + DeserializeOwned,
        O: Serialize + DeserializeOwned,
    {
        let child_request = request.as_child_request();
        write_cbor_frame(&mut self.stdin, &child_request)?;
        match read_cbor_frame::<_, ChildResponse<S, O>>(&mut self.stdout)? {
            ChildResponse::Result { state, output } => Ok(ProcessResult { state, output }),
            ChildResponse::Error { message } => Err(CuError::from(format!(
                "Python task process raised an exception:\n{message}"
            ))),
            ChildResponse::Ready { .. } => Err(CuError::from(
                "Python task process unexpectedly sent a second ready frame",
            )),
        }
    }

    fn stop(&mut self) -> CuResult<()> {
        let _ = write_cbor_frame(&mut self.stdin, &ChildRequest::<(), (), ()>::Shutdown);

        let deadline = Instant::now() + PROCESS_STOP_TIMEOUT;
        loop {
            if let Some(_status) = self
                .child
                .try_wait()
                .map_err(|e| CuError::new_with_cause("Failed to poll Python task process", e))?
            {
                return Ok(());
            }

            if Instant::now() >= deadline {
                self.child.kill().map_err(|e| {
                    CuError::new_with_cause("Failed to terminate Python task process", e)
                })?;
                self.child.wait().map_err(|e| {
                    CuError::new_with_cause("Failed to reap Python task process", e)
                })?;
                return Ok(());
            }

            thread::sleep(PROCESS_POLL_INTERVAL);
        }
    }
}

#[cfg(not(target_os = "macos"))]
struct EmbeddedBackend {
    call_process: Py<PyAny>,
    process_fn: Py<PyAny>,
}

#[cfg(not(target_os = "macos"))]
impl EmbeddedBackend {
    fn start(script: &Path) -> CuResult<Self> {
        Python::initialize();
        Python::attach(|py| {
            let code = CString::new(PYTHON_BOOTSTRAP).map_err(|e| {
                CuError::new_with_cause("Invalid embedded Python bootstrap code", e)
            })?;
            let filename = CString::new("cu_python_task_bootstrap.py")
                .map_err(|e| CuError::new_with_cause("Invalid bootstrap filename", e))?;
            let module_name = CString::new("cu_python_task_bootstrap")
                .map_err(|e| CuError::new_with_cause("Invalid bootstrap module name", e))?;

            let module = PyModule::from_code(
                py,
                code.as_c_str(),
                filename.as_c_str(),
                module_name.as_c_str(),
            )
            .map_err(python_error)?;
            let load_process = module
                .getattr("load_process_function")
                .map_err(python_error)?;
            let call_process = module.getattr("call_process").map_err(python_error)?;
            let process_fn = load_process
                .call1((script.display().to_string(),))
                .map_err(python_error)?;
            Ok(Self {
                call_process: call_process.unbind(),
                process_fn: process_fn.unbind(),
            })
        })
    }

    fn process<I, S, O>(
        &mut self,
        request: &ProcessRequest<I, S, O>,
    ) -> CuResult<ProcessResult<S, O>>
    where
        I: Serialize,
        S: Serialize + DeserializeOwned,
        O: Serialize + DeserializeOwned,
    {
        let request_value = to_value(request)
            .map_err(|e| CuError::new_with_cause("Failed to encode embedded Python request", e))?;
        let response_value = Python::attach(|py| {
            let call_process = self.call_process.bind(py);
            let process_fn = self.process_fn.bind(py);
            let request_py = value_to_py(&request_value, py).map_err(python_error)?;
            call_process
                .call1((process_fn, request_py))
                .map_err(python_error)
                .and_then(|response| py_to_value(&response).map_err(python_error))
        })?;
        response_value
            .deserialize_into::<ProcessResult<S, O>>()
            .map_err(|e| CuError::new_with_cause("Failed to decode embedded Python response", e))
    }

    fn stop(&mut self) -> CuResult<()> {
        Ok(())
    }
}

#[cfg(target_os = "macos")]
struct EmbeddedBackend;

#[cfg(target_os = "macos")]
impl EmbeddedBackend {
    fn start(_script: &Path) -> CuResult<Self> {
        Err(CuError::from(
            "Embedded Python tasks are not supported on macOS in this workspace",
        ))
    }

    fn process<I, S, O>(
        &mut self,
        _request: &ProcessRequest<I, S, O>,
    ) -> CuResult<ProcessResult<S, O>>
    where
        I: Serialize,
        S: Serialize + DeserializeOwned,
        O: Serialize + DeserializeOwned,
    {
        Err(CuError::from(
            "Embedded Python tasks are not supported on macOS in this workspace",
        ))
    }

    fn stop(&mut self) -> CuResult<()> {
        Ok(())
    }
}

#[cfg(not(target_os = "macos"))]
fn python_error(error: PyErr) -> CuError {
    CuError::from(format!("Embedded Python task error: {error}"))
}

fn python_command() -> CuResult<&'static str> {
    static PYTHON_COMMAND: OnceLock<Option<&'static str>> = OnceLock::new();
    match PYTHON_COMMAND.get_or_init(detect_python_command) {
        Some(command) => Ok(*command),
        None => Err(CuError::from(
            "Could not find a usable Python interpreter (`python3` or `python`) in PATH",
        )),
    }
}

fn detect_python_command() -> Option<&'static str> {
    PYTHON_COMMAND_CANDIDATES.iter().copied().find(|command| {
        Command::new(command)
            .arg("--version")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .map(|status| status.success())
            .unwrap_or(false)
    })
}

fn write_cbor_frame<W, T>(writer: &mut W, value: &T) -> CuResult<()>
where
    W: Write,
    T: Serialize,
{
    let value = to_value(value)
        .map_err(|e| CuError::new_with_cause("Failed to encode Python task CBOR value", e))?;
    let payload = encode_wire_value(&value)?;
    if payload.len() > MAX_CBOR_FRAME_BYTES {
        return Err(CuError::from(format!(
            "Python task frame exceeded {} bytes",
            MAX_CBOR_FRAME_BYTES
        )));
    }
    let len = u32::try_from(payload.len())
        .map_err(|_| CuError::from("Python task frame length exceeded u32"))?;
    writer
        .write_all(&len.to_le_bytes())
        .map_err(|e| CuError::new_with_cause("Failed to write Python task frame length", e))?;
    writer
        .write_all(&payload)
        .map_err(|e| CuError::new_with_cause("Failed to write Python task frame payload", e))?;
    writer
        .flush()
        .map_err(|e| CuError::new_with_cause("Failed to flush Python task frame", e))
}

fn read_cbor_frame<R, T>(reader: &mut R) -> CuResult<T>
where
    R: Read,
    T: DeserializeOwned,
{
    let mut len_bytes = [0_u8; 4];
    reader
        .read_exact(&mut len_bytes)
        .map_err(|e| CuError::new_with_cause("Failed to read Python task frame length", e))?;
    let len = u32::from_le_bytes(len_bytes) as usize;
    if len > MAX_CBOR_FRAME_BYTES {
        return Err(CuError::from(format!(
            "Python task frame exceeded {} bytes",
            MAX_CBOR_FRAME_BYTES
        )));
    }

    let mut payload = vec![0_u8; len];
    reader
        .read_exact(&mut payload)
        .map_err(|e| CuError::new_with_cause("Failed to read Python task frame payload", e))?;
    let value = decode_wire_value(&payload)?;
    value
        .deserialize_into::<T>()
        .map_err(|e| CuError::new_with_cause("Failed to decode Python task CBOR frame", e))
}

fn encode_wire_value(value: &CuValue) -> CuResult<Vec<u8>> {
    let mut encoder = CborEncoder::new(Vec::new());
    encode_wire_value_into(&mut encoder, value)
        .map_err(|e| CuError::new_with_cause("Failed to serialize Python task CBOR frame", e))?;
    Ok(encoder.into_writer())
}

fn encode_wire_value_into<W>(
    encoder: &mut CborEncoder<W>,
    value: &CuValue,
) -> Result<(), minicbor::encode::Error<W::Error>>
where
    W: minicbor::encode::Write,
{
    match value {
        CuValue::Bool(v) => {
            encoder.bool(*v)?;
        }
        CuValue::U8(v) => {
            encoder.u8(*v)?;
        }
        CuValue::U16(v) => {
            encoder.u16(*v)?;
        }
        CuValue::U32(v) => {
            encoder.u32(*v)?;
        }
        CuValue::U64(v) => {
            encoder.u64(*v)?;
        }
        CuValue::U128(v) => {
            if let Ok(v) = u64::try_from(*v) {
                encoder.u64(v)?;
            } else {
                encode_bignum(encoder, IanaTag::PosBignum, *v)?;
            }
        }
        CuValue::I8(v) => {
            encoder.i8(*v)?;
        }
        CuValue::I16(v) => {
            encoder.i16(*v)?;
        }
        CuValue::I32(v) => {
            encoder.i32(*v)?;
        }
        CuValue::I64(v) => {
            encoder.i64(*v)?;
        }
        CuValue::I128(v) => {
            if let Ok(v) = CborInt::try_from(*v) {
                encoder.int(v)?;
            } else {
                encode_bignum(encoder, IanaTag::NegBignum, (-1_i128 - *v) as u128)?;
            }
        }
        CuValue::F32(v) => {
            encoder.f32(*v)?;
        }
        CuValue::F64(v) => {
            encoder.f64(*v)?;
        }
        CuValue::Char(v) => {
            encoder.char(*v)?;
        }
        CuValue::String(v) => {
            encoder.str(v)?;
        }
        CuValue::Unit | CuValue::Option(None) => {
            encoder.null()?;
        }
        CuValue::Option(Some(v)) | CuValue::Newtype(v) => {
            encode_wire_value_into(encoder, v)?;
        }
        CuValue::Seq(values) => {
            encoder.array(values.len() as u64)?;
            for value in values {
                encode_wire_value_into(encoder, value)?;
            }
        }
        CuValue::Map(values) => {
            encoder.map(values.len() as u64)?;
            for (key, value) in values {
                encode_wire_value_into(encoder, key)?;
                encode_wire_value_into(encoder, value)?;
            }
        }
        CuValue::Bytes(value) => {
            encoder.bytes(value)?;
        }
        CuValue::CuTime(value) => {
            encoder.u64(value.0)?;
        }
    }
    Ok(())
}

fn encode_bignum<W>(
    encoder: &mut CborEncoder<W>,
    tag: IanaTag,
    value: u128,
) -> Result<(), minicbor::encode::Error<W::Error>>
where
    W: minicbor::encode::Write,
{
    let bytes = value.to_be_bytes();
    let start = bytes
        .iter()
        .position(|byte| *byte != 0)
        .unwrap_or(bytes.len() - 1);
    encoder.tag(tag)?.bytes(&bytes[start..])?;
    Ok(())
}

fn decode_wire_value(payload: &[u8]) -> CuResult<CuValue> {
    let mut decoder = CborDecoder::new(payload);
    let value = decode_wire_value_from(&mut decoder)
        .map_err(|e| CuError::new_with_cause("Failed to decode Python task CBOR frame", e))?;
    if decoder.position() != payload.len() {
        return Err(CuError::from(
            "Failed to decode Python task CBOR frame: trailing data",
        ));
    }
    Ok(canonicalize_wire_value(value))
}

fn decode_wire_value_from(
    decoder: &mut CborDecoder<'_>,
) -> Result<CuValue, minicbor::decode::Error> {
    match decoder.datatype()? {
        CborType::Bool => Ok(CuValue::Bool(decoder.bool()?)),
        CborType::Null => {
            decoder.null()?;
            Ok(CuValue::Option(None))
        }
        CborType::U8 => Ok(CuValue::U8(decoder.u8()?)),
        CborType::U16 => Ok(CuValue::U16(decoder.u16()?)),
        CborType::U32 => Ok(CuValue::U32(decoder.u32()?)),
        CborType::U64 => Ok(CuValue::U64(decoder.u64()?)),
        CborType::I8 => Ok(CuValue::I8(decoder.i8()?)),
        CborType::I16 => Ok(CuValue::I16(decoder.i16()?)),
        CborType::I32 => Ok(CuValue::I32(decoder.i32()?)),
        CborType::I64 => Ok(CuValue::I64(decoder.i64()?)),
        CborType::Int => decode_cbor_int(decoder.int()?),
        CborType::F32 => Ok(CuValue::F32(decoder.f32()?)),
        CborType::F64 => Ok(CuValue::F64(decoder.f64()?)),
        CborType::Bytes | CborType::BytesIndef => Ok(CuValue::Bytes(read_byte_string(decoder)?)),
        CborType::String | CborType::StringIndef => Ok(CuValue::String(read_text_string(decoder)?)),
        CborType::Array | CborType::ArrayIndef => decode_array(decoder),
        CborType::Map | CborType::MapIndef => decode_map(decoder),
        CborType::Tag => decode_tagged_value(decoder),
        other => Err(minicbor::decode::Error::message(format!(
            "unsupported Python task CBOR type {other}"
        ))),
    }
}

fn decode_cbor_int(value: CborInt) -> Result<CuValue, minicbor::decode::Error> {
    if let Ok(value) = u64::try_from(value) {
        Ok(CuValue::U64(value))
    } else if let Ok(value) = i64::try_from(value) {
        Ok(CuValue::I64(value))
    } else {
        Ok(CuValue::I128(i128::from(value)))
    }
}

fn decode_array(decoder: &mut CborDecoder<'_>) -> Result<CuValue, minicbor::decode::Error> {
    let len = decoder.array()?;
    let mut values = Vec::new();
    match len {
        Some(len) => {
            values.reserve(len as usize);
            for _ in 0..len {
                values.push(decode_wire_value_from(decoder)?);
            }
        }
        None => loop {
            if decoder.datatype()? == CborType::Break {
                decoder.skip()?;
                break;
            }
            values.push(decode_wire_value_from(decoder)?);
        },
    }
    Ok(CuValue::Seq(values))
}

fn decode_map(decoder: &mut CborDecoder<'_>) -> Result<CuValue, minicbor::decode::Error> {
    let len = decoder.map()?;
    let mut values = BTreeMap::new();
    match len {
        Some(len) => {
            for _ in 0..len {
                let key = decode_wire_value_from(decoder)?;
                let value = decode_wire_value_from(decoder)?;
                values.insert(key, value);
            }
        }
        None => loop {
            if decoder.datatype()? == CborType::Break {
                decoder.skip()?;
                break;
            }
            let key = decode_wire_value_from(decoder)?;
            let value = decode_wire_value_from(decoder)?;
            values.insert(key, value);
        },
    }
    Ok(CuValue::Map(values))
}

fn decode_tagged_value(decoder: &mut CborDecoder<'_>) -> Result<CuValue, minicbor::decode::Error> {
    match IanaTag::try_from(decoder.tag()?) {
        Ok(IanaTag::PosBignum) => {
            let value = decode_bignum(decoder)?;
            if let Ok(value) = u64::try_from(value) {
                Ok(CuValue::U64(value))
            } else {
                Ok(CuValue::U128(value))
            }
        }
        Ok(IanaTag::NegBignum) => {
            let value = decode_bignum(decoder)?;
            if value <= i64::MAX as u128 {
                Ok(CuValue::I64(-1 - value as i64))
            } else if let Ok(value) = i128::try_from(value) {
                Ok(CuValue::I128(-1 - value))
            } else {
                Err(minicbor::decode::Error::message(
                    "Python task CBOR negative bignum exceeded i128",
                ))
            }
        }
        Ok(other) => Err(minicbor::decode::Error::message(format!(
            "unsupported Python task CBOR tag {}",
            u64::from(other)
        ))),
        Err(tag) => Err(minicbor::decode::Error::message(format!(
            "unsupported Python task CBOR tag {tag}",
        ))),
    }
}

fn decode_bignum(decoder: &mut CborDecoder<'_>) -> Result<u128, minicbor::decode::Error> {
    let bytes = read_byte_string(decoder)?;
    let first = bytes
        .iter()
        .position(|byte| *byte != 0)
        .unwrap_or(bytes.len());
    let bytes = &bytes[first..];
    if bytes.len() > 16 {
        return Err(minicbor::decode::Error::message(
            "Python task CBOR bignum exceeded u128",
        ));
    }
    let mut buffer = [0_u8; 16];
    buffer[16 - bytes.len()..].copy_from_slice(bytes);
    Ok(u128::from_be_bytes(buffer))
}

fn read_byte_string(decoder: &mut CborDecoder<'_>) -> Result<Vec<u8>, minicbor::decode::Error> {
    match decoder.datatype()? {
        CborType::Bytes => Ok(decoder.bytes()?.to_vec()),
        CborType::BytesIndef => {
            let mut bytes = Vec::new();
            for chunk in decoder.bytes_iter()? {
                bytes.extend_from_slice(chunk?);
            }
            Ok(bytes)
        }
        other => Err(minicbor::decode::Error::message(format!(
            "expected byte string, found {other}"
        ))),
    }
}

fn read_text_string(decoder: &mut CborDecoder<'_>) -> Result<String, minicbor::decode::Error> {
    match decoder.datatype()? {
        CborType::String => Ok(decoder.str()?.to_owned()),
        CborType::StringIndef => {
            let mut text = String::new();
            for chunk in decoder.str_iter()? {
                text.push_str(chunk?);
            }
            Ok(text)
        }
        other => Err(minicbor::decode::Error::message(format!(
            "expected text string, found {other}"
        ))),
    }
}

fn canonicalize_wire_value(value: CuValue) -> CuValue {
    match value {
        CuValue::Option(None) => CuValue::Unit,
        CuValue::Option(Some(value)) | CuValue::Newtype(value) => canonicalize_wire_value(*value),
        CuValue::Seq(values) => CuValue::Seq(
            values
                .into_iter()
                .map(canonicalize_wire_value)
                .collect::<Vec<_>>(),
        ),
        CuValue::Map(values) => CuValue::Map(
            values
                .into_iter()
                .map(|(key, value)| (canonicalize_wire_value(key), canonicalize_wire_value(value)))
                .collect::<BTreeMap<_, _>>(),
        ),
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, MutexGuard, OnceLock};
    use tempfile::TempDir;

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    struct TestPayload {
        value: i32,
        flag: bool,
    }

    #[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    struct SeenState {
        seen: bool,
    }

    fn cwd_lock() -> MutexGuard<'static, ()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
            .lock()
            .expect("cwd lock poisoned")
    }

    struct CurrentDirGuard {
        original: PathBuf,
    }

    impl CurrentDirGuard {
        fn switch_to(path: &Path) -> Self {
            let original = std::env::current_dir().expect("cwd");
            std::env::set_current_dir(path).expect("switch cwd");
            Self { original }
        }
    }

    impl Drop for CurrentDirGuard {
        fn drop(&mut self) {
            std::env::set_current_dir(&self.original).expect("restore cwd");
        }
    }

    #[test]
    fn mode_defaults_to_process() {
        assert_eq!(parse_mode(None).unwrap(), PyTaskMode::Process);
    }

    #[test]
    fn mode_parses_embedded() {
        let mut cfg = ComponentConfig::new();
        cfg.set("mode", "embedded".to_string());
        assert_eq!(parse_mode(Some(&cfg)).unwrap(), PyTaskMode::Embedded);
    }

    #[test]
    fn invalid_mode_is_rejected() {
        let mut cfg = ComponentConfig::new();
        cfg.set("mode", "nope".to_string());
        let err = parse_mode(Some(&cfg)).expect_err("mode should fail");
        assert!(err.to_string().contains("Unsupported Python task mode"));
    }

    #[test]
    fn default_script_path_uses_python_task_py_from_cwd() {
        let _lock = cwd_lock();
        let temp_dir = TempDir::new().expect("temp dir");
        let _cwd = CurrentDirGuard::switch_to(temp_dir.path());
        let cwd = std::env::current_dir().expect("cwd after switch");
        let resolved = resolve_script_path(None).expect("resolve");
        assert_eq!(resolved, cwd.join(DEFAULT_SCRIPT_PATH));
    }

    #[test]
    fn relative_script_paths_are_absolutized() {
        let _lock = cwd_lock();
        let temp_dir = TempDir::new().expect("temp dir");
        let _cwd = CurrentDirGuard::switch_to(temp_dir.path());
        let cwd = std::env::current_dir().expect("cwd after switch");

        let mut cfg = ComponentConfig::new();
        cfg.set("script", "scripts/example.py".to_string());
        let resolved = resolve_script_path(Some(&cfg)).expect("resolve");

        assert_eq!(resolved, cwd.join("scripts/example.py"));
    }

    fn write_test_script(contents: &str) -> (TempDir, PathBuf) {
        let temp_dir = TempDir::new().expect("temp dir");
        let script = temp_dir.path().join("task.py");
        std::fs::write(&script, contents).expect("write script");
        (temp_dir, script)
    }

    #[test]
    fn process_backend_keeps_absent_output_payload_when_python_does_not_touch_it() {
        let (_temp_dir, script) =
            write_test_script("def process(inp, state, output):\n    state['seen'] = True\n");
        let mut backend = ProcessBackend::start(&script).expect("start backend");
        let output = PyCuMsg::from_output(&CuMsg::<TestPayload>::default());

        let result: ProcessResult<SeenState, PyCuMsg<TestPayload>> = backend
            .process(&ProcessRequest {
                input: (),
                state: SeenState::default(),
                output,
            })
            .expect("process request");
        backend.stop().expect("stop backend");

        assert!(result.state.seen);
        assert!(result.output.payload.is_none());
    }

    #[test]
    fn process_backend_supports_attribute_writes_on_absent_output_payload() {
        let (_temp_dir, script) = write_test_script(
            "def process(inp, state, output):\n    output.payload.value = 41\n    output.payload.flag = True\n",
        );
        let mut backend = ProcessBackend::start(&script).expect("start backend");
        let output = PyCuMsg::from_output(&CuMsg::<TestPayload>::default());

        let result: ProcessResult<(), PyCuMsg<TestPayload>> = backend
            .process(&ProcessRequest {
                input: (),
                state: (),
                output,
            })
            .expect("process request");
        backend.stop().expect("stop backend");

        assert_eq!(
            result.output.payload,
            Some(TestPayload {
                value: 41,
                flag: true,
            })
        );
    }

    #[test]
    fn cbor_wire_frames_round_trip_128_bit_integers() {
        let mut buffer = Vec::new();
        let large_u128 = u128::from(u64::MAX) + 99;
        let large_i128 = i128::from(i64::MIN) - 99;

        write_cbor_frame(&mut buffer, &large_u128).expect("serialize u128");
        let decoded_u128 = read_cbor_frame::<_, u128>(&mut std::io::Cursor::new(&buffer))
            .expect("deserialize u128");
        assert_eq!(decoded_u128, large_u128);

        buffer.clear();

        write_cbor_frame(&mut buffer, &large_i128).expect("serialize i128");
        let decoded_i128 = read_cbor_frame::<_, i128>(&mut std::io::Cursor::new(&buffer))
            .expect("deserialize i128");
        assert_eq!(decoded_i128, large_i128);
    }

    #[cfg(not(target_os = "macos"))]
    #[test]
    fn embedded_backend_supports_attribute_writes_on_absent_output_payload() {
        let (_temp_dir, script) = write_test_script(
            "def process(inp, state, output):\n    output.payload.value = 41\n    output.payload.flag = True\n",
        );
        let mut backend = EmbeddedBackend::start(&script).expect("start backend");
        let output = PyCuMsg::from_output(&CuMsg::<TestPayload>::default());

        let result: ProcessResult<(), PyCuMsg<TestPayload>> = backend
            .process(&ProcessRequest {
                input: (),
                state: (),
                output,
            })
            .expect("process request");
        backend.stop().expect("stop backend");

        assert_eq!(
            result.output.payload,
            Some(TestPayload {
                value: 41,
                flag: true,
            })
        );
    }
}
