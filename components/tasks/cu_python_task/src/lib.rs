use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
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
use pyo3::types::PyModule;

const DEFAULT_SCRIPT_PATH: &str = "python/task.py";
const MAX_JSON_FRAME_BYTES: usize = 16 * 1024 * 1024;
const PROCESS_START_TIMEOUT: Duration = Duration::from_secs(5);
const PROCESS_STOP_TIMEOUT: Duration = Duration::from_secs(2);
const PROCESS_POLL_INTERVAL: Duration = Duration::from_millis(10);
const PYTHON_BOOTSTRAP: &str = include_str!("bootstrap.py");
const PYTHON_COMMAND_CANDIDATES: &[&str] = &["python3", "python"];

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

pub trait PyInputSpec {
    type Input<'m>: CuMsgPack
    where
        Self: 'm;
    type Owned: Clone + Serialize + DeserializeOwned;

    fn to_owned(input: &Self::Input<'_>) -> Self::Owned;
}

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
    type Owned = CuMsg<T>;

    fn to_owned(output: &Self::Output<'_>) -> Self::Owned {
        output.clone()
    }

    fn replace_output(output: &mut Self::Output<'_>, owned: Self::Owned) {
        *output = owned;
    }
}

macro_rules! impl_py_output_spec_tuple {
    ($first_ty:ident, $second_ty:ident $(, $ty:ident)* $(,)?) => {
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
            type Owned = (CuMsg<$first_ty>, CuMsg<$second_ty> $(, CuMsg<$ty>)*);

            fn to_owned(output: &Self::Output<'_>) -> Self::Owned {
                output.clone()
            }

            fn replace_output(output: &mut Self::Output<'_>, owned: Self::Owned) {
                *output = owned;
            }
        }
    };
}

impl_py_output_spec_tuple!(T1, T2);
impl_py_output_spec_tuple!(T1, T2, T3);
impl_py_output_spec_tuple!(T1, T2, T3, T4);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5, T6);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5, T6, T7);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5, T6, T7, T8);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5, T6, T7, T8, T9);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5, T6, T7, T8, T9, T10);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11);
impl_py_output_spec_tuple!(T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Reflect)]
pub enum PyTaskMode {
    Process,
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
    kind: &'static str,
    input: I,
    state: S,
    output: O,
}

#[derive(Serialize, Deserialize)]
struct ProcessResult<S, O> {
    state: S,
    output: O,
}

#[derive(Serialize)]
struct ShutdownRequest {
    kind: &'static str,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum ChildResponse<S, O> {
    Ready,
    Result { state: S, output: O },
    Error { message: String },
}

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
        let bytes =
            serde_json::to_vec(&self.state).map_err(|e| EncodeError::OtherString(e.to_string()))?;
        Encode::encode(&bytes, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let bytes = <Vec<u8> as Decode<D::Context>>::decode(decoder)?;
        self.state =
            serde_json::from_slice(&bytes).map_err(|e| DecodeError::OtherString(e.to_string()))?;
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
            kind: "process",
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

            match read_json_frame::<_, ChildResponse<(), ()>>(&mut self.stdout) {
                Ok(ChildResponse::Ready) => return Ok(()),
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
        write_json_frame(&mut self.stdin, request)?;
        match read_json_frame::<_, ChildResponse<S, O>>(&mut self.stdout)? {
            ChildResponse::Result { state, output } => Ok(ProcessResult { state, output }),
            ChildResponse::Error { message } => Err(CuError::from(format!(
                "Python task process raised an exception:\n{message}"
            ))),
            ChildResponse::Ready => Err(CuError::from(
                "Python task process unexpectedly sent a second ready frame",
            )),
        }
    }

    fn stop(&mut self) -> CuResult<()> {
        let _ = write_json_frame(&mut self.stdin, &ShutdownRequest { kind: "shutdown" });

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
    call_process_json: Py<PyAny>,
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
            let call_process_json = module.getattr("call_process_json").map_err(python_error)?;
            let process_fn = load_process
                .call1((script.display().to_string(),))
                .map_err(python_error)?;
            Ok(Self {
                call_process_json: call_process_json.unbind(),
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
        let request_json = serde_json::to_string(request)
            .map_err(|e| CuError::new_with_cause("Failed to encode embedded Python request", e))?;
        let response_json = Python::attach(|py| {
            let call_process = self.call_process_json.bind(py);
            let process_fn = self.process_fn.bind(py);
            call_process
                .call1((process_fn, request_json.as_str()))
                .map_err(python_error)?
                .extract::<String>()
                .map_err(python_error)
        })?;
        serde_json::from_str::<ProcessResult<S, O>>(&response_json)
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

fn write_json_frame<W, T>(writer: &mut W, value: &T) -> CuResult<()>
where
    W: Write,
    T: Serialize,
{
    let payload = serde_json::to_vec(value)
        .map_err(|e| CuError::new_with_cause("Failed to serialize Python task frame", e))?;
    if payload.len() > MAX_JSON_FRAME_BYTES {
        return Err(CuError::from(format!(
            "Python task frame exceeded {} bytes",
            MAX_JSON_FRAME_BYTES
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

fn read_json_frame<R, T>(reader: &mut R) -> CuResult<T>
where
    R: Read,
    T: DeserializeOwned,
{
    let mut len_bytes = [0_u8; 4];
    reader
        .read_exact(&mut len_bytes)
        .map_err(|e| CuError::new_with_cause("Failed to read Python task frame length", e))?;
    let len = u32::from_le_bytes(len_bytes) as usize;
    if len > MAX_JSON_FRAME_BYTES {
        return Err(CuError::from(format!(
            "Python task frame exceeded {} bytes",
            MAX_JSON_FRAME_BYTES
        )));
    }

    let mut payload = vec![0_u8; len];
    reader
        .read_exact(&mut payload)
        .map_err(|e| CuError::new_with_cause("Failed to read Python task frame payload", e))?;
    serde_json::from_slice(&payload)
        .map_err(|e| CuError::new_with_cause("Failed to decode Python task frame", e))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, MutexGuard, OnceLock};
    use tempfile::TempDir;

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
}
