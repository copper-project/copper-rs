#![cfg_attr(not(feature = "python-bindings"), no_std)]
//! App-specific Python bindings for offline log inspection.
//!
//! This module demonstrates the intended Copper pattern for typed Python log
//! access:
//!
//! - keep the runtime in Rust
//! - generate the app-specific CopperList type with `gen_cumsgs!`
//! - expose a tiny `#[pymodule]` wrapper for offline analysis scripts
//!
//! This is not the same thing as running task logic in Python.

#[cfg(feature = "python-bindings")]
extern crate cu29 as bevy;

#[cfg(feature = "python-bindings")]
mod messages;

#[cfg(feature = "python-bindings")]
use cu29::prelude::*;
#[cfg(feature = "python-bindings")]
use pyo3::prelude::*;
#[cfg(feature = "python-vitfly-bindings")]
use pyo3::types::{PyBytes, PyDict};
#[cfg(feature = "python-bindings")]
use std::path::Path;

#[cfg(feature = "python-bindings")]
mod gnss_schema {
    use super::*;

    gen_cumsgs!("copperconfig.ron");

    pub type DataSet = cumsgs::gnss::CuStampedDataSet;
}

#[cfg(feature = "python-vitfly-bindings")]
mod vitfly_schema {
    use super::*;

    gen_cumsgs!("compute_vitfly_config.ron");

    pub type DataSet = CuStampedDataSet;
}

#[cfg(feature = "python-vitfly-bindings")]
type VitFlyCopperList = cu29::copperlist::CopperList<vitfly_schema::DataSet>;

#[cfg(feature = "python-bindings")]
const EXPECTED_MISSION: &str = "gnss";

#[cfg(feature = "python-bindings")]
fn ensure_expected_mission(unified_src_path: &str) -> PyResult<()> {
    cu29_export::assert_unified_log_mission(Path::new(unified_src_path), EXPECTED_MISSION)
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
}

#[cfg(feature = "python-bindings")]
#[pyfunction]
fn copperlist_iterator_unified(unified_src_path: &str, py: Python<'_>) -> PyResult<Py<PyAny>> {
    ensure_expected_mission(unified_src_path)?;
    cu29_export::copperlist_iterator_unified_typed_py::<gnss_schema::DataSet>(unified_src_path, py)
}

/// Iterator over the compute subsystem's complete VitFly inference samples.
///
/// The generic reflected CopperList iterator deliberately ignores `CuHandle`
/// contents. This specialized offline iterator exposes the logged depth handle
/// as little-endian f32 bytes while keeping the raster shape and associated
/// model context together.
#[cfg(feature = "python-vitfly-bindings")]
#[pyclass(unsendable)]
struct PyVitFlySampleIterator {
    inner: Box<dyn Iterator<Item = VitFlyCopperList> + Send>,
}

#[cfg(feature = "python-vitfly-bindings")]
#[pymethods]
impl PyVitFlySampleIterator {
    fn __iter__(slf: PyRefMut<Self>) -> PyRefMut<Self> {
        slf
    }

    fn __next__(&mut self, py: Python<'_>) -> Option<PyResult<Py<PyAny>>> {
        loop {
            let copperlist = self.inner.next()?;
            let depth_msg = copperlist.msgs.get_vitfly_depth_log_output();
            let Some(depth) = depth_msg.payload() else {
                continue;
            };

            let expected_len = depth.format.len_elements();
            let depth_bytes = depth.buffer_handle.with_inner(|values| {
                if values.len() < expected_len {
                    return None;
                }
                let mut bytes = Vec::with_capacity(expected_len * core::mem::size_of::<f32>());
                for value in values.iter().take(expected_len) {
                    bytes.extend_from_slice(&value.to_le_bytes());
                }
                Some(bytes)
            });
            let Some(depth_bytes) = depth_bytes else {
                return Some(Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "depth buffer for CopperList {} is shorter than its raster format",
                    copperlist.id
                ))));
            };

            let pose = copperlist
                .msgs
                .get_vitfly_context_output_0()
                .payload()
                .map(|pose| {
                    use cu29::units::si::angle::radian;
                    [
                        pose.roll.get::<radian>(),
                        pose.pitch.get::<radian>(),
                        pose.yaw.get::<radian>(),
                    ]
                });
            let desired_speed_mps =
                copperlist
                    .msgs
                    .get_vitfly_context_output_1()
                    .payload()
                    .map(|speed| {
                        use cu29::units::si::velocity::meter_per_second;
                        speed.get::<meter_per_second>()
                    });
            let task_velocity_mps = copperlist
                .msgs
                .get_vitfly_output()
                .payload()
                .map(velocity_mps);
            let command_velocity_mps =
                copperlist
                    .msgs
                    .get_vitfly_command_output()
                    .payload()
                    .map(|command| {
                        use cu29::units::si::velocity::meter_per_second;
                        [
                            command.north.get::<meter_per_second>(),
                            command.west.get::<meter_per_second>(),
                            command.up.get::<meter_per_second>(),
                        ]
                    });
            // This is the same precedence used by the simulator UI: the active,
            // conditioned command wins; otherwise it displays the task output.
            let preview_velocity_mps = command_velocity_mps.or(task_velocity_mps);

            let sample = PyDict::new(py);
            if let Err(error) = (|| -> PyResult<()> {
                sample.set_item("cl_id", copperlist.id)?;
                sample.set_item("frame_seq", depth.seq)?;
                sample.set_item("tov_ns", tov_time_ns(depth_msg.tov))?;
                sample.set_item("width", depth.format.width)?;
                sample.set_item("height", depth.format.height)?;
                sample.set_item("stride", depth.format.stride)?;
                sample.set_item("depth_f32_le", PyBytes::new(py, &depth_bytes))?;
                sample.set_item("pose_rpy_rad", pose.map(Vec::from))?;
                sample.set_item("desired_speed_mps", desired_speed_mps)?;
                sample.set_item("task_velocity_mps", task_velocity_mps.map(Vec::from))?;
                sample.set_item("command_velocity_mps", command_velocity_mps.map(Vec::from))?;
                sample.set_item("preview_velocity_mps", preview_velocity_mps.map(Vec::from))?;
                Ok(())
            })() {
                return Some(Err(error));
            }
            return Some(Ok(sample.into_any().unbind()));
        }
    }
}

#[cfg(feature = "python-vitfly-bindings")]
fn velocity_mps(velocity: &cu_vitfly::VitFlyVelocity) -> [f32; 3] {
    use cu29::units::si::velocity::meter_per_second;
    velocity.map(|axis| axis.get::<meter_per_second>())
}

#[cfg(feature = "python-vitfly-bindings")]
fn tov_time_ns(tov: Tov) -> Option<u64> {
    match tov {
        Tov::Time(time) => Some(time.as_nanos()),
        Tov::Range(range) => Some(range.end.as_nanos()),
        Tov::None => None,
    }
}

#[cfg(feature = "python-vitfly-bindings")]
#[pyfunction]
fn vitfly_sample_iterator_unified(unified_src_path: &str) -> PyResult<PyVitFlySampleIterator> {
    let path = Path::new(unified_src_path);
    cu29::logcodec::seed_effective_config_from_log::<vitfly_schema::DataSet>(path)
        .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(error.to_string()))?;
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .build()
        .map_err(|error| pyo3::exceptions::PyIOError::new_err(error.to_string()))?;
    let UnifiedLogger::Read(logger) = logger else {
        return Err(pyo3::exceptions::PyIOError::new_err(
            "expected a read-only unified logger for VitFly extraction",
        ));
    };
    let reader = UnifiedLoggerIOReader::new(logger, UnifiedLogType::CopperList);
    let inner = cu29_export::copperlists_reader::<vitfly_schema::DataSet>(reader);
    Ok(PyVitFlySampleIterator {
        inner: Box::new(inner),
    })
}

#[cfg(feature = "python-bindings")]
#[pyfunction]
fn runtime_lifecycle_iterator_unified(
    unified_src_path: &str,
    py: Python<'_>,
) -> PyResult<Py<PyAny>> {
    ensure_expected_mission(unified_src_path)?;
    cu29_export::runtime_lifecycle_iterator_unified_py(unified_src_path, py)
}

/// The compiled shared object is named `libcu_flight_controller_export.*`.
#[cfg(feature = "python-bindings")]
#[pymodule(name = "libcu_flight_controller_export")]
fn cu_flight_controller_export(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(copperlist_iterator_unified, m)?)?;
    m.add_function(wrap_pyfunction!(runtime_lifecycle_iterator_unified, m)?)?;
    #[cfg(feature = "python-vitfly-bindings")]
    {
        m.add_class::<PyVitFlySampleIterator>()?;
        m.add_function(wrap_pyfunction!(vitfly_sample_iterator_unified, m)?)?;
    }
    Ok(())
}
