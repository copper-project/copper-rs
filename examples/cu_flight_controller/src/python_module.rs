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
#[cfg(feature = "python-bindings")]
use std::path::Path;

#[cfg(feature = "python-bindings")]
gen_cumsgs!("copperconfig.ron");

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
    cu29_export::copperlist_iterator_unified_typed_py::<cumsgs::gnss::CuStampedDataSet>(
        unified_src_path,
        py,
    )
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
    Ok(())
}
