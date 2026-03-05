#[cfg(feature = "python-bindings")]
extern crate cu29 as bevy;

#[cfg(feature = "python-bindings")]
mod messages;

#[cfg(feature = "python-bindings")]
use cu29::prelude::*;
#[cfg(feature = "python-bindings")]
use pyo3::prelude::*;

#[cfg(feature = "python-bindings")]
gen_cumsgs!("copperconfig.ron");

#[cfg(feature = "python-bindings")]
#[pyfunction]
fn copperlist_iterator_unified(unified_src_path: &str, py: Python<'_>) -> PyResult<Py<PyAny>> {
    cu29_export::copperlist_iterator_unified_typed_py::<cumsgs::gnss::CuStampedDataSet>(
        unified_src_path,
        py,
    )
}

/// The compiled shared object is named `libcu_flight_controller_export.*`.
#[cfg(feature = "python-bindings")]
#[pymodule(name = "libcu_flight_controller_export")]
fn cu_flight_controller_export(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(copperlist_iterator_unified, m)?)?;
    Ok(())
}
