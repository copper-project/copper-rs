//! Python bindings for run-scoped catalog decoding without native registrations.

use crate::catalog::{
    CopperListValueReader, CuDecodedCopperList, copperlist_values_reader, read_value_decode_catalog,
};
use cu29::prelude::Value;
use pyo3::exceptions::{PyIOError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use std::path::Path;

#[pyclass]
struct PyCopperListValueIterator {
    reader: CopperListValueReader,
}

#[pymethods]
impl PyCopperListValueIterator {
    fn __iter__(slf: PyRefMut<'_, Self>) -> PyRefMut<'_, Self> {
        slf
    }
    fn __next__(&mut self, py: Python<'_>) -> Option<PyResult<Py<PyAny>>> {
        self.reader.next().map(|entry| {
            let entry = entry.map_err(|error| PyIOError::new_err(error.to_string()))?;
            copperlist_to_py(&entry, py)
        })
    }
}

/// Return the selected run's complete catalog as a dictionary.
#[pyfunction]
#[pyo3(signature = (path, run=None))]
fn value_decode_catalog_unified(
    py: Python<'_>,
    path: &str,
    run: Option<usize>,
) -> PyResult<Py<PyAny>> {
    let catalog = read_value_decode_catalog(Path::new(path), run)
        .map_err(|error| PyIOError::new_err(error.to_string()))?;
    serde_to_py(&catalog, py)
}

/// Iterate CopperLists as dictionaries, using only the selected embedded catalog.
#[pyfunction]
#[pyo3(signature = (path, run=None))]
fn copperlist_value_iterator_unified(
    path: &str,
    run: Option<usize>,
) -> PyResult<PyCopperListValueIterator> {
    let reader = copperlist_values_reader(Path::new(path), run)
        .map_err(|error| PyIOError::new_err(error.to_string()))?;
    Ok(PyCopperListValueIterator { reader })
}

pub(crate) fn add_functions(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyCopperListValueIterator>()?;
    module.add_function(wrap_pyfunction!(value_decode_catalog_unified, module)?)?;
    module.add_function(wrap_pyfunction!(copperlist_value_iterator_unified, module)?)?;
    Ok(())
}

fn copperlist_to_py(entry: &CuDecodedCopperList, py: Python<'_>) -> PyResult<Py<PyAny>> {
    let root = PyDict::new(py);
    root.set_item("id", entry.id)?;
    let messages = PyList::empty(py);
    for slot in &entry.msgs {
        let message = PyDict::new(py);
        message.set_item("task_id", &slot.task_id)?;
        message.set_item("original_payload_present", slot.original_payload_present)?;
        message.set_item("captured_payload_present", slot.captured_payload_present)?;
        message.set_item("tov", serde_to_py(&slot.tov, py)?)?;
        message.set_item("metadata", serde_to_py(&slot.metadata, py)?)?;
        message.set_item(
            "payload",
            match &slot.payload {
                Some(value) => value_to_py(value, py)?,
                None => py.None(),
            },
        )?;
        messages.append(message)?;
    }
    root.set_item("msgs", messages)?;
    Ok(root.into_any().unbind())
}

fn value_to_py(value: &Value, py: Python<'_>) -> PyResult<Py<PyAny>> {
    match value {
        Value::Map(values) => {
            let hashable = values.keys().all(|key| {
                matches!(
                    key,
                    Value::Bool(_)
                        | Value::U8(_)
                        | Value::U16(_)
                        | Value::U32(_)
                        | Value::U64(_)
                        | Value::U128(_)
                        | Value::I8(_)
                        | Value::I16(_)
                        | Value::I32(_)
                        | Value::I64(_)
                        | Value::I128(_)
                        | Value::F32(_)
                        | Value::F64(_)
                        | Value::Char(_)
                        | Value::String(_)
                        | Value::Unit
                        | Value::CuTime(_)
                )
            });
            let dict = PyDict::new(py);
            if hashable {
                for (key, value) in values {
                    dict.set_item(value_to_py(key, py)?, value_to_py(value, py)?)?;
                }
            } else {
                let pairs = PyList::empty(py);
                for (key, value) in values {
                    pairs.append((value_to_py(key, py)?, value_to_py(value, py)?))?;
                }
                dict.set_item("$map", pairs)?;
            }
            Ok(dict.into_any().unbind())
        }
        Value::Seq(values) => {
            let list = PyList::empty(py);
            for value in values {
                list.append(value_to_py(value, py)?)?;
            }
            Ok(list.into_any().unbind())
        }
        Value::Option(Some(value)) | Value::Newtype(value) => value_to_py(value, py),
        _ => crate::python::value_to_py(value, py),
    }
}

fn serde_to_py<T: serde::Serialize>(value: &T, py: Python<'_>) -> PyResult<Py<PyAny>> {
    let value =
        serde_json::to_value(value).map_err(|error| PyValueError::new_err(error.to_string()))?;
    json_value_to_py(&value, py)
}
fn json_value_to_py(value: &serde_json::Value, py: Python<'_>) -> PyResult<Py<PyAny>> {
    match value {
        serde_json::Value::Null => Ok(py.None()),
        serde_json::Value::Bool(value) => {
            Ok(value.into_pyobject(py)?.to_owned().into_any().unbind())
        }
        serde_json::Value::String(value) => Ok(value.into_pyobject(py)?.into_any().unbind()),
        serde_json::Value::Number(value) => {
            if let Some(value) = value.as_u64() {
                Ok(value.into_pyobject(py)?.into_any().unbind())
            } else if let Some(value) = value.as_i64() {
                Ok(value.into_pyobject(py)?.into_any().unbind())
            } else if let Some(value) = value.as_f64() {
                Ok(value.into_pyobject(py)?.into_any().unbind())
            } else {
                Err(PyValueError::new_err("Unsupported catalog number"))
            }
        }
        serde_json::Value::Array(values) => {
            let list = PyList::empty(py);
            for value in values {
                list.append(json_value_to_py(value, py)?)?;
            }
            Ok(list.into_any().unbind())
        }
        serde_json::Value::Object(values) => {
            let dict = PyDict::new(py);
            for (key, value) in values {
                dict.set_item(key, json_value_to_py(value, py)?)?;
            }
            Ok(dict.into_any().unbind())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_catalog_python_without_registered_native_decoder() {
        pyo3::Python::initialize();
        Python::attach(|py| {
            let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
            let path = dir.path().join("python.copper");
            crate::catalog_tests::fixture(&path, false, true);
            let catalog = value_decode_catalog_unified(py, path.to_str().unwrap(), None).unwrap();
            assert_eq!(
                catalog
                    .bind(py)
                    .get_item("mission")
                    .unwrap()
                    .extract::<String>()
                    .unwrap(),
                "drive"
            );
            let mut iterator =
                copperlist_value_iterator_unified(path.to_str().unwrap(), None).unwrap();
            let entry = iterator.__next__(py).unwrap().unwrap();
            let payload: u32 = entry
                .bind(py)
                .get_item("msgs")
                .unwrap()
                .get_item(0)
                .unwrap()
                .get_item("payload")
                .unwrap()
                .extract()
                .unwrap();
            assert_eq!(payload, 300);
            assert!(iterator.__next__(py).unwrap().is_ok());
            assert!(iterator.__next__(py).is_none());
            let wide = value_to_py(&Value::U128(u128::MAX), py).unwrap();
            assert_eq!(wide.bind(py).extract::<u128>().unwrap(), u128::MAX);
            let nan = value_to_py(&Value::F32(f32::NAN), py).unwrap();
            assert!(nan.bind(py).extract::<f64>().unwrap().is_nan());
        });
    }
    #[test]
    fn test_catalog_python_corruption_is_an_exception() {
        pyo3::Python::initialize();
        Python::attach(|py| {
            let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
            let path = dir.path().join("corrupt.copper");
            crate::catalog_tests::fixture(&path, true, true);
            let mut iterator =
                copperlist_value_iterator_unified(path.to_str().unwrap(), None).unwrap();
            assert!(iterator.__next__(py).unwrap().is_ok());
            let error = iterator.__next__(py).unwrap().unwrap_err();
            assert!(error.is_instance_of::<PyIOError>(py));
            assert!(error.to_string().contains("CopperList #1 slot 0"));
            assert!(iterator.__next__(py).is_none());
        });
    }
}
