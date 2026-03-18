use crate::Value;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use pyo3::types::{PyAny, PyBool, PyBytes, PyDict, PyFloat, PyInt, PyList, PyString, PyTuple};

pub fn value_to_py(value: &Value, py: Python<'_>) -> PyResult<Py<PyAny>> {
    match value {
        Value::Bool(v) => Ok(v.into_pyobject(py)?.to_owned().into()),
        Value::U8(v) => Ok(v.into_pyobject(py)?.into()),
        Value::U16(v) => Ok(v.into_pyobject(py)?.into()),
        Value::U32(v) => Ok(v.into_pyobject(py)?.into()),
        Value::U64(v) => Ok(v.into_pyobject(py)?.into()),
        Value::I8(v) => Ok(v.into_pyobject(py)?.into()),
        Value::I16(v) => Ok(v.into_pyobject(py)?.into()),
        Value::I32(v) => Ok(v.into_pyobject(py)?.into()),
        Value::I64(v) => Ok(v.into_pyobject(py)?.into()),
        Value::F32(v) => Ok(v.into_pyobject(py)?.into()),
        Value::F64(v) => Ok(v.into_pyobject(py)?.into()),
        Value::Char(v) => Ok(v.into_pyobject(py)?.into()),
        Value::String(v) => Ok(v.into_pyobject(py)?.into()),
        Value::Unit => Ok(py.None()),
        Value::Option(Some(v)) => value_to_py(v, py),
        Value::Option(None) => Ok(py.None()),
        Value::Newtype(v) => value_to_py(v, py),
        Value::Seq(values) => {
            let items: Vec<Py<PyAny>> = values
                .iter()
                .map(|item| value_to_py(item, py))
                .collect::<PyResult<_>>()?;
            Ok(PyList::new(py, items)?.into_pyobject(py)?.into())
        }
        Value::Map(values) => {
            let dict = PyDict::new(py);
            for (key, value) in values {
                dict.set_item(value_to_py(key, py)?, value_to_py(value, py)?)?;
            }
            Ok(dict.into_pyobject(py)?.into())
        }
        Value::Bytes(value) => Ok(PyBytes::new(py, value).into_any().unbind()),
        Value::CuTime(value) => Ok(value.0.into_pyobject(py)?.into()),
    }
}

pub fn py_to_value(value: &Bound<'_, PyAny>) -> PyResult<Value> {
    if value.is_none() {
        return Ok(Value::Unit);
    }
    if value.is_instance_of::<PyBool>() {
        return Ok(Value::Bool(value.extract()?));
    }
    if value.is_instance_of::<PyBytes>() {
        let bytes = value.cast::<PyBytes>()?;
        return Ok(Value::Bytes(bytes.as_bytes().to_vec()));
    }
    if value.is_instance_of::<PyString>() {
        return Ok(Value::String(value.extract()?));
    }
    if value.is_instance_of::<PyFloat>() {
        return Ok(Value::F64(value.extract()?));
    }
    if value.is_instance_of::<PyInt>() {
        if let Ok(v) = value.extract::<i64>() {
            return Ok(Value::I64(v));
        }
        if let Ok(v) = value.extract::<u64>() {
            return Ok(Value::U64(v));
        }
        return Err(PyTypeError::new_err(
            "Python integer is outside the supported 64-bit range",
        ));
    }
    if value.is_instance_of::<PyList>() {
        let list = value.cast::<PyList>()?;
        let items = list
            .iter()
            .map(|item| py_to_value(&item))
            .collect::<PyResult<Vec<_>>>()?;
        return Ok(Value::Seq(items));
    }
    if value.is_instance_of::<PyTuple>() {
        let tuple = value.cast::<PyTuple>()?;
        let items = tuple
            .iter()
            .map(|item| py_to_value(&item))
            .collect::<PyResult<Vec<_>>>()?;
        return Ok(Value::Seq(items));
    }
    if value.is_instance_of::<PyDict>() {
        let dict = value.cast::<PyDict>()?;
        let mut items = BTreeMap::new();
        for (key, value) in dict.iter() {
            items.insert(py_to_value(&key)?, py_to_value(&value)?);
        }
        return Ok(Value::Map(items));
    }
    Err(PyTypeError::new_err(
        "Unsupported Python value for cu29_value::Value conversion",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_maps_to_unit() {
        Python::initialize();
        Python::attach(|py| {
            let value = py_to_value(&py.None().bind(py)).expect("None to value");
            assert!(matches!(value, Value::Unit));
        });
    }

    #[test]
    fn roundtrip_nested_map_and_list() {
        Python::initialize();
        Python::attach(|py| {
            let mut inner = BTreeMap::new();
            inner.insert(Value::String("flag".into()), Value::Bool(true));

            let mut root = BTreeMap::new();
            root.insert(
                Value::String("items".into()),
                Value::Seq(vec![Value::I64(3), Value::Map(inner)]),
            );

            let original = Value::Map(root);
            let py_value = value_to_py(&original, py).expect("value to py");
            let roundtrip = py_to_value(py_value.bind(py)).expect("py to value");
            assert_eq!(roundtrip, original);
        });
    }
}
