# copper-value

[![license-badge][]][license]

`copper-value` provides a way to capture serialization value trees for later processing.
Customizations are made to enable a more compact representation for the structured logging of copper.

## Python Feature

With the `python` feature enabled, this crate also provides conversion helpers between
`cu29_value::Value` and Python objects via PyO3.

That bridge is used in two places:

- `cu-python-task`, where Copper temporarily converts task inputs/state/outputs so a
  Python function can mutate them
- `cu29-export`, where Copper data is exposed to Python for offline analysis

Conversion behavior is intentionally simple:

- `None` maps to `Value::Unit`
- Python lists and tuples map to `Value::Seq`
- Python dicts map to `Value::Map`
- Python integers are accepted up to 128-bit signed/unsigned range
- Python `bytes` maps to `Value::Bytes`

[license-badge]: https://img.shields.io/badge/license-MIT-lightgray.svg?style=flat-square
[license]: https://github.com/arcnmx/serde-value/blob/master/COPYING
