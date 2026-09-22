//! Scalar and component configuration values.

use super::BTreeMap;
#[cfg(not(feature = "std"))]
use super::imp::*;
#[cfg(not(feature = "std"))]
use alloc::boxed::Box;
#[cfg(not(feature = "std"))]
use alloc::vec;
use core::any::type_name;
use core::fmt;
use core::fmt::Display;
use cu29_traits::CuError;
use cu29_value::Value as CuValue;
use hashbrown::HashMap;
use ron::Number;
use ron::value::Value as RonValue;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Deserializer, Serialize};

/// This is the configuration of a component (like a task config or a monitoring config):w
/// It is a map of key-value pairs.
/// It is given to the new method of the task implementation.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ComponentConfig(pub HashMap<String, Value>);

/// Mapping between resource binding names and bundle-scoped resource ids.
#[allow(dead_code)]
impl Display for ComponentConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut first = true;
        let ComponentConfig(config) = self;
        write!(f, "{{")?;
        for (key, value) in config.iter() {
            if !first {
                write!(f, ", ")?;
            }
            write!(f, "{key}: {value}")?;
            first = false;
        }
        write!(f, "}}")
    }
}

// forward map interface
impl ComponentConfig {
    #[allow(dead_code)]
    pub fn new() -> Self {
        ComponentConfig(HashMap::new())
    }

    #[allow(dead_code)]
    pub fn get<T>(&self, key: &str) -> Result<Option<T>, ConfigError>
    where
        T: for<'a> TryFrom<&'a Value, Error = ConfigError>,
    {
        let ComponentConfig(config) = self;
        match config.get(key) {
            Some(value) => T::try_from(value).map(Some),
            None => Ok(None),
        }
    }

    #[allow(dead_code)]
    /// Retrieve a structured config value by deserializing it with cu29-value.
    ///
    /// Example RON:
    /// `{ "calibration": { "matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], "enabled": true } }`
    ///
    /// ```rust,ignore
    /// #[derive(serde::Deserialize)]
    /// struct CalibrationCfg {
    ///     matrix: [[f32; 3]; 3],
    ///     enabled: bool,
    /// }
    /// let cfg: CalibrationCfg = config.get_value("calibration")?.unwrap();
    /// ```
    pub fn get_value<T>(&self, key: &str) -> Result<Option<T>, ConfigError>
    where
        T: DeserializeOwned,
    {
        let ComponentConfig(config) = self;
        let Some(value) = config.get(key) else {
            return Ok(None);
        };
        let cu_value = ron_value_to_cu_value(&value.0).map_err(|err| err.with_key(key))?;
        cu_value
            .deserialize_into::<T>()
            .map(Some)
            .map_err(|err| ConfigError {
                message: format!(
                    "Config key '{key}' failed to deserialize as {}: {err}",
                    type_name::<T>()
                ),
            })
    }

    #[allow(dead_code)]
    pub fn deserialize_into<T>(&self) -> Result<T, ConfigError>
    where
        T: DeserializeOwned,
    {
        let mut map = BTreeMap::new();
        for (key, value) in &self.0 {
            let mapped_value = ron_value_to_cu_value(&value.0).map_err(|err| err.with_key(key))?;
            map.insert(CuValue::String(key.clone()), mapped_value);
        }

        CuValue::Map(map)
            .deserialize_into::<T>()
            .map_err(|err| ConfigError {
                message: format!(
                    "Config failed to deserialize as {}: {err}",
                    type_name::<T>()
                ),
            })
    }

    #[allow(dead_code)]
    pub fn set<T: Into<Value>>(&mut self, key: &str, value: T) {
        let ComponentConfig(config) = self;
        config.insert(key.to_string(), value.into());
    }

    #[allow(dead_code)]
    pub fn merge_from(&mut self, other: &ComponentConfig) {
        let ComponentConfig(config) = self;
        for (key, value) in &other.0 {
            config.insert(key.clone(), value.clone());
        }
    }
}

fn ron_value_to_cu_value(value: &RonValue) -> Result<CuValue, ConfigError> {
    match value {
        RonValue::Bool(v) => Ok(CuValue::Bool(*v)),
        RonValue::Char(v) => Ok(CuValue::Char(*v)),
        RonValue::String(v) => Ok(CuValue::String(v.clone())),
        RonValue::Bytes(v) => Ok(CuValue::Bytes(v.clone())),
        RonValue::Unit => Ok(CuValue::Unit),
        RonValue::Option(v) => {
            let mapped = match v {
                Some(inner) => Some(Box::new(ron_value_to_cu_value(inner)?)),
                None => None,
            };
            Ok(CuValue::Option(mapped))
        }
        RonValue::Seq(seq) => {
            let mut mapped = Vec::with_capacity(seq.len());
            for item in seq {
                mapped.push(ron_value_to_cu_value(item)?);
            }
            Ok(CuValue::Seq(mapped))
        }
        RonValue::Map(map) => {
            let mut mapped = BTreeMap::new();
            for (key, value) in map.iter() {
                let mapped_key = ron_value_to_cu_value(key)?;
                let mapped_value = ron_value_to_cu_value(value)?;
                mapped.insert(mapped_key, mapped_value);
            }
            Ok(CuValue::Map(mapped))
        }
        RonValue::Number(num) => match num {
            Number::I8(v) => Ok(CuValue::I8(*v)),
            Number::I16(v) => Ok(CuValue::I16(*v)),
            Number::I32(v) => Ok(CuValue::I32(*v)),
            Number::I64(v) => Ok(CuValue::I64(*v)),
            Number::U8(v) => Ok(CuValue::U8(*v)),
            Number::U16(v) => Ok(CuValue::U16(*v)),
            Number::U32(v) => Ok(CuValue::U32(*v)),
            Number::U64(v) => Ok(CuValue::U64(*v)),
            Number::F32(v) => Ok(CuValue::F32(v.0)),
            Number::F64(v) => Ok(CuValue::F64(v.0)),
            _ => Err(ConfigError {
                message: "Unsupported RON number variant".to_string(),
            }),
        },
    }
}

// The configuration Serialization format is as follows:
// (
//   tasks : [ (id: "toto", type: "zorglub::MyType", config: {...}),
//             (id: "titi", type: "zorglub::MyType2", config: {...})]
//   cnx : [ (src: "toto", dst: "titi", msg: "zorglub::MyMsgType"),...]
// )

/// Wrapper around the ron::Value to allow for custom serialization.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Value(pub(super) RonValue);

/// Scalar representation used by compile-time constants after RON parsing.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConstantNumber {
    Signed(i64),
    Unsigned(u64),
    Float(f64),
}

impl ConstantNumber {
    pub fn as_f64(self) -> f64 {
        match self {
            Self::Signed(value) => value as f64,
            Self::Unsigned(value) => value as f64,
            Self::Float(value) => value,
        }
    }
}

/// Rust scalar storage selected for a compile-time constant.
#[doc(hidden)]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, Default, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ConstantStorage {
    I8,
    I16,
    I32,
    I64,
    Isize,
    U8,
    U16,
    U32,
    U64,
    Usize,
    #[default]
    F32,
    F64,
}

impl ConstantStorage {
    pub const fn rust_type(self) -> &'static str {
        match self {
            Self::I8 => "i8",
            Self::I16 => "i16",
            Self::I32 => "i32",
            Self::I64 => "i64",
            Self::Isize => "isize",
            Self::U8 => "u8",
            Self::U16 => "u16",
            Self::U32 => "u32",
            Self::U64 => "u64",
            Self::Usize => "usize",
            Self::F32 => "f32",
            Self::F64 => "f64",
        }
    }

    pub const fn supports_quantity(self) -> bool {
        matches!(self, Self::F32 | Self::F64)
    }
}

/// One top-level `constants:` declaration.
#[doc(hidden)]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ConstantConfig {
    pub(super) id: String,
    #[serde(default, deserialize_with = "deserialize_constant_module")]
    module: Option<String>,
    #[serde(default)]
    pub(super) storage: Option<ConstantStorage>,
    pub(super) quantity: Option<cu29_units::constant::Quantity>,
    pub(super) unit: Option<cu29_units::constant::Unit>,
    pub(super) value: Option<Value>,
    #[serde(rename = "type")]
    pub(super) rust_type: Option<String>,
    pub(super) expression: Option<String>,
}

fn deserialize_constant_module<'de, D>(deserializer: D) -> Result<Option<String>, D::Error>
where
    D: Deserializer<'de>,
{
    Option::<String>::deserialize(deserializer).map(|module| {
        module.map(|module| {
            module
                .chars()
                .filter(|character| !character.is_whitespace())
                .collect()
        })
    })
}

impl ConstantConfig {
    pub const DEFAULT_MODULE: &'static str = "constants";

    pub fn id(&self) -> &str {
        &self.id
    }

    pub fn module_path(&self) -> &str {
        self.module.as_deref().unwrap_or(Self::DEFAULT_MODULE)
    }

    pub fn qualified_id(&self) -> String {
        format!("{}::{}", self.module_path(), self.id)
    }

    pub const fn storage(&self) -> ConstantStorage {
        match self.storage {
            Some(storage) => storage,
            None => ConstantStorage::F32,
        }
    }

    pub const fn quantity(&self) -> Option<cu29_units::constant::Quantity> {
        self.quantity
    }

    pub const fn explicit_unit(&self) -> Option<cu29_units::constant::Unit> {
        self.unit
    }

    pub fn expression_definition(&self) -> Option<(&str, &str)> {
        self.rust_type.as_deref().zip(self.expression.as_deref())
    }

    pub fn resolved_unit(&self) -> Result<Option<cu29_units::constant::Unit>, String> {
        let Some(quantity) = self.quantity else {
            return Ok(None);
        };
        if let Some(unit) = self.unit {
            return Ok(Some(unit));
        }
        let definition = cu29_units::constant::definition(quantity).ok_or_else(|| {
            format!(
                "Constant '{}' uses quantity '{}' which is missing from the unit catalogue",
                self.id,
                quantity.name()
            )
        })?;
        cu29_units::constant::Unit::from_name(definition.coherent_unit)
            .map(Some)
            .ok_or_else(|| {
                format!(
                    "Constant '{}' quantity '{}' has invalid coherent unit metadata '{}'",
                    self.id,
                    quantity.name(),
                    definition.coherent_unit
                )
            })
    }

    pub fn numbers(&self) -> Result<(bool, Vec<ConstantNumber>), String> {
        fn number(value: &RonValue) -> Result<ConstantNumber, String> {
            match value {
                RonValue::Number(number) => match number {
                    Number::I8(value) => Ok(ConstantNumber::Signed(i64::from(*value))),
                    Number::I16(value) => Ok(ConstantNumber::Signed(i64::from(*value))),
                    Number::I32(value) => Ok(ConstantNumber::Signed(i64::from(*value))),
                    Number::I64(value) => Ok(ConstantNumber::Signed(*value)),
                    Number::U8(value) => Ok(ConstantNumber::Unsigned(u64::from(*value))),
                    Number::U16(value) => Ok(ConstantNumber::Unsigned(u64::from(*value))),
                    Number::U32(value) => Ok(ConstantNumber::Unsigned(u64::from(*value))),
                    Number::U64(value) => Ok(ConstantNumber::Unsigned(*value)),
                    Number::F32(value) => Ok(ConstantNumber::Float(f64::from(value.0))),
                    Number::F64(value) => Ok(ConstantNumber::Float(value.0)),
                    _ => Err("unsupported numeric representation".to_string()),
                },
                _ => Err("expected a number".to_string()),
            }
        }

        let value = self
            .value
            .as_ref()
            .ok_or_else(|| format!("Constant '{}' does not declare a numeric value", self.id))?;
        match &value.0 {
            RonValue::Seq(values) => values
                .iter()
                .map(number)
                .collect::<Result<Vec<_>, _>>()
                .map(|values| (true, values)),
            value => number(value).map(|value| (false, vec![value])),
        }
        .map_err(|error| format!("Constant '{}': {error}", self.id))
    }

    pub fn normalized_f32(&self) -> Result<(bool, Vec<f32>), String> {
        let quantity = self.quantity.ok_or_else(|| {
            format!(
                "Constant '{}' does not declare a physical quantity",
                self.id
            )
        })?;
        let unit = self
            .resolved_unit()?
            .ok_or_else(|| format!("Constant '{}' has no resolved unit", self.id))?;
        let (is_array, numbers) = self.numbers()?;
        numbers
            .into_iter()
            .map(|number| {
                let value = number.as_f64() as f32;
                if !value.is_finite() {
                    return Err(format!("Constant '{}' values must be finite", self.id));
                }
                cu29_units::constant::normalize_f32(quantity, unit, value).ok_or_else(|| {
                    format!(
                        "Constant '{}' unit '{}' is not compatible with quantity '{}'",
                        self.id,
                        unit.name(),
                        quantity.name()
                    )
                })
            })
            .collect::<Result<Vec<_>, _>>()
            .map(|values| (is_array, values))
    }

    pub fn normalized_f64(&self) -> Result<(bool, Vec<f64>), String> {
        let quantity = self.quantity.ok_or_else(|| {
            format!(
                "Constant '{}' does not declare a physical quantity",
                self.id
            )
        })?;
        let unit = self
            .resolved_unit()?
            .ok_or_else(|| format!("Constant '{}' has no resolved unit", self.id))?;
        let (is_array, numbers) = self.numbers()?;
        numbers
            .into_iter()
            .map(|number| {
                let value = number.as_f64();
                if !value.is_finite() {
                    return Err(format!("Constant '{}' values must be finite", self.id));
                }
                cu29_units::constant::normalize_f64(quantity, unit, value).ok_or_else(|| {
                    format!(
                        "Constant '{}' unit '{}' is not compatible with quantity '{}'",
                        self.id,
                        unit.name(),
                        quantity.name()
                    )
                })
            })
            .collect::<Result<Vec<_>, _>>()
            .map(|values| (is_array, values))
    }

    /// Stable comparison key for detecting runtime attempts to change a baked constant.
    #[allow(dead_code)]
    pub fn semantic_fingerprint(&self) -> Result<u64, String> {
        const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
        const PRIME: u64 = 0x0000_0100_0000_01b3;

        fn hash_bytes(hash: &mut u64, bytes: &[u8]) {
            for byte in bytes {
                *hash ^= u64::from(*byte);
                *hash = hash.wrapping_mul(PRIME);
            }
        }

        let mut hash = OFFSET;
        if let Some((rust_type, expression)) = self.expression_definition() {
            hash_bytes(&mut hash, b"expression");
            hash_bytes(&mut hash, rust_type.as_bytes());
            hash_bytes(&mut hash, &[0]);
            hash_bytes(&mut hash, expression.as_bytes());
            return Ok(hash);
        }

        let storage = self.storage();
        hash_bytes(&mut hash, b"numeric");
        hash_bytes(&mut hash, storage.rust_type().as_bytes());
        hash_bytes(
            &mut hash,
            self.quantity
                .map_or("primitive", |quantity| quantity.name())
                .as_bytes(),
        );

        if self.quantity.is_some() {
            match storage {
                ConstantStorage::F32 => {
                    let (is_array, values) = self.normalized_f32()?;
                    hash_bytes(&mut hash, &[u8::from(is_array)]);
                    for value in values {
                        hash_bytes(&mut hash, &value.to_bits().to_le_bytes());
                    }
                }
                ConstantStorage::F64 => {
                    let (is_array, values) = self.normalized_f64()?;
                    hash_bytes(&mut hash, &[u8::from(is_array)]);
                    for value in values {
                        hash_bytes(&mut hash, &value.to_bits().to_le_bytes());
                    }
                }
                _ => {
                    return Err(format!(
                        "Constant '{}' quantity storage must be f32 or f64",
                        self.id
                    ));
                }
            }
            return Ok(hash);
        }

        let (is_array, numbers) = self.numbers()?;
        hash_bytes(&mut hash, &[u8::from(is_array)]);
        for number in numbers {
            match (storage, number) {
                (ConstantStorage::F32, number) => {
                    hash_bytes(&mut hash, &(number.as_f64() as f32).to_bits().to_le_bytes())
                }
                (ConstantStorage::F64, number) => {
                    hash_bytes(&mut hash, &number.as_f64().to_bits().to_le_bytes())
                }
                (_, ConstantNumber::Signed(value)) => hash_bytes(&mut hash, &value.to_le_bytes()),
                (_, ConstantNumber::Unsigned(value)) => hash_bytes(&mut hash, &value.to_le_bytes()),
                (_, ConstantNumber::Float(value)) => {
                    hash_bytes(&mut hash, &value.to_bits().to_le_bytes())
                }
            }
        }
        Ok(hash)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ConfigError {
    message: String,
}

impl ConfigError {
    fn type_mismatch(expected: &'static str, value: &Value) -> Self {
        ConfigError {
            message: format!("Expected {expected} but got {value:?}"),
        }
    }

    fn with_key(self, key: &str) -> Self {
        ConfigError {
            message: format!("Config key '{key}': {}", self.message),
        }
    }
}

impl Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ConfigError {}

#[cfg(not(feature = "std"))]
impl core::error::Error for ConfigError {}

impl From<ConfigError> for CuError {
    fn from(err: ConfigError) -> Self {
        CuError::from(err.to_string())
    }
}

// Macro for implementing From<T> for Value where T is a numeric type
macro_rules! impl_from_numeric_for_value {
    ($($source:ty),* $(,)?) => {
        $(impl From<$source> for Value {
            fn from(value: $source) -> Self {
                Value(RonValue::Number(value.into()))
            }
        })*
    };
}

// Implement From for common numeric types
impl_from_numeric_for_value!(i8, i16, i32, i64, u8, u16, u32, u64, f32, f64);

impl TryFrom<&Value> for bool {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::Bool(v)) = value {
            Ok(*v)
        } else {
            Err(ConfigError::type_mismatch("bool", value))
        }
    }
}

impl From<Value> for bool {
    fn from(value: Value) -> Self {
        if let Value(RonValue::Bool(v)) = value {
            v
        } else {
            panic!("Expected a Boolean variant but got {value:?}")
        }
    }
}
macro_rules! impl_from_value_for_int {
    ($($target:ty),* $(,)?) => {
        $(
            impl From<Value> for $target {
                fn from(value: Value) -> Self {
                    if let Value(RonValue::Number(num)) = value {
                        match num {
                            Number::I8(n) => n as $target,
                            Number::I16(n) => n as $target,
                            Number::I32(n) => n as $target,
                            Number::I64(n) => n as $target,
                            Number::U8(n) => n as $target,
                            Number::U16(n) => n as $target,
                            Number::U32(n) => n as $target,
                            Number::U64(n) => n as $target,
                            Number::F32(_) | Number::F64(_) => {
                                panic!("Expected an integer Number variant but got {num:?}")
                            }
                            _ => {
                                panic!("Expected an integer Number variant but got {num:?}")
                            }
                        }
                    } else {
                        panic!("Expected a Number variant but got {value:?}")
                    }
                }
            }
        )*
    };
}

impl_from_value_for_int!(u8, i8, u16, i16, u32, i32, u64, i64);

macro_rules! impl_try_from_value_for_int {
    ($($target:ty),* $(,)?) => {
        $(
            impl TryFrom<&Value> for $target {
                type Error = ConfigError;

                fn try_from(value: &Value) -> Result<Self, Self::Error> {
                    if let Value(RonValue::Number(num)) = value {
                        match num {
                            Number::I8(n) => Ok(*n as $target),
                            Number::I16(n) => Ok(*n as $target),
                            Number::I32(n) => Ok(*n as $target),
                            Number::I64(n) => Ok(*n as $target),
                            Number::U8(n) => Ok(*n as $target),
                            Number::U16(n) => Ok(*n as $target),
                            Number::U32(n) => Ok(*n as $target),
                            Number::U64(n) => Ok(*n as $target),
                            Number::F32(_) | Number::F64(_) => {
                                Err(ConfigError::type_mismatch("integer", value))
                            }
                            _ => {
                                Err(ConfigError::type_mismatch("integer", value))
                            }
                        }
                    } else {
                        Err(ConfigError::type_mismatch("integer", value))
                    }
                }
            }
        )*
    };
}

impl_try_from_value_for_int!(u8, i8, u16, i16, u32, i32, u64, i64);

impl TryFrom<&Value> for f64 {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::Number(num)) = value {
            let number = match num {
                Number::I8(n) => *n as f64,
                Number::I16(n) => *n as f64,
                Number::I32(n) => *n as f64,
                Number::I64(n) => *n as f64,
                Number::U8(n) => *n as f64,
                Number::U16(n) => *n as f64,
                Number::U32(n) => *n as f64,
                Number::U64(n) => *n as f64,
                Number::F32(n) => n.0 as f64,
                Number::F64(n) => n.0,
                _ => {
                    return Err(ConfigError::type_mismatch("number", value));
                }
            };
            Ok(number)
        } else {
            Err(ConfigError::type_mismatch("number", value))
        }
    }
}

impl From<Value> for f64 {
    fn from(value: Value) -> Self {
        if let Value(RonValue::Number(num)) = value {
            num.into_f64()
        } else {
            panic!("Expected a Number variant but got {value:?}")
        }
    }
}

//Basically just a copy of the From<Value> for f64.
impl TryFrom<&Value> for f32 {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::Number(num)) = value {
            let number = match num {
                Number::I8(n) => *n as f32,
                Number::I16(n) => *n as f32,
                Number::I32(n) => *n as f32,
                Number::I64(n) => *n as f32,
                Number::U8(n) => *n as f32,
                Number::U16(n) => *n as f32,
                Number::U32(n) => *n as f32,
                Number::U64(n) => *n as f32,
                Number::F32(n) => n.0,
                Number::F64(n) => n.0 as f32,
                _ => {
                    return Err(ConfigError::type_mismatch("number", value));
                }
            };
            Ok(number)
        } else {
            Err(ConfigError::type_mismatch("number", value))
        }
    }
}

impl From<Value> for f32 {
    fn from(value: Value) -> Self {
        if let Value(RonValue::Number(num)) = value {
            num.into_f64() as f32
        } else {
            panic!("Expected a Number variant but got {value:?}")
        }
    }
}

impl From<String> for Value {
    fn from(value: String) -> Self {
        Value(RonValue::String(value))
    }
}

impl TryFrom<&Value> for String {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::String(s)) = value {
            Ok(s.clone())
        } else {
            Err(ConfigError::type_mismatch("string", value))
        }
    }
}

impl From<Value> for String {
    fn from(value: Value) -> Self {
        if let Value(RonValue::String(s)) = value {
            s
        } else {
            panic!("Expected a String variant")
        }
    }
}

impl Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Value(value) = self;
        match value {
            RonValue::Number(n) => {
                let s = match n {
                    Number::I8(n) => n.to_string(),
                    Number::I16(n) => n.to_string(),
                    Number::I32(n) => n.to_string(),
                    Number::I64(n) => n.to_string(),
                    Number::U8(n) => n.to_string(),
                    Number::U16(n) => n.to_string(),
                    Number::U32(n) => n.to_string(),
                    Number::U64(n) => n.to_string(),
                    Number::F32(n) => n.0.to_string(),
                    Number::F64(n) => n.0.to_string(),
                    _ => panic!("Expected a Number variant but got {value:?}"),
                };
                write!(f, "{s}")
            }
            RonValue::String(s) => write!(f, "{s}"),
            RonValue::Bool(b) => write!(f, "{b}"),
            RonValue::Map(m) => write!(f, "{m:?}"),
            RonValue::Char(c) => write!(f, "{c:?}"),
            RonValue::Unit => write!(f, "unit"),
            RonValue::Option(o) => write!(f, "{o:?}"),
            RonValue::Seq(s) => write!(f, "{s:?}"),
            RonValue::Bytes(bytes) => write!(f, "{bytes:?}"),
        }
    }
}
