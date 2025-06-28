//! Schema trait and types for copper runtime
//!
//! This crate provides the Schema trait and SchemaType enum for describing
//! the structure of data types in the copper runtime system.

use indexmap::IndexMap;
use std::fmt::{Display, Formatter};

/// Type alias for schema field mappings to avoid exposing IndexMap dependency
pub type SchemaIndex = IndexMap<String, SchemaType>;

/// Represents the type information for schema generation
#[derive(Debug, Clone, PartialEq)]
pub enum SchemaType {
    UNIT,
    U8,
    U16,
    U32,
    U64,
    U128,
    Usize,
    I8,
    I16,
    I32,
    I64,
    I128,
    Isize,
    F32,
    F64,
    Bool,
    String,
    Vec(Box<SchemaType>),
    Array {
        element_type: Box<SchemaType>,
        size: usize,
    }, // Fixed-size array with compile-time known size
    Option(Box<SchemaType>),
    Tuple(Vec<SchemaType>),
    Struct {
        name: String,
        fields: SchemaIndex,
    },
    Custom(String),
}

impl Display for SchemaType {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            SchemaType::U8 => write!(f, "u8"),
            SchemaType::U16 => write!(f, "u16"),
            SchemaType::U32 => write!(f, "u32"),
            SchemaType::U64 => write!(f, "u64"),
            SchemaType::U128 => write!(f, "u128"),
            SchemaType::Usize => write!(f, "usize"),
            SchemaType::I8 => write!(f, "i8"),
            SchemaType::I16 => write!(f, "i16"),
            SchemaType::I32 => write!(f, "i32"),
            SchemaType::I64 => write!(f, "i64"),
            SchemaType::I128 => write!(f, "i128"),
            SchemaType::Isize => write!(f, "isize"),
            SchemaType::F32 => write!(f, "f32"),
            SchemaType::F64 => write!(f, "f64"),
            SchemaType::Bool => write!(f, "bool"),
            SchemaType::String => write!(f, "String"),
            SchemaType::Vec(inner) => write!(f, "Vec<{inner}>"),
            SchemaType::Array { element_type, size } => write!(f, "[{element_type}; {size}]"),
            SchemaType::Option(inner) => write!(f, "Option<{inner}>"),
            SchemaType::Tuple(types) => {
                write!(f, "(")?;
                for (i, ty) in types.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{ty}")?;
                }
                write!(f, ")")
            }
            SchemaType::Custom(name) => write!(f, "Custom({name})"),
            SchemaType::Struct { name, .. } => write!(f, "Struct {name}"),
            &SchemaType::UNIT => write!(f, "()"),
        }
    }
}

/// Trait for types that can provide schema information
pub trait Schema {
    /// Returns a SchemaIndex mapping field names to their types
    fn schema() -> SchemaIndex;

    /// Returns the type name of the struct
    fn type_name() -> &'static str;

    /// Returns the schema as a SchemaType::Struct for recursive inspection
    fn schema_type() -> SchemaType {
        SchemaType::Struct {
            name: Self::type_name().to_string(),
            fields: Self::schema(),
        }
    }

    /// Prints the schema in a human-readable format
    fn print_schema() {
        let schema = Self::schema();
        let type_name = Self::type_name();
        println!("Schema for {type_name}:");
        for (field_name, field_type) in schema {
            println!("  {field_name}: {field_type}");
        }
    }
}

impl Schema for () {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "()"
    }

    fn schema_type() -> SchemaType {
        SchemaType::UNIT
    }
}

// Schema implementations for primitive types
impl Schema for u8 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "u8"
    }

    fn schema_type() -> SchemaType {
        SchemaType::U8
    }
}

impl Schema for u16 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "u16"
    }

    fn schema_type() -> SchemaType {
        SchemaType::U16
    }
}

impl Schema for u32 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "u32"
    }

    fn schema_type() -> SchemaType {
        SchemaType::U32
    }
}

impl Schema for u64 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "u64"
    }

    fn schema_type() -> SchemaType {
        SchemaType::U64
    }
}

impl Schema for u128 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "u128"
    }

    fn schema_type() -> SchemaType {
        SchemaType::U128
    }
}

impl Schema for usize {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "usize"
    }

    fn schema_type() -> SchemaType {
        SchemaType::Usize
    }
}

impl Schema for i8 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "i8"
    }

    fn schema_type() -> SchemaType {
        SchemaType::I8
    }
}

impl Schema for i16 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "i16"
    }

    fn schema_type() -> SchemaType {
        SchemaType::I16
    }
}

impl Schema for i32 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "i32"
    }

    fn schema_type() -> SchemaType {
        SchemaType::I32
    }
}

impl Schema for i64 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "i64"
    }

    fn schema_type() -> SchemaType {
        SchemaType::I64
    }
}

impl Schema for i128 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "i128"
    }

    fn schema_type() -> SchemaType {
        SchemaType::I128
    }
}

impl Schema for isize {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "isize"
    }

    fn schema_type() -> SchemaType {
        SchemaType::Isize
    }
}

impl Schema for f32 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "f32"
    }

    fn schema_type() -> SchemaType {
        SchemaType::F32
    }
}

impl Schema for f64 {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "f64"
    }

    fn schema_type() -> SchemaType {
        SchemaType::F64
    }
}

impl Schema for bool {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "bool"
    }

    fn schema_type() -> SchemaType {
        SchemaType::Bool
    }
}

impl Schema for String {
    fn schema() -> SchemaIndex {
        IndexMap::new()
    }

    fn type_name() -> &'static str {
        "String"
    }

    fn schema_type() -> SchemaType {
        SchemaType::String
    }
}

macro_rules! impl_schema_for_tuples {
    ($(($($ty:ident),*)),*) => {
        $(
            impl<$($ty: Schema),*> Schema for ($($ty,)*) {
                fn schema() -> SchemaIndex {
                    IndexMap::new()
                }

                fn type_name() -> &'static str {
                    "tuple"
                }

                fn schema_type() -> SchemaType {
                    SchemaType::Tuple(vec![$($ty::schema_type()),*])
                }
            }
        )*
    };
}

// Apply the macro to generate Schema implementations for tuple sizes up to 5
impl_schema_for_tuples! {
    (T1, T2), (T1, T2, T3), (T1, T2, T3, T4), (T1, T2, T3, T4, T5)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_array_schema_type_display() {
        let array_type = SchemaType::Array {
            element_type: Box::new(SchemaType::F32),
            size: 8,
        };
        assert_eq!(format!("{array_type}"), "[f32; 8]");
    }

    #[test]
    fn test_array_schema_type_equality() {
        let array1 = SchemaType::Array {
            element_type: Box::new(SchemaType::F32),
            size: 8,
        };
        let array2 = SchemaType::Array {
            element_type: Box::new(SchemaType::F32),
            size: 8,
        };
        let array3 = SchemaType::Array {
            element_type: Box::new(SchemaType::F32),
            size: 16,
        };

        assert_eq!(array1, array2);
        assert_ne!(array1, array3);
    }

    #[test]
    fn test_nested_array_display() {
        let nested_array = SchemaType::Array {
            element_type: Box::new(SchemaType::Array {
                element_type: Box::new(SchemaType::I32),
                size: 3,
            }),
            size: 2,
        };
        assert_eq!(format!("{nested_array}"), "[[i32; 3]; 2]");
    }
}
