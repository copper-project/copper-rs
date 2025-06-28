#![doc = include_str!("../README.md")]

// backward compatibility
pub use cu29_runtime::config;
pub use cu29_runtime::copperlist;
pub use cu29_runtime::curuntime;
pub use cu29_runtime::cutask;
pub use cu29_runtime::input_msg;
pub use cu29_runtime::monitoring;
pub use cu29_runtime::output_msg;
pub use cu29_runtime::payload;
pub use cu29_runtime::simulation;

pub use bincode;
pub use cu29_clock as clock;
pub use cu29_runtime::config::read_configuration;
pub use cu29_traits::*;

pub mod prelude {
    pub use cu29_clock::*;
    pub use cu29_derive::*;
    pub use cu29_intern_strs::*;
    pub use cu29_log::*;
    pub use cu29_log_derive::*;
    pub use cu29_log_runtime::*;
    pub use cu29_runtime::app::*;
    pub use cu29_runtime::config::*;
    pub use cu29_runtime::copperlist::*;
    pub use cu29_runtime::curuntime::*;
    pub use cu29_runtime::cutask::*;
    pub use cu29_runtime::input_msg;
    pub use cu29_runtime::monitoring::*;
    pub use cu29_runtime::output_msg;
    pub use cu29_runtime::payload::*;
    pub use cu29_runtime::simulation::*;
    pub use cu29_runtime::*;
    pub use cu29_schema::*;
    pub use cu29_traits::*;
    pub use cu29_unifiedlog::*;
    pub use cu29_value::to_value;
    pub use cu29_value::Value;
    pub use pool::*;
}

#[cfg(test)]
mod tests {
    use super::prelude::*;

    #[derive(Schema, Debug)]
    struct TaMere {
        toto: u32,
        titi: f64,
        tutu: String,
    }

    #[derive(Schema, Debug)]
    struct ForeignStruct {
        value: i32,
        name: String,
    }

    #[test]
    fn test_schema_derive_functionality() {
        // Test the original request: TaMere::schema()
        let schema = TaMere::schema();
        assert_eq!(schema.len(), 3);
        assert_eq!(schema.get("toto"), Some(&SchemaType::U32));
        assert_eq!(schema.get("titi"), Some(&SchemaType::F64));
        assert_eq!(schema.get("tutu"), Some(&SchemaType::String));

        // Test type name
        assert_eq!(TaMere::type_name(), "TaMere");

        // Test foreign struct
        let foreign_schema = ForeignStruct::schema();
        assert_eq!(foreign_schema.len(), 2);
        assert_eq!(foreign_schema.get("value"), Some(&SchemaType::I32));
        assert_eq!(foreign_schema.get("name"), Some(&SchemaType::String));

        println!("✅ Schema derive macro working correctly!");
        println!("✅ TaMere::schema() returns: {:?}", schema);
        println!("✅ Ready for CSV export in cu29_export!");
    }
}
