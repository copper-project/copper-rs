//! Runtime reflection helpers built on top of `bevy_reflect`.

#[cfg(feature = "reflect")]
use alloc::format;
use alloc::string::String;
#[cfg(feature = "reflect")]
use alloc::vec::Vec;

#[cfg(feature = "reflect")]
pub use bevy_reflect::*;

#[cfg(feature = "reflect")]
pub trait ReflectTypePath: TypePath {}

#[cfg(feature = "reflect")]
impl<T: TypePath> ReflectTypePath for T {}

#[cfg(not(feature = "reflect"))]
pub use cu29_reflect_derive::Reflect;

#[cfg(not(feature = "reflect"))]
pub trait Reflect: 'static {}

#[cfg(not(feature = "reflect"))]
impl<T: 'static> Reflect for T {}

#[cfg(not(feature = "reflect"))]
pub trait TypePath {
    fn type_path() -> &'static str {
        core::any::type_name::<Self>()
    }

    fn short_type_path() -> &'static str {
        core::any::type_name::<Self>()
    }

    fn type_ident() -> Option<&'static str> {
        None
    }

    fn crate_name() -> Option<&'static str> {
        None
    }

    fn module_path() -> Option<&'static str> {
        None
    }
}

#[cfg(not(feature = "reflect"))]
pub trait ReflectTypePath {}

#[cfg(not(feature = "reflect"))]
impl<T> ReflectTypePath for T {}

#[cfg(not(feature = "reflect"))]
pub trait GetTypeRegistration {}

#[cfg(not(feature = "reflect"))]
impl<T> GetTypeRegistration for T {}

#[cfg(not(feature = "reflect"))]
#[derive(Debug, Default, Clone, Copy)]
pub struct TypeInfo;

#[cfg(not(feature = "reflect"))]
#[derive(Debug, Default)]
pub struct TypeRegistry;

#[cfg(not(feature = "reflect"))]
impl TypeRegistry {
    pub fn register<T>(&mut self) {
        let _ = core::any::type_name::<T>();
    }
}

/// Runtime task-reflect contract exposed by generated Copper applications.
pub trait ReflectTaskIntrospection {
    /// Returns a reflected immutable task instance for the given task id.
    fn reflect_task(&self, task_id: &str) -> Option<&dyn Reflect>;

    /// Returns a reflected mutable task instance for the given task id.
    fn reflect_task_mut(&mut self, task_id: &str) -> Option<&mut dyn Reflect>;

    /// Registers reflected schema types for this mission's app (tasks, messages, bridges).
    fn register_reflect_types(_registry: &mut TypeRegistry) {}
}

/// Dumps a stable, human-readable schema snapshot for the registered reflected types.
///
/// This is intended for diagnostics, examples, and contract validation.
#[cfg(feature = "reflect")]
pub fn dump_type_registry_schema(registry: &TypeRegistry) -> String {
    let mut entries: Vec<(&'static str, String)> = registry
        .iter()
        .map(|registration| {
            let info = registration.type_info();
            (info.type_path(), format!("{info:#?}"))
        })
        .collect();

    entries.sort_by(|(left, _), (right, _)| left.cmp(right));

    let mut dump = String::new();
    for (type_path, info_dump) in entries {
        dump.push_str("=== ");
        dump.push_str(type_path);
        dump.push_str(" ===\n");
        dump.push_str(&info_dump);
        dump.push('\n');
    }
    dump
}

#[cfg(not(feature = "reflect"))]
pub fn dump_type_registry_schema(_registry: &TypeRegistry) -> String {
    String::new()
}
