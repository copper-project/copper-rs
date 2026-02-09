//! Runtime reflection helpers built on top of `bevy_reflect`.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

pub use bevy_reflect::*;

/// Runtime task-introspection contract exposed by generated Copper applications.
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
