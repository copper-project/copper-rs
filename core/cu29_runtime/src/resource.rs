//! Resource descriptors and utilities to hand resources to tasks and bridges.
//!
//! The `resources` module that the derive macro generates for each mission
//! exposes typed `ResourceKey` values and bundle metadata. This module provides
//! the runtime side to register resources (either eagerly via bundle providers
//! declared in the Copper config, or manually with `add_bundle_prebuilt`) and
//! to hydrate the `Resources` associated type declared by each task/bridge.

use crate::config::ComponentConfig;
use core::any::Any;
use core::fmt;
use core::marker::PhantomData;
use cu29_traits::{CuError, CuResult};
use hashbrown::HashMap;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::boxed::Box;
    pub use alloc::format;
    pub use alloc::string::String;
    pub use alloc::sync::Arc;
}

#[cfg(feature = "std")]
mod imp {
    pub use std::boxed::Box;
    pub use std::string::String;
    pub use std::sync::Arc;
}

use imp::*;

/// Lightweight wrapper used when a task needs to take ownership of a resource.
pub struct Owned<T>(pub T);

/// Wrapper used when a task needs to borrow a resource that remains managed by
/// the `ResourceManager`.
pub struct Borrowed<'r, T>(pub &'r T);

enum ResourceEntry {
    Owned(Box<dyn Any + Send + Sync>),
    Shared(Arc<dyn Any + Send + Sync>),
}

impl ResourceEntry {
    fn as_shared<T: 'static + Send + Sync>(&self) -> Option<&T> {
        match self {
            ResourceEntry::Shared(arc) => arc.downcast_ref::<T>(),
            ResourceEntry::Owned(boxed) => boxed.downcast_ref::<T>(),
        }
    }

    fn into_owned<T: 'static + Send + Sync>(self) -> Option<T> {
        match self {
            ResourceEntry::Owned(boxed) => boxed.downcast::<T>().map(|b| *b).ok(),
            ResourceEntry::Shared(_) => None,
        }
    }
}

/// Manages the concrete resources available to tasks and bridges.
#[derive(Default)]
pub struct ResourceManager {
    entries: HashMap<String, ResourceEntry>,
}

impl ResourceManager {
    /// Register an owned resource under a fully-qualified path such as
    /// `fc.spi_1`.
    pub fn add_owned<T: 'static + Send + Sync>(
        &mut self,
        path: impl Into<String>,
        value: T,
    ) -> CuResult<()> {
        let path = path.into();
        if self.entries.contains_key(&path) {
            return Err(CuError::from(format!(
                "Resource '{path}' already registered"
            )));
        }
        self.entries
            .insert(path, ResourceEntry::Owned(Box::new(value)));
        Ok(())
    }

    /// Register a shared (borrowed) resource. Callers keep an `Arc` while tasks
    /// receive references.
    pub fn add_shared<T: 'static + Send + Sync>(
        &mut self,
        path: impl Into<String>,
        value: Arc<T>,
    ) -> CuResult<()> {
        let path = path.into();
        if self.entries.contains_key(&path) {
            return Err(CuError::from(format!(
                "Resource '{path}' already registered"
            )));
        }
        self.entries.insert(
            path,
            ResourceEntry::Shared(value as Arc<dyn Any + Send + Sync>),
        );
        Ok(())
    }

    /// Returns true if any resource with the given bundle prefix (e.g. `fc.`)
    /// is already registered.
    pub fn has_bundle(&self, bundle_id: &str) -> bool {
        let prefix = format!("{bundle_id}.");
        self.entries.keys().any(|k| k.starts_with(&prefix))
    }

    /// Borrow a shared resource by path.
    pub fn borrow<'r, T: 'static + Send + Sync>(&'r self, path: &str) -> CuResult<Borrowed<'r, T>> {
        let entry = self
            .entries
            .get(path)
            .ok_or_else(|| CuError::from(format!("Resource '{path}' not found")))?;
        entry
            .as_shared::<T>()
            .map(Borrowed)
            .ok_or_else(|| CuError::from(format!("Resource '{path}' has unexpected type")))
    }

    /// Move out an owned resource by path.
    pub fn take<T: 'static + Send + Sync>(&mut self, path: &str) -> CuResult<Owned<T>> {
        let entry = self
            .entries
            .remove(path)
            .ok_or_else(|| CuError::from(format!("Resource '{path}' not found")))?;
        entry.into_owned::<T>().map(Owned).ok_or_else(|| {
            CuError::from(format!(
                "Resource '{path}' is not owned or has unexpected type"
            ))
        })
    }

    /// Insert a prebuilt bundle by running a caller-supplied function. This is
    /// the escape hatch for resources that must be constructed in application
    /// code (for example, owning handles to embedded peripherals).
    pub fn add_bundle_prebuilt(
        &mut self,
        bundle_id: &str,
        builder: impl FnOnce(&str, &mut ResourceManager) -> CuResult<()>,
    ) -> CuResult<()> {
        builder(bundle_id, self)
    }
}

/// Trait implemented by resource binding structs passed to task/bridge
/// constructors. Implementors pull the concrete resources they need from the
/// `ResourceManager`, using the symbolic mapping provided in the Copper config
/// (`resources: { name: "bundle.id" }`).
pub trait ResourceBindings<'r>: Sized {
    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&HashMap<String, String>>,
    ) -> CuResult<Self>;
}

impl<'r> ResourceBindings<'r> for () {
    fn from_bindings(
        _manager: &'r mut ResourceManager,
        _mapping: Option<&HashMap<String, String>>,
    ) -> CuResult<Self> {
        Ok(())
    }
}

/// Bundle providers implement this trait to populate the `ResourceManager` with
/// concrete resources for a given bundle id.
pub trait ResourceBundle {
    fn build(
        bundle_id: &str,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()>;
}

/// Typed identifier for a resource entry.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct ResourceKey<Id, T = ()> {
    id: Id,
    _marker: PhantomData<fn() -> T>,
}

impl<Id, T> ResourceKey<Id, T> {
    pub const fn new(id: Id) -> Self {
        Self {
            id,
            _marker: PhantomData,
        }
    }

    pub const fn id(&self) -> &Id {
        &self.id
    }

    /// Reinterpret this key as pointing to a concrete resource type.
    pub fn typed<U>(self) -> ResourceKey<Id, U> {
        ResourceKey {
            id: self.id,
            _marker: PhantomData,
        }
    }
}

impl<Id: fmt::Debug, T> fmt::Debug for ResourceKey<Id, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ResourceKey").field("id", &self.id).finish()
    }
}

/// Static declaration of a single resource path bound to a key.
#[derive(Copy, Clone, Debug)]
pub struct ResourceDecl<Id> {
    pub key: ResourceKey<Id>,
    pub path: &'static str,
}

impl<Id> ResourceDecl<Id> {
    pub const fn new(key: ResourceKey<Id>, path: &'static str) -> Self {
        Self { key, path }
    }
}

/// Static metadata describing how to create a bundle of resources.
///
/// The runtime will call the provider identified by `provider_path` to populate
/// the resources listed in `resources`.
#[derive(Copy, Clone, Debug)]
pub struct ResourceProvider<Id: 'static> {
    pub id: &'static str,
    pub provider_path: &'static str,
    pub resources: &'static [ResourceDecl<Id>],
}

impl<Id: 'static> ResourceProvider<Id> {
    pub const fn new(
        id: &'static str,
        provider_path: &'static str,
        resources: &'static [ResourceDecl<Id>],
    ) -> Self {
        Self {
            id,
            provider_path,
            resources,
        }
    }
}

/// Helper macro to declare the tuple type of resources a task needs, and to
/// destructure that tuple with named bindings.
#[macro_export]
macro_rules! resources {
    ($($name:ident : $ty:ty),+ $(,)?) => {
        ( $( $ty , )+ )
    };
    { $($name:ident),+ $(,)? } => {
        ( $( $name ),+, )
    };
}
