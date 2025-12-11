//! Resource key and provider descriptors generated from Copper configs.
//!
//! This module only contains type-level descriptors. The runtime wiring that
//! instantiates and hands resources to tasks will build on top of these types.

use core::fmt;
use core::marker::PhantomData;

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
