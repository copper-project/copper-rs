//! Resource descriptors and utilities to hand resources to tasks and bridges.
//! User view: in `copperconfig.ron`, map the binding names your tasks/bridges
//! expect to the resources exported by your board bundle. Exclusive things
//! (like a serial port) should be bound once; shared things (like a telemetry
//! bus `Arc`) can be bound to multiple consumers.
//!
//! ```ron
//! (
//!     resources: [ ( id: "board", provider: "board_crate::BoardBundle" ) ],
//!     bridges: [
//!         ( id: "crsf", type: "cu_crsf::CrsfBridge<SerialPort, SerialError>",
//!           resources: { serial: "board.serial0" } // pick whichever serial port you want
//!         ),
//!     ],
//!     tasks: [
//!         ( id: "telemetry", type: "app::TelemetryTask",
//!           resources: { bus: "board.telemetry_bus" } // shared: borrowed
//!         ),
//!     ],
//! )
//! ```
//!
//! Writing your own task/bridge? Add a small `Resources` struct and implement
//! `ResourceBindings` to pull the names you declared:
//! ```rust,ignore
//! pub struct TelemetryResources<'r> { pub bus: Borrowed<'r, TelemetryBus> }
//! impl<'r> ResourceBindings<'r> for TelemetryResources<'r> {
//!     fn from_bindings(mgr: &'r mut ResourceManager, map: Option<&ResourceMapping>) -> CuResult<Self> {
//!         let key = map.expect("bus binding").get("bus").expect("bus").typed();
//!         Ok(Self { bus: mgr.borrow(key)? })
//!     }
//! }
//! pub fn new_with(_cfg: Option<&ComponentConfig>, res: TelemetryResources<'_>) -> CuResult<Self> {
//!     Ok(Self { bus: res.bus })
//! }
//! ```
//! Otherwise, use config to point to the right board resource and you're done.

use crate::config::ComponentConfig;
use core::any::Any;
use core::fmt;
use core::marker::PhantomData;
use cu29_traits::{CuError, CuResult};

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

/// Lightweight wrapper used when a task needs to take ownership of a resource.
pub struct Owned<T>(pub T);

/// Wrapper used when a task needs to borrow a resource that remains managed by
/// the `ResourceManager`.
pub struct Borrowed<'r, T>(pub &'r T);

/// A resource can be exclusive (most common case) or shared.
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

/// Typed identifier for a resource entry.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct ResourceKey<T = ()> {
    // This index is unique per mission
    index: usize,
    _boo: PhantomData<fn() -> T>,
}

impl<T> ResourceKey<T> {
    pub const fn new(index: usize) -> Self {
        Self {
            index,
            _boo: PhantomData,
        }
    }

    pub const fn index(&self) -> usize {
        self.index
    }

    /// Reinterpret this key as pointing to a concrete resource type.
    pub fn typed<U>(self) -> ResourceKey<U> {
        ResourceKey {
            index: self.index,
            _boo: PhantomData,
        }
    }
}

impl<T> fmt::Debug for ResourceKey<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ResourceKey")
            .field("index", &self.index)
            .finish()
    }
}

/// Static declaration of a single resource path bound to a key.
#[derive(Copy, Clone, Debug)]
pub struct ResourceDecl {
    pub key: ResourceKey,
    pub path: &'static str,
}

impl ResourceDecl {
    pub const fn new(key: ResourceKey, path: &'static str) -> Self {
        Self { key, path }
    }
}

/// Static mapping between user-defined binding names (e.g. "bus", "irq") and
/// resource keys. Backed by a slice to avoid runtime allocation.
#[derive(Clone, Copy)]
pub struct ResourceMapping {
    entries: &'static [(&'static str, ResourceKey)],
}

impl ResourceMapping {
    pub const fn new(entries: &'static [(&'static str, ResourceKey)]) -> Self {
        Self { entries }
    }

    pub fn get(&self, name: &str) -> Option<ResourceKey> {
        self.entries
            .iter()
            .find(|(entry_name, _)| *entry_name == name)
            .map(|(_, key)| *key)
    }
}

/// Manages the concrete resources available to tasks and bridges.
pub struct ResourceManager {
    entries: Box<[Option<ResourceEntry>]>,
}

impl ResourceManager {
    /// Creates a new manager sized for the number of resources generated for
    /// the current mission.
    pub fn new(num_resources: usize) -> Self {
        let mut entries = Vec::with_capacity(num_resources);
        entries.resize_with(num_resources, || None);
        Self {
            entries: entries.into_boxed_slice(),
        }
    }

    /// Register an owned resource in the slot identified by `key`.
    pub fn add_owned<T: 'static + Send + Sync>(
        &mut self,
        key: ResourceKey<T>,
        value: T,
    ) -> CuResult<()> {
        let idx = key.index();
        if self.entries[idx].is_some() {
            return Err(CuError::from("Resource already registered"));
        }
        self.entries[idx] = Some(ResourceEntry::Owned(Box::new(value)));
        Ok(())
    }

    /// Register a shared (borrowed) resource. Callers keep an `Arc` while tasks
    /// receive references.
    pub fn add_shared<T: 'static + Send + Sync>(
        &mut self,
        key: ResourceKey<T>,
        value: Arc<T>,
    ) -> CuResult<()> {
        let idx = key.index();
        if self.entries[idx].is_some() {
            return Err(CuError::from("Resource already registered"));
        }
        self.entries[idx] = Some(ResourceEntry::Shared(value as Arc<dyn Any + Send + Sync>));
        Ok(())
    }

    /// Borrow a shared resource by key.
    pub fn borrow<'r, T: 'static + Send + Sync>(
        &'r self,
        key: ResourceKey<T>,
    ) -> CuResult<Borrowed<'r, T>> {
        let idx = key.index();
        let entry = self
            .entries
            .get(idx)
            .and_then(|opt| opt.as_ref())
            .ok_or_else(|| CuError::from("Resource not found"))?;
        entry
            .as_shared::<T>()
            .map(Borrowed)
            .ok_or_else(|| CuError::from("Resource has unexpected type"))
    }

    /// Borrow a shared `Arc`-backed resource by key, cloning the `Arc` for the caller.
    #[cfg(feature = "std")]
    pub fn borrow_shared_arc<T: 'static + Send + Sync>(
        &self,
        key: ResourceKey<T>,
    ) -> CuResult<Arc<T>> {
        let idx = key.index();
        let entry = self
            .entries
            .get(idx)
            .and_then(|opt| opt.as_ref())
            .ok_or_else(|| CuError::from("Resource not found"))?;
        match entry {
            ResourceEntry::Shared(arc) => arc
                .clone()
                .downcast::<T>()
                .map_err(|_| CuError::from("Resource has unexpected type")),
            ResourceEntry::Owned(_) => Err(CuError::from("Resource is owned, not shared")),
        }
    }

    /// Move out an owned resource by key.
    pub fn take<T: 'static + Send + Sync>(&mut self, key: ResourceKey<T>) -> CuResult<Owned<T>> {
        let idx = key.index();
        let entry = self.entries.get_mut(idx).and_then(|opt| opt.take());
        let entry = entry.ok_or_else(|| CuError::from("Resource not found"))?;
        entry
            .into_owned::<T>()
            .map(Owned)
            .ok_or_else(|| CuError::from("Resource is not owned or has unexpected type"))
    }

    /// Insert a prebuilt bundle by running a caller-supplied function. This is
    /// the escape hatch for resources that must be constructed in application
    /// code (for example, owning handles to embedded peripherals).
    pub fn add_bundle_prebuilt(
        &mut self,
        builder: impl FnOnce(&mut ResourceManager) -> CuResult<()>,
    ) -> CuResult<()> {
        builder(self)
    }
}

/// Trait implemented by resource binding structs passed to task/bridge
/// constructors. Implementors pull the concrete resources they need from the
/// `ResourceManager`, using the symbolic mapping provided in the Copper config
/// (`resources: { name: "bundle.resource" }`).
pub trait ResourceBindings<'r>: Sized {
    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceMapping>,
    ) -> CuResult<Self>;
}

impl<'r> ResourceBindings<'r> for () {
    fn from_bindings(
        _manager: &'r mut ResourceManager,
        _mapping: Option<&ResourceMapping>,
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
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()>;
}

/// Static metadata describing how to create a bundle of resources.
///
/// The runtime will call the provider identified by `provider_path` to populate
/// the resources listed in `resources`.
#[derive(Copy, Clone, Debug)]
pub struct ResourceProvider {
    pub id: &'static str,
    pub provider_path: &'static str,
    pub resources: &'static [ResourceDecl],
}

impl ResourceProvider {
    pub const fn new(
        id: &'static str,
        provider_path: &'static str,
        resources: &'static [ResourceDecl],
    ) -> Self {
        Self {
            id,
            provider_path,
            resources,
        }
    }
}

// The legacy `resources!` macro was replaced by the proc-macro in `cu29_derive::resources`.
// Import it from `cu29_derive` or `cu29::prelude`.

#[cfg(feature = "std")]
pub struct ThreadPoolBundle;

#[cfg(feature = "std")]
impl ResourceBundle for ThreadPoolBundle {
    fn build(
        bundle_id: &str,
        config: Option<&ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        use rayon::ThreadPoolBuilder;

        const DEFAULT_THREADS: usize = 2;
        let threads: usize = config
            .and_then(|cfg| cfg.get::<u64>("threads"))
            .map(|v| v as usize)
            .unwrap_or(DEFAULT_THREADS);

        let pool = ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .map_err(|e| CuError::from(format!("Failed to build threadpool: {e}")))?;

        let key = resources
            .iter()
            .find(|decl| decl.path == format!("{bundle_id}.bg_threads"))
            .ok_or_else(|| {
                CuError::from("ThreadPoolBundle missing required resource 'bg_threads'")
            })?
            .key
            .typed();

        manager.add_shared(key, Arc::new(pool))
    }
}
