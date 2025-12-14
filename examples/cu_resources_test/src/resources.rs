use cu29::resource::{ResourceBundle, ResourceDecl, ResourceKey, ResourceManager};
use cu29::{CuError, CuResult};
use parking_lot::Mutex;
use std::sync::Arc;

#[derive(Debug)]
pub struct SharedBus {
    label: String,
    value: Mutex<i64>,
}

impl SharedBus {
    pub fn new(label: String) -> Self {
        Self {
            label,
            value: Mutex::new(0),
        }
    }

    pub fn set(&self, v: i64) {
        *self.value.lock() = v;
    }

    pub fn get(&self) -> i64 {
        *self.value.lock()
    }

    pub fn label(&self) -> &str {
        &self.label
    }
}

#[derive(Debug)]
pub struct OwnedCounter {
    next: i64,
}

impl OwnedCounter {
    pub fn new(start: i64) -> Self {
        Self { next: start }
    }

    pub fn next(&mut self) -> i64 {
        let v = self.next;
        self.next += 1;
        v
    }
}

#[derive(Default)]
pub struct GlobalLog {
    entries: Mutex<Vec<String>>,
}

impl GlobalLog {
    pub fn push(&self, entry: impl Into<String>) {
        self.entries.lock().push(entry.into());
    }
}

fn lookup(resources: &[ResourceDecl], bundle: &str, name: &str) -> CuResult<ResourceKey> {
    let path = format!("{bundle}.{name}");
    resources
        .iter()
        .find(|decl| decl.path == path)
        .map(|decl| decl.key)
        .ok_or_else(|| CuError::from(format!("Resource '{path}' not declared")))
}

pub struct BoardBundle;

impl ResourceBundle for BoardBundle {
    fn build(
        bundle_id: &str,
        config: Option<&cu29::config::ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let label: String = config
            .and_then(|cfg| cfg.get("label"))
            .unwrap_or_else(|| format!("{bundle_id}-bus"));
        let offset: i64 = config.and_then(|cfg| cfg.get("offset")).unwrap_or(0);

        let bus_key = lookup(resources, bundle_id, "bus")?.typed();
        let counter_key = lookup(resources, bundle_id, "counter")?.typed();
        let tag_key = lookup(resources, bundle_id, "tag")?.typed();

        let bus = Arc::new(SharedBus::new(label.clone()));
        manager.add_shared(bus_key, Arc::new(bus))?;
        manager.add_owned(counter_key, OwnedCounter::new(offset))?;
        manager.add_shared(tag_key, Arc::new(Arc::new(label)))?;
        Ok(())
    }
}

pub struct ExtraBundle;

impl ResourceBundle for ExtraBundle {
    fn build(
        bundle_id: &str,
        config: Option<&cu29::config::ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let note: String = config
            .and_then(|cfg| cfg.get("note"))
            .unwrap_or_else(|| format!("{bundle_id}-note"));
        let note_key = lookup(resources, bundle_id, "note")?.typed();
        manager.add_owned(note_key, note)?;
        Ok(())
    }
}

pub struct GlobalBundle;

impl ResourceBundle for GlobalBundle {
    fn build(
        bundle_id: &str,
        _config: Option<&cu29::config::ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let log_key = lookup(resources, bundle_id, "log")?.typed();
        manager.add_shared(log_key, Arc::new(Arc::new(GlobalLog::default())))?;
        Ok(())
    }
}
