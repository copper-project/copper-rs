use cu29::bevy_reflect;
use cu29::reflect::Reflect;
use cu29::resource::{ResourceBundle, ResourceManager};
use cu29::{CuResult, bundle_resources};
use parking_lot::Mutex;
use std::sync::Arc;

#[derive(Clone, Debug, Reflect)]
#[reflect(from_reflect = false)]
pub struct SharedBus {
    #[reflect(ignore)]
    inner: Arc<SharedBusInner>,
}

#[derive(Debug)]
struct SharedBusInner {
    label: String,
    value: Mutex<i64>,
}

impl SharedBus {
    pub fn new(label: String) -> Self {
        Self {
            inner: Arc::new(SharedBusInner {
                label,
                value: Mutex::new(0),
            }),
        }
    }

    pub fn set(&self, v: i64) {
        *self.inner.value.lock() = v;
    }

    pub fn get(&self) -> i64 {
        *self.inner.value.lock()
    }

    pub fn label(&self) -> &str {
        &self.inner.label
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

#[derive(Clone, Default, Reflect)]
#[reflect(from_reflect = false)]
pub struct GlobalLog {
    #[reflect(ignore)]
    entries: Arc<Mutex<Vec<String>>>,
}

impl GlobalLog {
    pub fn push(&self, entry: impl Into<String>) {
        self.entries.lock().push(entry.into());
    }
}

pub struct BoardBundle;

bundle_resources!(BoardBundle: Bus, Counter, Tag);

impl ResourceBundle for BoardBundle {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&cu29::config::ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let label: String = match config {
            Some(cfg) => cfg
                .get::<String>("label")?
                .unwrap_or_else(|| format!("{}-bus", bundle.bundle_id())),
            None => format!("{}-bus", bundle.bundle_id()),
        };
        let offset: i64 = match config {
            Some(cfg) => cfg.get::<i64>("offset")?.unwrap_or(0),
            None => 0,
        };

        let bus_key = bundle.key(BoardBundleId::Bus);
        let counter_key = bundle.key(BoardBundleId::Counter);
        let tag_key = bundle.key(BoardBundleId::Tag);

        let bus = Arc::new(SharedBus::new(label.clone()));
        manager.add_shared(bus_key, bus.clone())?;
        manager.add_owned(counter_key, OwnedCounter::new(offset))?;
        let tag = Arc::new(label);
        manager.add_shared(tag_key, tag.clone())?;
        Ok(())
    }
}

pub struct ExtraBundle;

bundle_resources!(ExtraBundle: Note);

impl ResourceBundle for ExtraBundle {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&cu29::config::ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let note: String = match config {
            Some(cfg) => cfg
                .get::<String>("note")?
                .unwrap_or_else(|| format!("{}-note", bundle.bundle_id())),
            None => format!("{}-note", bundle.bundle_id()),
        };
        let note_key = bundle.key(ExtraBundleId::Note);
        manager.add_owned(note_key, note)?;
        Ok(())
    }
}

pub struct GlobalBundle;

bundle_resources!(GlobalBundle: Log);

impl ResourceBundle for GlobalBundle {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        _config: Option<&cu29::config::ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let log_key = bundle.key(GlobalBundleId::Log);
        let log = Arc::new(GlobalLog::default());
        manager.add_shared(log_key, log.clone())?;
        Ok(())
    }
}
