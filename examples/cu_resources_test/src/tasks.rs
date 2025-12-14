use crate::bridges::BusReading;
use crate::resources::{GlobalLog, OwnedCounter, SharedBus};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Default, Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct Tick;

pub struct TriggerTask;

impl Freezable for TriggerTask {}

impl CuSrcTask for TriggerTask {
    type Output<'m> = CuMsg<Tick>;
    type Resources<'r> = ();

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self)
    }

    fn process<'o>(&mut self, _clock: &RobotClock, output: &mut Self::Output<'o>) -> CuResult<()> {
        info!("[trigger] tick");
        output.set_payload(Tick);
        Ok(())
    }
}

pub struct SensorTask {
    counter: Owned<OwnedCounter>,
    bus: Arc<SharedBus>,
    tag: Arc<String>,
    global: Arc<GlobalLog>,
}

impl Freezable for SensorTask {}

impl CuTask for SensorTask {
    type Resources<'r> = SensorResources;
    type Input<'m> = CuMsg<Tick>;
    type Output<'m> = CuMsg<BusReading>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        let SensorResources {
            counter,
            bus,
            tag,
            global,
        } = resources;
        Ok(Self {
            counter,
            bus,
            tag,
            global,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        _input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let value = self.counter.0.next();
        self.bus.set(value);
        self.global.push(format!(
            "sensor {} ({}) -> {}",
            self.tag.as_str(),
            self.bus.label(),
            value
        ));
        info!(
            "[sensor] {} ({}) -> {}",
            self.tag.as_str(),
            self.bus.label(),
            value
        );
        output.set_payload(BusReading {
            tag: (*self.tag).clone(),
            value,
        });
        Ok(())
    }
}

pub struct InspectorTask {
    bus: Arc<SharedBus>,
    note: Arc<String>,
    global: Arc<GlobalLog>,
}

impl Freezable for InspectorTask {}

impl CuSinkTask for InspectorTask {
    type Input<'m> = CuMsg<BusReading>;
    type Resources<'r> = InspectorResources;

    fn new_with(
        _config: Option<&ComponentConfig>,
        resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        let InspectorResources { bus, note, global } = resources;
        Ok(Self { bus, note, global })
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(reading) = input.payload() {
            self.global.push(format!(
                "inspector got {} from {} (note {})",
                reading.value,
                reading.tag.clone(),
                self.note.as_str()
            ));
            info!(
                "[inspector] got {} from {} (note {})",
                reading.value,
                reading.tag.clone(),
                self.note.as_str()
            );
        } else {
            let current = self.bus.get();
            self.global.push(format!(
                "inspector polled {} (note {})",
                current,
                self.note.as_str()
            ));
            info!(
                "[inspector] polled {} (note {})",
                current,
                self.note.as_str()
            );
        }
        Ok(())
    }
}

pub struct SensorResources {
    pub counter: Owned<OwnedCounter>,
    pub bus: Arc<SharedBus>,
    pub tag: Arc<String>,
    pub global: Arc<GlobalLog>,
}

impl ResourceBindings<'_> for SensorResources {
    fn from_bindings(
        manager: &mut ResourceManager,
        mapping: Option<&ResourceMapping>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| CuError::from("missing sensor bindings"))?;
        let counter = manager.take(mapping.get("counter").unwrap().typed())?;
        let bus = manager.borrow(mapping.get("bus").unwrap().typed::<Arc<SharedBus>>())?;
        let tag = manager.borrow(mapping.get("tag").unwrap().typed::<Arc<String>>())?;
        let global = manager.borrow(mapping.get("global").unwrap().typed::<Arc<GlobalLog>>())?;
        Ok(Self {
            counter,
            bus: bus.0.clone(),
            tag: tag.0.clone(),
            global: global.0.clone(),
        })
    }
}

pub struct InspectorResources {
    pub bus: Arc<SharedBus>,
    pub note: Arc<String>,
    pub global: Arc<GlobalLog>,
}

impl ResourceBindings<'_> for InspectorResources {
    fn from_bindings(
        manager: &mut ResourceManager,
        mapping: Option<&ResourceMapping>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| CuError::from("missing inspector bindings"))?;
        let bus = manager.borrow(mapping.get("bus").unwrap().typed::<Arc<SharedBus>>())?;
        let note = manager.borrow(mapping.get("note").unwrap().typed::<Arc<String>>())?;
        let global = manager.borrow(mapping.get("global").unwrap().typed::<Arc<GlobalLog>>())?;
        Ok(Self {
            bus: bus.0.clone(),
            note: note.0.clone(),
            global: global.0.clone(),
        })
    }
}
