use crate::bridges::BusReading;
use crate::resources::{GlobalLog, OwnedCounter, SharedBus};
use cu29::prelude::*;
use cu29::resources;
use serde::{Deserialize, Serialize};

#[derive(
    Default, Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode, Reflect,
)]
pub struct Tick;

#[derive(Reflect)]
pub struct TriggerTask;

impl Freezable for TriggerTask {}

impl CuSrcTask for TriggerTask {
    type Output<'m> = CuMsg<Tick>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process<'o>(&mut self, ctx: &CuContext, output: &mut Self::Output<'o>) -> CuResult<()> {
        info!(ctx, "[trigger] tick");
        output.set_payload(Tick);
        Ok(())
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SensorTask {
    #[reflect(ignore)]
    counter: Owned<OwnedCounter>,
    #[reflect(ignore)]
    bus: SharedBus,
    tag: String,
    #[reflect(ignore)]
    global: GlobalLog,
}

impl Freezable for SensorTask {}

impl CuTask for SensorTask {
    type Resources<'r> = SensorResources;
    type Input<'m> = CuMsg<Tick>;
    type Output<'m> = CuMsg<BusReading>;

    fn new(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        let sensor_resources::Resources {
            counter,
            bus,
            tag,
            global,
        } = resources;
        Ok(Self {
            counter,
            bus: (*bus).clone(),
            tag: (*tag).clone(),
            global: (*global).clone(),
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        _input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let value = self.counter.0.next();
        self.bus.set(value);
        record(
            ctx,
            &self.global,
            format!("sensor {} ({}) -> {}", self.tag, self.bus.label(), value),
        );
        output.set_payload(BusReading {
            tag: self.tag.clone(),
            value,
        });
        Ok(())
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct InspectorTask {
    #[reflect(ignore)]
    bus: SharedBus,
    note: String,
    #[reflect(ignore)]
    global: GlobalLog,
}

impl Freezable for InspectorTask {}

impl CuSinkTask for InspectorTask {
    type Input<'m> = CuMsg<BusReading>;
    type Resources<'r> = InspectorResources;

    fn new(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        let inspector_resources::Resources { bus, note, global } = resources;
        Ok(Self {
            bus: (*bus).clone(),
            note: (*note).clone(),
            global: (*global).clone(),
        })
    }

    fn process<'i>(&mut self, ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(reading) = input.payload() {
            record(
                ctx,
                &self.global,
                format!(
                    "inspector got {} from {} (note {})",
                    reading.value, reading.tag, self.note
                ),
            );
        } else {
            let current = self.bus.get();
            record(
                ctx,
                &self.global,
                format!("inspector polled {} (note {})", current, self.note),
            );
        }
        Ok(())
    }
}

fn record(ctx: &CuContext, global: &GlobalLog, message: impl Into<String>) {
    let msg = message.into();
    global.push(msg.clone());
    info!(ctx, "{msg}");
}

mod sensor_resources {
    use super::*;

    resources!({
        counter => Owned<OwnedCounter>,
        bus => Shared<SharedBus>,
        tag => Shared<String>,
        global => Shared<GlobalLog>,
    });
}

mod inspector_resources {
    use super::*;

    resources!({
        bus => Shared<SharedBus>,
        note => Shared<String>,
        global => Shared<GlobalLog>,
    });
}

type SensorResources = sensor_resources::Resources;
type InspectorResources = inspector_resources::Resources;
