use cu29::prelude::*;
use cu29::resource::{BundleContext, ResourceBundle, ResourceManager};
use cu29::{bundle_resources, resources};
use cu29_derive::copper_runtime;

use std::string::String;

pub struct TestBundle;

bundle_resources!(
    TestBundle:
        Serial3 = "serial3",
        Gpio0 = "gpio0",
        I2c1 = "i2c1"
);

impl ResourceBundle for TestBundle {
    fn build(
        bundle: BundleContext<Self>,
        _config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        manager.add_owned(bundle.key(TestBundleId::Serial3), String::from("serial"))?;
        manager.add_owned(bundle.key(TestBundleId::Gpio0), String::from("pin"))?;
        manager.add_owned(bundle.key(TestBundleId::I2c1), String::from("bus"))?;
        Ok(())
    }
}

#[derive(Reflect)]
struct NoopSource;

impl Freezable for NoopSource {}

impl CuSrcTask for NoopSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(1);
        Ok(())
    }
}

mod sink_resources {
    use super::*;

    resources!({
        serial => Owned<String>,
        pin => Owned<String>,
        bus => Owned<String>,
    });
}

type SinkResources = sink_resources::Resources;

#[derive(Reflect)]
struct UsesNamedResourcesSink;

impl Freezable for UsesNamedResourcesSink {}

impl CuSinkTask for UsesNamedResourcesSink {
    type Resources<'r> = SinkResources;
    type Input<'m> = input_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        let _serial = resources.serial.0;
        let _pin = resources.pin.0;
        let _bus = resources.bus.0;
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "config/resource_bundle_slot_name_case_valid.ron")]
struct App {}

fn main() {}
