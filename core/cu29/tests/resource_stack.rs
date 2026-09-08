use cu29::prelude::*;

pub struct Base;
cu29::bundle_resources!(Base: Value);
impl ResourceBundle for Base {
    fn build(
        bundle: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        manager.add_owned(bundle.key(BaseId::Value), 42u32)
    }
}
mod inputs {
    cu29::resources!({ value => Owned<u32> });
}
pub struct Wrapped;
cu29::bundle_resources!(Wrapped: Value);
impl ResourceBundle for Wrapped {
    const INPUT_NAMES: &'static [&'static str] = <inputs::Resources as ResourceBindings>::NAMES;
    fn build(
        bundle: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let inputs = bundle.inputs::<inputs::Resources>(manager)?;
        manager.add_owned(bundle.key(WrappedId::Value), inputs.value.0 + 1)
    }
}
#[derive(Reflect)]
struct Source;
impl Freezable for Source {}
impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &CuContext, out: &mut Self::Output<'_>) -> CuResult<()> {
        out.set_payload(0);
        Ok(())
    }
}
#[copper_runtime(config = "tests/resource_stack.ron")]
struct App {}

#[test]
fn construction_is_dependency_ordered_with_stable_keys() {
    let config =
        cu29::config::CuConfig::deserialize_ron(include_str!("resource_stack.ron")).unwrap();
    let mut manager = default::resources_instanciator(&config).unwrap();
    let value = manager
        .take::<u32>(BundleIndex::new(0).key(WrappedId::Value))
        .unwrap();
    assert_eq!(value.0, 43);
    assert!(
        manager
            .take::<u32>(BundleIndex::new(1).key(BaseId::Value))
            .is_err()
    );
}

#[test]
fn runtime_configuration_cannot_rewire_compiled_inputs() {
    let mut config =
        cu29::config::CuConfig::deserialize_ron(include_str!("resource_stack.ron")).unwrap();
    config.resources[0]
        .resources
        .as_mut()
        .unwrap()
        .insert("value".into(), "base.other".into());
    assert!(default::resources_instanciator(&config).is_err());
}
