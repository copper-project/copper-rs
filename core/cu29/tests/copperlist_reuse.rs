use cu29::prelude::*;

#[derive(Reflect)]
struct ReuseSource;

impl Freezable for ReuseSource {}

impl CuSrcTask for ReuseSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(42);
        Ok(())
    }
}

#[copper_runtime(
    config = "tests/config/copperlist_reuse_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct ReuseApp {}

#[test]
fn generated_copperlist_clears_payload_on_slot_reuse() {
    let mut manager = CuListsManager::<default::CuStampedDataSet, 1>::new();

    manager
        .create()
        .expect("fresh slot")
        .msgs
        .0
        .0
        .set_payload(99);
    let _ = manager.pop().expect("pop populated slot");

    let reused = manager.create().expect("reuse slot");
    assert!(reused.msgs.0.0.payload().is_none());
}
