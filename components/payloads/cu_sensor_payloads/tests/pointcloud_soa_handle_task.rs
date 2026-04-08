use core::fmt::Debug;

use cu_sensor_payloads::PointCloudSoaHandle;
use cu29::prelude::*;

#[derive(Reflect)]
#[reflect(from_reflect = false)]
struct PointCloudHandleTask;

impl Freezable for PointCloudHandleTask {}

impl CuTask for PointCloudHandleTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(PointCloudSoaHandle<8>);
    type Output<'m> = output_msg!(PointCloudSoaHandle<8>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            output.set_payload(payload.clone());
        } else {
            output.clear_payload();
        }

        Ok(())
    }
}

#[test]
fn pointcloud_soa_handle_satisfies_task_message_bounds() {
    fn assert_debug<T: Debug>() {}
    fn assert_payload<T: CuMsgPayload>() {}
    fn assert_task<T: CuTask>() {}

    assert_task::<PointCloudHandleTask>();
    assert_debug::<PointCloudSoaHandle<8>>();
    assert_payload::<PointCloudSoaHandle<8>>();
}
