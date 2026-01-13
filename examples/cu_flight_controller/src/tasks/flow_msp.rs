use super::*;
use cu_msp_bridge::MspRequestBatch;
use cu_msp_lib::structs::MspRequest;

pub struct FlowMspLogger;

impl Freezable for FlowMspLogger {}

impl CuSinkTask for FlowMspLogger {
    type Input<'m> = CuMsg<MspRequestBatch>;
    type Resources<'r> = ();

    fn new(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        let Some(batch) = input.payload() else {
            return Ok(());
        };

        for request in batch.iter() {
            match request {
                MspRequest::MspSensorOpticFlow(flow) => {
                    debug!(
                        "                                                                              msp flow q={} dx={} dy={}",
                        flow.quality, flow.motion_x, flow.motion_y
                    );
                }
                MspRequest::MspSensorRangefinder(range) => {
                    debug!(
                        "msp range q={} dist_mm={}",
                        range.quality, range.distance_mm
                    );
                }
                _ => {}
            }
        }

        Ok(())
    }
}
