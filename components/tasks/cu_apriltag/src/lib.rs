use apriltag::Detector;
use cu29::prelude::*;

struct AprilTags {
    detector: Detector,
}

impl Freezable for AprilTags {}

impl<'cl> CuTask<'cl> for AprilTags {
    type Input = ();
    type Output = ();

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        todo!()
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        todo!()
    }
}
