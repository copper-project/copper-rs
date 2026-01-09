use super::*;
resources!({
    led => Owned<spin::Mutex<GreenLed>>,
});

pub struct ActivityLed {
    on: bool,
    led: spin::Mutex<GreenLed>,
}

impl Freezable for ActivityLed {}

impl CuSinkTask for ActivityLed {
    type Input<'m> = CuMsg<ControlInputs>;
    type Resources<'r> = Resources;

    fn new_with(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            on: false,
            led: resources.led.0,
        })
    }

    fn process<'i>(&mut self, _clock: &RobotClock, _inputs: &Self::Input<'i>) -> CuResult<()> {
        // Toggle the green LED so we can see if the Copper loop is alive.
        let mut led = self.led.lock();
        if self.on {
            led.set_low();
        } else {
            led.set_high();
        }
        self.on = !self.on;
        Ok(())
    }
}
