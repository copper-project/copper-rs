use super::*;

trait LedBackend {
    fn set_on(&mut self, on: bool);
}

#[cfg(feature = "firmware")]
type ActivityLedBackend = spin::Mutex<cu_micoairh743::GreenLed>;

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
type ActivityLedBackend = crate::sim_support::SimActivityLed;

#[cfg(feature = "firmware")]
impl LedBackend for ActivityLedBackend {
    fn set_on(&mut self, on: bool) {
        let mut led = self.lock();
        if on {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
impl LedBackend for ActivityLedBackend {
    fn set_on(&mut self, on: bool) {
        self.set(on);
    }
}

#[cfg(feature = "firmware")]
resources!({
    led => Owned<ActivityLedBackend>,
});

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct ActivityLed {
    on: bool,
    #[reflect(ignore)]
    led: ActivityLedBackend,
}

impl Freezable for ActivityLed {}

impl CuSinkTask for ActivityLed {
    type Input<'m> = CuMsg<ControlInputs>;
    #[cfg(feature = "firmware")]
    type Resources<'r> = Resources;
    #[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        #[cfg(feature = "firmware")]
        let mut led = _resources.led.0;
        #[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
        let mut led = crate::sim_support::sim_activity_led();

        led.set_on(false);
        Ok(Self { on: false, led })
    }

    fn process<'i>(&mut self, _ctx: &CuContext, _inputs: &Self::Input<'i>) -> CuResult<()> {
        self.on = !self.on;
        self.led.set_on(self.on);
        Ok(())
    }
}
