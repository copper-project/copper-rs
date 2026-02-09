#[cfg(feature = "firmware")]
extern crate alloc;

use cu29::prelude::*;

#[cfg(feature = "firmware")]
mod imp {
    pub use alloc::boxed::Box;
    pub use embedded_hal::digital::OutputPin;
    pub use spin::{Mutex, Once};

    pub trait LedPin: Send {
        fn set(&mut self, on: bool);
    }

    pub struct LedWrap<P: OutputPin + Send>(P);

    impl<P: OutputPin + Send> LedPin for LedWrap<P> {
        fn set(&mut self, on: bool) {
            if on {
                let _ = self.0.set_high();
            } else {
                let _ = self.0.set_low();
            }
        }
    }

    pub static LED: Once<Mutex<Box<dyn LedPin>>> = Once::new();
    pub fn register_led<P>(p: P)
    where
        P: OutputPin + Send + 'static,
    {
        LED.call_once(|| Mutex::new(Box::new(LedWrap(p))));
    }

    pub fn set_led(on: bool) {
        if let Some(m) = LED.get() {
            // info!("LED on: {}", on);
            let mut g = m.lock();
            g.set(on);
        }
    }
}

#[cfg(feature = "host")]
mod imp {
    pub fn set_led(on: bool) {
        println!("LED set to: {}", on);
    }
}

pub use imp::*;

#[derive(Reflect)]
pub struct BooleanSource {
    state: bool,
}
impl Freezable for BooleanSource {}
impl CuSrcTask for BooleanSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(bool);
    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { state: false })
    }
    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        self.state = !self.state;
        new_msg.tov = Tov::Time(clock.now());
        *new_msg.payload_mut().as_mut().unwrap() = self.state;
        let end = clock.now() + CuDuration::from_millis(1000);
        while clock.now() < end {
            core::hint::spin_loop();
        }
        Ok(())
    }
}

#[derive(Reflect)]
pub struct LEDSink;
impl Freezable for LEDSink {}
impl CuSinkTask for LEDSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(bool);
    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(&v) = input.payload().as_ref() {
            set_led(*v);
        }
        info!("LEDSink got: {:?}", input.payload());
        Ok(())
    }
}
