use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle, Freezable};
use cu29::output_msg;
use cu29::{CuError, CuResult};
use lazy_static::lazy_static;
use rppal::gpio::{Gpio, InputPin};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;
use std::thread;
use std::thread::JoinHandle;
use std::time::Duration;

lazy_static! {
    static ref GPIO: Gpio = Gpio::new().expect("Could not create GPIO bindings");
}

#[derive(Default, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct EncoderPayload {
    ticks: i32,
}

pub struct Encoder {
    clk_pin: u8, // signals a tick
    dat_pin: u8, // gives the direction of the tick
    ticks_count: Arc<AtomicI32>,
    thread_handle: Option<JoinHandle<()>>,
}

impl Freezable for Encoder {
    // pin is derived from the config, so we keep the default implementation.
}

impl CuTaskLifecycle for Encoder {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or("Encoder needs a config with clk_pin and dat_pin.")?;
        let config = &config.0;

        let clk_pin_nb_value = config.get("clk_pin").ok_or("Encoder needs a clk_pin")?;
        let clk_pin: u8 = clk_pin_nb_value.clone().into();

        let dat_pin_nb_value = config.get("dat_pin").ok_or("Encoder needs a dat_pin")?;
        let dat_pin: u8 = dat_pin_nb_value.clone().into();

        Ok(Self {
            clk_pin,
            dat_pin,
            ticks_count: Arc::new(AtomicI32::new(0)),
            thread_handle: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let ticks_count = self.ticks_count.clone();

        let clk_pin: InputPin = GPIO
            .get(self.clk_pin)
            .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
            .into_input();
        let dat_pin: InputPin = GPIO
            .get(self.dat_pin)
            .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
            .into_input();

        let mut last_clk_state = true;
        let handle: JoinHandle<()> = thread::spawn(move || loop {
            if clk_pin.is_high() {
                if !last_clk_state {
                    if dat_pin.is_low() {
                        ticks_count.fetch_add(1, Ordering::SeqCst);
                    } else {
                        ticks_count.fetch_sub(1, Ordering::SeqCst);
                    }
                    last_clk_state = true;
                } else {
                    last_clk_state = false;
                }
            }

            thread::sleep(Duration::from_millis(1));
        });

        self.thread_handle = Some(handle);
        Ok(())
    }
}

impl<'cl> CuSrcTask<'cl> for Encoder {
    type Output = output_msg!('cl, EncoderPayload);

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        new_msg.metadata.tov = Some(clock.now()).into();
        let ticks = self.ticks_count.load(Ordering::SeqCst);
        new_msg.set_payload(EncoderPayload { ticks });
        Ok(())
    }
}
