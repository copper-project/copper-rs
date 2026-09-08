#![cfg_attr(not(feature = "std"), no_std)]

//! HC-12 FU3 radio setup happens once, before the runtime starts. The host UART
//! must already match the module baud rate (factory default: 9600, 8N1).
use core::marker::PhantomData;
use cu_serial::SerialIo;
use cu29::prelude::*;
use cu29::resource::{
    BundleContext, NamedResourceBundleDecl, ResourceBindings, ResourceBundle, ResourceBundleDecl,
    ResourceManager,
};
use embedded_hal::{delay::DelayNs, digital::OutputPin};

const COMMAND_TIMEOUT_MS: u32 = 500;
const RESPONSE_IDLE_MS: u32 = 20;

/// A configured radio. Keeping SET owned prevents another consumer from putting
/// the module into command mode while data is flowing.
pub struct Hc12<S, P> {
    serial: S,
    _set: P,
}

impl<S, P> core::fmt::Debug for Hc12<S, P> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("Hc12")
    }
}

impl<S: SerialIo, P: OutputPin> Hc12<S, P> {
    pub fn configure(
        mut serial: S,
        mut set: P,
        delay: &mut impl DelayNs,
        channel: u8,
    ) -> CuResult<Self> {
        if !(1..=127).contains(&channel) {
            return Err(CuError::from("HC-12 channel must be 1..=127"));
        }
        // Allow the module's reset interval to elapse before entering AT mode.
        set.set_high()
            .map_err(|_| CuError::from("HC-12 SET high failed"))?;
        delay.delay_ms(200);
        set.set_low()
            .map_err(|_| CuError::from("HC-12 SET low failed"))?;
        delay.delay_ms(40);
        let result: CuResult<()> = (|| {
            // Discard stale bytes with a fixed startup budget.
            let mut discard = [0; 64];
            for _ in 0..16 {
                if serial
                    .try_read(&mut discard)
                    .map_err(|_| CuError::from("HC-12 UART read failed"))?
                    == 0
                {
                    break;
                }
            }
            expect(&mut serial, delay, b"AT", b"OK")?;
            let (reply, len) = command(&mut serial, delay, b"AT+RF")?;
            if trim(&reply[..len]) != b"OK+FU3" {
                expect(&mut serial, delay, b"AT+FU3", b"OK+FU3")?;
            }
            let digits = [
                b'0' + channel / 100,
                b'0' + (channel / 10) % 10,
                b'0' + channel % 10,
            ];
            let (reply, len) = command(&mut serial, delay, b"AT+RC")?;
            let expected_query = [
                b'O', b'K', b'+', b'R', b'C', digits[0], digits[1], digits[2],
            ];
            let expected = [b'O', b'K', b'+', b'C', digits[0], digits[1], digits[2]];
            if trim(&reply[..len]) != expected_query && trim(&reply[..len]) != expected {
                let cmd = [b'A', b'T', b'+', b'C', digits[0], digits[1], digits[2]];
                expect(&mut serial, delay, &cmd, &expected)?;
            }
            Ok(())
        })();
        // Restore transparent mode even when a command failed.
        let restore = set
            .set_high()
            .map_err(|_| CuError::from("HC-12 SET high failed"));
        delay.delay_ms(80);
        result?;
        restore?;
        Ok(Self { serial, _set: set })
    }
}

fn trim(mut bytes: &[u8]) -> &[u8] {
    while bytes.first().is_some_and(u8::is_ascii_whitespace) {
        bytes = &bytes[1..];
    }
    while bytes.last().is_some_and(u8::is_ascii_whitespace) {
        bytes = &bytes[..bytes.len() - 1];
    }
    bytes
}

fn command(
    serial: &mut impl SerialIo,
    delay: &mut impl DelayNs,
    bytes: &[u8],
) -> CuResult<([u8; 64], usize)> {
    let mut written = 0;
    let mut reply = [0; 64];
    let mut len = 0;
    let mut idle = 0;
    for _ in 0..COMMAND_TIMEOUT_MS {
        if written < bytes.len() {
            written += serial
                .try_write(&bytes[written..])
                .map_err(|_| CuError::from("HC-12 UART write failed"))?;
        } else {
            let count = serial
                .try_read(&mut reply[len..])
                .map_err(|_| CuError::from("HC-12 UART read failed"))?;
            len += count;
            if len == reply.len() {
                return Err(CuError::from("HC-12 AT reply too long"));
            }
            if count == 0 && len > 0 {
                idle += 1;
            } else {
                idle = 0;
            }
            if idle >= RESPONSE_IDLE_MS {
                return Ok((reply, len));
            }
        }
        delay.delay_ms(1);
    }
    Err(CuError::from(
        "HC-12 AT command timed out; check SET wiring and UART baud rate",
    ))
}

fn expect(
    serial: &mut impl SerialIo,
    delay: &mut impl DelayNs,
    cmd: &[u8],
    expected: &[u8],
) -> CuResult<()> {
    let (reply, len) = command(serial, delay, cmd)?;
    if trim(&reply[..len]) != expected {
        return Err(CuError::from("Unexpected HC-12 AT reply"));
    }
    Ok(())
}

impl<S: SerialIo, P> embedded_io::ErrorType for Hc12<S, P> {
    type Error = S::Error;
}
impl<S: SerialIo, P> SerialIo for Hc12<S, P> {
    fn try_read(&mut self, bytes: &mut [u8]) -> Result<usize, Self::Error> {
        self.serial.try_read(bytes)
    }
    fn try_write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
        self.serial.try_write(bytes)
    }
}

mod inputs {
    cu29::resources!(for<S, P, D> where S: Send + Sync + 'static, P: Send + Sync + 'static, D: Send + Sync + 'static {
        serial => Owned<S>, set => Owned<P>, delay => Owned<D>,
    });
}

pub struct Hc12Resources<S, P, D>(PhantomData<(S, P, D)>);
struct Slots;
cu29::bundle_resources!(Slots: Serial);
pub use SlotsId as Hc12ResourceId;
impl<S, P, D> ResourceBundleDecl for Hc12Resources<S, P, D> {
    type Id = Hc12ResourceId;
}
impl<S, P, D> NamedResourceBundleDecl for Hc12Resources<S, P, D> {
    const NAMES: &'static [&'static str] = <Slots as NamedResourceBundleDecl>::NAMES;
}
impl<S, P, D> ResourceBundle for Hc12Resources<S, P, D>
where
    S: SerialIo + Send + Sync + 'static,
    P: OutputPin + Send + Sync + 'static,
    D: DelayNs + Send + Sync + 'static,
{
    const INPUT_NAMES: &'static [&'static str] =
        <inputs::Resources<S, P, D> as ResourceBindings>::NAMES;
    fn build(
        bundle: BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let channel = config
            .and_then(|cfg| cfg.get::<u8>("channel").transpose())
            .transpose()?
            .ok_or_else(|| CuError::from("HC-12 requires a channel in RON"))?;
        let mut inputs = bundle.inputs::<inputs::Resources<S, P, D>>(manager)?;
        let radio = Hc12::configure(inputs.serial.0, inputs.set.0, &mut inputs.delay.0, channel)?;
        manager.add_owned(bundle.key(Hc12ResourceId::Serial), radio)
    }
}

#[cfg(feature = "std")]
pub mod host {
    use super::*;
    /// Delay provider for HC-12 startup configuration.
    pub struct StartupDelay;
    impl DelayNs for StartupDelay {
        fn delay_ns(&mut self, ns: u32) {
            std::thread::sleep(std::time::Duration::from_nanos(u64::from(ns)));
        }
    }
    pub struct DelayResources;
    cu29::bundle_resources!(DelayResources: Delay);
    impl ResourceBundle for DelayResources {
        fn build(
            bundle: BundleContext<Self>,
            _: Option<&ComponentConfig>,
            manager: &mut ResourceManager,
        ) -> CuResult<()> {
            manager.add_owned(bundle.key(DelayResourcesId::Delay), StartupDelay)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::convert::Infallible;
    use std::{
        collections::VecDeque,
        sync::{Arc, Mutex},
        vec::Vec,
    };
    #[derive(Default)]
    struct State {
        commands: Vec<Vec<u8>>,
        delays: Vec<u32>,
        levels: Vec<bool>,
    }
    struct Uart {
        state: Arc<Mutex<State>>,
        tx: Vec<u8>,
        rx: VecDeque<u8>,
        silent: bool,
    }
    impl embedded_io::ErrorType for Uart {
        type Error = Infallible;
    }
    impl SerialIo for Uart {
        fn try_read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
            let count = out.len().min(self.rx.len()).min(2);
            for byte in &mut out[..count] {
                *byte = self.rx.pop_front().unwrap();
            }
            Ok(count)
        }
        fn try_write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
            let count = bytes.len().min(2);
            self.tx.extend_from_slice(&bytes[..count]);
            if count == bytes.len() {
                let cmd = core::mem::take(&mut self.tx);
                let reply: &[u8] = match cmd.as_slice() {
                    b"AT" => b"OK",
                    b"AT+RF" => b"OK+FU3",
                    b"AT+RC" => b"OK+RC001",
                    b"AT+C021" => b"OK+C021",
                    _ => panic!("unexpected command"),
                };
                self.state.lock().unwrap().commands.push(cmd);
                if !self.silent {
                    self.rx.extend(reply);
                }
            }
            Ok(count)
        }
    }
    struct Pin(Arc<Mutex<State>>);
    impl embedded_hal::digital::ErrorType for Pin {
        type Error = Infallible;
    }
    impl OutputPin for Pin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0.lock().unwrap().levels.push(false);
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0.lock().unwrap().levels.push(true);
            Ok(())
        }
    }
    struct Delay(Arc<Mutex<State>>);
    impl DelayNs for Delay {
        fn delay_ns(&mut self, ns: u32) {
            self.0.lock().unwrap().delays.push(ns);
        }
    }
    fn setup(channel: u8, silent: bool) -> (CuResult<Hc12<Uart, Pin>>, Arc<Mutex<State>>) {
        let state = Arc::new(Mutex::new(State::default()));
        let uart = Uart {
            state: state.clone(),
            tx: Vec::new(),
            rx: VecDeque::new(),
            silent,
        };
        let result = Hc12::configure(uart, Pin(state.clone()), &mut Delay(state.clone()), channel);
        (result, state)
    }
    #[test]
    fn configures_channel_and_restores_transparent_mode() {
        let (radio, state) = setup(21, false);
        assert!(radio.is_ok());
        let state = state.lock().unwrap();
        assert_eq!(
            state.commands,
            [
                b"AT".to_vec(),
                b"AT+RF".to_vec(),
                b"AT+RC".to_vec(),
                b"AT+C021".to_vec()
            ]
        );
        assert_eq!(state.levels, [true, false, true]);
        assert_eq!(state.delays[0], 200_000_000);
        assert_eq!(state.delays[1], 40_000_000);
        assert_eq!(*state.delays.last().unwrap(), 80_000_000);
    }
    #[test]
    fn unchanged_channel_does_not_write_persistent_settings() {
        let (radio, state) = setup(1, false);
        assert!(radio.is_ok());
        assert_eq!(state.lock().unwrap().commands.len(), 3);
    }
    #[test]
    fn timeout_restores_set_and_invalid_channel_never_touches_hardware() {
        let (radio, state) = setup(21, true);
        assert!(radio.is_err());
        assert_eq!(state.lock().unwrap().levels, [true, false, true]);
        let (radio, state) = setup(0, false);
        assert!(radio.is_err());
        assert!(state.lock().unwrap().levels.is_empty());
    }
}
