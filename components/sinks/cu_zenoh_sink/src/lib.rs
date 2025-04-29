use cu29::clock::RobotClock;
use cu29::{bincode, prelude::*};

use zenoh::key_expr::KeyExpr;
use zenoh::Config;
use zenoh::Error as ZenohError;

use std::marker::PhantomData;

/// This is a sink task that sends messages to a zenoh topic.
/// P is the payload type of the messages.
/// Copper messages and Zenoh payloads are compatible.
pub struct ZenohSink<P>
where
    P: CuMsgPayload,
{
    _marker: PhantomData<P>,
    config: ZenohConfig,
    ctx: Option<ZenohContext>,
}

pub struct ZenohConfig {
    config: zenoh::Config,
    topic: String,
}

pub struct ZenohContext {
    session: zenoh::Session,
    publisher: zenoh::pubsub::Publisher<'static>,
}

fn cu_error(msg: &str, error: ZenohError) -> CuError {
    CuError::new_with_cause(msg, error.as_ref())
}

fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    |e| cu_error(msg, e)
}

impl<P> Freezable for ZenohSink<P> where P: CuMsgPayload {}

impl<'cl, P> CuSinkTask<'cl> for ZenohSink<P>
where
    P: CuMsgPayload + 'cl + 'static,
{
    type Input = input_msg!('cl, P);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or(CuError::from("ZenohSink: Missing configuration"))?;

        // Get json zenoh config
        let session_config = config.get::<String>("zenoh_config_file").map_or(
            // Or default zenoh config otherwise
            CuResult::Ok(Config::default()),
            |s| -> CuResult<zenoh::Config> {
                Config::from_file(&s)
                    .map_err(cu_error_map("ZenohSink: Failed to create zenoh config"))
            },
        )?;

        let topic = config.get::<String>("topic").unwrap_or("copper".to_owned());

        Ok(Self {
            _marker: Default::default(),
            config: ZenohConfig {
                config: session_config,
                topic,
            },
            ctx: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let session = zenoh::Wait::wait(zenoh::open(self.config.config.clone()))
            .map_err(cu_error_map("ZenohSink: Failed to open session"))?;

        let key_expr = KeyExpr::<'static>::new(self.config.topic.clone())
            .map_err(cu_error_map("ZenohSink: Invalid topic string"))?;

        debug!("Zenoh session open");
        let publisher = zenoh::Wait::wait(session.declare_publisher(key_expr))
            .map_err(cu_error_map("ZenohSink: Failed to create publisher"))?;

        self.ctx = Some(ZenohContext { session, publisher });
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let ctx = self
            .ctx
            .as_mut()
            .ok_or_else(|| CuError::from("ZenohSink: Context not found"))?;

        let encoded =
            bincode::encode_to_vec(input, bincode::config::standard()).expect("Encoding failed");
        zenoh::Wait::wait(ctx.publisher.put(encoded))
            .map_err(cu_error_map("ZenohSink: Failed to put value"))?;
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(ZenohContext { session, publisher }) = self.ctx.take() {
            zenoh::Wait::wait(publisher.undeclare())
                .map_err(cu_error_map("ZenohSink: Failed to undeclare publisher"))?;
            zenoh::Wait::wait(session.close())
                .map_err(cu_error_map("ZenohSink: Failed to close session"))?;
        }
        debug!("ZenohSink: Stopped");
        Ok(())
    }
}
