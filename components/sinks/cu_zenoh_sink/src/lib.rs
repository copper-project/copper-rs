use cu29::clock::RobotClock;
use cu29::{bincode, prelude::*};

use zenoh::Config;
use zenoh::Error as ZenohError;
use zenoh::key_expr::KeyExpr;

use std::marker::PhantomData;

/// This is a sink task that sends messages to a zenoh topic.
/// P is the payload type of the messages.
/// Copper messages and Zenoh payloads are compatible.
#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct ZenohSink<P>
where
    P: CuMsgPayload,
{
    #[reflect(ignore)]
    _marker: PhantomData<fn() -> P>,
    #[reflect(ignore)]
    config: ZenohConfig,
    #[reflect(ignore)]
    ctx: Option<ZenohContext>,
}

pub struct ZenohConfig {
    config: Config,
    topic: String,
}

pub struct ZenohContext {
    session: zenoh::Session,
    publisher: zenoh::pubsub::Publisher<'static>,
}

fn cu_error(msg: &str, error: ZenohError) -> CuError {
    CuError::from(msg).add_cause(&error.to_string())
}

fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    |e| cu_error(msg, e)
}

impl<P> Freezable for ZenohSink<P> where P: CuMsgPayload {}

impl<P> cu29::reflect::TypePath for ZenohSink<P>
where
    P: CuMsgPayload + 'static,
{
    fn type_path() -> &'static str {
        "cu_zenoh_sink::ZenohSink"
    }

    fn short_type_path() -> &'static str {
        "ZenohSink"
    }

    fn type_ident() -> Option<&'static str> {
        Some("ZenohSink")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_zenoh_sink")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_zenoh_sink")
    }
}

impl<P> CuSinkTask for ZenohSink<P>
where
    P: CuMsgPayload + 'static,
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!(P);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or(CuError::from("ZenohSink: Missing configuration"))?;

        // Get json zenoh config
        let session_config = match config.get::<String>("zenoh_config_file")? {
            Some(path) => Config::from_file(&path)
                .map_err(cu_error_map("ZenohSink: Failed to create zenoh config"))?,
            None => Config::default(),
        };

        let topic = config
            .get::<String>("topic")?
            .unwrap_or("copper".to_owned());

        Ok(Self {
            _marker: PhantomData,
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

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
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
