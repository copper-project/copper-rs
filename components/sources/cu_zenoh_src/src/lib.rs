use cu29::clock::RobotClock;
use cu29::{bincode, prelude::*};

use zenoh::Config;
use zenoh::Error as ZenohError;
use zenoh::handlers::FifoChannelHandler;
use zenoh::key_expr::KeyExpr;
use zenoh::sample::Sample;
use zenoh::sample::SampleKind;

use std::marker::PhantomData;

/// This is a source task that receives messages from a zenoh topic.
/// P is the payload type of the messages.
/// Copper messages and Zenoh payloads are compatible.
pub struct ZenohSrc<P>
where
    P: CuMsgPayload,
{
    _marker: PhantomData<P>,
    config: ZenohConfig,
    ctx: Option<ZenohContext>,
}

pub struct ZenohConfig {
    config: Config,
    topic: String,
}

pub struct ZenohContext {
    session: zenoh::Session,
    subscriber: zenoh::pubsub::Subscriber<FifoChannelHandler<Sample>>,
}

fn cu_error(msg: &str, error: ZenohError) -> CuError {
    CuError::from(msg).add_cause(&error.to_string())
}

fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    |e| cu_error(msg, e)
}

impl<P> Freezable for ZenohSrc<P> where P: CuMsgPayload {}

impl<P> CuSrcTask for ZenohSrc<P>
where
    P: CuMsgPayload + 'static,
{
    type Resources<'r> = ();
    type Output<'m> = output_msg!(P);

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or(CuError::from("ZenohSrc: Missing configuration"))?;

        // Get json zenoh config
        let session_config = config.get::<String>("zenoh_config_file").map_or(
            // Or default zenoh config otherwise
            Ok(Config::default()),
            |s| -> CuResult<Config> {
                Config::from_file(&s)
                    .map_err(cu_error_map("ZenohSrc: Failed to create zenoh config"))
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
            .map_err(cu_error_map("ZenohSrc: Failed to open session"))?;

        let key_expr = KeyExpr::<'static>::new(self.config.topic.clone())
            .map_err(cu_error_map("ZenohSrc: Invalid topic string"))?;

        debug!("Zenoh session open");
        let subscriber = zenoh::Wait::wait(session.declare_subscriber(key_expr))
            .map_err(cu_error_map("ZenohSrc: Failed to create subscriber"))?;

        self.ctx = Some(ZenohContext {
            session,
            subscriber,
        });
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let ctx = self
            .ctx
            .as_mut()
            .ok_or_else(|| CuError::from("ZenohSrc: Context not found"))?;

        let sample = ctx.subscriber.try_recv().map_err(|e| {
            CuError::from("ZenohSrc: Failed to receive sample").add_cause(&e.to_string())
        })?;

        let Some(sample) = sample else {
            debug!("ZenohSrc: No message received");
            return Ok(());
        };

        if sample.kind() == SampleKind::Delete {
            debug!("ZenohSrc: Skipping delete sample");
            return Ok(());
        }

        let payload = sample.payload().to_bytes();
        let (decoded, _): (CuMsg<P>, usize) =
            bincode::decode_from_slice(payload.as_ref(), bincode::config::standard())
                .map_err(|e| CuError::new_with_cause("ZenohSrc: Failed to decode payload", e))?;

        *new_msg = decoded;
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(ZenohContext {
            session,
            subscriber,
        }) = self.ctx.take()
        {
            zenoh::Wait::wait(subscriber.undeclare())
                .map_err(cu_error_map("ZenohSrc: Failed to undeclare subscriber"))?;
            zenoh::Wait::wait(session.close())
                .map_err(cu_error_map("ZenohSrc: Failed to close session"))?;
        }
        debug!("ZenohSrc: Stopped");
        Ok(())
    }
}
