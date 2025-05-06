mod attachment;
mod error;
mod keyexpr;
mod liveliness;
mod node;
mod topic;

use attachment::encode_attachment;
use cdr::{CdrBe, Infinite};
use cu29::clock::RobotClock;
use cu29::prelude::*;
use cu_ros_payloads::RosMsgAdapter;
use error::cu_error_map;
use liveliness::{node_liveliness, publisher_liveliness};
use node::Node;
use topic::Topic;
use zenoh::bytes::Encoding;
use zenoh::Config;

use std::marker::PhantomData;

// Only one node per session
const NODE_ID: u32 = 0;

// Only one publisher per session
const PUBLISHER_ID: u32 = NODE_ID + 1;

/// This is a sink task that sends ROS-compatible messages to a Zenoh topic.
/// P is the payload type of the messages, which must be convertible to ROS format.
/// Hence the payload type must implement the `RosMsgAdapter` trait.
pub struct ZenohRosSink<P>
where
    P: CuMsgPayload + RosMsgAdapter<'static>,
{
    _marker: PhantomData<P>,
    config: ZenohRosConfig,
    ctx: Option<ZenohRosContext>,
}

pub struct ZenohRosConfig {
    session: zenoh::Config,
    domain_id: u32,
    namespace: String,
    node: String,
    topic: String,
}

pub struct ZenohRosContext {
    session: zenoh::Session,
    publisher: zenoh::pubsub::Publisher<'static>,
    // Token have to be kept alive (undeclared on drop)
    #[allow(dead_code)]
    node_token: zenoh::liveliness::LivelinessToken,
    #[allow(dead_code)]
    publisher_token: zenoh::liveliness::LivelinessToken,
    sequence_number: u64,
}

impl<P> Freezable for ZenohRosSink<P> where P: CuMsgPayload + RosMsgAdapter<'static> {}

impl<'cl, P> CuSinkTask<'cl> for ZenohRosSink<P>
where
    P: CuMsgPayload + RosMsgAdapter<'static> + 'cl,
{
    type Input = input_msg!('cl, P);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or(CuError::from("ZenohRosSink: Missing configuration"))?;

        // Get json zenoh config
        let session_config = config.get::<String>("zenoh_config_json").map_or(
            // Or default zenoh config otherwise
            CuResult::Ok(Config::default()),
            |s| -> CuResult<zenoh::Config> {
                Config::from_json5(&s)
                    .map_err(cu_error_map("ZenohRosSink: Failed to create zenoh config"))
            },
        )?;

        Ok(Self {
            _marker: Default::default(),
            config: ZenohRosConfig {
                session: session_config,
                domain_id: config.get::<u32>("domain_id").unwrap_or(0),
                namespace: config.get::<String>("namespace").unwrap_or("node".into()),
                node: config.get::<String>("node").unwrap_or("node".into()),
                topic: config.get::<String>("topic").unwrap_or("copper".into()),
            },
            ctx: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let session = zenoh::Wait::wait(zenoh::open(self.config.session.clone()))
            .map_err(cu_error_map("ZenohRosSink: Failed to open session"))?;

        debug!("Zenoh session open");

        let zid = session.zid();

        let node = Node {
            domain_id: self.config.domain_id,
            zid,
            id: NODE_ID,
            namespace: self.config.namespace.as_ref(),
            name: self.config.node.as_ref(),
        };

        let topic = Topic::new::<P>(self.config.topic.as_ref());

        let node_token =
            zenoh::Wait::wait(session.liveliness().declare_token(node_liveliness(&node)?))
                .map_err(cu_error_map(
                    "ZenohRosSink: Failed to declare node liveliness token",
                ))?;

        let publisher_token = zenoh::Wait::wait(
            session
                .liveliness()
                .declare_token(publisher_liveliness(&node, &topic, PUBLISHER_ID)?),
        )
        .map_err(cu_error_map(
            "ZenohRosSink: Failed to declare topic liveliness token",
        ))?;

        // Format the key expression according to ROS 2 conventions
        let publisher = zenoh::Wait::wait(session.declare_publisher(topic.pubsub_keyexpr(&node)?))
            .map_err(cu_error_map("ZenohRosSink: Failed to create publisher"))?;

        self.ctx = Some(ZenohRosContext {
            session,
            publisher,
            node_token,
            publisher_token,
            sequence_number: 0,
        });
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let ctx = self
            .ctx
            .as_mut()
            .ok_or(CuError::from("ZenohRosSink: Context not found"))?;

        // Convert the payload to ROS-compatible CDR format
        let payload = input.payload().ok_or(CuError::from(
            "ZenohRosSink: Cannot send empty payload through ROS bridge",
        ))?;

        let ros_payload = payload.convert();
        let serial_ros_payload = cdr::serialize::<_, _, CdrBe>(&ros_payload, Infinite)
            .map_err(|e| CuError::new_with_cause("ZenohRosSink: Failed to serialize payload", e))?;

        let serial_metadata = encode_attachment(ctx.sequence_number, clock, &ctx.session.zid());

        ctx.sequence_number += 1;

        zenoh::Wait::wait(
            ctx.publisher
                .put(serial_ros_payload)
                .encoding(Encoding::APPLICATION_CDR)
                .attachment(serial_metadata),
        )
        .map_err(cu_error_map("ZenohRosSink: Failed to put value"))?;

        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(ZenohRosContext {
            session,
            publisher,
            node_token: _,
            publisher_token: _,
            sequence_number: _,
        }) = self.ctx.take()
        {
            zenoh::Wait::wait(publisher.undeclare())
                .map_err(cu_error_map("ZenohRosSink: Failed to undeclare publisher"))?;
            zenoh::Wait::wait(session.close())
                .map_err(cu_error_map("ZenohRosSink: Failed to close session"))?;
        }
        debug!("ZenohRosSink: Stopped");
        Ok(())
    }
}
