use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::prelude::*;
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

#[derive(Clone, Debug, Default, Decode, Encode)]
pub struct IceorixCuMsg<P: CuMsgPayload>(CuMsg<P>);

unsafe impl<P: CuMsgPayload> ZeroCopySend for IceorixCuMsg<P> {}
/// This is a sink task that sends messages to an iceoryx2 service.
/// P is the payload type of the messages.
/// Copper messages and Iceoryx2 payloads are compatible.
pub struct IceoryxSink<P>
where
    P: CuMsgPayload + 'static, // TODO: Maybe with something a little more generic we can be ROS2 compatible
{
    service_name: ServiceName,
    node: iceoryx2::prelude::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, IceorixCuMsg<P>, ()>>,
    publisher: Option<Publisher<ipc::Service, IceorixCuMsg<P>, ()>>,
}

impl<P> Freezable for IceoryxSink<P> where P: CuMsgPayload {}

impl<'cl, P> CuSinkTask<'cl> for IceoryxSink<P>
where
    P: CuMsgPayload + 'cl + 'static,
{
    type Input = input_msg!('cl, P);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("IceoryxSink: Missing configuration."))?;
        let service_name_str = config.get::<String>("service").ok_or_else(|| {
            CuError::from("IceoryxSink: Configuration requires 'service' key (string).")
        })?;

        debug!(
            "IceoryxSink: Configuring service name: {}",
            service_name_str.as_str()
        );

        let service_name = ServiceName::new(service_name_str.as_str()).map_err(|e| {
            CuError::new_with_cause("IceoryxSink: Failed to create service name.", e)
        })?;
        let node: iceoryx2::prelude::Node<ipc::Service> =
            NodeBuilder::new().create::<ipc::Service>().map_err(|e| {
                CuError::new_with_cause(
                    format!("IceoryxSink({service_name_str}): Failed to create node.").as_str(),
                    e,
                )
            })?;

        Ok(Self {
            service_name,
            node,
            service: None,
            publisher: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<IceorixCuMsg<P>>()
            .open_or_create()
            .map_err(|e| {
                CuError::new_with_cause(
                    format!(
                        "IceoryxSink({}): Failed to create service.",
                        self.service_name
                    )
                    .as_str(),
                    e,
                )
            })?;

        let publisher = service.publisher_builder().create().map_err(|e| {
            CuError::new_with_cause(
                format!(
                    "IceoryxSink({}): Failed to create publisher.",
                    self.service_name
                )
                .as_str(),
                e,
            )
        })?;

        self.service = Some(service);
        self.publisher = Some(publisher);
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let publisher = self.publisher.as_mut().ok_or_else(|| {
            CuError::from(
                format!("IceoryxSink({}): Publisher not found.", self.service_name).as_str(),
            )
        })?;

        let dst = publisher.loan_uninit().map_err(|e| {
            CuError::new_with_cause(
                format!("IceoryxSink({}): Failed to loan uninit.", self.service_name).as_str(),
                e,
            )
        })?;

        let dst = dst.write_payload(IceorixCuMsg(input.clone()));

        dst.send().map_err(|e| {
            CuError::new_with_cause(
                format!(
                    "IceoryxSink({}): Failed to send message.",
                    self.service_name
                )
                .as_str(),
                e,
            )
        })?;

        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.service = None;
        self.publisher = None;
        debug!("IceoryxSink({}): Stopped.", self.service_name.as_str());
        Ok(())
    }
}
