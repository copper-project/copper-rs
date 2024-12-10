use cu29::clock::RobotClock;
use cu29::prelude::*;
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

/// This is a sink task that sends messages to an iceoryx2 service.
/// P is the payload type of the messages.
/// Copper messages and Iceoryx2 payloads are compatible.
pub struct IceoryxSink<P>
where
    P: CuMsgPayload, // TODO: Maybe with something a little more generic we can be ROS2 compatible
{
    service_name: ServiceName,
    node: iceoryx2::prelude::Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, CuMsg<P>, ()>>,
    publisher: Option<Publisher<ipc::Service, CuMsg<P>, ()>>,
}

impl<P> Freezable for IceoryxSink<P> where P: CuMsgPayload {}

impl<'cl, P> CuSinkTask<'cl> for IceoryxSink<P>
where
    P: CuMsgPayload + 'cl,
{
    type Input = input_msg!('cl, P);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("You need a config."))?;
        let service_name = config
            .get::<String>("service")
            .ok_or_else(|| CuError::from("You need a service name"))?;

        debug!("Service name: {}", service_name.as_str());

        let service_name = ServiceName::new(service_name.as_str())
            .map_err(|e| CuError::new_with_cause("Failed to create service name.", e))?;
        let node: iceoryx2::prelude::Node<ipc::Service> = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("Failed to create node.", e))?;

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
            .publish_subscribe::<CuMsg<P>>()
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("Failed to create service.", e))?;

        let publisher = service
            .publisher_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("Failed to create publisher.", e))?;

        self.service = Some(service);
        self.publisher = Some(publisher);
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let publisher = self
            .publisher
            .as_mut()
            .ok_or_else(|| CuError::from("Publisher not found."))?;

        let dst = publisher
            .loan_uninit()
            .map_err(|e| CuError::new_with_cause("Failed to loan uninit.", e))?;

        let dst = dst.write_payload(input.clone());

        dst.send()
            .map_err(|e| CuError::new_with_cause("Failed to send message.", e))?;

        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.publisher = None;
        Ok(())
    }
}
