use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::Freezable;
use cu29::cutask::{CuMsg, CuMsgPayload, CuSrcTask};
use cu29::output_msg;
use cu29_log_derive::debug;
use cu29_traits::{CuError, CuResult};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

/// This is a source task that receives messages from an iceoryx2 service.
/// P is the payload type of the messages.
pub struct IceoryxSrc<P>
where
    P: CuMsgPayload,
{
    service_name: ServiceName,
    node: Node<ipc::Service>,
    service: Option<PortFactory<ipc::Service, CuMsg<P>, ()>>,
    subscriber: Option<Subscriber<ipc::Service, CuMsg<P>, ()>>,
}

impl<P> Freezable for IceoryxSrc<P> where P: CuMsgPayload {}

impl<'cl, P> CuSrcTask<'cl> for IceoryxSrc<P>
where
    P: CuMsgPayload + 'cl,
{
    type Output = output_msg!('cl, P);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("You need a config"))?;
        let service_name = config
            .get::<String>("service")
            .ok_or_else(|| CuError::from("You need a service name"))?;

        debug!("Service name: {}", service_name.as_str());

        let service_name = ServiceName::new(service_name.as_str())
            .map_err(|e| CuError::new_with_cause("Failed to create service name.", e))?;

        let node: Node<ipc::Service> = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("Failed to create node.", e))?;

        Ok(Self {
            service_name,
            node,
            service: None,
            subscriber: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let service = self
            .node
            .service_builder(&self.service_name)
            .publish_subscribe::<CuMsg<P>>()
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("Failed to create service.", e))?;

        let subscriber = service
            .subscriber_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("Failed to create subscriber.", e))?;

        self.subscriber = Some(subscriber);
        self.service = Some(service);
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let sub = self
            .subscriber
            .as_ref()
            .ok_or_else(|| CuError::from("Subscriber not found"))?;

        if let Some(icemsg) = sub
            .receive()
            .map_err(|e| CuError::new_with_cause("Error receiving message.", e))?
        {
            new_msg.set_payload(
                icemsg
                    .payload()
                    .payload()
                    .ok_or(CuError::from("Failed to get payload."))?
                    .clone(),
            );
            new_msg.metadata.tov = icemsg.payload().metadata.tov.clone();
        } else {
            debug!("No message received");
        }

        Ok(())
    }
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.service = None;
        self.subscriber = None;
        Ok(())
    }
}
