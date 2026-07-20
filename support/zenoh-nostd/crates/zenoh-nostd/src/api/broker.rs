use alloc::collections::BTreeMap;
use alloc::{boxed::Box, sync::Arc};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    mutex::{Mutex, MutexGuard},
};
use zenoh_proto::BrokerError;
use zenoh_proto::msgs::NetworkMessage;
use zenoh_proto::{Endpoint, fields::ZenohIdProto};

use crate::io::transport::{TransportLink, ZTransportLinkTx};
use crate::{config::ZBrokerConfig, io::driver::Driver, platform::ZLinkManager};

type Link<Config> = <<Config as ZBrokerConfig>::LinkManager as ZLinkManager>::Link<'static>;

struct StaticDriver<Config>
where
    Config: ZBrokerConfig + 'static,
{
    driver: core::mem::ManuallyDrop<Driver<'static, Link<Config>, Config::Buff>>,
    ptr: *mut TransportLink<Link<Config>, Config::Buff>,
}

impl<Config> Drop for StaticDriver<Config>
where
    Config: ZBrokerConfig,
{
    fn drop(&mut self) {
        unsafe { core::mem::ManuallyDrop::drop(&mut self.driver) };
        unsafe { drop(Box::from_raw(self.ptr)) };
    }
}

pub struct BrokerState<Config>
where
    Config: ZBrokerConfig + 'static,
{
    north: Option<(ZenohIdProto, Arc<StaticDriver<Config>>)>,
    south: BTreeMap<ZenohIdProto, Arc<StaticDriver<Config>>>,
}

pub struct Broker<Config>
where
    Config: ZBrokerConfig + 'static,
{
    config: &'static Config,
    state: Mutex<NoopRawMutex, BrokerState<Config>>,
}

impl<Config> Broker<Config>
where
    Config: ZBrokerConfig,
{
    pub fn new(config: &'static Config) -> Self {
        Self {
            config,
            state: Mutex::new(BrokerState {
                north: None,
                south: BTreeMap::new(),
            }),
        }
    }

    pub(crate) async fn state(&self) -> MutexGuard<'_, NoopRawMutex, BrokerState<Config>> {
        self.state.lock().await
    }

    async fn update(
        north: bool,
        id: ZenohIdProto,
        state: &mut BrokerState<Config>,
        msg: NetworkMessage<'_>,
        bytes: &[u8],
    ) -> core::result::Result<(), BrokerError> {
        if north {
            for south in state.south.values() {
                south
                    .driver
                    .tx()
                    .await
                    .send_optimized_ref(core::iter::once((msg.as_ref(), bytes)))
                    .await?;
            }
        } else {
            if let Some((_, north)) = &state.north {
                north
                    .driver
                    .tx()
                    .await
                    .send_optimized_ref(core::iter::once((msg.as_ref(), bytes)))
                    .await?;
            }

            for (_, south) in state.south.iter().filter(|(sid, _)| **sid != id) {
                south
                    .driver
                    .tx()
                    .await
                    .send_optimized_ref(core::iter::once((msg.as_ref(), bytes)))
                    .await?;
            }
        }

        Ok(())
    }

    async fn update_north(
        id: ZenohIdProto,
        state: &mut BrokerState<Config>,
        msg: NetworkMessage<'_>,
        bytes: &[u8],
    ) -> core::result::Result<(), BrokerError> {
        Self::update(true, id, state, msg, bytes).await
    }

    async fn update_south(
        id: ZenohIdProto,
        state: &mut BrokerState<Config>,
        msg: NetworkMessage<'_>,
        bytes: &[u8],
    ) -> core::result::Result<(), BrokerError> {
        Self::update(false, id, state, msg, bytes).await
    }

    pub async fn open(&self, endpoint: Endpoint<'_>) -> core::result::Result<(), BrokerError> {
        loop {
            let transport = Box::leak(Box::new(
                self.config
                    .transports()
                    .connect(endpoint.clone(), self.config.buff())
                    .await?,
            ));

            let driver = Arc::new(StaticDriver {
                ptr: transport,
                driver: core::mem::ManuallyDrop::new(Driver::new(transport)),
            });

            self.state().await.north = Some((driver.driver.zid(), driver.clone()));

            if let Err(e) = driver
                .driver
                .run(&self.state, Self::update_north)
                .await
                .map_err(|e| e.flatten_map::<BrokerError>())
            {
                zenoh_proto::error!("Error on north: {}", e);
            }

            self.state().await.north.take();
        }
    }

    pub async fn accept(&self, endpoint: Endpoint<'_>) -> core::result::Result<(), BrokerError> {
        loop {
            let transport = Box::leak(Box::new(
                self.config
                    .transports()
                    .listen(endpoint.clone(), self.config.buff())
                    .await?,
            ));

            let driver = Arc::new(StaticDriver {
                ptr: transport,
                driver: core::mem::ManuallyDrop::new(Driver::new(transport)),
            });

            self.state()
                .await
                .south
                .insert(driver.driver.zid(), driver.clone());

            if let Err(e) = driver
                .driver
                .run(&self.state, Self::update_south)
                .await
                .map_err(|e| e.flatten_map::<BrokerError>())
            {
                zenoh_proto::error!("Error on south: {}", e);
            }

            self.state().await.south.remove(&driver.driver.zid());
        }
    }
}

#[macro_export]
macro_rules! __broker {
    ($CONFIG:ty: $config:expr) => {{
        static CONFIG: static_cell::StaticCell<$CONFIG> = static_cell::StaticCell::new();
        let config = CONFIG.init($config);

        static BROKER: static_cell::StaticCell<$crate::broker::Broker<$CONFIG>> =
            static_cell::StaticCell::new();

        BROKER.init($crate::broker::Broker::new(config)) as &'static $crate::broker::Broker<$CONFIG>
    }};
}
