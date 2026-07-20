use dyn_utils::{DynObject, storage::RawOrBox};
use embassy_sync::channel::{DynamicReceiver, DynamicSender};
use zenoh_proto::{exts::QoS, fields::*, msgs::*, *};

#[cfg(feature = "alloc")]
use crate::api::callbacks::AllocCallbacks;

use crate::{
    api::{
        arg::SampleRef,
        callbacks::{AsyncCallback, DynCallback, FixedCapacityCallbacks, SyncCallback, ZCallbacks},
        sample::Sample,
        session::Session,
    },
    config::ZSessionConfig,
    io::transport::ZTransportLinkTx,
};

pub type FixedCapacitySubCallbacks<
    'a,
    const CAPACITY: usize,
    Callback = RawOrBox<16>,
    Future = RawOrBox<128>,
> = FixedCapacityCallbacks<'a, SampleRef, CAPACITY, Callback, Future>;

#[cfg(feature = "alloc")]
pub type AllocSubCallbacks<'a, Callback = RawOrBox<16>, Future = RawOrBox<128>> =
    AllocCallbacks<'a, SampleRef, Callback, Future>;

pub struct Subscriber<'a, 'res, Config, OwnedSample = (), const CHANNEL: bool = false>
where
    Config: ZSessionConfig,
{
    id: u32,
    ke: &'static keyexpr,
    session: &'a Session<'res, Config>,
    receiver: Option<DynamicReceiver<'res, OwnedSample>>,
}

impl<'a, 'res, Config, OwnedSample, const CHANNEL: bool>
    Subscriber<'a, 'res, Config, OwnedSample, CHANNEL>
where
    Config: ZSessionConfig,
{
    #[allow(dead_code)]
    async fn undeclare(self) -> core::result::Result<(), SessionError> {
        let msg = Declare {
            body: DeclareBody::UndeclareSubscriber(UndeclareSubscriber {
                id: self.id,
                ..Default::default()
            }),
            ..Default::default()
        };

        self.session.state().await.sub_callbacks.remove(self.id)?;

        self.session
            .driver
            .tx()
            .await
            .send(core::iter::once(NetworkMessage {
                reliability: Reliability::default(),
                qos: exts::QoS::default(),
                body: NetworkBody::Declare(msg),
            }))
            .await?;

        todo!("Also stop the channel if any")
    }

    pub fn keyexpr(&self) -> &keyexpr {
        self.ke
    }
}

impl<'a, 'res, Config, OwnedSample> Subscriber<'a, 'res, Config, OwnedSample, true>
where
    Config: ZSessionConfig,
{
    pub fn try_recv(&self) -> Option<OwnedSample> {
        self.receiver.as_ref().unwrap().try_receive().ok()
    }

    pub async fn recv(&self) -> Option<OwnedSample> {
        Some(self.receiver.as_ref().unwrap().receive().await)
    }
}

type CallbackStorage<'res, Config> =
    <<Config as ZSessionConfig>::SubCallbacks<'res> as ZCallbacks<'res, SampleRef>>::Callback;

type FutureStorage<'res, Config> =
    <<Config as ZSessionConfig>::SubCallbacks<'res> as ZCallbacks<'res, SampleRef>>::Future;

pub struct SubscriberBuilder<
    'a,
    'res,
    Config,
    OwnedSample = (),
    const READY: bool = false,
    const CHANNEL: bool = false,
> where
    Config: ZSessionConfig,
{
    session: &'a Session<'res, Config>,
    ke: &'static keyexpr,
    callback: Option<
        DynCallback<'res, CallbackStorage<'res, Config>, FutureStorage<'res, Config>, SampleRef>,
    >,
    receiver: Option<DynamicReceiver<'res, OwnedSample>>,
}

impl<'a, 'res, Config> SubscriberBuilder<'a, 'res, Config, (), false, false>
where
    Config: ZSessionConfig,
{
    pub(crate) fn new(session: &'a Session<'res, Config>, ke: &'static keyexpr) -> Self {
        Self {
            session,
            ke,
            callback: None,
            receiver: None,
        }
    }

    pub fn callback(
        self,
        callback: impl AsyncFnMut(&Sample<'_>) + 'res,
    ) -> SubscriberBuilder<'a, 'res, Config, (), true, false> {
        SubscriberBuilder {
            session: self.session,
            ke: self.ke,
            callback: Some(DynObject::new(AsyncCallback::new(callback))),
            receiver: None,
        }
    }

    pub fn callback_sync(
        self,
        callback: impl FnMut(&Sample<'_>) + 'res,
    ) -> SubscriberBuilder<'a, 'res, Config, (), true, false> {
        SubscriberBuilder {
            session: self.session,
            ke: self.ke,
            callback: Some(DynObject::new(SyncCallback::new(callback))),
            receiver: None,
        }
    }

    pub fn channel<OwnedSample, E>(
        self,
        sender: DynamicSender<'res, OwnedSample>,
        receiver: DynamicReceiver<'res, OwnedSample>,
    ) -> SubscriberBuilder<'a, 'res, Config, OwnedSample, true, true>
    where
        OwnedSample: for<'any> TryFrom<&'any Sample<'any>, Error = E>,
    {
        SubscriberBuilder {
            session: self.session,
            ke: self.ke,
            callback: Some(DynObject::new(AsyncCallback::new(
                async move |resp: &'_ Sample<'_>| {
                    if let Ok(resp) = OwnedSample::try_from(resp) {
                        sender.send(resp).await;
                    } else {
                        zenoh_proto::error!(
                            "{}: Couldn't convert to a transferable sample",
                            zenoh_proto::zctx!()
                        )
                    }
                },
            ))),
            receiver: Some(receiver),
        }
    }
}

impl<'a, 'res, Config, OwnedSample, const CHANNEL: bool>
    SubscriberBuilder<'a, 'res, Config, OwnedSample, true, CHANNEL>
where
    Config: ZSessionConfig,
{
    pub async fn finish(
        self,
    ) -> core::result::Result<Subscriber<'a, 'res, Config, OwnedSample, CHANNEL>, SessionError>
    {
        let mut state = self.session.state().await;
        let id = state.next();

        if let Some(callback) = self.callback {
            state.sub_callbacks.insert(id, self.ke, None, callback)?;
        }

        let msg = Declare {
            body: DeclareBody::DeclareSubscriber(DeclareSubscriber {
                id,
                wire_expr: WireExpr::from(self.ke),
            }),
            ..Default::default()
        };

        self.session
            .driver
            .tx()
            .await
            .send(core::iter::once(NetworkMessage {
                reliability: Reliability::default(),
                qos: QoS::default(),
                body: NetworkBody::Declare(msg),
            }))
            .await?;

        Ok(Subscriber {
            ke: self.ke,
            id,
            session: self.session,
            receiver: self.receiver,
        })
    }
}

impl<'res, Config> Session<'res, Config>
where
    Config: ZSessionConfig,
{
    pub fn declare_subscriber(&self, ke: &'static keyexpr) -> SubscriberBuilder<'_, 'res, Config> {
        SubscriberBuilder::new(self, ke)
    }
}
