use core::ops::DerefMut;
use embassy_futures::select::{Either3, select3};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    mutex::{Mutex, MutexGuard},
};
use embassy_time::{Duration, Instant, Timer};
use zenoh_proto::{EitherError, TransportLinkError, fields::ZenohIdProto, msgs::NetworkMessage};

use crate::{
    io::transport::{
        TransportLink, TransportLinkRx, TransportLinkTx, ZTransportLinkRx, ZTransportLinkTx,
    },
    platform::ZLink,
};

pub struct Driver<'res, Link, Buff>
where
    Link: ZLink + 'res,
{
    zid: ZenohIdProto,
    tx: Mutex<NoopRawMutex, TransportLinkTx<'res, Link::Tx<'res>, Buff>>,
    rx: Mutex<NoopRawMutex, TransportLinkRx<'res, Link::Rx<'res>, Buff>>,
}

impl<'res, Link, Buff> Driver<'res, Link, Buff>
where
    Link: ZLink,
{
    pub fn new(transport: &'res mut TransportLink<Link, Buff>) -> Self {
        let zid = transport.transport().other_zid;

        let (tx, rx) = transport.split();

        Self {
            zid,
            tx: Mutex::new(tx),
            rx: Mutex::new(rx),
        }
    }

    #[allow(dead_code)]
    pub fn zid(&self) -> ZenohIdProto {
        self.zid
    }

    pub async fn tx(
        &self,
    ) -> MutexGuard<'_, NoopRawMutex, TransportLinkTx<'res, Link::Tx<'res>, Buff>> {
        self.tx.lock().await
    }

    pub async fn run<State, E, Update>(
        &self,
        state: &Mutex<NoopRawMutex, State>,
        mut update: Update,
    ) -> core::result::Result<(), EitherError<TransportLinkError, E>>
    where
        Buff: AsMut<[u8]> + AsRef<[u8]>,
        Update: for<'any> AsyncFnMut(
            ZenohIdProto,
            &mut State,
            NetworkMessage<'any>,
            &'any [u8],
        ) -> core::result::Result<(), E>,
    {
        let mut rx = self.rx.lock().await;

        let start = Instant::now();

        loop {
            let (write_lease, read_lease) = self.sync(start, start.elapsed(), &mut rx).await;
            if rx.transport().closed() {
                return Err(EitherError::A(TransportLinkError::TransportClosed));
            }

            match select3(write_lease, read_lease, rx.recv()).await {
                Either3::First(_) => {
                    let mut tx_guard = self.tx.lock().await;
                    let tx = tx_guard.deref_mut();

                    if tx.transport().should_close(start.elapsed().into()) {
                        // TODO: send Close msg
                        break Err(EitherError::A(TransportLinkError::TransportClosed));
                    }

                    if tx.transport().should_send_keepalive(start.elapsed().into()) {
                        zenoh_proto::trace!("Sending Keepalive");
                        tx.keepalive().await?;
                    }

                    continue;
                }
                Either3::Third(res) => {
                    let mut state = state.lock().await;

                    for msg in res? {
                        update(self.zid, &mut state, msg.0, msg.1)
                            .await
                            .map_err(EitherError::B)?;
                    }

                    continue;
                }
                _ => {}
            }

            if rx.transport().should_close(start.elapsed().into()) {
                // TODO: Try send Close msg
                break Err(EitherError::A(TransportLinkError::TransportClosed));
            }
        }
    }

    pub async fn sync(
        &self,
        start: Instant,
        now: Duration,
        rx: &mut TransportLinkRx<'res, Link::Rx<'res>, Buff>,
    ) -> (Timer, Timer) {
        let mut tx_guard = self.tx.lock().await;
        let tx = tx_guard.deref_mut();

        rx.transport_mut().sync(Some(tx.transport()), now.into());
        tx.transport_mut().sync(Some(rx.transport()), now.into());

        let write_lease = start + tx.transport().next_timeout().try_into().unwrap();
        let read_lease = start + rx.transport().next_timeout().try_into().unwrap();

        (Timer::at(write_lease), Timer::at(read_lease))
    }
}
