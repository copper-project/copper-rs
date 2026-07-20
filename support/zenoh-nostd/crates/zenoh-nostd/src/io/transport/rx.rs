use zenoh_sansio::{TransportRx, ZTransportRx};

use super::{ZLinkRx, ZTransportLinkRx};

pub struct TransportLinkRx<'transport, LinkRx, Buff> {
    link: LinkRx,
    transport: &'transport mut TransportRx<Buff>,
}

impl<'transport, LinkRx, Buff> TransportLinkRx<'transport, LinkRx, Buff> {
    pub fn new(link: LinkRx, transport: &'transport mut TransportRx<Buff>) -> Self {
        Self { link, transport }
    }

    pub fn transport(&self) -> &TransportRx<Buff> {
        self.transport
    }

    pub fn transport_mut(&mut self) -> &mut TransportRx<Buff> {
        self.transport
    }
}

impl<'transport, LinkRx, Buff> ZTransportLinkRx for TransportLinkRx<'transport, LinkRx, Buff>
where
    LinkRx: ZLinkRx,
    Buff: AsMut<[u8]> + AsRef<[u8]>,
{
    fn rx(&mut self) -> (&mut impl ZLinkRx, &mut impl ZTransportRx) {
        (&mut self.link, self.transport)
    }
}
