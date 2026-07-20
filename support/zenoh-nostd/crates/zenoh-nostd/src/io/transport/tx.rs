use zenoh_sansio::{TransportTx, ZTransportTx};

use super::{ZLinkTx, ZTransportLinkTx};

pub struct TransportLinkTx<'transport, LinkTx, Buff> {
    link: LinkTx,
    transport: &'transport mut TransportTx<Buff>,
}

impl<'transport, LinkTx, Buff> TransportLinkTx<'transport, LinkTx, Buff> {
    pub fn new(link: LinkTx, transport: &'transport mut TransportTx<Buff>) -> Self {
        Self { link, transport }
    }

    pub fn transport(&self) -> &TransportTx<Buff> {
        self.transport
    }

    pub fn transport_mut(&mut self) -> &mut TransportTx<Buff> {
        self.transport
    }
}

impl<'transport, LinkTx, Buff> ZTransportLinkTx for TransportLinkTx<'transport, LinkTx, Buff>
where
    LinkTx: ZLinkTx,
    Buff: AsMut<[u8]> + AsRef<[u8]>,
{
    fn tx(&mut self) -> (&mut impl ZLinkTx, &mut impl ZTransportTx) {
        (&mut self.link, self.transport)
    }
}
