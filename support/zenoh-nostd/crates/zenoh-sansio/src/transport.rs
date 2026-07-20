use core::fmt::Display;
use core::time::Duration;

use establishment::Description;
use zenoh_proto::{fields::*, msgs::*};

pub(crate) mod establishment;

mod handshake;
mod rx;
mod traits;
mod tx;

pub use handshake::*;
pub use rx::*;
pub use traits::*;
pub use tx::*;

use crate::transport::establishment::State;

pub struct TransportBuilder<Buff> {
    zid: ZenohIdProto,
    batch_size: u16,
    lease: Duration,
    resolution: Resolution,

    buff: Buff,
}

impl<Buff> TransportBuilder<Buff> {
    fn new(buff: Buff) -> Self
    where
        Buff: AsRef<[u8]>,
    {
        TransportBuilder {
            zid: ZenohIdProto::default(),
            batch_size: buff.as_ref().len() as u16,
            lease: Duration::from_secs(10),
            resolution: Resolution::default(),
            buff,
        }
    }
    pub fn with_zid(mut self, zid: ZenohIdProto) -> Self {
        self.zid = zid;
        self
    }

    pub fn with_batch_size(mut self, batch_size: u16) -> Self {
        self.batch_size = batch_size;
        self
    }

    pub fn with_lease(mut self, lease: Duration) -> Self {
        self.lease = lease;
        self
    }

    pub fn with_resolution(mut self, resolution: Resolution) -> Self {
        self.resolution = resolution;
        self
    }

    pub fn with_buff<NewBuff>(self, buff: NewBuff) -> TransportBuilder<NewBuff> {
        TransportBuilder {
            zid: self.zid,
            batch_size: self.batch_size,
            lease: self.lease,
            resolution: self.resolution,
            buff,
        }
    }

    pub fn codec(self) -> Transport<Buff>
    where
        Buff: Clone,
    {
        Transport {
            tx: TransportTx::new(
                self.buff.clone(),
                self.batch_size as usize,
                0,
                self.resolution,
                self.lease,
            ),
            rx: TransportRx::new(
                self.buff,
                self.batch_size as usize,
                0,
                self.resolution,
                self.lease,
            ),
            mine_zid: self.zid,
            other_zid: self.zid,
        }
    }

    pub fn listen<T, E, Read, Write>(
        self,
        handle: T,
        read: Read,
        write: Write,
    ) -> Handshake<Buff, T, Read, Write>
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: FnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: FnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        let state = State::WaitingInitSyn {
            mine_zid: self.zid,
            mine_batch_size: self.batch_size,
            mine_resolution: self.resolution,
            mine_lease: self.lease,
        };

        let tx = TransportTx::new(
            self.buff.clone(),
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        let rx = TransportRx::new(
            self.buff,
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        Handshake::PendingRecv {
            state,
            prefixed: false,
            tx,
            rx,
            handle,
            read,
            write,
        }
    }

    pub fn listen_async<T, E, Read, Write>(
        self,
        handle: T,
        read: Read,
        write: Write,
    ) -> Handshake<Buff, T, Read, Write>
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: AsyncFnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: AsyncFnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        let state = State::WaitingInitSyn {
            mine_zid: self.zid,
            mine_batch_size: self.batch_size,
            mine_resolution: self.resolution,
            mine_lease: self.lease,
        };

        let tx = TransportTx::new(
            self.buff.clone(),
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        let rx = TransportRx::new(
            self.buff,
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        Handshake::PendingRecv {
            state,
            prefixed: false,
            tx,
            rx,
            handle,
            read,
            write,
        }
    }

    pub fn connect<T, E, Read, Write>(
        self,
        handle: T,
        read: Read,
        write: Write,
    ) -> Handshake<Buff, T, Read, Write>
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: FnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: FnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        let state = State::WaitingInitAck {
            mine_zid: self.zid,
            mine_batch_size: self.batch_size,
            mine_resolution: self.resolution,
            mine_lease: self.lease,
        };

        let tx = TransportTx::new(
            self.buff.clone(),
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        let rx = TransportRx::new(
            self.buff,
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        Handshake::PendingInit {
            state,
            init: InitSyn {
                identifier: InitIdentifier {
                    zid: self.zid,
                    ..Default::default()
                },
                resolution: InitResolution {
                    resolution: self.resolution,
                    batch_size: BatchSize(self.batch_size),
                },
                ..Default::default()
            },
            prefixed: false,
            tx,
            rx,
            handle,
            read,
            write,
        }
    }

    pub fn connect_async<T, E, Read, Write>(
        self,
        handle: T,
        read: Read,
        write: Write,
    ) -> Handshake<Buff, T, Read, Write>
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: AsyncFnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: AsyncFnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        let state = State::WaitingInitAck {
            mine_zid: self.zid,
            mine_batch_size: self.batch_size,
            mine_resolution: self.resolution,
            mine_lease: self.lease,
        };

        let tx = TransportTx::new(
            self.buff.clone(),
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        let rx = TransportRx::new(
            self.buff,
            self.batch_size as usize,
            0,
            self.resolution,
            self.lease,
        );

        Handshake::PendingInit {
            state,
            init: InitSyn {
                identifier: InitIdentifier {
                    zid: self.zid,
                    ..Default::default()
                },
                resolution: InitResolution {
                    resolution: self.resolution,
                    batch_size: BatchSize(self.batch_size),
                },
                ..Default::default()
            },
            prefixed: false,
            tx,
            rx,
            handle,
            read,
            write,
        }
    }
}

pub struct Transport<Buff> {
    pub tx: TransportTx<Buff>,
    pub rx: TransportRx<Buff>,

    pub mine_zid: ZenohIdProto,
    pub other_zid: ZenohIdProto,
}

impl<Buff> Transport<Buff> {
    pub fn builder(buff: Buff) -> TransportBuilder<Buff>
    where
        Buff: AsRef<[u8]>,
    {
        TransportBuilder::new(buff)
    }

    pub(crate) fn new(description: Description, tx: Buff, rx: Buff) -> Self {
        Self {
            tx: TransportTx::new(
                tx,
                description.batch_size as usize,
                description.mine_sn,
                description.resolution,
                description.mine_lease,
            ),
            rx: TransportRx::new(
                rx,
                description.batch_size as usize,
                description.other_sn,
                description.resolution,
                description.other_lease,
            ),
            mine_zid: description.mine_zid,
            other_zid: description.other_zid,
        }
    }

    pub fn sync(&mut self, now: Duration) {
        let Self { tx, rx, .. } = self;
        rx.sync(Some(tx), now);
        tx.sync(Some(rx), now);
    }

    pub fn closed(&self) -> bool {
        self.tx.closed() || self.rx.closed()
    }

    pub fn split(&mut self) -> (&mut TransportTx<Buff>, &mut TransportRx<Buff>) {
        (&mut self.tx, &mut self.rx)
    }
}
