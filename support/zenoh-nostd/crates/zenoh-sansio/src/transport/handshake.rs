use core::fmt::Display;

use zenoh_proto::{EitherError, TransportError, msgs::InitSyn};

use crate::{
    Transport, TransportRx, TransportTx, ZTransportRx, ZTransportTx,
    establishment::{Description, State},
};

pub enum Handshake<Buff, T, Read, Write> {
    PendingInit {
        #[allow(private_interfaces)]
        state: State,

        init: InitSyn<'static>,
        prefixed: bool,

        tx: TransportTx<Buff>,
        rx: TransportRx<Buff>,

        handle: T,

        read: Read,
        write: Write,
    },

    PendingRecv {
        #[allow(private_interfaces)]
        state: State,

        prefixed: bool,

        tx: TransportTx<Buff>,
        rx: TransportRx<Buff>,

        handle: T,

        read: Read,
        write: Write,
    },
    Ready {
        #[allow(private_interfaces)]
        description: Description,

        tx: TransportTx<Buff>,
        rx: TransportRx<Buff>,
    },
    Opened,
}

pub struct HandshakeReady<'a, Buff, T, Read, Write> {
    handshake: &'a mut Handshake<Buff, T, Read, Write>,
}

impl<'a, Buff, T, Read, Write> HandshakeReady<'a, Buff, T, Read, Write> {
    pub fn open(self) -> Transport<Buff> {
        if let Handshake::Ready {
            description,
            tx,
            rx,
        } = core::mem::replace(self.handshake, Handshake::Opened)
        {
            Transport::new(description, tx.into_inner(), rx.into_inner())
        } else {
            unreachable!()
        }
    }
}

impl<Buff, T, Read, Write> Handshake<Buff, T, Read, Write> {
    pub fn prefixed(mut self) -> Self {
        if let Self::PendingRecv { prefixed, .. } | Self::PendingInit { prefixed, .. } = &mut self {
            *prefixed = true;
        }

        self
    }

    pub fn with_prefixed(mut self, p: bool) -> Self {
        if let Self::PendingRecv { prefixed, .. } | Self::PendingInit { prefixed, .. } = &mut self {
            *prefixed = p;
        }

        self
    }

    #[allow(clippy::type_complexity)]
    pub fn poll<E>(
        &mut self,
    ) -> core::result::Result<
        Option<HandshakeReady<'_, Buff, T, Read, Write>>,
        EitherError<TransportError, E>,
    >
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: FnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: FnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        match self {
            Self::Opened => Ok(None),
            Self::Ready { .. } => Ok(Some(HandshakeReady { handshake: self })),
            Self::PendingInit {
                init,
                prefixed,
                tx,
                handle,
                write,
                ..
            } => {
                tx.init_syn(init);

                if let Some(bytes) = tx.flush(*prefixed) {
                    write(handle, bytes).map_err(EitherError::B)?;
                }

                if let Self::PendingInit {
                    state,
                    prefixed,
                    tx,
                    rx,
                    handle,
                    read,
                    write,
                    ..
                } = core::mem::replace(self, Self::Opened)
                {
                    *self = Self::PendingRecv {
                        state,
                        prefixed,
                        tx,
                        rx,
                        handle,
                        read,
                        write,
                    };
                } else {
                    unreachable!()
                }

                Ok(None)
            }
            Self::PendingRecv {
                state,
                prefixed,
                tx,
                rx,
                handle,
                read,
                write,
            } => {
                if let Some(description) = state.description() {
                    if let Self::PendingRecv { tx, rx, .. } = core::mem::replace(self, Self::Opened)
                    {
                        *self = Self::Ready {
                            description,
                            tx,
                            rx,
                        };

                        return Ok(Some(HandshakeReady { handshake: self }));
                    } else {
                        unreachable!()
                    }
                }

                rx.decode_with(|bytes| read(handle, bytes), *prefixed)?;
                let resp = rx
                    .flush_transport()
                    .map(|msg| state.poll(msg))
                    .filter_map(|response| response.0);

                for resp in resp {
                    tx.transport(resp)
                }

                if let Some(bytes) = tx.flush(*prefixed) {
                    write(handle, bytes).map_err(EitherError::B)?;
                }

                Ok(None)
            }
        }
    }

    pub async fn poll_async<E>(
        &mut self,
    ) -> core::result::Result<
        Option<HandshakeReady<'_, Buff, T, Read, Write>>,
        EitherError<TransportError, E>,
    >
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: AsyncFnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: AsyncFnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        match self {
            Self::Opened => Ok(None),
            Self::Ready { .. } => Ok(Some(HandshakeReady { handshake: self })),
            Self::PendingInit {
                init,
                prefixed,
                tx,
                handle,
                write,
                ..
            } => {
                tx.init_syn(init);

                if let Some(bytes) = tx.flush(*prefixed) {
                    write(handle, bytes).await.map_err(EitherError::B)?;
                }

                if let Self::PendingInit {
                    state,
                    prefixed,
                    tx,
                    rx,
                    handle,
                    read,
                    write,
                    ..
                } = core::mem::replace(self, Self::Opened)
                {
                    *self = Self::PendingRecv {
                        state,
                        prefixed,
                        tx,
                        rx,
                        handle,
                        read,
                        write,
                    };
                } else {
                    unreachable!()
                }

                Ok(None)
            }
            Self::PendingRecv {
                state,
                prefixed,
                tx,
                rx,
                handle,
                read,
                write,
            } => {
                if let Some(description) = state.description() {
                    if let Self::PendingRecv { tx, rx, .. } = core::mem::replace(self, Self::Opened)
                    {
                        *self = Self::Ready {
                            description,
                            tx,
                            rx,
                        };

                        return Ok(Some(HandshakeReady { handshake: self }));
                    } else {
                        unreachable!()
                    }
                }

                rx.decode_with_async(async |bytes| read(handle, bytes).await, *prefixed)
                    .await?;

                let resp = rx
                    .flush_transport()
                    .map(|msg| state.poll(msg))
                    .filter_map(|response| response.0);
                for resp in resp {
                    tx.transport(resp);
                }

                if let Some(bytes) = tx.flush(*prefixed) {
                    write(handle, bytes).await.map_err(EitherError::B)?;
                }

                Ok(None)
            }
        }
    }

    pub fn finish<E>(
        mut self,
    ) -> core::result::Result<Transport<Buff>, EitherError<TransportError, E>>
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: FnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: FnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        loop {
            if let Some(ready) = self.poll()? {
                break Ok(ready.open());
            }
        }
    }

    pub async fn finish_async<E>(
        mut self,
    ) -> core::result::Result<Transport<Buff>, EitherError<TransportError, E>>
    where
        E: Display,
        Buff: Clone + AsMut<[u8]> + AsRef<[u8]>,
        Read: AsyncFnMut(&mut T, &mut [u8]) -> core::result::Result<usize, E>,
        Write: AsyncFnMut(&mut T, &[u8]) -> core::result::Result<(), E>,
    {
        loop {
            if let Some(ready) = self.poll_async().await? {
                break Ok(ready.open());
            }
        }
    }
}
