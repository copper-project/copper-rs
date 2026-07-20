use core::time::Duration;

use sha3::{
    Shake128,
    digest::{ExtendableOutput, Update, XofReader},
};

use zenoh_proto::{TransportError, ZDecode, fields::*, msgs::*};

/// Everything that describes an Opened Transport between two peers
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct Description {
    pub mine_zid: ZenohIdProto,

    pub batch_size: u16,
    pub resolution: Resolution,

    pub mine_lease: Duration,
    pub other_lease: Duration,

    pub mine_sn: u32,
    pub other_sn: u32,

    pub other_zid: ZenohIdProto,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum State {
    WaitingInitSyn {
        /// Mine zid
        mine_zid: ZenohIdProto,
        /// Mine startup batch_size
        mine_batch_size: u16,
        /// Mine startup resolution
        mine_resolution: Resolution,
        /// Mine lease,
        mine_lease: Duration,
    },
    WaitingOpenSyn {
        /// Mine zid
        mine_zid: ZenohIdProto,
        /// Mine startup batch_size
        mine_batch_size: u16,
        /// Mine startup resolution
        mine_resolution: Resolution,
        /// Mine lease,
        mine_lease: Duration,
    },
    WaitingInitAck {
        /// Mine zid
        mine_zid: ZenohIdProto,
        /// Mine startup batch_size
        mine_batch_size: u16,
        /// Mine startup resolution
        mine_resolution: Resolution,
        /// Mine lease,
        mine_lease: Duration,
    },
    WaitingOpenAck {
        /// Mine zid
        mine_zid: ZenohIdProto,
        /// Negotiated batch_size
        batch_size: u16,
        /// Negotiated resolution
        resolution: Resolution,
        /// Computed sn,
        sn: u32,
        /// Mine lease,
        mine_lease: Duration,
        /// Peer zid
        other_zid: ZenohIdProto,
    },
    Opened(Description),
}

impl State {
    pub(crate) fn poll<'a>(
        &mut self,
        input: (TransportMessage<'a>, &'a [u8]),
    ) -> (Option<TransportMessage<'a>>, Option<Description>) {
        if let Self::Opened(description) = &self {
            return (None, Some(*description));
        }

        let (msg, buff) = input;

        match msg {
            // Don't do any computation at this stage. Pass the relevant values as the cookie
            // for future computation.
            TransportMessage::InitSyn(syn) => match *self {
                Self::WaitingInitSyn {
                    mine_zid,
                    mine_batch_size,
                    mine_resolution,
                    mine_lease,
                } => {
                    zenoh_proto::debug!(
                        "Received InitSyn on transport {:?} -> NEW!({:?})",
                        mine_zid,
                        syn.identifier.zid
                    );

                    *self = Self::WaitingOpenSyn {
                        mine_zid,
                        mine_batch_size,
                        mine_resolution,
                        mine_lease,
                    };

                    (
                        Some(TransportMessage::InitAck(InitAck {
                            identifier: InitIdentifier {
                                zid: mine_zid,
                                ..Default::default()
                            },
                            resolution: InitResolution {
                                resolution: mine_resolution,
                                batch_size: BatchSize(mine_batch_size),
                            },
                            cookie: buff, // TODO: cypher ChaCha20
                            ..Default::default()
                        })),
                        None,
                    )
                }
                _ => zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidState),
            },
            // Negotiate values, pass the cookie back
            TransportMessage::InitAck(ack) => match *self {
                Self::WaitingInitAck {
                    mine_zid,
                    mine_batch_size,
                    mine_resolution,
                    mine_lease,
                } => {
                    zenoh_proto::debug!(
                        "Received InitAck on transport {:?} -> ({:?})",
                        mine_zid,
                        ack.identifier.zid
                    );

                    let batch_size = mine_batch_size.min(ack.resolution.batch_size.0);
                    let resolution = {
                        let mut res = Resolution::default();
                        let i_fsn_res = ack.resolution.resolution.get(Field::FrameSN);
                        let m_fsn_res = mine_resolution.get(Field::FrameSN);
                        if i_fsn_res > m_fsn_res {
                            zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidAttribute);
                        }
                        res.set(Field::FrameSN, i_fsn_res);
                        let i_rid_res = ack.resolution.resolution.get(Field::RequestID);
                        let m_rid_res = mine_resolution.get(Field::RequestID);
                        if i_rid_res > m_rid_res {
                            zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidAttribute);
                        }
                        res.set(Field::RequestID, i_rid_res);
                        res
                    };
                    let sn = {
                        let mut hasher = Shake128::default();
                        hasher.update(&mine_zid.as_le_bytes()[..mine_zid.size()]);
                        hasher
                            .update(&ack.identifier.zid.as_le_bytes()[..ack.identifier.zid.size()]);
                        let mut array = 0_u32.to_le_bytes();
                        hasher.finalize_xof().read(&mut array);
                        u32::from_le_bytes(array)
                            & match mine_resolution.get(Field::FrameSN) {
                                Bits::U8 => u8::MAX as u32 >> 1,
                                Bits::U16 => u16::MAX as u32 >> 2,
                                Bits::U32 => u32::MAX >> 4,
                                Bits::U64 => u64::MAX as u32 >> 1,
                            }
                    };

                    *self = Self::WaitingOpenAck {
                        mine_zid,
                        batch_size,
                        resolution,
                        sn,
                        mine_lease,
                        other_zid: ack.identifier.zid,
                    };

                    (
                        Some(TransportMessage::OpenSyn(OpenSyn {
                            lease: mine_lease,
                            sn,
                            cookie: ack.cookie,
                            ..Default::default()
                        })),
                        None,
                    )
                }
                _ => zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidState),
            },
            // Negotiate values, open the transport. Ack the open
            TransportMessage::OpenSyn(open) => match *self {
                Self::WaitingOpenSyn {
                    mine_zid,
                    mine_batch_size,
                    mine_resolution,
                    mine_lease,
                } => {
                    // TODO: decypher cookie ChaCha20
                    let syn = match <InitSyn as ZDecode>::z_decode(&mut &open.cookie[..]) {
                        Ok(syn) => syn,
                        Err(e) => {
                            zenoh_proto::zbail!(@ret (None, None), e)
                        }
                    };

                    zenoh_proto::debug!(
                        "Received OpenSyn on transport {:?} -> ({:?})",
                        mine_zid,
                        syn.identifier.zid
                    );

                    let batch_size = mine_batch_size.min(syn.resolution.batch_size.0);
                    let resolution = {
                        let mut res = Resolution::default();
                        let i_fsn_res = syn.resolution.resolution.get(Field::FrameSN);
                        let m_fsn_res = mine_resolution.get(Field::FrameSN);
                        if i_fsn_res > m_fsn_res {
                            zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidAttribute);
                        }
                        res.set(Field::FrameSN, i_fsn_res);
                        let i_rid_res = syn.resolution.resolution.get(Field::RequestID);
                        let m_rid_res = mine_resolution.get(Field::RequestID);
                        if i_rid_res > m_rid_res {
                            zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidAttribute);
                        }
                        res.set(Field::RequestID, i_rid_res);
                        res
                    };
                    let sn = {
                        let mut hasher = Shake128::default();
                        hasher.update(&mine_zid.as_le_bytes()[..mine_zid.size()]);
                        hasher
                            .update(&syn.identifier.zid.as_le_bytes()[..syn.identifier.zid.size()]);
                        let mut array = 0_u32.to_le_bytes();
                        hasher.finalize_xof().read(&mut array);
                        u32::from_le_bytes(array)
                            & match mine_resolution.get(Field::FrameSN) {
                                Bits::U8 => u8::MAX as u32 >> 1,
                                Bits::U16 => u16::MAX as u32 >> 2,
                                Bits::U32 => u32::MAX >> 4,
                                Bits::U64 => u64::MAX as u32 >> 1,
                            }
                    };

                    let description = Description {
                        mine_zid,
                        batch_size,
                        resolution,
                        mine_lease,
                        other_lease: open.lease,
                        mine_sn: sn,
                        other_sn: open.sn,
                        other_zid: syn.identifier.zid,
                    };

                    *self = Self::Opened(description);

                    (
                        Some(TransportMessage::OpenAck(OpenAck {
                            lease: mine_lease,
                            sn,
                            ..Default::default()
                        })),
                        Some(description),
                    )
                }
                _ => zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidState),
            },
            // Open the transport
            TransportMessage::OpenAck(ack) => match *self {
                Self::WaitingOpenAck {
                    mine_zid,
                    batch_size,
                    resolution,
                    sn,
                    mine_lease,
                    other_zid,
                } => {
                    zenoh_proto::debug!(
                        "Received OpenAck on transport {:?} -> ({:?})",
                        mine_zid,
                        other_zid,
                    );

                    let description = Description {
                        mine_zid,
                        batch_size,
                        resolution,
                        mine_lease,
                        other_lease: ack.lease,
                        mine_sn: sn,
                        other_sn: ack.sn,
                        other_zid,
                    };

                    *self = Self::Opened(description);

                    (None, Some(description))
                }
                _ => zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidState),
            },
            _ => zenoh_proto::zbail!(@ret (None, None), TransportError::InvalidState),
        }
    }

    pub(crate) fn description(&self) -> Option<Description> {
        match self {
            Self::Opened(description) => Some(*description),
            _ => None,
        }
    }
}
