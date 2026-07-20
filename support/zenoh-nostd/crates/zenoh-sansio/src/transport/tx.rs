use core::time::Duration;

use zenoh_proto::{
    TransportError, ZEncode, ZWriteable,
    fields::Resolution,
    msgs::{
        Close, FrameHeader, KeepAlive, MessageRef, NetworkMessage, NetworkMessageRef,
        TransportMessage, TransportMessageRef,
    },
};

use crate::{ZTransportTx, transport::TransportRx};

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum State {
    Opened,
    Used,
    Synchronized { last_sent: Duration },
    Closed,
}

#[derive(Debug)]
pub struct TransportTx<Buff> {
    buff: Buff,
    cursor: usize,
    batch_size: usize,

    sn: u32,
    resolution: Resolution,
    last_frame: Option<FrameHeader>,
    lease: Duration,

    state: State,
}

impl<Buff> TransportTx<Buff> {
    pub(crate) fn new(
        buff: Buff,

        batch_size: usize,
        sn: u32,
        resolution: Resolution,
        lease: Duration,
    ) -> Self {
        Self {
            buff,
            cursor: 2,
            batch_size,
            sn,
            resolution,
            last_frame: None,
            lease,
            state: State::Opened,
        }
    }

    pub(crate) fn into_inner(self) -> Buff {
        self.buff
    }

    pub fn sync(&mut self, rx: Option<&TransportRx<Buff>>, now: Duration) {
        if let Some(rx) = rx
            && rx.closed()
        {
            self.state = State::Closed;
            return;
        }

        if self.should_close(now) {
            self.state = State::Closed;
        }

        if matches!(self.state, State::Used | State::Opened) {
            self.state = State::Synchronized { last_sent: now };
        };
    }

    pub fn next_timeout(&self) -> Duration {
        match self.state {
            State::Opened | State::Closed | State::Used => Duration::from_secs(0),
            State::Synchronized { last_sent } => last_sent + self.lease / 4,
        }
    }

    pub fn should_send_keepalive(&self, now: Duration) -> bool {
        match self.state {
            State::Opened | State::Closed | State::Used => false,
            State::Synchronized { last_sent } => now > last_sent + self.lease / 4,
        }
    }

    pub fn should_close(&self, now: Duration) -> bool {
        match self.state {
            State::Opened | State::Closed | State::Used => false,
            State::Synchronized { last_sent } => now > last_sent + self.lease,
        }
    }

    pub fn closed(&self) -> bool {
        matches!(self.state, State::Closed)
    }

    pub(crate) fn encode(&mut self, msg: MessageRef<'_>, bytes: Option<&[u8]>) -> Option<usize>
    where
        Buff: AsMut<[u8]> + AsRef<[u8]>,
    {
        let max = core::cmp::min(self.buff.as_ref().len(), self.batch_size);
        let mut buff = &mut self.buff.as_mut()[self.cursor..max];

        let start = buff.len();

        let reliability = self.last_frame.as_ref().map(|r| &r.reliability);
        let qos = self.last_frame.as_ref().map(|r| &r.qos);

        match msg {
            MessageRef::Network(msg) => {
                let r = msg.reliability;
                let q = msg.qos;

                let header = if reliability != Some(&r) || qos != Some(&q) {
                    let header = FrameHeader {
                        reliability: r,
                        sn: self.sn,
                        qos: q,
                    };

                    header.z_encode(&mut buff).ok()?;

                    // TODO: wrap with resolution
                    let _ = self.resolution;
                    self.sn = self.sn.wrapping_add(1);

                    Some(header)
                } else {
                    None
                };

                if let Some(bytes) = bytes {
                    buff.write_exact(bytes).ok()?;
                } else {
                    msg.body.z_encode(&mut buff).ok()?;
                }

                if let Some(header) = header {
                    self.last_frame = Some(header);
                }
            }
            MessageRef::Transport(msg) => {
                self.last_frame.take();

                if let Some(bytes) = bytes {
                    buff.write_exact(bytes).ok()?;
                } else {
                    msg.z_encode(&mut buff).ok()?;
                }
            }
        };

        self.cursor += start - buff.len();
        Some(start - buff.len())
    }
}

impl<Buff> ZTransportTx for TransportTx<Buff>
where
    Buff: AsMut<[u8]> + AsRef<[u8]>,
{
    fn keepalive(&mut self) {
        self.transport(TransportMessage::KeepAlive(KeepAlive));
    }

    fn init_syn(&mut self, syn: &zenoh_proto::msgs::InitSyn) {
        self.transport_ref(TransportMessageRef::InitSyn(syn));
    }

    fn init_ack(&mut self, ack: &zenoh_proto::msgs::InitAck) {
        self.transport_ref(TransportMessageRef::InitAck(ack));
    }

    fn open_syn(&mut self, syn: &zenoh_proto::msgs::OpenSyn) {
        self.transport_ref(TransportMessageRef::OpenSyn(syn));
    }

    fn open_ack(&mut self, ack: &zenoh_proto::msgs::OpenAck) {
        self.transport_ref(TransportMessageRef::OpenAck(ack));
    }

    fn close(&mut self) {
        self.transport(TransportMessage::Close(Close::default()));
    }

    fn transport(&mut self, msg: TransportMessage) {
        let len = self
            .encode(MessageRef::Transport(msg.as_ref()), None)
            .iter()
            .sum::<usize>();

        if len != 0 {
            self.state = State::Used;
        }
    }

    fn transport_ref(&mut self, msg: TransportMessageRef) {
        let len = self
            .encode(MessageRef::Transport(msg), None)
            .iter()
            .sum::<usize>();

        if len != 0 {
            self.state = State::Used;
        }
    }

    fn encode<'a>(&mut self, msgs: impl Iterator<Item = NetworkMessage<'a>>) {
        let len = msgs
            .filter_map(|msg| self.encode(MessageRef::Network(msg.as_ref()), None))
            .sum::<usize>();

        if len != 0 {
            self.state = State::Used;
        }
    }

    fn encode_ref<'a>(&mut self, msgs: impl Iterator<Item = NetworkMessageRef<'a>>) {
        let len = msgs
            .filter_map(|msg| self.encode(MessageRef::Network(msg), None))
            .sum::<usize>();

        if len != 0 {
            self.state = State::Used;
        }
    }

    fn encode_optimized<'a>(&mut self, msgs: impl Iterator<Item = (NetworkMessage<'a>, &'a [u8])>) {
        let len = msgs
            .filter_map(|msg| self.encode(MessageRef::Network(msg.0.as_ref()), Some(msg.1)))
            .sum::<usize>();

        if len != 0 {
            self.state = State::Used;
        }
    }

    fn encode_optimized_ref<'a>(
        &mut self,
        msgs: impl Iterator<Item = (NetworkMessageRef<'a>, &'a [u8])>,
    ) {
        let len = msgs
            .filter_map(|msg| self.encode(MessageRef::Network(msg.0), Some(msg.1)))
            .sum::<usize>();

        if len != 0 {
            self.state = State::Used;
        }
    }

    fn flush_prefixed(&mut self) -> Option<&'_ [u8]> {
        let size = core::cmp::min(
            self.buff.as_ref().len(),
            core::cmp::min(self.batch_size, self.cursor),
        );

        if size < 2 {
            zenoh_proto::zbail!(@None TransportError::TransportTxFull);
        }

        let len = ((size - 2) as u16).to_le_bytes();
        self.buff.as_mut()[..2].copy_from_slice(&len);
        self.clear();

        let buff_ref = &self.buff.as_ref()[..size];
        if size > 0 { Some(buff_ref) } else { None }
    }

    fn flush_raw(&mut self) -> Option<&'_ [u8]> {
        let size = core::cmp::min(
            self.buff.as_ref().len(),
            core::cmp::min(self.batch_size, self.cursor),
        );

        self.clear();

        let buff_ref = &self.buff.as_ref()[2..size];
        if size > 0 { Some(buff_ref) } else { None }
    }

    fn clear(&mut self) {
        self.cursor = 2;
        self.last_frame.take();
    }
}
