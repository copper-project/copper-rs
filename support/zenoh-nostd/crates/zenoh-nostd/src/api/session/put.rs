use zenoh_proto::{exts::*, fields::*, msgs::*, *};

use crate::{api::session::Session, config::ZSessionConfig, io::transport::ZTransportLinkTx};

pub struct PutBuilder<'a, 'res, Config>
where
    Config: ZSessionConfig,
{
    pub(crate) session: &'a Session<'res, Config>,

    pub(crate) ke: &'a keyexpr,
    pub(crate) payload: &'a [u8],

    pub(crate) encoding: Encoding<'a>,
    pub(crate) timestamp: Option<Timestamp>,
    pub(crate) attachment: Option<Attachment<'a>>,
}

impl<'a, 'res, Config> PutBuilder<'a, 'res, Config>
where
    Config: ZSessionConfig,
{
    pub(crate) fn new(
        session: &'a Session<'res, Config>,
        ke: &'a keyexpr,
        payload: &'a [u8],
    ) -> Self {
        Self {
            session,
            ke,
            payload,
            encoding: Encoding::default(),
            timestamp: None,
            attachment: None,
        }
    }

    pub fn payload(mut self, payload: &'a [u8]) -> Self {
        self.payload = payload;
        self
    }

    pub fn encoding(mut self, encoding: Encoding<'a>) -> Self {
        self.encoding = encoding;
        self
    }

    pub fn timestamp(mut self, timestamp: Timestamp) -> Self {
        self.timestamp = Some(timestamp);
        self
    }

    pub fn attachment(mut self, attachment: &'a [u8]) -> Self {
        self.attachment = Some(Attachment { buffer: attachment });
        self
    }

    pub async fn finish(self) -> core::result::Result<(), SessionError> {
        let msg = Push {
            wire_expr: WireExpr::from(self.ke),
            payload: PushBody::Put(Put {
                payload: self.payload,
                encoding: self.encoding,
                timestamp: self.timestamp,
                attachment: self.attachment,
                ..Default::default()
            }),
            timestamp: self.timestamp,
            ..Default::default()
        };

        Ok(self
            .session
            .driver
            .tx()
            .await
            .send(core::iter::once(NetworkMessage {
                reliability: Reliability::default(),
                qos: QoS::default(),
                body: NetworkBody::Push(msg),
            }))
            .await?)
    }
}

impl<'res, Config> Session<'res, Config>
where
    Config: ZSessionConfig,
{
    pub fn put<'a>(&'a self, ke: &'a keyexpr, payload: &'a [u8]) -> PutBuilder<'a, 'res, Config> {
        PutBuilder::new(self, ke, payload)
    }
}
