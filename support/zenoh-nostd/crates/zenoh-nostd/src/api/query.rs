use core::str::FromStr;

use zenoh_proto::{CollectionError, SessionError, keyexpr};

use crate::{api::session::Session, config::ZSessionConfig};

pub struct QueryableQuery<'a, 'res, Config>
where
    Config: ZSessionConfig,
{
    session: &'a Session<'res, Config>,
    rid: u32,
    ke: &'a keyexpr,
    parameters: Option<&'a str>,
    payload: Option<&'a [u8]>,
    finalized: bool,
}

impl<'a, 'res, Config> QueryableQuery<'a, 'res, Config>
where
    Config: ZSessionConfig,
{
    pub(crate) fn new(
        session: &'a Session<'res, Config>,
        rid: u32,
        ke: &'a keyexpr,
        parameters: Option<&'a str>,
        payload: Option<&'a [u8]>,
    ) -> Self {
        Self {
            session,
            rid,
            ke,
            parameters,
            payload,
            finalized: false,
        }
    }

    pub fn keyexpr(&self) -> &keyexpr {
        self.ke
    }

    pub fn parameters(&self) -> Option<&str> {
        self.parameters
    }

    pub fn payload(&self) -> Option<&[u8]> {
        self.payload
    }

    pub async fn reply(
        &self,
        ke: &keyexpr,
        payload: &[u8],
    ) -> core::result::Result<(), SessionError> {
        self.session.reply(self.rid, ke, payload).await
    }

    pub async fn err(
        &self,
        ke: &keyexpr,
        payload: &[u8],
    ) -> core::result::Result<(), SessionError> {
        self.session.err(self.rid, ke, payload).await
    }

    pub async fn finalize(&mut self) -> core::result::Result<(), SessionError> {
        if !self.finalized {
            self.session.finalize(self.rid).await?;
            self.finalized = true
        }

        Ok(())
    }
}

pub struct FixedCapacityQueryableQuery<
    Config,
    const MAX_KEYEXPR: usize,
    const MAX_PARAMETERS: usize,
    const MAX_PAYLOAD: usize,
> where
    Config: ZSessionConfig + 'static,
{
    session: &'static Session<'static, Config>,
    rid: u32,
    ke: heapless::String<MAX_KEYEXPR>,
    parameters: Option<heapless::String<MAX_PARAMETERS>>,
    payload: Option<heapless::Vec<u8, MAX_PAYLOAD>>,
    finalized: bool,
}

impl<Config, const MAX_KEYEXPR: usize, const MAX_PARAMETERS: usize, const MAX_PAYLOAD: usize>
    FixedCapacityQueryableQuery<Config, MAX_KEYEXPR, MAX_PARAMETERS, MAX_PAYLOAD>
where
    Config: ZSessionConfig,
{
    pub fn keyexpr(&self) -> &keyexpr {
        keyexpr::from_str_unchecked(self.ke.as_str())
    }

    pub fn parameters(&self) -> Option<&str> {
        self.parameters.as_ref().map(|p| p.as_str())
    }

    pub fn payload(&self) -> Option<&[u8]> {
        self.payload.as_ref().map(|p| p.as_slice())
    }

    pub async fn reply(
        &self,
        ke: &keyexpr,
        payload: &[u8],
    ) -> core::result::Result<(), SessionError> {
        self.session.reply(self.rid, ke, payload).await
    }

    pub async fn err(
        &self,
        ke: &keyexpr,
        payload: &[u8],
    ) -> core::result::Result<(), SessionError> {
        self.session.err(self.rid, ke, payload).await
    }

    pub async fn finalize(&mut self) -> core::result::Result<(), SessionError> {
        if !self.finalized {
            self.session.finalize(self.rid).await?;
            self.finalized = true
        }

        Ok(())
    }
}

impl<'a, Config, const MAX_KEYEXPR: usize, const MAX_PARAMETERS: usize, const MAX_PAYLOAD: usize>
    TryFrom<(
        &QueryableQuery<'a, 'static, Config>,
        &'static Session<'static, Config>,
    )> for FixedCapacityQueryableQuery<Config, MAX_KEYEXPR, MAX_PARAMETERS, MAX_PAYLOAD>
where
    Config: ZSessionConfig,
{
    type Error = CollectionError;

    fn try_from(
        value: (
            &QueryableQuery<'a, 'static, Config>,
            &'static Session<'static, Config>,
        ),
    ) -> Result<Self, Self::Error> {
        let (value, session) = value;

        Ok(Self {
            session,
            rid: value.rid,
            ke: heapless::String::from_str(value.keyexpr().as_str())
                .map_err(|_| CollectionError::CollectionTooSmall)?,
            parameters: value
                .parameters
                .map(heapless::String::from_str)
                .transpose()
                .map_err(|_| CollectionError::CollectionTooSmall)?,
            payload: value
                .payload
                .map(heapless::Vec::from_slice)
                .transpose()
                .map_err(|_| CollectionError::CollectionTooSmall)?,
            finalized: value.finalized,
        })
    }
}

#[cfg(feature = "alloc")]
pub struct AllocQueryableQuery<Config>
where
    Config: ZSessionConfig + 'static,
{
    session: &'static Session<'static, Config>,
    rid: u32,
    ke: alloc::string::String,
    parameters: Option<alloc::string::String>,
    payload: Option<alloc::vec::Vec<u8>>,
    finalized: bool,
}

#[cfg(feature = "alloc")]
impl<Config> AllocQueryableQuery<Config>
where
    Config: ZSessionConfig,
{
    pub fn keyexpr(&self) -> &keyexpr {
        keyexpr::from_str_unchecked(self.ke.as_str())
    }

    pub fn parameters(&self) -> Option<&str> {
        self.parameters.as_deref()
    }

    pub fn payload(&self) -> Option<&[u8]> {
        self.payload.as_deref()
    }

    pub async fn reply(
        &self,
        ke: &keyexpr,
        payload: &[u8],
    ) -> core::result::Result<(), SessionError> {
        self.session.reply(self.rid, ke, payload).await
    }

    pub async fn err(
        &self,
        ke: &keyexpr,
        payload: &[u8],
    ) -> core::result::Result<(), SessionError> {
        self.session.err(self.rid, ke, payload).await
    }

    pub async fn finalize(&mut self) -> core::result::Result<(), SessionError> {
        if !self.finalized {
            self.session.finalize(self.rid).await?;
            self.finalized = true
        }

        Ok(())
    }
}

#[cfg(feature = "alloc")]
impl<'a, Config>
    TryFrom<(
        &QueryableQuery<'a, 'static, Config>,
        &'static Session<'static, Config>,
    )> for AllocQueryableQuery<Config>
where
    Config: ZSessionConfig,
{
    type Error = CollectionError;

    fn try_from(
        value: (
            &QueryableQuery<'a, 'static, Config>,
            &'static Session<'static, Config>,
        ),
    ) -> Result<Self, Self::Error> {
        let (value, session) = value;

        Ok(Self {
            session,
            rid: value.rid,
            ke: alloc::string::String::from(value.keyexpr().as_str()),
            parameters: value.parameters.map(alloc::string::String::from),
            payload: value.payload.map(alloc::vec::Vec::from),
            finalized: value.finalized,
        })
    }
}
