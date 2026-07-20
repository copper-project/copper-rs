use super::sample::*;
use zenoh_proto::{CollectionError, keyexpr};

#[derive(Debug)]
pub enum GetResponse<'a> {
    Ok(Sample<'a>),
    Err(Sample<'a>),
}

impl<'a> GetResponse<'a> {
    pub fn ok(ke: &'a keyexpr, payload: &'a [u8]) -> Self {
        Self::Ok(Sample::new(ke, payload))
    }

    pub fn err(ke: &'a keyexpr, payload: &'a [u8]) -> Self {
        Self::Err(Sample::new(ke, payload))
    }
}

#[derive(Debug)]
pub enum FixedCapacityGetResponse<const MAX_KEYEXPR: usize, const MAX_PAYLOAD: usize> {
    Ok(FixedCapacitySample<MAX_KEYEXPR, MAX_PAYLOAD>),
    Err(FixedCapacitySample<MAX_KEYEXPR, MAX_PAYLOAD>),
}

impl<const MAX_KEYEXPR: usize, const MAX_PAYLOAD: usize>
    FixedCapacityGetResponse<MAX_KEYEXPR, MAX_PAYLOAD>
{
    pub fn as_ref(&self) -> GetResponse<'_> {
        match self {
            Self::Ok(sample) => GetResponse::Ok(sample.as_ref()),
            Self::Err(sample) => GetResponse::Err(sample.as_ref()),
        }
    }
}

impl<const MAX_KEYEXPR: usize, const MAX_PAYLOAD: usize> TryFrom<&GetResponse<'_>>
    for FixedCapacityGetResponse<MAX_KEYEXPR, MAX_PAYLOAD>
{
    type Error = CollectionError;

    fn try_from(value: &GetResponse<'_>) -> Result<Self, Self::Error> {
        match value {
            GetResponse::Ok(sample) => Ok(Self::Ok(sample.try_into()?)),
            GetResponse::Err(sample) => Ok(Self::Err(sample.try_into()?)),
        }
    }
}

#[cfg(feature = "alloc")]
#[derive(Debug)]
pub enum AllocGetResponse {
    Ok(AllocSample),
    Err(AllocSample),
}

#[cfg(feature = "alloc")]
impl AllocGetResponse {
    pub fn as_ref(&self) -> GetResponse<'_> {
        match self {
            Self::Ok(sample) => GetResponse::Ok(sample.as_ref()),
            Self::Err(sample) => GetResponse::Err(sample.as_ref()),
        }
    }
}

#[cfg(feature = "alloc")]
impl TryFrom<&GetResponse<'_>> for AllocGetResponse {
    type Error = CollectionError;

    fn try_from(value: &GetResponse<'_>) -> Result<Self, Self::Error> {
        match value {
            GetResponse::Ok(sample) => Ok(Self::Ok(sample.try_into()?)),
            GetResponse::Err(sample) => Ok(Self::Err(sample.try_into()?)),
        }
    }
}
