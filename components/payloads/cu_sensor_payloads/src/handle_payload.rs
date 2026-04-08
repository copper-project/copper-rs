use alloc::boxed::Box;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use core::fmt::Debug;
use core::marker::PhantomData;
use cu29::prelude::*;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

pub trait CuHandlePayloadMeta {
    const TYPE_PATH: &'static str;
    const SHORT_TYPE_PATH: &'static str;
    const TYPE_IDENT: Option<&'static str>;
    const CRATE_NAME: Option<&'static str>;
    const MODULE_PATH: Option<&'static str>;
}

pub trait CuHandlePayloadInit: Debug + Send + Sync + 'static {
    fn boxed_init() -> Box<Self>;
}

#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    #[reflect(ignore)]
    pub handle: CuHandle<T>,
    #[reflect(ignore)]
    _meta: PhantomData<M>,
}

impl<T, M> Debug for CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CuHandlePayload")
            .field("handle", &self.handle)
            .finish()
    }
}

impl<T, M> Clone for CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    fn clone(&self) -> Self {
        Self {
            handle: self.handle.clone(),
            _meta: PhantomData,
        }
    }
}

impl<T, M> Default for CuHandlePayload<T, M>
where
    T: CuHandlePayloadInit,
    M: CuHandlePayloadMeta + 'static,
{
    fn default() -> Self {
        Self::from_box(T::boxed_init())
    }
}

impl<T, M> TypePath for CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    fn type_path() -> &'static str {
        M::TYPE_PATH
    }

    fn short_type_path() -> &'static str {
        M::SHORT_TYPE_PATH
    }

    fn type_ident() -> Option<&'static str> {
        M::TYPE_IDENT
    }

    fn crate_name() -> Option<&'static str> {
        M::CRATE_NAME
    }

    fn module_path() -> Option<&'static str> {
        M::MODULE_PATH
    }
}

impl<T, M> CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    pub fn from_handle(handle: CuHandle<T>) -> Self {
        Self {
            handle,
            _meta: PhantomData,
        }
    }

    pub fn new_detached(inner: T) -> Self {
        Self::from_handle(CuHandle::new_detached(inner))
    }

    pub fn from_box(inner: Box<T>) -> Self {
        Self::from_handle(CuHandle::new_detached_box(inner))
    }

    pub fn with_inner<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        self.handle.with_inner(|inner| f(inner.as_ref()))
    }

    pub fn with_inner_mut<R>(&mut self, f: impl FnOnce(&mut T) -> R) -> R {
        self.handle.with_inner_mut(|inner| f(inner.as_mut()))
    }
}

impl<T, M> Serialize for CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + Serialize + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.with_inner(|inner| inner.serialize(serializer))
    }
}

impl<'de, T, M> Deserialize<'de> for CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + Deserialize<'de> + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let inner = Box::<T>::deserialize(deserializer)?;
        Ok(Self::from_box(inner))
    }
}

impl<T, M> Encode for CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + Encode + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.with_inner(|inner| inner.encode(encoder))
    }
}

impl<T, M> Decode<()> for CuHandlePayload<T, M>
where
    T: Debug + Send + Sync + Decode<()> + 'static,
    M: CuHandlePayloadMeta + 'static,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let inner = Box::<T>::decode(decoder)?;
        Ok(Self::from_box(inner))
    }
}
