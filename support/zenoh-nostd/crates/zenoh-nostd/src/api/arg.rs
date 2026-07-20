use core::marker::PhantomData;

use crate::{api::query::QueryableQuery, config::ZSessionConfig};

use super::{response::*, sample::*};

pub trait ZArg {
    type Of<'a>
    where
        Self: 'a;
}

pub struct GetResponseRef;
pub struct SampleRef;
pub struct QueryableQueryRef<'res, Config>(PhantomData<&'res Config>);

impl ZArg for GetResponseRef {
    type Of<'a> = &'a GetResponse<'a>;
}

impl ZArg for SampleRef {
    type Of<'a> = &'a Sample<'a>;
}

impl<'res, Config> ZArg for QueryableQueryRef<'res, Config>
where
    Config: ZSessionConfig,
{
    type Of<'a>
        = &'a QueryableQuery<'a, 'res, Config>
    where
        Self: 'a;
}
