#![cfg(all(feature = "derive", feature = "std"))]

extern crate std;

extern crate cu_bincode as bincode;
use bincode::{Decode, Encode};
use std::borrow::Cow;

#[derive(Clone, Encode, Decode)]
pub struct Foo<'a>(Cow<'a, str>);
