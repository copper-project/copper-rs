#![cfg(feature = "derive")]

extern crate cu_bincode as bincode;
use bincode::{Decode, Encode};

#[derive(Encode, Decode, Clone)]
pub struct A;
#[derive(Encode, Decode, Clone)]
pub struct B<T>
where
    T: Clone + Encode + Decode<()>,
{
    pub t: T,
}

#[derive(Encode, Decode)]
pub struct MyStruct<T>
where
    T: Clone + Encode + Decode<()>,
{
    pub a: A,
    pub b: B<T>,
}
