#![cfg(all(feature = "derive", feature = "std"))]

extern crate cu_bincode as bincode;
use bincode::{Decode, Encode};

#[derive(Encode, Decode)]
struct Foo<Bar = ()> {
    x: Bar,
}
