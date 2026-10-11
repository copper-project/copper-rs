#![cfg(feature = "derive")]

extern crate cu_bincode as bincode;

#[derive(bincode::Encode, bincode::Decode)]
pub struct Eg<D, E> {
    data: (D, E),
}
