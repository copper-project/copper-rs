#![cfg(all(feature = "std", feature = "derive"))]

extern crate std;

extern crate cu_bincode as bincode;
use std::collections::BTreeMap;

#[derive(bincode::Encode)]
struct AllTypes(BTreeMap<u8, AllTypes>);

#[test]
fn test_issue_459() {
    let _result = bincode::encode_to_vec(AllTypes(BTreeMap::new()), bincode::config::standard());
}
