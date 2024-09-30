use std::path::Path;

use cu29_traits::{CuError, CuResult};
use rkv::backend::Lmdb;
use rkv::{Rkv, StoreOptions};

use byteorder::{ByteOrder, LittleEndian};

/// Rebuild the interned string index in memory.
pub fn read_interned_strings(index: &Path) -> CuResult<Vec<String>> {
    let mut all_strings = Vec::<String>::new();
    let env = Rkv::new::<Lmdb>(index).map_err(|e| {
        CuError::from("Could not open the string index. Check the path.")
            .add_cause(e.to_string().as_str())
    })?;

    let index_to_string = env
        .open_single("index_to_string", StoreOptions::default())
        .map_err(|e| CuError::new_with_cause("Could not open the index_to_string store", e))?;
    let db_reader = env.read().unwrap();
    let ri = index_to_string.iter_start(&db_reader);
    let mut i = ri.expect("Failed to start iterator");
    while let Some(Ok(v)) = i.next() {
        let (k, v) = v;
        let index = LittleEndian::read_u32(k) as usize;

        if let rkv::Value::Str(s) = v {
            if all_strings.len() <= index {
                all_strings.resize(index + 1, String::new());
            }

            all_strings[index] = s.to_string();
        }
    }
    Ok(all_strings)
}
