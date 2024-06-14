use bincode::config::standard;
use bincode::decode_from_std_read;
use byteorder::{ByteOrder, LittleEndian};
use copper_log::{rebuild_logline, CuLogEntry};
use copper_traits::{CuError, CuResult};
use rkv::backend::Lmdb;
use rkv::{Rkv, StoreOptions};
use std::io::Read;
use std::path::Path;

/// Full dump of the copper structured log from its binary representation.
/// src: the source of the log data
/// index: the path to the index file (containing the interned strings constructed at build time)
pub fn full_log_dump(mut src: impl Read, index: &Path) -> CuResult<()> {
    let all_strings = read_interned_strings(index)?;
    loop {
        let entry = decode_from_std_read::<CuLogEntry, _, _>(&mut src, standard());
        if entry.is_err() {
            break;
        }
        let entry = entry.unwrap();

        let result = rebuild_logline(&all_strings, entry);
        if result.is_err() {
            println!("Failed to rebuild log line: {:?}", result);
            continue;
        }
        println!("Copper: {}", result.unwrap());
    }
    Ok(())
}

/// Rebuild the interned string index in memory.
pub fn read_interned_strings(index: &Path) -> CuResult<Vec<String>> {
    let mut all_strings = Vec::<String>::new();
    let env = Rkv::new::<Lmdb>(index).map_err(|e| {
        CuError::from("Could not open the string index. Check the path.")
            .add_cause(e.to_string().as_str())
    })?;

    let index_to_string = env
        .open_single("index_to_string", StoreOptions::default())
        .expect("Failed to open index_to_string store");
    let db_reader = env.read().unwrap();
    let ri = index_to_string.iter_start(&db_reader);
    let mut i = ri.expect("Failed to start iterator");
    while let Some(Ok(v)) = i.next() {
        let (k, v) = v;
        let index = LittleEndian::read_u32(&k) as usize;

        if let rkv::Value::Str(s) = v {
            if all_strings.len() <= index as usize {
                all_strings.resize(index as usize + 1, String::new());
            }

            all_strings[index] = s.to_string();
            println!("{} -> {}", index, s);
        }
    }
    Ok(all_strings)
}

#[cfg(test)]
mod tests {

    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_extract_copper_log() {
        let hex_string = "01 01 00 01 0C 05 7A 61 72 6D 61";
        let bytes: Vec<u8> = hex_string
            .split_whitespace()
            .map(|s| u8::from_str_radix(s, 16).expect("Parse error"))
            .collect();

        let reader = Cursor::new(bytes.as_slice());
        full_log_dump(reader, Path::new("test/copper_log_index"));
    }
}
