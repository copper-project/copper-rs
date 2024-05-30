use bincode::config::standard;
use bincode::decode_from_std_read;
use byteorder::{ByteOrder, LittleEndian};
use copper_log_runtime::CuLogEntry;
use rkv::backend::Lmdb;
use rkv::{Rkv, StoreOptions, Writer};
use std::io::Read;
use std::path::Path;

pub fn extract_copper_log_index(mut src: impl Read, index: &Path) {
    let mut all_strings = Vec::<String>::new();
    let env = Rkv::new::<Lmdb>(&index).unwrap();
    let index_to_string = env
        .open_single("index_to_string", StoreOptions::default())
        .expect("Failed to open index_to_string store");
    let db_eader = env.read().unwrap();
    let ri = index_to_string.iter_start(&db_eader);
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
    let entry = decode_from_std_read::<CuLogEntry, _, _>(&mut src, standard());
    let entry = entry.expect("Failed to decode CuLogEntry");
    println!(
        "Entry: {} -> {} with params {:?}",
        entry.msg_index, all_strings[entry.msg_index as usize], entry.params
    );
    //FIXME: Entry: 1 -> Just a string {} with params [I16(61)]
    compile_error!("ici");
}

#[cfg(test)]
mod tests {

    use super::*;
    use std::io::Cursor;
    /*
        Just a string {} CuLogEntry { msg_index: 1, paramname_indexes: [0], params: [String("zarma")] }
        anonymous param constants {} {} CuLogEntry { msg_index: 2, paramname_indexes: [0, 0], params: [U16(42), U8(43)] }
        named param constants {} {} CuLogEntry { msg_index: 3, paramname_indexes: [4, 5], params: [I32(3), I32(2)] }
        mixed named param constants, {} {} {} CuLogEntry { msg_index: 6, paramname_indexes: [0, 4, 5], params: [I32(54), I32(3), I32(2)] }
        complex tuple CuLogEntry { msg_index: 7, paramname_indexes: [0], params: [Seq([I32(1), String("toto"), F64(3.34), Bool(true), Char('a')])] }
        Struct CuLogEntry { msg_index: 8, paramname_indexes: [0], params: [Map({String("a"): I32(3), String("b"): I32(4)})] }
        Allocations: +{}B -{}B CuLogEntry { msg_index: 1, paramname_indexes: [2, 3], params: [U64(1003932), U64(1002876)] }
         AFTER CLOSE {}  CuLogEntry { msg_index: 10, paramname_indexes: [0], params: [String("AFTER CLOSE")] }
    */

    // const stored_log: &[u8] = include_bytes!("../test/teststructlog.copper");

    #[test]
    fn test_extract_copper_log() {
        let hex_string = "01 01 00 01 05 7A 61 72 6D 61 02 02 00 00 02 2A 2B 03 02 04 05 02 06 04 06 03 00 04 05 03 6C 06 04 07 01 00 01 05 02 04 74 6F 74 6F B8 1E 85 EB 51 B8 0A 40 01 61 08 01 00 01 02 01 61 06 01 62 08 01 02 02 03 02 FC 9C 51 0F 00 FC 7C 4D 0F 00 0A 01 00 01 0B 41 46 54 45 52 20 43 4C 4F 53 45 43 4C 4F 53 45 00 00 00 00";
        let bytes: Vec<u8> = hex_string
            .split_whitespace()
            .map(|s| u8::from_str_radix(s, 16).expect("Parse error"))
            .collect();
        let mut reader = Cursor::new(bytes.as_slice());
        extract_copper_log_index(reader, Path::new("test/copper_log_index"));
    }
}
