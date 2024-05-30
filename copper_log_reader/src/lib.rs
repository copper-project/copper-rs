use bincode::config::standard;
use bincode::decode_from_std_read;
use byteorder::{ByteOrder, LittleEndian};
use copper_log_runtime::CuLogEntry;
use rkv::backend::Lmdb;
use rkv::{Rkv, StoreOptions, Writer};
use std::collections::HashMap;
use std::io::Read;
use std::path::Path;
use strfmt::strfmt;

pub fn extract_copper_log_with_index(mut src: impl Read, index: &Path) {
    let mut all_strings = Vec::<String>::new();
    let env = Rkv::new::<Lmdb>(&index).unwrap();
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
    loop {
        let entry = decode_from_std_read::<CuLogEntry, _, _>(&mut src, standard());
        if entry.is_err() {
            break;
        }
        let entry = entry.unwrap();
        let mut format_string = all_strings[entry.msg_index as usize].clone();
        let mut vars = HashMap::new();

        for (i, param) in entry.params.iter().enumerate() {
            let param_as_string = format!("{}", param);
            if entry.paramname_indexes[i] == 0 {
                // Anonymous parameter
                format_string = format_string.replacen("{}", &param_as_string, 1);
            } else {
                // Named parameter
                let name = all_strings[entry.paramname_indexes[i] as usize].clone();
                vars.insert(name, param_as_string);
            }
        }

        // Use strfmt to replace named parameters
        let result = strfmt(&format_string, &vars).unwrap();
        println!("Copper: {}", result);
    }
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
        extract_copper_log_with_index(reader, Path::new("test/copper_log_index"));
    }
}
