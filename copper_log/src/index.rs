use lazy_static::lazy_static;
use rkv::backend::{Lmdb, LmdbDatabase};
use rkv::backend::{LmdbEnvironment, LmdbRwTransaction};
use rkv::{MultiStore, Rkv, SingleStore, StoreOptions, Value, Writer};
use std::fs;
use std::path::Path;
use std::sync::Mutex;

type SStore = SingleStore<LmdbDatabase>;
type MStore = MultiStore<LmdbDatabase>;
type IndexType = u32;

lazy_static! {
    static ref RKV: Mutex<Rkv<LmdbEnvironment>> = {
        let outdir = std::env::var("OUT_DIR").expect("no OUT_DIR set, build.rs must be broken");
        let path = Path::new(&outdir).join("copper_log_index");
        if !path.exists() {
            fs::create_dir_all(&path).unwrap();
        }
        println!("CLog: Storing log index at: {:?}", path);
        let env = Rkv::new::<Lmdb>(&path).unwrap();
        Mutex::new(env)
    };
    static ref DBS: Mutex<(SStore, SStore, SStore, MStore)> = {
        let env = RKV.lock().unwrap();
        let counter = env.open_single("counter", StoreOptions::create()).unwrap();
        let index_to_string = env
            .open_single("index_to_string", StoreOptions::create())
            .unwrap();
        let string_to_index = env
            .open_single("string_to_index", StoreOptions::create())
            .unwrap();
        let index_to_callsites = env
            .open_multi("index_to_callsites", StoreOptions::create())
            .unwrap();

        Mutex::new((
            counter,
            index_to_string,
            string_to_index,
            index_to_callsites,
        ))
    };
}

pub fn check_and_insert(filename: &str, line_number: u32, log_string: &str) -> Option<IndexType> {
    let (counter_store, index_to_string, string_to_index, index_to_callsite) =
        &mut *DBS.lock().unwrap();
    let index = {
        let env = RKV.lock().unwrap();
        // If this string already exists in the store, return the index
        {
            let reader = env.read().unwrap();
            let entry = string_to_index.get(&reader, log_string);
            // check if log_string is already in the string_to_index store
            if let Ok(Some(Value::U64(index))) = string_to_index.get(&reader, log_string) {
                println!(
                    "CLog: Returning existing index #{} -> {}",
                    index, log_string
                );
                return Some(index as IndexType);
            };
        }
        let mut writer = env.write().unwrap();
        let next_index = get_next_index(&mut writer, counter_store).unwrap();
        // Insert the new string into the store
        index_to_string
            .put(
                &mut writer,
                next_index.to_le_bytes(),
                &Value::Str(log_string),
            )
            .unwrap();
        string_to_index
            .put(&mut writer, log_string, &Value::U64(next_index as u64))
            .unwrap();
        index_to_callsite
            .put(
                &mut writer,
                next_index.to_le_bytes(),
                &Value::Str(format!("{}:{}", filename, line_number).as_str()),
            )
            .unwrap();
        writer.commit().unwrap();
        Some(next_index)
    };
    println!("CLog: Inserted #{} -> {}", index.unwrap(), log_string);
    index
}

const COUNTER_KEY: &str = "__counter__";
fn get_next_index(
    writer: &mut Writer<LmdbRwTransaction>,
    counter_store: &SStore,
) -> Result<IndexType, Box<dyn std::error::Error>> {
    let current_counter = match counter_store.get(writer, COUNTER_KEY)? {
        Some(Value::U64(value)) => value as IndexType,
        _ => 0,
    };

    let next_counter = current_counter + 1;
    counter_store.put(writer, COUNTER_KEY, &Value::U64(next_counter as u64))?;
    Ok(next_counter)
}
