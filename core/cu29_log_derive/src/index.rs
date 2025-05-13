use cu29_log::default_log_index_dir;
use rkv::backend::{Lmdb, LmdbDatabase};
use rkv::backend::{LmdbEnvironment, LmdbRwTransaction};
use rkv::{MultiStore, Rkv, SingleStore, StoreOptions, Value, Writer};
use std::fs;
use std::sync::Mutex;
use std::sync::OnceLock;

type SStore = SingleStore<LmdbDatabase>;
type MStore = MultiStore<LmdbDatabase>;
type IndexType = u32;

#[cfg(feature = "macro_debug")]
const COLORED_PREFIX_BUILD_LOG: &str = "\x1b[32mCLog:\x1b[0m";

#[cfg(feature = "macro_debug")]
macro_rules! build_log {
    ($($arg:tt)*) => {
        eprintln!("{} {}", COLORED_PREFIX_BUILD_LOG, format!($($arg)*));
    };
}

static RKV: OnceLock<Mutex<Rkv<LmdbEnvironment>>> = OnceLock::new();
static DBS: OnceLock<Mutex<(SStore, SStore, SStore, MStore)>> = OnceLock::new();

fn rkv() -> &'static Mutex<Rkv<LmdbEnvironment>> {
    RKV.get_or_init(|| {
        let target_dir = default_log_index_dir();

        // Should never happen I believe.
        if !target_dir.exists() {
            fs::create_dir_all(&target_dir).unwrap();
        }

        #[cfg(feature = "macro_debug")]
        {
            build_log!(
                "=================================================================================="
            );
            build_log!("Interned strings are stored in: {:?}", target_dir);
            build_log!("   [r] is reused index and [n] is new index.");
            build_log!(
                "=================================================================================="
            );
        }

        let env = Rkv::new::<Lmdb>(&target_dir).unwrap();
        Mutex::new(env)
    })
}

fn dbs() -> &'static Mutex<(SStore, SStore, SStore, MStore)> {
    DBS.get_or_init(|| {
        let env = rkv().lock().unwrap();

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
    })
}

pub fn intern_string(s: &str) -> Option<IndexType> {
    let (counter_store, index_to_string, string_to_index, _) = &mut *dbs().lock().unwrap();
    let index = {
        let env = rkv().lock().unwrap();
        // If this string already exists in the store, return the index
        {
            let reader = env.read().unwrap();
            // check if log_string is already in the string_to_index store
            if let Ok(Some(Value::U64(index))) = string_to_index.get(&reader, s) {
                #[cfg(feature = "macro_debug")]
                {
                    build_log!("#{:0>3} [r] -> {}.", index, s);
                }
                return Some(index as IndexType);
            };
        }
        let mut writer = env.write().unwrap();
        let next_index = get_next_index(&mut writer, counter_store).unwrap();
        // Insert the new string into the store
        index_to_string
            .put(&mut writer, next_index.to_le_bytes(), &Value::Str(s))
            .unwrap();
        string_to_index
            .put(&mut writer, s, &Value::U64(next_index as u64))
            .unwrap();
        writer.commit().unwrap();
        Some(next_index)
    };
    #[cfg(feature = "macro_debug")]
    {
        build_log!("#{:0>3} [n] -> {}.", index.unwrap(), s);
    }
    index
}

#[allow(dead_code)]
pub fn record_callsite(filename: &str, line_number: u32) -> Option<IndexType> {
    intern_string(format!("{filename}:{line_number}").as_str())
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
