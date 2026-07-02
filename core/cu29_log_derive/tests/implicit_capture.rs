use cu29_clock::RobotClock;
use cu29_log::{CuLogEntry, CuLogLevel};
use cu29_log_derive::debug;
use cu29_log_runtime::{
    LoggerRuntime, NullLog, log_debug_mode, scoped_live_log_listener, unregister_live_log_listener,
};
use cu29_traits::{CuResult, WriteStream};
use cu29_value::{Value, to_value};
use std::sync::{Arc, Mutex};

type ObservedLog = Arc<Mutex<Option<(CuLogEntry, String, Vec<String>)>>>;

#[derive(Debug)]
struct CaptureLog(Arc<Mutex<Vec<CuLogEntry>>>);

impl WriteStream<CuLogEntry> for CaptureLog {
    fn log(&mut self, obj: &CuLogEntry) -> CuResult<()> {
        self.0.lock().unwrap().push(obj.clone());
        Ok(())
    }
}

#[test]
fn debug_implicitly_captures_named_placeholders() {
    let entries = Arc::new(Mutex::new(Vec::new()));
    let observed: ObservedLog = Arc::new(Mutex::new(None));
    let _runtime = LoggerRuntime::init(
        RobotClock::default(),
        CaptureLog(entries.clone()),
        None::<NullLog>,
    );
    unregister_live_log_listener();

    let observed_clone = observed.clone();
    let _listener = scoped_live_log_listener(move |entry, format_str, param_names| {
        *observed_clone.lock().unwrap() = Some((
            entry.clone(),
            format_str.to_string(),
            param_names
                .iter()
                .map(|name| name.to_string())
                .collect::<Vec<_>>(),
        ));
    });

    let hash = "0x000000000";
    debug!("Hash: {hash}");

    let entries = entries.lock().unwrap();
    assert_eq!(entries.len(), 1);
    assert_eq!(entries[0].level, CuLogLevel::Debug);
    assert_eq!(
        entries[0].params.as_slice(),
        [Value::String("0x000000000".to_string())]
    );

    let (entry, format_str, param_names) = observed.lock().unwrap().clone().unwrap();
    assert_eq!(format_str, "Hash: {hash}");
    assert_eq!(param_names, vec!["hash"]);
    assert_eq!(
        entry.params.as_slice(),
        [Value::String("0x000000000".to_string())]
    );
}
