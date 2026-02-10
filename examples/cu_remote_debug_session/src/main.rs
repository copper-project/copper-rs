use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::bincode::{Decode, Encode};
use cu29::prelude::CopperList;
use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::{
    RemoteDebugPaths, RemoteDebugZenohClient, RemoteDebugZenohServer, SessionOpenParams, WireCodec,
};
use cu29_helpers::basic_copper_setup;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::fs;
use std::path::Path;
use std::thread;
use std::time::Duration;

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct CounterMsg {
    pub value: u32,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct AccumMsg {
    pub sum: u32,
}

#[derive(Reflect)]
pub struct CounterSrc {
    pub next: u32,
}
impl Freezable for CounterSrc {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}
impl CuSrcTask for CounterSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CounterMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { next: 0 })
    }

    fn process(&mut self, _clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.next += 1;
        output.set_payload(CounterMsg { value: self.next });
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Accumulator {
    pub sum: u32,
}
impl Freezable for Accumulator {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.sum, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.sum = Decode::decode(decoder)?;
        Ok(())
    }
}
impl CuTask for Accumulator {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CounterMsg);
    type Output<'m> = output_msg!(AccumMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { sum: 0 })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(msg) = input.payload() {
            self.sum += msg.value;
            output.set_payload(AccumMsg { sum: self.sum });
        } else {
            output.clear_payload();
        }
        Ok(())
    }
}

#[derive(Reflect)]
pub struct SpySink {
    pub last: Option<u32>,
}
impl Freezable for SpySink {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.last, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.last = Decode::decode(decoder)?;
        Ok(())
    }
}
impl CuSinkTask for SpySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(AccumMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { last: None })
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        self.last = input.payload().map(|p| p.sum);
        Ok(())
    }
}

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct DebugApp {}

const LOG_PATH: &str = "logs/cu_remote_debug_session.copper";
const REPLAY_LOG_PATH: &str = "logs/cu_remote_debug_session_replay.copper";
const LOG_SLAB_SIZE: Option<usize> = Some(256 * 1024 * 1024);
const REPLAY_LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);

fn clean_logs() -> CuResult<()> {
    if let Ok(entries) = fs::read_dir("logs") {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|s| s.to_str())
                && name.starts_with("cu_remote_debug_session")
            {
                let _ = fs::remove_file(&path);
            }
        }
    }
    Ok(())
}

fn record_log() -> CuResult<()> {
    println!("[record] Cleaning existing session logs");
    clean_logs()?;
    println!("[record] Creating mock clock and logger context");
    let (clock, clock_mock) = RobotClock::mock();
    let ctx = basic_copper_setup(
        Path::new(LOG_PATH),
        LOG_SLAB_SIZE,
        /*text_log=*/ false,
        Some(clock.clone()),
    )?;

    let mut sim_cb = |_step: default::SimStep| SimOverride::ExecuteByRuntime;

    println!("[record] Building debug app");
    let mut app = DebugAppBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    println!("[record] Starting tasks");
    app.start_all_tasks(&mut sim_cb)?;
    for i in 0..128u32 {
        clock_mock.increment(CuDuration::from_millis(10));
        app.run_one_iteration(&mut sim_cb)?;
        if (i + 1) % 32 == 0 {
            println!("[record] Completed {}/128 iterations", i + 1);
        }
    }
    println!("[record] Stopping tasks");
    app.stop_all_tasks(&mut sim_cb)?;
    println!("[record] Log recording complete");
    Ok(())
}

fn time_of(cl: &CopperList<default::CuStampedDataSet>) -> Option<CuTime> {
    Option::<CuTime>::from(cl.msgs.get_src_output().metadata.process_time.start)
}

fn build_cb<'a>(
    cl: &'a CopperList<default::CuStampedDataSet>,
    _clock_for_cb: RobotClock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
    let src_out = cl.msgs.get_src_output().clone();
    Box::new(move |step: default::SimStep<'_>| match step {
        default::SimStep::Src(CuTaskCallbackState::Process(_input, output)) => {
            *output = src_out.clone();
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    })
}

fn app_factory(_params: &SessionOpenParams) -> CuResult<(DebugApp, RobotClock, RobotClockMock)> {
    println!("[server] Building replay app instance");
    let (clock, clock_mock) = RobotClock::mock();
    let ctx = basic_copper_setup(
        Path::new(REPLAY_LOG_PATH),
        REPLAY_LOG_SLAB_SIZE,
        /*text_log=*/ false,
        Some(clock.clone()),
    )?;

    let mut sim_cb = |_step: default::SimStep| SimOverride::ExecuteByRuntime;
    let app = DebugAppBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    Ok((app, clock, clock_mock))
}

fn call_ok(
    client: &RemoteDebugZenohClient,
    session_id: Option<&str>,
    method: &str,
    params: Value,
) -> CuResult<Value> {
    let response = client.call(session_id, method, params)?;
    if !response.ok {
        let msg = response
            .error
            .map(|e| format!("{}: {}", e.code, e.message))
            .unwrap_or_else(|| "unknown remote error".to_string());
        return Err(CuError::from(format!("RPC {method} failed: {msg}")));
    }
    Ok(response.result.unwrap_or(Value::Null))
}

fn call_step(
    client: &RemoteDebugZenohClient,
    session_id: Option<&str>,
    method: &str,
    params: Value,
) -> CuResult<Value> {
    match session_id {
        Some(id) => println!("[rpc] -> {method} (session: {id})"),
        None => println!("[rpc] -> {method}"),
    }
    let result = call_ok(client, session_id, method, params)?;
    println!("[rpc] <- {method} ok");
    Ok(result)
}

fn run_remote_debug_session() -> CuResult<()> {
    println!("[session] Preparing remote debug paths");
    let paths = RemoteDebugPaths::new("copper/examples/cu_remote_debug_session/debug/v1");

    let server_paths = paths.clone();
    let server_handle = thread::spawn(move || -> CuResult<()> {
        println!("[server] Starting RemoteDebugZenohServer");
        let mut server = RemoteDebugZenohServer::<
            DebugApp,
            default::CuStampedDataSet,
            _,
            _,
            MmapSectionStorage,
            MmapUnifiedLoggerWrite,
            _,
        >::new(server_paths, app_factory, build_cb, time_of)?;
        println!("[server] Serving until admin.stop");
        let result = server.serve_until_stopped();
        println!("[server] Server loop exited");
        result
    });

    println!("[session] Waiting for server startup");
    thread::sleep(Duration::from_millis(300));

    println!("[session] Creating remote debug client");
    let client = RemoteDebugZenohClient::builder(paths.clone(), "demo_client")
        .codec(WireCodec::Cbor)
        .build()?;

    let open = call_step(
        &client,
        None,
        "session.open",
        json!({
            "log_base": LOG_PATH,
            "cache_cap": 8,
            "role": "controller",
            "codecs": ["cbor", "json"],
        }),
    )?;
    let session_id = open
        .get("session_id")
        .and_then(Value::as_str)
        .ok_or_else(|| CuError::from("missing session_id"))?
        .to_string();
    println!("[session] Opened session_id={session_id}");

    let _ = call_step(
        &client,
        Some(&session_id),
        "session.capabilities",
        json!({}),
    )?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "nav.seek",
        json!({ "target": {"kind": "cl", "cl": 7}, "resolve": "exact" }),
    )?;

    let _ = call_step(&client, Some(&session_id), "timeline.get_cursor", json!({}))?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "timeline.get_cl",
        json!({
            "include_payloads": true,
            "include_metadata": true,
            "include_raw": true,
        }),
    )?;
    let ts_list = call_step(
        &client,
        Some(&session_id),
        "timeline.list",
        json!({
            "from": {"kind": "cl", "cl": 4},
            "to": {"kind": "cl", "cl": 4},
        }),
    )?;
    let ts_ns = ts_list
        .get("items")
        .and_then(Value::as_array)
        .and_then(|items| items.first())
        .and_then(|item| item.get("ts_ns"))
        .and_then(Value::as_u64)
        .ok_or_else(|| CuError::from("missing timestamp in timeline.list"))?;
    println!("[session] Using timestamp ts_ns={ts_ns} from timeline.list");

    let _ = call_step(
        &client,
        Some(&session_id),
        "timeline.get_cl",
        json!({
            "at": {
                "target": {"kind": "ts", "ts_ns": ts_ns},
                "mutate_cursor": false,
                "resolve": "at_or_after"
            },
            "include_payloads": true,
            "include_metadata": false,
            "include_raw": false,
        }),
    )?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "nav.seek",
        json!({ "target": {"kind": "ts", "ts_ns": ts_ns}, "resolve": "at_or_after" }),
    )?;

    let run_until = call_step(
        &client,
        Some(&session_id),
        "nav.run_until",
        json!({
            "target": {"kind": "cl", "cl": 20},
            "resolve": "exact",
            "max_steps": 1024,
            "timeout_ms": 1000,
            "progress_every_n_steps": 4,
        }),
    )?;
    let op_id = run_until
        .get("op_id")
        .and_then(Value::as_str)
        .ok_or_else(|| CuError::from("missing op_id from nav.run_until"))?
        .to_string();
    println!("[session] Received op_id={op_id} from nav.run_until");

    let _ = call_step(
        &client,
        Some(&session_id),
        "session.cancel",
        json!({"op_id": op_id}),
    )?;

    let _ = call_step(&client, Some(&session_id), "nav.step", json!({"delta": -1}))?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "nav.replay",
        json!({
            "at": {
                "target": {"kind": "cl", "cl": 6},
                "mutate_cursor": true,
                "resolve": "exact"
            }
        }),
    )?;

    let _ = call_step(&client, Some(&session_id), "schema.get_stack", json!({}))?;

    let types = call_step(
        &client,
        Some(&session_id),
        "schema.list_types",
        json!({"filter": ""}),
    )?;
    let first_type = types
        .get("type_paths")
        .and_then(Value::as_array)
        .and_then(|arr| arr.first())
        .and_then(Value::as_str)
        .ok_or_else(|| CuError::from("schema.list_types returned no type"))?
        .to_string();
    println!("[session] First type from schema.list_types: {first_type}");

    let _ = call_step(
        &client,
        Some(&session_id),
        "schema.get_type",
        json!({"type_path": first_type, "format": "jsonschema"}),
    )?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "schema.get_payload_map",
        json!({}),
    )?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "state.inspect",
        json!({
            "path": "/tasks/accum",
            "depth": 4,
            "at": {
                "target": {"kind": "cl", "cl": 12},
                "mutate_cursor": false,
                "resolve": "at_or_after"
            }
        }),
    )?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "state.read",
        json!({
            "path": "/tasks/accum",
            "format": "json",
            "at": {
                "target": {"kind": "ts", "ts_ns": ts_ns},
                "mutate_cursor": false,
                "resolve": "at_or_after"
            }
        }),
    )?;

    let _ = call_step(
        &client,
        Some(&session_id),
        "state.search",
        json!({
            "query": "sum",
            "at": {
                "target": {"kind": "cl", "cl": 16},
                "mutate_cursor": false,
                "resolve": "at_or_before"
            }
        }),
    )?;

    let watch_open = call_step(
        &client,
        Some(&session_id),
        "state.watch.open",
        json!({
            "path": "/tasks/accum",
            "mode": "on_cursor_change",
            "at": {
                "target": {"kind": "ts", "ts_ns": ts_ns},
                "mutate_cursor": true,
                "resolve": "at_or_after"
            }
        }),
    )?;
    let watch_id = watch_open
        .get("watch_id")
        .and_then(Value::as_u64)
        .ok_or_else(|| CuError::from("missing watch_id"))?;
    println!("[session] Opened state watch_id={watch_id}");

    let _ = call_step(
        &client,
        Some(&session_id),
        "state.watch.close",
        json!({"watch_id": watch_id}),
    )?;

    let _ = call_step(&client, Some(&session_id), "health.ping", json!({}))?;
    let _ = call_step(&client, Some(&session_id), "health.stats", json!({}))?;

    let _ = call_step(&client, Some(&session_id), "session.close", json!({}))?;

    let _ = call_step(&client, None, "admin.stop", json!({}))?;

    println!("[session] Waiting for server thread to finish");
    let server_result = server_handle
        .join()
        .map_err(|_| CuError::from("remote debug server thread panicked"))?;
    server_result?;
    println!("[session] Server thread finished cleanly");

    println!("Remote debug API demo completed successfully (all endpoints exercised).");
    Ok(())
}

fn main() -> CuResult<()> {
    println!("[main] Recording baseline log");
    record_log()?;
    println!("[main] Running remote debug API session");
    run_remote_debug_session()
}
