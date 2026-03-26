use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub mod messages {
    use super::*;

    #[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct Ping {
        pub seq: u64,
        pub note: String,
    }

    #[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct Pong {
        pub seq: u64,
        pub reply: String,
    }
}

pub struct DemoRunOptions {
    pub log_path: PathBuf,
    pub instance_id: u32,
    pub iterations: Option<usize>,
}

pub fn default_log_path(file_name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join(file_name)
}

pub fn parse_run_options(default_log_file: &str) -> CuResult<DemoRunOptions> {
    let mut log_path = default_log_path(default_log_file);
    let mut instance_id = 0u32;
    let mut iterations = None;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--log" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --log"))?;
                log_path = PathBuf::from(value);
            }
            "--instance-id" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --instance-id"))?;
                instance_id = value.parse::<u32>().map_err(|e| {
                    CuError::from(format!("invalid --instance-id '{value}'"))
                        .add_cause(e.to_string().as_str())
                })?;
            }
            "--iterations" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --iterations"))?;
                iterations = Some(value.parse::<usize>().map_err(|e| {
                    CuError::from(format!("invalid --iterations '{value}'"))
                        .add_cause(e.to_string().as_str())
                })?);
            }
            other => {
                return Err(CuError::from(format!(
                    "unsupported argument '{other}', expected --log, --instance-id or --iterations"
                )));
            }
        }
    }

    if let Some(parent) = log_path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| {
            CuError::from(format!(
                "failed to create log directory '{}'",
                parent.display()
            ))
            .add_cause(e.to_string().as_str())
        })?;
    }

    Ok(DemoRunOptions {
        log_path,
        instance_id,
        iterations,
    })
}

pub mod bridges {
    use super::messages;
    use cu_zenoh_bridge::ZenohBridge;
    use cu29::prelude::*;

    tx_channels! {
        pub struct DemoTxChannels : DemoTxId {
            ping_bin => messages::Ping = "demo/ping/bin",
            ping_json => messages::Ping = "demo/ping/json",
            ping_cbor => messages::Ping = "demo/ping/cbor",
            pong_bin => messages::Pong = "demo/pong/bin",
            pong_json => messages::Pong = "demo/pong/json",
            pong_cbor => messages::Pong = "demo/pong/cbor",
        }
    }

    rx_channels! {
        pub struct DemoRxChannels : DemoRxId {
            ping_bin => messages::Ping = "demo/ping/bin",
            ping_json => messages::Ping = "demo/ping/json",
            ping_cbor => messages::Ping = "demo/ping/cbor",
            pong_bin => messages::Pong = "demo/pong/bin",
            pong_json => messages::Pong = "demo/pong/json",
            pong_cbor => messages::Pong = "demo/pong/cbor",
        }
    }

    pub type DemoZenohBridge = ZenohBridge<DemoTxChannels, DemoRxChannels>;
}

pub mod tasks {
    use super::messages::{Ping, Pong};
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct PingSource {
        seq: u64,
        label: String,
    }

    impl Freezable for PingSource {}

    impl CuSrcTask for PingSource {
        type Output<'m> = output_msg!(Ping);
        type Resources<'r> = ();

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let label = match config {
                Some(cfg) => cfg
                    .get::<String>("label")?
                    .unwrap_or_else(|| "ping".to_string()),
                None => "ping".to_string(),
            };
            Ok(Self { seq: 0, label })
        }

        fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            output.set_payload(Ping {
                seq: self.seq,
                note: format!("{}#{}", self.label, self.seq),
            });
            output.tov = Tov::Time(ctx.now());
            self.seq += 1;
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct PongResponder {
        label: String,
    }

    impl Freezable for PongResponder {}

    impl CuTask for PongResponder {
        type Input<'m> = input_msg!(Ping);
        type Output<'m> = output_msg!(Pong);
        type Resources<'r> = ();

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let label = match config {
                Some(cfg) => cfg
                    .get::<String>("label")?
                    .unwrap_or_else(|| "pong".to_string()),
                None => "pong".to_string(),
            };
            Ok(Self { label })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            if let Some(ping) = input.payload() {
                output.set_payload(Pong {
                    seq: ping.seq,
                    reply: format!("{} -> {}", self.label, ping.note),
                });
                output.tov = Tov::Time(ctx.now());
            } else {
                output.clear_payload();
            }
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct PongSink {
        label: String,
    }

    impl Freezable for PongSink {}

    impl CuSinkTask for PongSink {
        type Input<'m> = input_msg!(Pong);
        type Resources<'r> = ();

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let label = match config {
                Some(cfg) => cfg
                    .get::<String>("label")?
                    .unwrap_or_else(|| "sink".to_string()),
                None => "sink".to_string(),
            };
            Ok(Self { label })
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(pong) = input.payload() {
                debug!(
                    "{}: got pong seq={} reply={}",
                    self.label.as_str(),
                    pong.seq,
                    pong.reply.as_str()
                );
            }
            Ok(())
        }
    }
}
