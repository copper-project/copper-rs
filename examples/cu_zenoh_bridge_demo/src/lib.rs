use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

pub mod messages {
    use super::*;
    use cu29::prelude::*;

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

        fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
            output.set_payload(Ping {
                seq: self.seq,
                note: format!("{}#{}", self.label, self.seq),
            });
            output.tov = Tov::Time(clock.now());
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
            clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            if let Some(ping) = input.payload() {
                output.set_payload(Pong {
                    seq: ping.seq,
                    reply: format!("{} -> {}", self.label, ping.note),
                });
                output.tov = Tov::Time(clock.now());
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

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
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
