//! Run the robot watchdog and stream its CopperLists to the operator.
use cu_teleops_deadman::*;
use cu29::logstream::{
    FecTelemetryTx, LogStreamPlan, TelemetryBufferConfig, protocol::CuTelemetryTx,
};
use cu29::prelude::*;
use cu29::teleops::{ReturnPolicy, with_teleops};
use cu29_logstream_udp::{CuUdpLogStreamRx, CuUdpLogStreamTx};
use std::error::Error;

#[copper_runtime(config = "robotconfig.ron")]
struct Robot {}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let bind = args.next().unwrap_or_else(|| "127.0.0.1:7441".into());
    let peer = args.next().unwrap_or_else(|| "127.0.0.1:7442".into());
    if args.next().is_some() {
        return Err("usage: robot [bind_addr [twin_addr]]".into());
    }
    let (data_tx, return_rx) = endpoints(&bind, &peer)?;
    let (publish, inboxes) = Controls::inboxes()?;
    let (path, _lock) = lock_log("robot")?;
    let robot = Robot::builder()
        .with_log_path(path, None)?
        .with_resources(move |cfg| {
            let mut manager = default::resources_instanciator(cfg)?;
            manager.add_owned(
                default::resources::bundles::OPS.key(OpsId::Button),
                inboxes.button,
            )?;
            Ok(manager)
        })
        .with_telemetry_tx(
            move |clock| robot_tx(data_tx, return_rx, publish, clock),
            TelemetryBufferConfig {
                capture_count: 64,
                keyframe_count: 8,
                ..TelemetryBufferConfig::new(512, 512)
            },
        )
        .build()?;
    println!("Robot listening on {bind}, streaming to {peer}");
    robot.run_until_shutdown()?;
    Ok(())
}

fn robot_tx(
    data_tx: CuUdpLogStreamTx,
    return_rx: CuUdpLogStreamRx,
    publish: controls::Publishers,
    clock: RobotClock,
) -> CuResult<impl CuTelemetryTx> {
    let config = CuConfig::deserialize_ron(include_str!("../fec.ron"))?;
    let plan = LogStreamPlan::resolve(&config.log_streaming.unwrap().destinations[0])
        .map_err(|err| CuError::new_with_cause("invalid FEC plan", err))?;
    let fec = FecTelemetryTx::new(data_tx, plan, clock.clone())?;
    let policy = ReturnPolicy::<RETURN_PACKET_BYTES> { feedback: None };
    Ok(with_teleops(
        fec,
        return_rx,
        publish,
        clock,
        policy,
        |capture| {
            let (list, _) =
                cu29::logstream::capture::decode_capture::<default::CuStampedDataSet>(capture)
                    .ok()?;
            list.copperlist.msgs.get_tick_output().payload().copied()
        },
        |result| {
            if let Err(rejected) = result {
                eprintln!("Rejected input {rejected:?}");
            }
        },
    ))
}
