//! Local UDP endpoints and the robot telemetry worker.
use crate::{controls, default};
use cu29::logstream::{FecTelemetryTx, LogStreamPlan, protocol::CuTelemetryTx};
use cu29::prelude::*;
use cu29::teleops::{ReturnPolicy, with_teleops};
use cu29_logstream_udp::{CuUdpLogStreamConfig, CuUdpLogStreamRx, CuUdpLogStreamTx};
use std::error::Error;

/// Complete UDP return datagram budget shared by this demo’s console and robot.
pub(super) const RETURN_PACKET_BYTES: usize = 256;

pub(super) fn loopback() -> Result<(CuUdpLogStreamTx, CuUdpLogStreamRx), Box<dyn Error>> {
    let loopback = "127.0.0.1:0".parse()?;
    let (_, rx) = CuUdpLogStreamConfig::new(loopback).open()?;
    let mut sender = CuUdpLogStreamConfig::new(loopback);
    sender.remote_addr = Some(rx.local_addr()?);
    Ok((sender.open()?.0.ok_or("missing sender")?, rx))
}

pub(super) fn robot_tx(
    data_tx: CuUdpLogStreamTx,
    return_rx: CuUdpLogStreamRx,
    publish: controls::Publishers,
    clock: RobotClock,
) -> CuResult<impl CuTelemetryTx> {
    let config = CuConfig::deserialize_ron(include_str!("../fec.ron"))?;
    let plan = LogStreamPlan::resolve(&config.log_streaming.unwrap().destinations[0])
        .map_err(|err| CuError::new_with_cause("invalid loopback FEC plan", err))?;
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
