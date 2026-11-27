//! Graph types and UDP configuration shared by the robot and its live twin.

mod policy;
use cu29::prelude::*;
use cu29_logstream_udp::{CuUdpLogStreamConfig, CuUdpLogStreamRx, CuUdpLogStreamTx};
pub use policy::{ActuationGate, Command, Deadman, Safety, Tick};
use std::{error::Error, fs::File, path::PathBuf};

#[allow(dead_code)]
#[derive(cu29::prelude::Teleops)]
pub struct Controls {
    pub button: Latest<Command>,
}
pub struct Ops;
bundle_resources!(Ops: Button);
impl ResourceBundle for Ops {
    fn build(
        _: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        _: &mut ResourceManager,
    ) -> CuResult<()> {
        Ok(())
    }
}

/// Maximum age of a permit, 150 ms in robot time.
pub const PERMIT_LIFETIME: CuDuration = CuDuration(150_000_000);

pub const RETURN_PACKET_BYTES: usize = 256;

pub fn endpoints(
    bind: &str,
    peer: &str,
) -> Result<(CuUdpLogStreamTx, CuUdpLogStreamRx), Box<dyn Error>> {
    let mut config = CuUdpLogStreamConfig::new(bind.parse()?);
    config.remote_addr = Some(peer.parse()?);
    let (tx, rx) = config.open()?;
    Ok((tx.ok_or("missing UDP peer")?, rx))
}

pub fn lock_log(role: &str) -> Result<(PathBuf, File), Box<dyn Error>> {
    let root = PathBuf::from("logs");
    std::fs::create_dir_all(&root)?;
    let lock = File::options()
        .create(true)
        .truncate(false)
        .write(true)
        .open(root.join(format!("{role}.lock")))?;
    lock.try_lock()
        .map_err(|err| format!("Cannot lock {role} recording: {err}"))?;
    Ok((root.join(format!("{role}.copper")), lock))
}
