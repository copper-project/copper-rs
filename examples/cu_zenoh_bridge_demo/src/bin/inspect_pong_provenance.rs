use cu29::prelude::*;
use cu29_export::copperlists_reader;

pub mod bridges {
    pub use cu_zenoh_bridge_demo::bridges::*;
}

pub mod messages {
    pub use cu_zenoh_bridge_demo::messages::*;
}

pub mod tasks {
    pub use cu_zenoh_bridge_demo::tasks::*;
}

#[copper_runtime(config = "pong_config.ron")]
struct InspectPongSupport {}

fn main() {
    if let Err(err) = drive() {
        eprintln!("inspect-pong-provenance failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let log_path = std::env::args()
        .nth(1)
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| cu_zenoh_bridge_demo::default_log_path("zenoh_pong.copper"));
    inspect::<default::CuStampedDataSet>(&log_path)
}

fn inspect<P>(log_path: &std::path::Path) -> CuResult<()>
where
    P: CopperListTuple,
{
    let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
        .file_base_name(log_path)
        .build()
        .map_err(|e| CuError::from(format!("failed to open log '{}': {e}", log_path.display())))?
    else {
        return Err(CuError::from("expected a readable unified logger"));
    };

    let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::CopperList);
    let slot_ids = P::get_all_task_ids();
    let mut found = false;

    for copperlist in copperlists_reader::<P>(&mut reader) {
        for (slot_id, msg) in slot_ids.iter().zip(copperlist.cumsgs()) {
            if let Some(origin) = msg.metadata().bridge_origin() {
                found = true;
                println!(
                    "{slot_id} local_cl={} origin={{subsystem_code={}, instance_id={}, cl_id={}}}",
                    copperlist.id, origin.subsystem_code, origin.instance_id, origin.cl_id
                );
            }
        }
    }

    if !found {
        println!("No bridge provenance found in {}", log_path.display());
    }

    Ok(())
}
