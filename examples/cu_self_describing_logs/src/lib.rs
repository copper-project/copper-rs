//! Automatic startup catalog recording in a single-crate application.
#![cfg(feature = "self-describing-logs")]

pub mod payloads;

use cu29::prelude::*;
use payloads::WheelSample;

#[derive(Reflect)]
pub struct WheelSource;
impl Freezable for WheelSource {}
impl CuSrcTask for WheelSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(WheelSample);
    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(WheelSample {
            ticks: 42,
            timestamp: ctx.now(),
            ..Default::default()
        });
        output.tov = Tov::Time(ctx.now());
        Ok(())
    }
}

#[derive(Reflect)]
pub struct WheelFilter;
impl Freezable for WheelFilter {}
impl CuTask for WheelFilter {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(WheelSample);
    type Output<'m> = output_msg!(WheelSample);
    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(sample) = input.payload() {
            output.set_payload(sample.clone());
        }
        output.tov = input.tov;
        Ok(())
    }
}

#[derive(Reflect)]
pub struct WheelSink;
impl Freezable for WheelSink {}
impl CuSinkTask for WheelSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(WheelSample);
    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &CuContext, _: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct WheelApplication {}

pub use default::WheelApplication as Application;

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::bincode;
    use cu29_value::catalog::ValueDecodeCatalog;
    use cu29_value::decode::ValueDecodeLimits;

    #[copper_runtime(config = "missionconfig.ron")]
    struct MissionApplication {}

    #[test]
    fn every_mission_shares_one_static_catalog_and_matching_append_reuses_it() {
        use std::sync::{Arc, Mutex};
        let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("missions.copper");
        let open = |append| {
            let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
                .file_base_name(&path)
                .preallocated_size(1024 * 1024)
                .write(true)
                .create(true)
                .append(append)
                .build()
                .unwrap()
            else {
                panic!("writer")
            };
            Arc::new(Mutex::new(logger))
        };
        let shared = open(false);
        let first = alpha::MissionApplication::builder()
            .with_logger::<memmap::MmapSectionStorage, UnifiedLoggerWrite>(shared.clone())
            .with_resources(|_| {
                let mut reader = UnifiedLoggerRead::new(&path).unwrap();
                assert!(
                    reader
                        .application_metadata()
                        .unwrap()
                        .unwrap()
                        .catalog_offset
                        > 0
                );
                assert!(
                    reader
                        .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
                        .unwrap()
                        .is_some()
                );
                Ok(ResourceManager::new(&[]))
            })
            .build()
            .unwrap();
        let mut first = first.start().unwrap();
        first.run_one_iteration().unwrap();
        let second = beta::MissionApplication::builder()
            .with_logger::<memmap::MmapSectionStorage, UnifiedLoggerWrite>(shared.clone())
            .build()
            .unwrap();
        let mut second = second.start().unwrap();
        second.run_one_iteration().unwrap();
        drop(first.stop().unwrap());
        drop(second.stop().unwrap());
        drop(shared);
        let before = UnifiedLoggerRead::new(&path)
            .unwrap()
            .application_metadata()
            .unwrap()
            .unwrap();
        let shared = open(true);
        drop(
            alpha::MissionApplication::builder()
                .with_logger::<memmap::MmapSectionStorage, UnifiedLoggerWrite>(shared.clone())
                .build()
                .unwrap()
                .start()
                .unwrap()
                .stop()
                .unwrap(),
        );
        drop(shared);
        let mut reader = UnifiedLoggerRead::new(&path).unwrap();
        assert_eq!(reader.application_metadata().unwrap().unwrap(), before);
        assert_eq!(before.missions, ["alpha", "beta"]);
        let catalog = ValueDecodeCatalog::from_blob(
            &reader
                .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
                .unwrap()
                .unwrap(),
        )
        .unwrap();
        assert_eq!(catalog.version, 2);
        assert!(
            reader
                .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
                .unwrap()
                .is_none()
        );
        assert_eq!(catalog.missions.len(), 2);
        assert_eq!(catalog.missions[0].mission_index, 0);
        assert_eq!(catalog.missions[1].mission_index, 1);
        assert_eq!(
            catalog.missions[0].slots[0].binding,
            catalog.missions[1].slots[0].binding
        );
        assert_eq!(
            catalog
                .description
                .schemas
                .iter()
                .filter(|schema| schema.type_path.ends_with("::WheelSample"))
                .count(),
            1
        );
        let mut reader = UnifiedLoggerRead::new(&path).unwrap();
        let mut runs = std::collections::BTreeSet::new();
        loop {
            let header = reader.raw_skip_section().unwrap();
            if header.entry_type == UnifiedLogType::LastEntry {
                break;
            }
            if header.context.run_id > 0 {
                runs.insert(header.context.run_id);
            }
        }
        assert_eq!(runs.into_iter().collect::<Vec<_>>(), [1, 2, 3]);
    }

    #[test]
    fn test_catalog_is_recorded_automatically_at_startup() {
        let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("wheel.copper");
        let app = WheelApplication::builder()
            .with_log_path(&path, Some(32 * 1024 * 1024))
            .unwrap()
            .with_resources(|_| Ok(ResourceManager::new(&[])))
            .build()
            .unwrap();
        let mut app = app.start().unwrap();
        app.run_one_iteration().unwrap();
        app.run_one_iteration().unwrap();
        drop(app.stop().unwrap());
        let logger = UnifiedLoggerBuilder::new()
            .file_base_name(&path)
            .build()
            .unwrap();
        let UnifiedLogger::Read(mut reader) = logger else {
            panic!("read logger")
        };
        let recorded = reader
            .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
            .unwrap()
            .unwrap();
        assert!(
            reader
                .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
                .unwrap()
                .is_none()
        );
        let catalog = ValueDecodeCatalog::from_blob(&recorded).unwrap();
        let wheel = catalog.missions[0]
            .slots
            .iter()
            .find(|slot| slot.task_id == "wheel")
            .unwrap();
        let filter = catalog.missions[0]
            .slots
            .iter()
            .find(|slot| slot.task_id == "filter")
            .unwrap();
        assert_eq!(wheel.binding, filter.binding);
        assert!(
            catalog
                .description
                .schemas
                .iter()
                .any(|schema| schema.type_path == "cu_self_describing_logs::payloads::WheelSample")
        );
        let payload = WheelSample {
            ticks: 42,
            ..Default::default()
        };
        let bytes = bincode::encode_to_vec(&payload, bincode::config::standard()).unwrap();
        assert!(
            bincode::encode_to_vec(&catalog, bincode::config::standard())
                .unwrap()
                .len()
                > recorded.len()
        );
        let mut description = catalog.description;
        description.root = wheel.binding.unwrap();
        let (tree, used) = description
            .decode(
                &bytes,
                bincode::config::standard(),
                ValueDecodeLimits::default(),
            )
            .unwrap();
        assert_eq!(used, bytes.len());
        let Value::Map(fields) = tree else {
            panic!("wheel fields")
        };
        assert_eq!(
            fields.get(&Value::String("ticks".into())),
            Some(&Value::U32(42))
        );
    }
}
