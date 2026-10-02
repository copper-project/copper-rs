//! Host packaging and startup-recording integration example.
#![cfg(feature = "self-describing-logs")]

pub use cu_self_describing_payloads as payloads;

use cu29::prelude::*;
use payloads::WheelSample;

include!(concat!(env!("OUT_DIR"), "/catalog.rs"));

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

    #[test]
    fn test_host_embedded_catalog_is_recorded_verbatim_once_at_startup() {
        let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("wheel.copper");
        let app = WheelApplication::builder()
            .with_value_decode_catalog(VALUE_DECODE_CATALOG)
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
        assert_eq!(recorded, VALUE_DECODE_CATALOG);
        assert!(
            reader
                .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
                .unwrap()
                .is_none()
        );
        let catalog = ValueDecodeCatalog::from_blob(&recorded).unwrap();
        let wheel = catalog
            .slots
            .iter()
            .find(|slot| slot.task_id == "wheel")
            .unwrap();
        let filter = catalog
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
                .any(|schema| schema.type_path == "cu_self_describing_payloads::WheelSample")
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

    #[test]
    fn test_startup_rejects_truncated_header_and_unsupported_version() {
        let result = WheelApplication::builder()
            .with_value_decode_catalog(&VALUE_DECODE_CATALOG[..8])
            .build();
        assert!(
            result
                .err()
                .unwrap()
                .to_string()
                .contains("Invalid embedded")
        );
        static BAD_VERSION: &[u8] = b"CUVDCAT\0\x02\x00";
        let result = WheelApplication::builder()
            .with_value_decode_catalog(BAD_VERSION)
            .build();
        assert!(result.err().unwrap().to_string().contains("unsupported"));
    }
}
