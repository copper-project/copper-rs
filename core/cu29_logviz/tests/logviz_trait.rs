use cu29::prelude::CuResult;
use cu29_logviz::{LogvizDataSet, logviz_emit_dataset};
use rerun::RecordingStream;

struct Dummy;

impl LogvizDataSet for Dummy {
    fn logviz_emit(&self, _rec: &RecordingStream) -> CuResult<()> {
        Ok(())
    }
}

#[test]
fn logviz_trait_compiles() {
    let _ = Dummy;
}

#[test]
fn logviz_emit_dataset_smoke() {
    let rec = RecordingStream::disabled();
    let dummy = Dummy;
    assert!(logviz_emit_dataset(&dummy, &rec).is_ok());
}
