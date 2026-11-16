//! A deterministic motion source and a sink using the payload's native Rerun representation.

use crate::SoftArm;
use cu_rerun::rerun::RecordingStream;
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use std::time::Duration;

/// One 12-second gesture at the graph's 30 Hz rate.
#[derive(Reflect)]
pub struct Motion {
    pub frame: u64,
}

impl Freezable for Motion {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.frame, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.frame = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for Motion {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(SoftArm);

    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self { frame: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let phase = std::f32::consts::TAU * (self.frame % 360) as f32 / 360.0;
        output.set_payload(SoftArm::at_phase(phase));
        output.tov = Tov::Time(ctx.now());
        if self.frame.is_multiple_of(360) {
            info!(ctx, "Soft-arm gesture {}", self.frame / 360);
        }
        self.frame += 1;
        Ok(())
    }
}

/// The application supplies the stream, so the sink works with a live viewer or an RRD file.
pub struct ViewerBundle;
bundle_resources!(ViewerBundle: Recording);

impl ResourceBundle for ViewerBundle {
    fn build(
        _: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        _: &mut ResourceManager,
    ) -> CuResult<()> {
        Err(CuError::from(
            "Supply the Rerun recording with the application's resource factory",
        ))
    }
}

/// Construct the single bundle declared in `copperconfig.ron`.
pub fn viewer_resources(recording: RecordingStream) -> CuResult<ResourceManager> {
    let mut resources = ResourceManager::new(&[ViewerBundleId::COUNT]);
    resources.add_owned(
        BundleIndex::new(0).key(ViewerBundleId::Recording),
        recording,
    )?;
    Ok(resources)
}

mod sink_resources {
    use super::*;
    cu29::resources!({ recording => Owned<RecordingStream> });
}

/// Log the Copper message as one custom Rerun entity.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct RerunSink {
    #[reflect(ignore)]
    recording: RecordingStream,
}

impl Freezable for RerunSink {}

impl CuSinkTask for RerunSink {
    type Resources<'r> = sink_resources::Resources;
    type Input<'m> = input_msg!(SoftArm);

    fn new(_: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            recording: resources.recording.0,
        })
    }

    fn process(&mut self, _: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(arm) = input.payload() {
            if let Tov::Time(time) = input.tov {
                self.recording
                    .set_time("cu_time", Duration::from_nanos(time.as_nanos()));
            }
            self.recording.log("world/soft_arm", arm).map_err(|error| {
                CuError::new_with_cause("Failed to log soft arm to Rerun", error)
            })?;
        }
        Ok(())
    }

    fn stop(&mut self, _: &CuContext) -> CuResult<()> {
        self.recording
            .flush_blocking()
            .map_err(|error| CuError::new_with_cause("Failed to flush Rerun recording", error))
    }
}
