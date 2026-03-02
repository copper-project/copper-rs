use super::*;
use cu_gnss_payloads::{
    GnssAccuracy, GnssCommandAck, GnssEpochTime, GnssFixSolution, GnssInfoText, GnssRawUbxFrame,
    GnssRfStatus, GnssSatelliteState, GnssSatsInView, GnssSignalState,
};
use cu29::units::si::angle::degree;

#[cfg(feature = "firmware")]
type GnssBackend = cu_gnss_ublox::UbxSourceTask<cu_micoairh743::Uart3Port>;

#[cfg(all(feature = "sim", not(feature = "firmware")))]
type GnssBackend = cu_gnss_ublox::UbxSource;

type GnssBackendOutput<'m> = (
    CuMsg<GnssEpochTime>,
    CuMsg<GnssFixSolution>,
    CuMsg<GnssAccuracy>,
    CuMsg<GnssSatsInView>,
    CuMsg<GnssSatelliteState>,
    CuMsg<GnssSignalState>,
    CuMsg<GnssRfStatus>,
    CuMsg<GnssInfoText>,
    CuMsg<GnssCommandAck>,
    CuMsg<GnssRawUbxFrame>,
);

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct GnssSource {
    #[reflect(ignore)]
    backend: GnssBackend,
}

impl Freezable for GnssSource {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.backend.freeze(encoder)
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        self.backend.thaw(decoder)
    }
}

impl CuSrcTask for GnssSource {
    type Resources<'r> = <GnssBackend as CuSrcTask>::Resources<'r>;
    type Output<'m> = output_msg!(GnssFixSolution);

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let backend = GnssBackend::new(config, resources)?;
        Ok(Self { backend })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.backend.start(ctx)
    }

    fn preprocess(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.backend.preprocess(ctx)
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let mut backend_out: GnssBackendOutput<'_> = Default::default();
        self.backend.process(ctx, &mut backend_out)?;

        output.clear_payload();
        output.tov = backend_out.1.tov;
        if let Some(fix) = backend_out.1.payload() {
            output.set_payload(fix.clone());
        }
        output.metadata = backend_out.1.metadata.clone();

        Ok(())
    }

    fn postprocess(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.backend.postprocess(ctx)
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.backend.stop(ctx)
    }
}

#[derive(Reflect, Default)]
pub struct GnssFixSink {
    last_log: Option<CuTime>,
}

impl Freezable for GnssFixSink {}

impl CuSinkTask for GnssFixSink {
    type Input<'m> = CuMsg<GnssFixSolution>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self::default())
    }

    fn process<'i>(&mut self, ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
        let Some(fix) = input.payload() else {
            return Ok(());
        };

        let now = ctx.now();
        let should_log = self
            .last_log
            .map(|last| now - last >= CuDuration::from_secs(1))
            .unwrap_or(true);
        if should_log {
            self.last_log = Some(now);
            info!(
                "gnss lat={:.7} lon={:.7} sats={} fix_ok={}",
                fix.latitude.get::<degree>(),
                fix.longitude.get::<degree>(),
                fix.num_satellites_used,
                fix.gnss_fix_ok
            );
        }

        Ok(())
    }
}
