use super::*;
use cu_gnss_payloads::GnssFixSolution;

#[cfg(feature = "firmware")]
pub type GnssSource = cu_gnss_ublox::UbxSourceTask<cu_micoairh743::Uart3Port>;

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
pub type GnssSource = crate::sim_support::SimGnssSource;

#[derive(Reflect, Default)]
pub struct GnssFixSink {
    last_log: Option<CuTime>,
}

impl Freezable for GnssFixSink {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        cu29::bincode::Encode::encode(&self.last_log, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.last_log = cu29::bincode::Decode::decode(decoder)?;
        Ok(())
    }
}

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
                "gnss lat={} lon={} sats={} fix_ok={}",
                fix.position.latitude_degrees(),
                fix.position.longitude_degrees(),
                fix.num_satellites_used,
                fix.gnss_fix_ok
            );
        }

        Ok(())
    }
}
