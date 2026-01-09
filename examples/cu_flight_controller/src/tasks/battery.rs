use crate::debug_rl;
use super::*;


resources!({
    battery_adc => Owned<cu_micoairh743::BatteryAdc>,
});

pub struct BatteryAdcSource {
    adc: cu_micoairh743::BatteryAdc,
    vref_mv: u32,
    vbat_scale: u32,
    vbat_res_div_val: u32,
    vbat_res_div_mult: u32,
}

impl Freezable for BatteryAdcSource {}

impl CuSrcTask for BatteryAdcSource {
    type Output<'m> = output_msg!(BatteryVoltage);
    type Resources<'r> = battery::Resources;

    fn new_with(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let vref_mv = cfg_u32(config, "vref_mv", BATTERY_VREF_MV_DEFAULT).max(1);
        let vbat_scale = cfg_u32(config, "vbat_scale", BATTERY_VBAT_SCALE_DEFAULT);
        let vbat_res_div_val =
            cfg_u32(config, "vbat_res_div_val", BATTERY_VBAT_RES_DIV_VAL_DEFAULT).max(1);
        let vbat_res_div_mult = cfg_u32(
            config,
            "vbat_res_div_mult",
            BATTERY_VBAT_RES_DIV_MULT_DEFAULT,
        )
            .max(1);
        Ok(Self {
            adc: resources.battery_adc.0,
            vref_mv,
            vbat_scale,
            vbat_res_div_val,
            vbat_res_div_mult,
        })
    }

    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        let now = clock.now();
        let raw = self.adc.read_raw_blocking();
        let slope = self.adc.slope().max(1);
        let raw_u64 = u64::from(raw);
        let slope_u64 = u64::from(slope);
        let vref_mv = u64::from(self.vref_mv);
        let vbat_scale = u64::from(self.vbat_scale);
        let vbat_res_div_val = u64::from(self.vbat_res_div_val);
        let vbat_res_div_mult = u64::from(self.vbat_res_div_mult);

        // Match Betaflight voltageAdcToVoltage: output in 0.01V steps.
        let denom = slope_u64.saturating_mul(vbat_res_div_val).max(1);
        let mut numer = raw_u64.saturating_mul(vbat_scale).saturating_mul(vref_mv);
        numer /= 10;
        let mut centivolts = (numer + denom / 2) / denom;
        centivolts /= vbat_res_div_mult.max(1);
        let centivolts = centivolts.min(u64::from(u16::MAX)) as u16;

        output.set_payload(BatteryVoltage { centivolts });
        output.tov = Tov::Time(now);
        debug_rl!(
        &LOG_TELEMETRY,
        now,
        "vbat adc raw={} slope={} vref_mv={} centivolts={}",
        raw,
        slope,
        self.vref_mv,
        centivolts
    );
        Ok(())
    }
}
