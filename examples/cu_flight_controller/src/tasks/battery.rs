use super::*;
use cu29::units::si::electric_potential::volt;
use cu29::units::si::f32::ElectricPotential;

#[derive(Reflect)]
struct BatteryAdcCalibration {
    vref_mv: u32,
    vbat_scale: u32,
    vbat_res_div_val: u32,
    vbat_res_div_mult: u32,
}

trait BatteryAdcBackend {
    fn read_centivolts(&mut self, calib: &BatteryAdcCalibration) -> u16;
}

#[cfg(feature = "firmware")]
type BatteryAdcBackendImpl = cu_micoairh743::BatteryAdc;

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
type BatteryAdcBackendImpl = crate::sim_support::SimBatteryAdc;

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
const SIM_BATTERY_BASE_VOLTAGE_V: f32 = 16.0;
#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
const SIM_BATTERY_SAG_MAX_RATIO: f32 = 0.08;

#[cfg(feature = "firmware")]
impl BatteryAdcBackend for BatteryAdcBackendImpl {
    fn read_centivolts(&mut self, calib: &BatteryAdcCalibration) -> u16 {
        let raw_u64 = u64::from(self.read_raw_blocking());
        let slope_u64 = u64::from(self.slope().max(1));
        let vref_mv = u64::from(calib.vref_mv);
        let vbat_scale = u64::from(calib.vbat_scale);
        let vbat_res_div_val = u64::from(calib.vbat_res_div_val);
        let vbat_res_div_mult = u64::from(calib.vbat_res_div_mult.max(1));

        // Match Betaflight voltageAdcToVoltage: output in 0.01V steps.
        let denom = slope_u64.saturating_mul(vbat_res_div_val).max(1);
        let mut numer = raw_u64.saturating_mul(vbat_scale).saturating_mul(vref_mv);
        numer /= 10;
        let mut centivolts = (numer + denom / 2) / denom;
        centivolts /= vbat_res_div_mult;
        centivolts.min(u64::from(u16::MAX)) as u16
    }
}

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
impl BatteryAdcBackend for BatteryAdcBackendImpl {
    fn read_centivolts(&mut self, _calib: &BatteryAdcCalibration) -> u16 {
        let voltage_v = self.read_voltage_v();
        let centivolts = (voltage_v * 100.0).round().clamp(0.0, f32::from(u16::MAX));
        centivolts as u16
    }
}

#[cfg(feature = "firmware")]
resources!({
    battery_adc => Owned<BatteryAdcBackendImpl>,
});

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct BatteryAdcSource {
    #[reflect(ignore)]
    adc: BatteryAdcBackendImpl,
    calib: BatteryAdcCalibration,
}

impl Freezable for BatteryAdcSource {}

impl CuSrcTask for BatteryAdcSource {
    type Output<'m> = output_msg!(BatteryVoltage);
    #[cfg(feature = "firmware")]
    type Resources<'r> = Resources;
    #[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let calib = BatteryAdcCalibration {
            vref_mv: cfg_u32(config, "vref_mv", BATTERY_VREF_MV_DEFAULT)?.max(1),
            vbat_scale: cfg_u32(config, "vbat_scale", BATTERY_VBAT_SCALE_DEFAULT)?,
            vbat_res_div_val: cfg_u32(
                config,
                "vbat_res_div_val",
                BATTERY_VBAT_RES_DIV_VAL_DEFAULT,
            )?
            .max(1),
            vbat_res_div_mult: cfg_u32(
                config,
                "vbat_res_div_mult",
                BATTERY_VBAT_RES_DIV_MULT_DEFAULT,
            )?
            .max(1),
        };

        #[cfg(feature = "firmware")]
        let adc = _resources.battery_adc.0;
        #[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
        let adc = crate::sim_support::sim_battery_adc(
            SIM_BATTERY_BASE_VOLTAGE_V,
            SIM_BATTERY_SAG_MAX_RATIO,
        );

        Ok(Self { adc, calib })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let now = ctx.now();
        let centivolts = self.adc.read_centivolts(&self.calib);
        output.set_payload(BatteryVoltage {
            voltage: ElectricPotential::new::<volt>(centivolts as f32 / 100.0),
        });
        output.tov = Tov::Time(now);
        Ok(())
    }
}
