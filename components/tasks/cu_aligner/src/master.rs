//! Selection and configuration for master-driven alignment.
use cu29::prelude::*;
use serde::Deserialize;

/// How a sensor sample is selected relative to the master's point ToV.
#[derive(Debug, Clone, Copy, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum Selection {
    /// Closest sample; equal distances prefer the earlier sample.
    #[default]
    Nearest,
    /// Most recent sample no later than the master.
    LatestBefore,
}

#[doc(hidden)]
pub use circular_buffer::FixedCircularBuffer;

/// Validated settings shared by generated master tasks.
#[doc(hidden)]
pub struct Settings {
    pub selection: Selection,
    pub max_offset: CuDuration,
    pub max_wait: CuDuration,
    pub retention: CuDuration,
}

impl Settings {
    pub fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let config = config.ok_or_else(|| CuError::from("Master aligner config missing"))?;
        let required_ms = |key| -> CuResult<CuDuration> {
            let value = config
                .get::<u32>(key)?
                .ok_or_else(|| CuError::from("Master aligner timing setting missing"))?;
            Ok(CuDuration::from_millis(u64::from(value)))
        };
        let settings = Self {
            selection: config
                .get_value::<Selection>("selection")?
                .unwrap_or_default(),
            max_offset: required_ms("max_offset_ms")?,
            max_wait: required_ms("max_wait_ms")?,
            retention: required_ms("stale_data_horizon_ms")?,
        };
        if settings.retention < settings.max_offset + settings.max_wait {
            return Err(CuError::from(
                "Master aligner retention must cover max_offset_ms + max_wait_ms",
            ));
        }
        Ok(settings)
    }
}

/// Validate before mutating any buffers. Equal timestamps are repeated inputs.
#[doc(hidden)]
pub fn input_time<P: CuMsgPayload>(
    msg: &CuMsg<P>,
    previous: Option<CuTime>,
) -> CuResult<Option<CuTime>> {
    if msg.payload().is_none() {
        return Ok(None);
    }
    let Tov::Time(time) = msg.tov else {
        return Err(CuError::from("Master alignment requires point ToVs"));
    };
    if previous.is_some_and(|previous| time < previous) {
        return Err(CuError::from(
            "Master aligner input timestamps must be monotonic",
        ));
    }
    Ok((previous != Some(time)).then_some(time))
}

#[doc(hidden)]
pub fn select<'a, P: CuMsgPayload, const N: usize>(
    buffer: &'a crate::buffers::TimeboundCircularBuffer<N, P, CuMsgMetadata>,
    reference: CuTime,
    settings: &Settings,
) -> Option<&'a CuMsg<P>> {
    buffer
        .inner
        .iter()
        .filter_map(|msg| {
            let Tov::Time(time) = msg.tov else {
                return None;
            };
            if matches!(settings.selection, Selection::LatestBefore) && time > reference {
                return None;
            }
            let distance = time.as_nanos().abs_diff(reference.as_nanos());
            (distance <= settings.max_offset.as_nanos()).then_some((distance, time, msg))
        })
        .min_by_key(|(distance, time, _)| (*distance, *time))
        .map(|(_, _, msg)| msg)
}

/// Define a task emitting one tuple of complete messages per matched master.
///
/// Inputs are numbered consecutively from zero. Each declaration supplies buffer
/// capacity and payload type, e.g. `0 => { 8, Image }`. See the crate README.
#[macro_export]
macro_rules! define_master_task {
    ($name:ident, master: $master:tt, $($index:tt => { $capacity:expr, $payload:ty }),+ $(,)?) => {
        #[derive(cu29::prelude::Reflect)]
        #[reflect(from_reflect = false)]
        pub struct $name {
            #[reflect(ignore)]
            buffers: ($($crate::buffers::TimeboundCircularBuffer<$capacity, $payload, cu29::prelude::CuMsgMetadata>,)+),
            #[reflect(ignore)]
            arrivals: $crate::master::FixedCircularBuffer<cu29::prelude::CuTime, { [$($capacity),+][$master] }>,
            #[reflect(ignore)]
            last_seen: [Option<cu29::prelude::CuTime>; [$($capacity),+].len()],
            #[reflect(ignore)]
            settings: $crate::master::Settings,
        }

        impl cu29::prelude::Freezable for $name {
            fn freeze<E: cu29::bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), cu29::bincode::error::EncodeError> {
                use cu29::bincode::Encode;
                $(self.buffers.$index.freeze(encoder)?;)+
                Encode::encode(&self.last_seen, encoder)?;
                Encode::encode(&(self.arrivals.len() as u64), encoder)?;
                for arrival in self.arrivals.iter() { Encode::encode(arrival, encoder)?; }
                Ok(())
            }
            fn thaw<D: cu29::bincode::de::Decoder>(&mut self, decoder: &mut D) -> Result<(), cu29::bincode::error::DecodeError> {
                use cu29::bincode::Decode;
                $(self.buffers.$index.thaw(decoder)?;)+
                self.last_seen = Decode::decode(decoder)?;
                let len: u64 = Decode::decode(decoder)?;
                if len != self.buffers.$master.inner.len() as u64 {
                    return Err(cu29::bincode::error::DecodeError::Other("Master queue length mismatch"));
                }
                self.arrivals.clear();
                for _ in 0..len { self.arrivals.push_back(Decode::decode(decoder)?); }
                Ok(())
            }
        }

        impl cu29::prelude::CuTask for $name {
            type Resources<'r> = ();
            type Input<'m> = ($(&'m cu29::prelude::CuMsg<$payload>,)+);
            type Output<'m> = cu29::prelude::CuMsg<($(cu29::prelude::CuMsg<$payload>,)+)>;

            fn new(config: Option<&cu29::prelude::ComponentConfig>, _resources: ()) -> cu29::prelude::CuResult<Self> {
                if [$($capacity),+].contains(&0) {
                    return Err(cu29::prelude::CuError::from("Aligner buffer capacities must be positive"));
                }
                Ok(Self {
                    buffers: ($( $crate::buffers::TimeboundCircularBuffer::<$capacity, $payload, cu29::prelude::CuMsgMetadata>::new(), )+),
                    arrivals: $crate::master::FixedCircularBuffer::new(),
                    last_seen: [None; [$($capacity),+].len()],
                    settings: $crate::master::Settings::new(config)?,
                })
            }

            fn process(&mut self, ctx: &cu29::prelude::CuContext, input: &Self::Input<'_>, output: &mut Self::Output<'_>) -> cu29::prelude::CuResult<()> {
                use cu29::prelude::*;
                output.clear_payload();
                output.tov = Tov::None;
                // Validate all inputs before committing any state changes.
                let times = [$($crate::master::input_time(input.$index, self.last_seen[$index])?,)+];
                let now = ctx.now();
                let horizon = now - self.settings.retention;
                $(
                    if $index != $master { self.buffers.$index.purge(horizon); }
                    if let Some(time) = times[$index] {
                        self.buffers.$index.push(input.$index.clone());
                        self.last_seen[$index] = Some(time);
                    }
                )+
                if times[$master].is_some() { self.arrivals.push_back(now); }

                // Drop unmatched/expired masters, emitting at most one observation per call.
                while let Some(arrival) = self.arrivals.front().copied() {
                    if now - arrival < self.settings.max_wait { break; }
                    let master = self.buffers.$master.inner.front().expect("master queue invariant");
                    let Tov::Time(reference) = master.tov else { unreachable!() };
                    let selected = ($(
                        if $index == $master { self.buffers.$index.inner.front() }
                        else { $crate::master::select(&self.buffers.$index, reference, &self.settings) },
                    )+);
                    let complete = reference >= horizon $(&& selected.$index.is_some())+;
                    if complete {
                        let payload = output.payload_mut().get_or_insert_with(Default::default);
                        $(payload.$index.clone_from(selected.$index.expect("complete selection"));)+
                        output.tov = Tov::Time(reference);
                    }
                    self.arrivals.pop_front();
                    self.buffers.$master.inner.pop_front();
                    if complete { break; }
                }
                Ok(())
            }
        }
    };
}
