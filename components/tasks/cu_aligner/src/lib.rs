#![doc = include_str!("../README.md")]

pub mod buffers;

/// Define a task that aligns incoming messages based on their timestamps
/// See module doc for use.
#[macro_export]
macro_rules! define_task {
    ($name:ident, $($index:tt => { $mis:expr, $mos:expr, $p:ty }),+) => {

       paste::paste! {
            $crate::buffers::alignment_buffers!(
                AlignmentBuffers,
                $(
                    [<buffer $index>]: TimeboundCircularBuffer<$mis, CuStampedData<$p, CuMsgMetadata>>
                ),*
            );
        }

        pub struct $name {
            aligner: AlignmentBuffers,
        }

        impl Freezable for $name {}

        impl CuTask for $name {
    type Resources<'r> = ();
            type Input<'m> = input_msg!('m, $($p),*);
            type Output<'m> = output_msg!(($(
                cu29::payload::CuArray<$p, { $mos }>
            ),*));

            fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
            where
                Self: Sized,
            {
                let config = config.ok_or_else(|| cu29::CuError::from("Config Missing"))?;
                let target_alignment_window: u64 =
                    config.get::<u32>("target_alignment_window_ms").ok_or_else(|| cu29::CuError::from("Missing target_alignment_window"))?.into();
                let stale_data_horizon: u64 =
                    config.get::<u32>("stale_data_horizon_ms").ok_or_else(|| cu29::CuError::from("Missing stale_data_horizon"))?.into();

                Ok(Self {
                    aligner: AlignmentBuffers::new(cu29_clock::CuDuration(target_alignment_window as u64 * 1_000_000),cu29_clock::CuDuration(stale_data_horizon as u64 * 1_000_000)),
                })
            }

            fn preprocess(&mut self, clock: &cu29_clock::RobotClock) -> CuResult<()> {
                self.aligner.purge(clock.now());
                Ok(())
            }

            fn process(
                &mut self,
                _clock: &cu29::clock::RobotClock,
                input: &Self::Input<'_>,
                output: &mut Self::Output<'_>,
            ) -> CuResult<()> {
                // add the incoming data into the buffers
                // input is a tuple of &CuMsg<T> for each T in the input
                paste::paste! {
                    $(
                        self.aligner.[<buffer $index>].push(input.$index.clone());
                    )*
                }


                let tuple_of_iters = self.aligner.get_latest_aligned_data();
                if tuple_of_iters.is_none() {
                    return Ok(());
                }

                // this is a tuple of iterators of CuStampedDataSet
                let tuple_of_iters = tuple_of_iters.unwrap();

                // Populate the CuArray fields in the output message
                let output_payload = output.payload_mut().get_or_insert_with(Default::default);
                $(
                    output_payload.$index.fill_from_iter(tuple_of_iters.$index.map(|msg| msg.payload().unwrap().clone()));
                )*
                Ok(())
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use cu29::CuResult;
    use cu29::config::ComponentConfig;
    use cu29::cutask::CuMsg;
    use cu29::cutask::CuTask;
    use cu29::cutask::Freezable;
    use cu29::cutask::{CuMsgMetadata, CuStampedData};
    use cu29::input_msg;
    use cu29::output_msg;
    use cu29::payload::CuArray;

    define_task!(AlignerTask, 0 => { 10, 5, f32 }, 1 => { 5, 10, i32 });
    #[test]
    fn test_aligner_smoketest() {
        let mut config = ComponentConfig::default();
        config.set("target_alignment_window_ms", 1000);
        config.set("stale_data_horizon_ms", 2000);
        let mut aligner = AlignerTask::new(Some(&config)).unwrap();
        let m1 = CuStampedData::<f32, CuMsgMetadata>::default();
        let m2 = CuStampedData::<i32, CuMsgMetadata>::default();
        let input: <AlignerTask as CuTask>::Input<'_> = (&m1, &m2);
        let m3 = CuStampedData::<(CuArray<f32, 5>, CuArray<i32, 10>), CuMsgMetadata>::default();
        let mut output: <AlignerTask as CuTask>::Output<'_> = m3;

        let clock = cu29::clock::RobotClock::new();
        let result = aligner.process(&clock, &input, &mut output);
        assert!(result.is_ok());
    }
    mod string_payload {
        use super::*;
        use cu29::clock::{CuDuration, Tov};

        define_task!(StringAlignerTask, 0 => { 4, 4, String }, 1 => { 4, 4, String });

        #[test]
        fn test_aligner_string_payload() {
            let mut config = ComponentConfig::default();
            config.set("target_alignment_window_ms", 10);
            config.set("stale_data_horizon_ms", 1000);
            let mut aligner = StringAlignerTask::new(Some(&config)).unwrap();

            let mut left = CuStampedData::<String, CuMsgMetadata>::new(Some("left".to_string()));
            let mut right = CuStampedData::<String, CuMsgMetadata>::new(Some("right".to_string()));
            let tov_time = CuDuration::from_millis(100);
            left.tov = Tov::Time(tov_time);
            right.tov = Tov::Time(tov_time);

            let input: <StringAlignerTask as CuTask>::Input<'_> = (&left, &right);
            let mut output =
                CuStampedData::<(CuArray<String, 4>, CuArray<String, 4>), CuMsgMetadata>::default();

            let clock = cu29::clock::RobotClock::new();
            aligner.process(&clock, &input, &mut output).unwrap();

            let payload = output.payload().unwrap();
            assert_eq!(payload.0.len(), 1);
            assert_eq!(payload.1.len(), 1);
            assert_eq!(payload.0.as_slice()[0], "left");
            assert_eq!(payload.1.as_slice()[0], "right");
        }
    }
}
