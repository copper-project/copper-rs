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
                    [<buffer $index>]: TimeboundCircularBuffer<$mis, CuMsg<$p>>
                ),*
            );
        }

        pub struct $name {
            aligner: AlignmentBuffers,
        }

        impl Freezable for $name {}

        impl<'cl> CuTask<'cl> for $name {
            type Input = input_msg!('cl, $($p),*);
            type Output = output_msg!('cl, ($(
                cu29::payload::CuArray<$p, { $mos }>
            ),*));

            fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
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
                input: Self::Input,
                output: Self::Output,
            ) -> CuResult<()> {
                // add the incoming data into the buffers
                // input is a tuple of &'cl CuMsg<T> for each T in the input
                paste::paste! {
                    $(
                        self.aligner.[<buffer $index>].push(input.$index.clone());
                    )*
                }


                let tuple_of_iters = self.aligner.get_latest_aligned_data();
                if tuple_of_iters.is_none() {
                    return Ok(());
                }

                // this is a tuple of iterators of CuMsgs
                let tuple_of_iters = tuple_of_iters.unwrap();

                // Populate the CuArray fields in the output message
                $(
                    output.payload_mut().as_mut().unwrap().$index.fill_from_iter(tuple_of_iters.$index.map(|msg| *msg.payload().unwrap()));
                )*
                Ok(())
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::define_task;
    use cu29::config::ComponentConfig;
    use cu29::cutask::CuMsg;
    use cu29::cutask::CuTask;
    use cu29::cutask::Freezable;
    use cu29::input_msg;
    use cu29::output_msg;
    use cu29::payload::CuArray;
    use cu29::CuResult;

    define_task!(AlignerTask, 0 => { 10, 5, f32 }, 1 => { 5, 10, i32 });

    #[test]
    fn test_aligner_smoketest() {
        let mut config = ComponentConfig::default();
        config.set("target_alignment_window_ms", 1000);
        config.set("stale_data_horizon_ms", 2000);
        let mut aligner = AlignerTask::new(Some(&config)).unwrap();
        let m1 = CuMsg::<f32>::default();
        let m2 = CuMsg::<i32>::default();
        let input: <AlignerTask as CuTask>::Input = (&m1, &m2);
        let mut m3 = CuMsg::<(CuArray<f32, 5>, CuArray<i32, 10>)>::default();
        let output: <AlignerTask as CuTask>::Output = &mut m3;

        let clock = cu29::clock::RobotClock::new();
        let result = aligner.process(&clock, input, output);
        assert!(result.is_ok());
    }
}
