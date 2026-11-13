#![doc = include_str!("../README.md")]

pub mod buffers;

/// Define a task that aligns incoming messages based on their timestamps
/// See module doc for use.
#[macro_export]
macro_rules! define_task {
    ($name:ident, $($index:tt => { $mis:expr, $mos:expr, $p:ty }),+) => {

       paste::paste! {
            #[allow(unused_imports)]
            use cu29::prelude::*;

            $crate::buffers::alignment_buffers!(
                AlignmentBuffers,
                $(
                    [<buffer $index>]: TimeboundCircularBuffer<$mis, CuStampedData<$p, CuMsgMetadata>>
                ),*
            );
        }

        #[derive(Reflect)]
        #[reflect(from_reflect = false)]
        pub struct $name {
            #[reflect(ignore)]
            aligner: AlignmentBuffers,
        }

        impl Freezable for $name {
            fn freeze<E: cu29::bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), cu29::bincode::error::EncodeError> {
                self.aligner.freeze(encoder)
            }

            fn thaw<D: cu29::bincode::de::Decoder>(&mut self, decoder: &mut D) -> Result<(), cu29::bincode::error::DecodeError> {
                self.aligner.thaw(decoder)
            }
        }

        impl CuTask for $name {
    type Resources<'r> = ();
            type Input<'m> = input_msg!('m, $($p),*);
            type Output<'m> = output_msg!(($(
                CuArray<CuMsg<$p>, { $mos }>
            ),*));

            fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
            where
                Self: Sized,
            {
                let config = config.ok_or_else(|| CuError::from("Config Missing"))?;
                let target_alignment_window_ms: u64 = config
                    .get::<u32>("target_alignment_window_ms")?
                    .ok_or_else(|| CuError::from("Missing target_alignment_window"))?
                    .into();
                let stale_data_horizon_ms: u64 = config
                    .get::<u32>("stale_data_horizon_ms")?
                    .ok_or_else(|| CuError::from("Missing stale_data_horizon"))?
                    .into();

                let target_alignment_window =
                    cu29_clock::CuDuration(target_alignment_window_ms * 1_000_000);
                let stale_data_horizon =
                    cu29_clock::CuDuration(stale_data_horizon_ms * 1_000_000);

                Ok(Self {
                    aligner: AlignmentBuffers::new(target_alignment_window, stale_data_horizon),
                })
            }

            fn preprocess(&mut self, ctx: &CuContext) -> CuResult<()> {
                self.aligner.purge(ctx.now());
                Ok(())
            }

            fn process(
                &mut self,
                _ctx: &CuContext,
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


                // this is a tuple of iterators of CuStampedDataSet
                let Some(tuple_of_iters) = self.aligner.get_latest_aligned_data() else {
                    return Ok(());
                };

                // Populate the CuArray fields in the output message
                let output_payload = output.payload_mut().get_or_insert_with(Default::default);
                $(
                    output_payload.$index.fill_from_iter(tuple_of_iters.$index.cloned());
                )*
                Ok(())
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use cu29::prelude::*;

    define_task!(AlignerTask, 0 => { 10, 5, f32 }, 1 => { 5, 10, i32 });
    #[test]
    fn test_aligner_smoketest() {
        let mut config = ComponentConfig::default();
        config.set("target_alignment_window_ms", 1000);
        config.set("stale_data_horizon_ms", 2000);
        let mut aligner = AlignerTask::new(Some(&config), ()).unwrap();
        let m1 = CuStampedData::<f32, CuMsgMetadata>::default();
        let m2 = CuStampedData::<i32, CuMsgMetadata>::default();
        let input: <AlignerTask as CuTask>::Input<'_> = (&m1, &m2);
        let m3 = CuStampedData::<(CuArray<CuMsg<f32>, 5>, CuArray<CuMsg<i32>, 10>), CuMsgMetadata>::default();
        let mut output: <AlignerTask as CuTask>::Output<'_> = m3;

        let ctx = CuContext::new_with_clock();
        let result = aligner.process(&ctx, &input, &mut output);
        assert!(result.is_ok());
    }
    #[test]
    fn test_aligner_preserves_messages_and_serialization() {
        let mut config = ComponentConfig::default();
        config.set("target_alignment_window_ms", 20);
        config.set("stale_data_horizon_ms", 1000);
        let mut aligner = AlignerTask::new(Some(&config), ()).unwrap();
        let ctx = CuContext::new_with_clock();
        let mut output = <AlignerTask as CuTask>::Output::default();

        let mut left = CuMsg::new(Some(1.5_f32));
        left.tov = Tov::Time(CuTime::from_millis(90));
        left.metadata.process_time.start = CuTime::from_millis(91).into();
        left.metadata.process_time.end = CuTime::from_millis(92).into();
        left.metadata.set_status("left sample");
        left.metadata.set_origin(CuMsgOrigin {
            subsystem_code: 1,
            instance_id: 2,
            cl_id: 3,
        });
        let mut right = CuMsg::new(Some(42_i32));
        right.tov = Tov::Range(CuTimeRange {
            start: CuTime::from_millis(85),
            end: CuTime::from_millis(89),
        });
        right.metadata.process_time.start = CuTime::from_millis(93).into();
        right.metadata.process_time.end = CuTime::from_millis(94).into();
        right.metadata.set_status("right sample");
        right.metadata.set_origin(CuMsgOrigin {
            subsystem_code: 4,
            instance_id: 5,
            cl_id: 6,
        });
        aligner
            .process(&ctx, &(&left, &right), &mut output)
            .unwrap();
        // Advance both streams so the window includes the earlier point and range.
        let mut next_left = CuMsg::new(Some(2.5_f32));
        next_left.tov = Tov::Time(CuTime::from_millis(100));
        let mut next_right = CuMsg::new(Some(43_i32));
        next_right.tov = next_left.tov;
        aligner
            .process(&ctx, &(&next_left, &next_right), &mut output)
            .unwrap();

        let config = cu29::bincode::config::standard();
        let bytes = cu29::bincode::encode_to_vec(&output, config).unwrap();
        let (decoded, consumed): (<AlignerTask as CuTask>::Output<'_>, usize) =
            cu29::bincode::decode_from_slice(&bytes, config).unwrap();
        assert_eq!(consumed, bytes.len());
        for output in [&output, &decoded] {
            let (left_window, right_window) = output.payload().unwrap();
            assert_eq!(left_window.len(), 2);
            assert_eq!(right_window.len(), 2);
            for (actual, expected) in left_window.as_slice().iter().zip([&left, &next_left]) {
                assert_eq!(
                    cu29::bincode::encode_to_vec(actual, config).unwrap(),
                    cu29::bincode::encode_to_vec(expected, config).unwrap(),
                );
            }
            for (actual, expected) in right_window.as_slice().iter().zip([&right, &next_right]) {
                assert_eq!(
                    cu29::bincode::encode_to_vec(actual, config).unwrap(),
                    cu29::bincode::encode_to_vec(expected, config).unwrap(),
                );
            }
        }
    }

    mod string_payload {
        use super::*;

        define_task!(StringAlignerTask, 0 => { 4, 4, String }, 1 => { 4, 4, String });

        #[test]
        fn test_aligner_string_payload() {
            let mut config = ComponentConfig::default();
            config.set("target_alignment_window_ms", 10);
            config.set("stale_data_horizon_ms", 1000);
            let mut aligner = StringAlignerTask::new(Some(&config), ()).unwrap();

            let mut left = CuStampedData::<String, CuMsgMetadata>::new(Some("left".to_string()));
            let mut right = CuStampedData::<String, CuMsgMetadata>::new(Some("right".to_string()));
            let tov_time = CuTime::from_millis(100);
            left.tov = Tov::Time(tov_time);
            right.tov = Tov::Time(tov_time);

            let input: <StringAlignerTask as CuTask>::Input<'_> = (&left, &right);
            let mut output = CuStampedData::<
                (CuArray<CuMsg<String>, 4>, CuArray<CuMsg<String>, 4>),
                CuMsgMetadata,
            >::default();

            let ctx = CuContext::new_with_clock();
            aligner.process(&ctx, &input, &mut output).unwrap();

            let payload = output.payload().unwrap();
            assert_eq!(payload.0.len(), 1);
            assert_eq!(payload.1.len(), 1);
            assert_eq!(
                payload.0.as_slice()[0].payload().map(String::as_str),
                Some("left")
            );
            assert_eq!(
                payload.1.as_slice()[0].payload().map(String::as_str),
                Some("right")
            );
        }
    }
}
