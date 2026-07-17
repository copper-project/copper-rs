#[cfg(test)]
mod tests {
    mod tasks {
        use cu29::prelude::*;

        #[derive(Reflect)]
        pub struct Source;

        impl Freezable for Source {}

        impl CuSrcTask for Source {
            type Resources<'r> = ();
            type Output<'m> = output_msg!(u64);

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                Ok(Self)
            }

            fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
                output.set_payload(1);
                Ok(())
            }
        }

        #[derive(Reflect)]
        pub struct Sink;

        impl Freezable for Sink {}

        impl CuSinkTask for Sink {
            type Resources<'r> = ();
            type Input<'m> = input_msg!(u64);

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self> {
                Ok(Self)
            }

            fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
                Ok(())
            }
        }
    }

    use cu29::prelude::*;

    #[copper_runtime(config = "tests/forwarding/main.ron")]
    struct FeatureForwardingApp {}

    #[test]
    fn cargo_feature_reaches_the_runtime_macro() {
        let resolved = FeatureForwardingApp::original_config();

        assert!(resolved.contains("base_source"));
        if cfg!(feature = "jetson-mipi") {
            assert!(resolved.contains("feature_source"));
        } else {
            assert!(!resolved.contains("feature_source"));
        }
        assert!(!resolved.contains("Feature"));
    }
}
