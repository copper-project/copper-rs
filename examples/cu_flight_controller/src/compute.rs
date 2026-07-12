use cu29::prelude::*;

mod tasks {
    use cu29::prelude::*;

    /// Compile-only source that keeps the compute subsystem graph valid until the
    /// real `cu_zed::Zed` source is wired in.
    #[derive(Reflect)]
    pub struct ComputeHeartbeatSource;

    impl Freezable for ComputeHeartbeatSource {}

    impl CuSrcTask for ComputeHeartbeatSource {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(u64);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            output.set_payload(ctx.cl_id());
            output.tov = Tov::Time(ctx.now());
            Ok(())
        }
    }

    /// Terminal placeholder for the compute-local graph.
    #[derive(Reflect)]
    pub struct ComputeHeartbeatSink;

    impl Freezable for ComputeHeartbeatSink {}

    impl CuSinkTask for ComputeHeartbeatSink {
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

#[copper_runtime(config = "multi_copper.ron", subsystem = "compute")]
struct ComputeApp {}

const LOG_SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn main() {
    if let Err(err) = drive() {
        eprintln!("quad-compute failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let mut app = ComputeApp::builder()
        .with_log_path("logs/compute.copper", LOG_SLAB_SIZE)?
        .build()?;
    app.run()
}
