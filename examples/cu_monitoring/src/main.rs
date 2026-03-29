use cu29::prelude::*;
use std::path::PathBuf;

pub mod tasks {
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct ExampleSrc {}

    impl Freezable for ExampleSrc {}

    impl CuSrcTask for ExampleSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            new_msg.set_payload(42);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct ExampleTask {}

    impl Freezable for ExampleTask {}

    impl CuTask for ExampleTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            output.set_payload(input.payload().unwrap() + 1);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct ExampleSink {}

    impl Freezable for ExampleSink {}

    impl CuSinkTask for ExampleSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
    }
}

struct ExampleMonitor {
    components: &'static [MonitorComponentMetadata],
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

impl CuMonitor for ExampleMonitor {
    fn new(metadata: CuMonitoringMetadata, _runtime: CuMonitoringRuntime) -> CuResult<Self> {
        debug!("Monitoring: created: mission={}", metadata.mission_id());
        Ok(Self {
            components: metadata.components(),
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!("Monitoring: started: {}", ctx.now());
        Ok(())
    }

    fn process_copperlist(&self, _ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
        debug!("Monitoring: Processing copperlist...");
        for entry in view.entries() {
            let component_name = self.components[entry.component_id.index()].id();
            debug!(
                "Component: {} (slot {}) -> {}",
                component_name,
                entry.culist_slot.index(),
                entry.msg
            );
        }
        Ok(())
    }

    fn process_error(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        error: &CuError,
    ) -> Decision {
        let component_name = self.components[component_id.index()].id();
        debug!(
            "Monitoring: Processing error component: {} step: {} error: {}",
            component_name, step, error
        );
        Decision::Ignore
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!("Monitoring: stopped: {}", ctx.now());
        Ok(())
    }
}
const SLAB_SIZE: Option<usize> = None;
fn main() {
    let logger_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/monitor.copper");

    debug!("Logger created at {}.", path = &logger_path);
    debug!("Creating application... ");
    let mut application = App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime");
    debug!("Running... starting clock: {}.", application.clock().now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application.run().expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    debug!("End of program: {}.", application.clock().now());
}
