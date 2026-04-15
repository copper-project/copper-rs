use cu29::prelude::*;
use std::fs;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "multi_copper.ron", subsystem = "robot")]
struct App {}

#[derive(Reflect)]
struct CalibrationReporter {
    label: String,
    gyro_bias: [f64; 3],
}

impl Freezable for CalibrationReporter {}

impl CuSrcTask for CalibrationReporter {
    type Output<'m> = output_msg!(i64);
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config =
            config.ok_or_else(|| CuError::from("CalibrationReporter requires configuration"))?;
        let label = config
            .get::<String>("label")?
            .unwrap_or_else(|| "base-robot".to_string());
        let gyro_bias = config
            .get_value::<[f64; 3]>("gyro_bias")?
            .unwrap_or([0.0, 0.0, 0.0]);
        Ok(Self { label, gyro_bias })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        println!(
            "runtime task: instance_id={} label={} gyro_bias={:?}",
            ctx.instance_id(),
            self.label.as_str(),
            self.gyro_bias
        );
        info!(
            "instance_id={} label={} gyro_bias={:?}",
            ctx.instance_id(),
            self.label.as_str(),
            self.gyro_bias
        );
        output.set_payload(i64::from(ctx.instance_id()));
        output.tov = Tov::Time(ctx.now());
        Ok(())
    }
}

fn manifest_path(path: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
}

fn parse_instance_id() -> CuResult<u32> {
    let mut instance_id = 0u32;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--instance-id" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --instance-id"))?;
                instance_id = value.parse::<u32>().map_err(|err| {
                    CuError::from(format!("invalid --instance-id '{value}'"))
                        .add_cause(err.to_string().as_str())
                })?;
            }
            other => {
                return Err(CuError::from(format!(
                    "unsupported argument '{other}', expected --instance-id"
                )));
            }
        }
    }
    Ok(instance_id)
}

fn print_effective_config(instance_id: u32) -> CuResult<()> {
    let multi_path = manifest_path("multi_copper.ron");
    let multi = read_multi_configuration(
        multi_path
            .to_str()
            .ok_or_else(|| CuError::from("invalid multi_copper.ron path"))?,
    )?;
    let config = multi.resolve_subsystem_config_for_instance("robot", instance_id)?;
    let graph = config.get_graph(None)?;
    let reporter_id = graph
        .get_node_id_by_name("reporter")
        .ok_or_else(|| CuError::from("reporter task missing from effective config"))?;
    let reporter = graph
        .get_node(reporter_id)
        .ok_or_else(|| CuError::from("reporter task weight missing"))?;
    let reporter_cfg = reporter
        .get_instance_config()
        .ok_or_else(|| CuError::from("reporter config missing"))?;
    let label = reporter_cfg
        .get::<String>("label")?
        .unwrap_or_else(|| "base-robot".to_string());
    let gyro_bias = reporter_cfg
        .get_value::<[f64; 3]>("gyro_bias")?
        .unwrap_or([0.0, 0.0, 0.0]);

    println!("effective config: instance_id={instance_id} label={label} gyro_bias={gyro_bias:?}");
    Ok(())
}

fn main() {
    if let Err(err) = drive() {
        eprintln!("cu-instance-overrides-demo failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let instance_id = parse_instance_id()?;
    print_effective_config(instance_id)?;

    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).map_err(|err| {
        CuError::from("failed to switch to example directory").add_cause(err.to_string().as_str())
    })?;

    let logger_path = manifest_path(&format!("logs/instance_{instance_id}.copper"));
    if let Some(parent) = logger_path.parent() {
        fs::create_dir_all(parent)
            .map_err(|err| CuError::new_with_cause("failed to create log directory", err))?;
    }

    let mut app = App::builder()
        .with_log_path(&logger_path, None)?
        .with_instance_id(instance_id)
        .build()?;
    app.start_all_tasks()?;
    app.run_one_iteration()?;
    app.stop_all_tasks()?;
    Ok(())
}
