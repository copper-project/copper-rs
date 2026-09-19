use cu29::config::SchedulingPolicy;
use cu29::config::read_configuration;
use cu29::planner::CuPlan;
use cu29::planner::CuPlanPlacement;
use cu29::planner::CuPlanWorker;
use cu29::prelude::*;
use std::env;
use std::fs;
use std::path::PathBuf;

fn worker(id: String, steps: Vec<u32>) -> CuPlanWorker {
    CuPlanWorker {
        id,
        placement: CuPlanPlacement::Thread {
            cpu: None,
            policy: SchedulingPolicy::Fair,
        },
        steps,
    }
}

fn main() -> CuResult<()> {
    let compute_workers = env::args()
        .nth(1)
        .ok_or_else(|| CuError::from("missing compute worker count"))?
        .parse::<u32>()
        .map_err(|error| CuError::new_with_cause("invalid compute worker count", error))?;
    if compute_workers == 0 || compute_workers > 32 {
        return Err(CuError::from("compute worker count must be within 1..=32"));
    }

    let output = env::args()
        .nth(2)
        .map(PathBuf::from)
        .ok_or_else(|| CuError::from("missing output path"))?;
    let config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("copperconfig.ron");
    let config = read_configuration(
        config_path
            .to_str()
            .ok_or_else(|| CuError::from("config path is not UTF-8"))?,
    )?;
    let mut plan = CuPlan::from_config_cyclic(&config, compute_workers)?;

    for mission in plan.missions.values_mut() {
        let mut source = Vec::new();
        let mut compute = vec![Vec::new(); compute_workers as usize];
        let mut terminal = Vec::new();

        for (index, step) in mission.steps.iter().enumerate() {
            let index = u32::try_from(index)
                .map_err(|error| CuError::new_with_cause("plan is too large", error))?;
            if step.key.contains("|task:src|") {
                source.push(index);
            } else if step.key.contains("|task:band_") {
                compute[step.copperlist as usize].push(index);
            } else {
                terminal.push(index);
            }
        }

        let mut workers = Vec::with_capacity(compute.len() + 2);
        workers.push(worker("source".to_string(), source));
        workers.extend(
            compute
                .into_iter()
                .enumerate()
                .map(|(index, steps)| worker(format!("compute_{index}"), steps)),
        );
        workers.push(worker("terminal".to_string(), terminal));
        mission.workers = workers;
    }

    plan.validate(&config)?;
    let serialized = plan.serialize_ron()?.replace("policy: Fair,", "");
    let fragment = format!(
        "(\n    runtime: (\n        planner: (\n            kind: ExplicitSchedule,\n            config: {{\n                \"plan\": {serialized},\n            }},\n        ),\n    ),\n)\n"
    );
    fs::write(&output, fragment)
        .map_err(|error| CuError::new_with_cause("failed to write schedule fragment", error))?;
    Ok(())
}
