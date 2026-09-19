//! Mechanical stage placement using the shared schedule representation.

use super::CuPlan;
use super::CuPlanDependency;
use super::CuPlanPlacement;
use super::CuPlanWorker;
use super::CuPlanner;
use super::PlanningInput;
use super::Serial;
use super::assemble_runtime_plan_with_planner;
use super::mission_graphs;
use super::schedule::PlanShape;
use crate::config::ComponentConfig;
use crate::config::RT_POOL;
use crate::config::SchedulingPolicy;
use crate::curuntime::CuExecutionUnit;
use crate::curuntime::CuStepPhase;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::vec;
use alloc::vec::Vec;
use cu29_traits::CuError;
use cu29_traits::CuResult;

/// Place each process stage on a dedicated worker in graph order.
pub struct Pipeline {
    max_in_flight: Option<u32>,
}

impl Pipeline {
    pub(super) fn new(params: Option<&ComponentConfig>) -> CuResult<Self> {
        let max_in_flight = params
            .map(|params| params.get_value::<u32>("max_in_flight"))
            .transpose()
            .map_err(|error| CuError::from(format!("Pipeline: {error}")))?
            .flatten();
        Ok(Self { max_in_flight })
    }
}

impl CuPlanner for Pipeline {
    fn plan(&self, input: &PlanningInput<'_>) -> CuResult<CuPlan> {
        let config = input.config;
        let capacity = config
            .logging
            .as_ref()
            .and_then(|logging| logging.copperlist_count)
            .unwrap_or(super::DEFAULT_COPPERLIST_COUNT);
        let max_in_flight = self.max_in_flight.unwrap_or(
            u32::try_from(capacity).map_err(|_| CuError::from("Pipeline capacity exceeds u32"))?,
        );
        let pool = config
            .runtime
            .as_ref()
            .and_then(|runtime| runtime.thread_pools.iter().find(|pool| pool.id == RT_POOL));
        let mut missions = BTreeMap::new();
        for (mission, graph) in mission_graphs(config) {
            let mut assembled = assemble_runtime_plan_with_planner(config, graph, &Serial)?;
            let mut groups = BTreeMap::new();
            let mut order = Vec::new();
            for unit in core::mem::take(&mut assembled.execution.steps) {
                let CuExecutionUnit::Step(step) = &unit else {
                    return Err(CuError::from("Pipeline requires process steps"));
                };
                if step.phase != CuStepPhase::AnytimeRefine {
                    order.push(step.node_id);
                }
                groups
                    .entry(step.node_id)
                    .or_insert_with(Vec::new)
                    .push(unit);
            }
            assembled.execution.steps = order
                .into_iter()
                .flat_map(|node| groups.remove(&node).expect("stage group exists"))
                .collect();
            let mut plan =
                PlanShape::new(&assembled, config, graph, &mission, &[])?.serial_plan()?;
            plan.max_in_flight = max_in_flight;
            let mut workers: Vec<CuPlanWorker> = Vec::new();
            for (index, unit) in assembled.execution.steps.iter().enumerate() {
                let CuExecutionUnit::Step(step) = unit else {
                    return Err(CuError::from("Pipeline requires process steps"));
                };
                let occurrence = u32::try_from(index)
                    .map_err(|_| CuError::from("Pipeline has too many steps"))?;
                if step.phase == CuStepPhase::AnytimeRefine {
                    workers
                        .last_mut()
                        .ok_or_else(|| CuError::from("Refinement has no base stage"))?
                        .steps
                        .push(occurrence);
                } else {
                    let worker = workers.len();
                    let cpu = pool
                        .and_then(|pool| pool.affinity.as_ref())
                        .filter(|cpus| !cpus.is_empty())
                        .map(|cpus| cpus[worker % cpus.len()]);
                    workers.push(CuPlanWorker {
                        id: format!("stage_{worker}"),
                        placement: CuPlanPlacement::Thread {
                            cpu,
                            policy: pool.map_or(SchedulingPolicy::Fair, |pool| pool.policy),
                        },
                        steps: vec![occurrence],
                    });
                }
                if occurrence > 0 {
                    let edge = CuPlanDependency {
                        from: occurrence - 1,
                        to: occurrence,
                        cycle_lag: 0,
                    };
                    if !plan.dependencies.contains(&edge) {
                        plan.dependencies.push(edge);
                    }
                }
            }
            plan.workers = workers;
            missions.insert(mission, plan);
        }
        let plan = CuPlan {
            concurrent_resources: Vec::new(),
            missions,
        };
        plan.validate(config)?;
        Ok(plan)
    }
}
