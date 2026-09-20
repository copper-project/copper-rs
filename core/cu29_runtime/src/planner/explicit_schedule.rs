//! Portable exact process schedules and their build-time enforcement.

use super::AssembledPlan;
use super::CuMissionPlan;
use super::CuPlanner;
use super::LaneOccurrence;
use super::LanePlan;
use super::LaneWorker;
use super::PlanningInput;
use super::Serial;
use super::StepOrder;
use super::assemble_from_order;
use super::assemble_runtime_plan_with_planner;
use super::build_plan_graph;
use super::mission_graphs;
use super::schedule::PlanShape;
use super::step_key;
use crate::config::ComponentConfig;
use crate::config::CuConfig;
use crate::config::CuGraph;
use crate::config::NodeId;
use crate::config::PlannerConfig;
use crate::config::PlannerKind;
use crate::config::RuntimeConfig;
use crate::config::Value;
use crate::curuntime::CuExecutionUnit;
use crate::curuntime::CuStepPhase;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use cu29_traits::CuError;
use cu29_traits::CuResult;
use serde::Deserialize;
use serde::Serialize;

/// An experimental portable process schedule for every mission in an app.
///
/// Copper validates a plan against the static task graph and generates typed
/// calls and CopperList slots at compile time. The serialized shape is the
/// schema: incompatible additions or removals are rejected by serde.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuPlan {
    /// Resources (`bundle.resource`) that several components may use concurrently.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub concurrent_resources: Vec<String>,
    /// Schedule indexed by mission id (`default` without named missions).
    pub missions: BTreeMap<String, CuMissionPlan>,
}

impl CuPlan {
    /// Resolve and validate the scheduler selected by this configuration.
    pub fn from_config(config: &CuConfig) -> CuResult<Self> {
        super::resolve_schedule(config)
    }

    /// Export the canonical schedule for a repeating number of CopperLists.
    pub fn from_config_cyclic(config: &CuConfig, copperlists_per_cycle: u32) -> CuResult<Self> {
        if copperlists_per_cycle == 0 {
            return Err(CuError::from("copperlists_per_cycle must be positive"));
        }
        let mut missions = BTreeMap::new();
        for (mission, graph) in mission_graphs(config) {
            let assembled = assemble_runtime_plan_with_planner(config, graph, &Serial)?;
            missions.insert(
                mission.clone(),
                PlanShape::new(&assembled, config, graph, &mission, &[])?
                    .cyclic_plan(copperlists_per_cycle)?,
            );
        }
        Ok(Self {
            concurrent_resources: Vec::new(),
            missions,
        })
    }

    /// Validate inventory, placement, precedence, recurrence, and capacity.
    pub fn validate(&self, config: &CuConfig) -> CuResult<()> {
        self.validate_orders(config).map(|_| ())
    }

    fn validate_orders(&self, config: &CuConfig) -> CuResult<BTreeMap<String, Vec<usize>>> {
        let missions = mission_graphs(config);
        if !self.missions.keys().eq(missions.iter().map(|(id, _)| id)) {
            return Err(CuError::from(
                "Execution plan must contain exactly the configured missions",
            ));
        }
        let mut orders = BTreeMap::new();
        for (mission, graph) in missions {
            let canonical = assemble_runtime_plan_with_planner(config, graph, &Serial)?;
            let shape = PlanShape::new(
                &canonical,
                config,
                graph,
                &mission,
                &self.concurrent_resources,
            )?;
            let order = self.missions[&mission]
                .validate(config, &shape)
                .map_err(|error| CuError::from(format!("Plan for mission '{mission}': {error}")))?;
            orders.insert(mission, order);
        }
        Ok(orders)
    }

    /// Raise CopperList storage to the largest admission window in this plan.
    pub fn provide_capacity(&self, config: &mut CuConfig) {
        let needed = self
            .missions
            .values()
            .map(|mission| mission.max_in_flight as usize)
            .max()
            .unwrap_or(0);
        let current = config
            .logging
            .as_ref()
            .and_then(|logging| logging.copperlist_count)
            .unwrap_or(super::DEFAULT_COPPERLIST_COUNT);
        if needed > current {
            config
                .logging
                .get_or_insert_with(Default::default)
                .copperlist_count = Some(needed);
        }
    }

    /// Serialize this plan as human-editable RON.
    pub fn serialize_ron(&self) -> CuResult<String> {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
            .map_err(|error| CuError::new_with_cause("Could not serialize execution plan", error))
    }

    /// Parse a plan from RON.
    pub fn deserialize_ron(text: &str) -> CuResult<Self> {
        ron::from_str(text)
            .map_err(|error| CuError::new_with_cause("Could not parse execution plan", error))
    }

    /// Read a standalone plan file in host tooling or a build script.
    #[cfg(feature = "std")]
    pub fn read(path: &std::path::Path) -> CuResult<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|error| CuError::new_with_cause("Could not read execution plan", error))?;
        Self::deserialize_ron(&text)
    }

    /// Write a standalone plan file in host tooling or a build script.
    #[cfg(feature = "std")]
    pub fn write(&self, path: &std::path::Path) -> CuResult<()> {
        std::fs::write(path, self.serialize_ron()?)
            .map_err(|error| CuError::new_with_cause("Could not write execution plan", error))
    }
}

/// Enforce an exact saved process schedule during runtime generation.
#[derive(Clone, Debug)]
pub struct ExplicitSchedule {
    plan: CuPlan,
}

impl CuPlanner for ExplicitSchedule {
    fn plan(&self, _input: &PlanningInput<'_>) -> CuResult<CuPlan> {
        Ok(self.plan.clone())
    }
}

impl ExplicitSchedule {
    /// Take ownership of a saved plan.
    pub fn new(plan: CuPlan) -> Self {
        Self { plan }
    }

    /// Validate all missions and select this exact plan in `config`.
    pub fn apply(&self, config: &mut CuConfig) -> CuResult<()> {
        let mut prepared = config.clone();
        self.plan.provide_capacity(&mut prepared);
        self.plan.validate(&prepared)?;
        let value = cu29_value::to_value(&self.plan)
            .map_err(|error| CuError::new_with_cause("Could not encode execution plan", error))?;
        let value = Value::deserialize(value)
            .map_err(|error| CuError::new_with_cause("Could not embed execution plan", error))?;
        let mut params = ComponentConfig::default();
        params.set("plan", value);
        prepared
            .runtime
            .get_or_insert_with(RuntimeConfig::default)
            .planner = Some(PlannerConfig {
            kind: PlannerKind::ExplicitSchedule,
            config: Some(params),
        });
        *config = prepared;
        Ok(())
    }

    pub(super) fn assemble(
        &self,
        config: &CuConfig,
        graph: &CuGraph,
        mission: &str,
    ) -> CuResult<AssembledPlan> {
        let orders = self.plan.validate_orders(config)?;
        self.assemble_steps(config, graph, mission, &orders[mission])
            .map_err(|error| {
                CuError::from(format!(
                    "ExplicitSchedule plan for mission '{mission}': {error}"
                ))
            })
    }

    fn assemble_steps(
        &self,
        config: &CuConfig,
        graph: &CuGraph,
        mission: &str,
        order: &[usize],
    ) -> CuResult<AssembledPlan> {
        let plan = self
            .plan
            .missions
            .get(mission)
            .ok_or_else(|| CuError::from("Missing mission"))?;
        let requested = if plan.is_serial() {
            plan.serial_keys()?
        } else {
            plan.layout_keys(order)
        };
        let canonical = assemble_runtime_plan_with_planner(config, graph, &Serial)?;
        let keys = execution_keys(&canonical, mission)?;
        let by_key: BTreeMap<_, _> = keys.iter().enumerate().map(|(i, key)| (key, i)).collect();
        let mut node_order = Vec::with_capacity(canonical.entities.len());
        for key in &requested {
            let index = *by_key
                .get(key)
                .ok_or_else(|| CuError::from(format!("Unknown process step '{key}'")))?;
            let CuExecutionUnit::Step(step) = &canonical.execution.steps[index] else {
                return Err(CuError::from("Nested loops cannot be materialized"));
            };
            if step.phase != CuStepPhase::AnytimeRefine {
                node_order.push(step.node_id);
            }
        }
        let mut assembled =
            assemble_from_order(build_plan_graph(config, graph)?, StepOrder(node_order))?;
        let remapped_keys = execution_keys(&assembled, mission)?;
        let mut units: BTreeMap<_, _> = remapped_keys
            .into_iter()
            .zip(assembled.execution.steps)
            .collect();
        assembled.execution.steps = requested
            .iter()
            .map(|key| {
                units.remove(key).ok_or_else(|| {
                    CuError::from(format!("Could not materialize process step '{key}'"))
                })
            })
            .collect::<CuResult<Vec<_>>>()?;
        assembled.background = plan.background.clone();
        if !plan.is_serial() {
            let step_of: BTreeMap<_, _> = requested
                .iter()
                .enumerate()
                .map(|(index, key)| (key.as_str(), index))
                .collect();
            assembled.lanes = Some(LanePlan {
                copperlists_per_cycle: plan.copperlists_per_cycle,
                max_in_flight: plan.max_in_flight,
                occurrences: plan
                    .steps
                    .iter()
                    .map(|step| LaneOccurrence {
                        step: step_of[step.key.as_str()],
                        copperlist: step.copperlist,
                    })
                    .collect(),
                workers: plan
                    .workers
                    .iter()
                    .map(|worker| LaneWorker {
                        id: worker.id.clone(),
                        placement: worker.placement.clone(),
                        occurrences: worker.steps.iter().map(|&index| index as usize).collect(),
                    })
                    .collect(),
                dispatcher: plan.dispatcher.clone(),
                dependencies: plan.dependencies.clone(),
            });
        }
        Ok(assembled)
    }
}

pub(super) fn execution_keys(plan: &AssembledPlan, mission: &str) -> CuResult<Vec<String>> {
    let mut refines: BTreeMap<NodeId, u32> = BTreeMap::new();
    plan.execution
        .steps
        .iter()
        .map(|unit| {
            let CuExecutionUnit::Step(step) = unit else {
                return Err(CuError::from(
                    "Nested loops are not supported by portable plans",
                ));
            };
            let ordinal = if step.phase == CuStepPhase::AnytimeRefine {
                let next = refines.entry(step.node_id).or_default();
                *next += 1;
                Some(*next)
            } else {
                None
            };
            Ok(step_key(
                mission,
                &plan.entities[step.node_id as usize],
                step.phase,
                ordinal,
            ))
        })
        .collect()
}
