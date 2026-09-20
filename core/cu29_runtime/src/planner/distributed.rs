//! Build-time coordination derived from a validated worker schedule.

use super::AssembledPlan;
use super::CuPlanPlacement;
use super::LaneOccurrence;
use super::LanePlan;
use super::LaneWorker;
use crate::curuntime::CuExecutionLoop;
use crate::curuntime::CuExecutionStep;
use crate::curuntime::CuExecutionUnit;
use crate::curuntime::CuStepPhase;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::vec;
use alloc::vec::Vec;
use cu29_traits::CuError;
use cu29_traits::CuResult;

/// A resolved periodic schedule. All analysis and allocation happen at build time.
#[doc(hidden)]
#[derive(Clone, Debug)]
pub struct DistributedSchedule {
    pub plan: LanePlan,
    pub storage_slots: usize,
    pub async_output: bool,
    pub stage_of_step: Vec<Option<usize>>,
    pub stages: usize,
    pub base_of: Vec<Option<usize>>,
    pub previous_phase: Vec<Option<usize>>,
    pub dependencies: Vec<(usize, usize, u32)>,
    pub reference_dependencies: Vec<(usize, usize, u32)>,
    pub owner: Vec<usize>,
    pub finish_counts: Vec<u32>,
    pub publish: Vec<bool>,
    pub progress: Vec<u64>,
    pub worker_lengths: Vec<u64>,
    pub completion: Vec<Vec<(usize, u64)>>,
}

impl DistributedSchedule {
    /// Normalize a resolved serial or fixed worker plan without changing its order.
    pub fn from_assembled(assembled: &AssembledPlan) -> CuResult<Self> {
        let plan = assembled.lanes.clone().unwrap_or_else(|| LanePlan {
            copperlists_per_cycle: 1,
            max_in_flight: 1,
            occurrences: (0..assembled.execution.steps.len())
                .map(|step| LaneOccurrence {
                    step,
                    copperlist: 0,
                })
                .collect(),
            workers: vec![LaneWorker {
                id: "main".into(),
                placement: CuPlanPlacement::Main,
                occurrences: (0..assembled.execution.steps.len()).collect(),
            }],
            dispatcher: None,
            dependencies: Vec::new(),
        });
        Self::compile(&plan, &assembled.execution)
    }

    pub fn is_serial(&self) -> bool {
        self.plan.workers.len() == 1
            && self.plan.max_in_flight == 1
            && self.plan.copperlists_per_cycle == 1
            && self.plan.workers[0].placement == CuPlanPlacement::Main
    }

    pub fn isolate_outputs(&self) -> bool {
        self.plan.workers.len() > 1
    }

    pub fn slot_alignment(&self) -> usize {
        if self.is_serial() { 1 } else { 128 }
    }

    pub fn storage_reuse_slots(&self) -> usize {
        if self.async_output {
            self.storage_slots
        } else {
            self.plan.max_in_flight as usize
        }
    }

    pub fn with_async_output(mut self, enabled: bool) -> Self {
        self.async_output = enabled;
        self
    }

    pub fn with_storage_slots(mut self, slots: usize) -> CuResult<Self> {
        if slots < self.plan.max_in_flight as usize {
            return Err(CuError::from(
                "Arena capacity is smaller than schedule concurrency",
            ));
        }
        self.storage_slots = slots;
        Ok(self)
    }

    /// Compile a structurally validated plan into executable coordination.
    pub fn compile(plan: &LanePlan, execution: &CuExecutionLoop) -> CuResult<Self> {
        let steps: Vec<&CuExecutionStep> = execution
            .steps
            .iter()
            .map(|unit| match unit {
                CuExecutionUnit::Step(step) => Ok(step.as_ref()),
                CuExecutionUnit::Loop(_) => Err(CuError::from(
                    "Execution loops are not supported by the worker executor",
                )),
            })
            .collect::<CuResult<_>>()?;
        if plan.workers.len() == 1
            && plan.max_in_flight == 1
            && plan.copperlists_per_cycle == 1
            && plan.workers[0].placement == CuPlanPlacement::Main
        {
            let count = plan.occurrences.len();
            let mut finish_counts = vec![0; count];
            if let Some(&last) = plan.workers[0].occurrences.last() {
                finish_counts[last] = count as u32;
            }
            return Ok(Self {
                storage_slots: 1,
                async_output: false,
                plan: plan.clone(),
                stage_of_step: (0..steps.len()).map(Some).collect(),
                stages: steps.len(),
                base_of: vec![None; count],
                previous_phase: vec![None; count],
                dependencies: Vec::new(),
                reference_dependencies: Vec::new(),
                owner: vec![0; count],
                finish_counts,
                publish: vec![false; count],
                progress: (1..=count as u64).collect(),
                worker_lengths: vec![count as u64],
                completion: vec![vec![(0, count as u64)]],
            });
        }

        let mut stage_of_step = Vec::with_capacity(steps.len());
        let mut next_stage = 0usize;
        for step in &steps {
            if step.phase == CuStepPhase::AnytimeRefine {
                stage_of_step.push(None);
            } else {
                stage_of_step.push(Some(next_stage));
                next_stage += 1;
            }
        }
        let occurrences = &plan.occurrences;
        let occurrence_at = |step: usize, copperlist: u32| {
            occurrences.iter().position(|occurrence| {
                occurrence.step == step && occurrence.copperlist == copperlist
            })
        };
        let mut base_of = vec![None; occurrences.len()];
        let mut previous_phase = vec![None; occurrences.len()];
        for (index, occurrence) in occurrences.iter().enumerate() {
            let step = steps[occurrence.step];
            if step.phase != CuStepPhase::AnytimeRefine {
                continue;
            }
            let base_step = (0..occurrence.step).rev().find(|&candidate| {
                steps[candidate].node_id == step.node_id
                    && steps[candidate].phase == CuStepPhase::AnytimeBase
            });
            let previous_step = (0..occurrence.step)
                .rev()
                .find(|&candidate| steps[candidate].node_id == step.node_id);
            let (Some(base_step), Some(previous_step)) = (base_step, previous_step) else {
                return Err(CuError::from(format!(
                    "Worker executor: refine step of task '{}' has no base step",
                    step.node.get_id()
                )));
            };
            base_of[index] = occurrence_at(base_step, occurrence.copperlist);
            previous_phase[index] = occurrence_at(previous_step, occurrence.copperlist);
        }

        let resolve = |index: usize| base_of[index].unwrap_or(index);
        let mut dependencies = Vec::new();
        for edge in &plan.dependencies {
            let (from, to, lag) = (edge.from as usize, edge.to as usize, edge.cycle_lag);
            if base_of[to].is_some() {
                if previous_phase[to] == Some(from) && lag == 0 {
                    continue;
                }
                let occurrence = occurrences[to];
                return Err(CuError::from(format!(
                    "Worker executor: refine step of task '{}' (CL {}) may only depend on its previous phase",
                    steps[occurrence.step].node.get_id(),
                    occurrence.copperlist
                )));
            }
            dependencies.push((resolve(from), to, lag));
        }

        let mut owner = vec![usize::MAX; occurrences.len()];
        let mut finish_counts = vec![0u32; occurrences.len()];
        for (worker_id, worker) in plan.workers.iter().enumerate() {
            let mut last_and_count = BTreeMap::new();
            for &index in &worker.occurrences {
                owner[index] = worker_id;
                if base_of[index].is_none() {
                    let entry = last_and_count
                        .entry(occurrences[index].copperlist)
                        .or_insert((index, 0u32));
                    entry.0 = index;
                    entry.1 += 1;
                }
            }
            for (_, (last, count)) in last_and_count {
                finish_counts[last] = count;
            }
        }
        let reference_dependencies = dependencies.clone();
        dependencies.retain(|&(from, to, _)| owner[from] != owner[to]);
        let mut publish = vec![false; occurrences.len()];
        for &(from, _, _) in &dependencies {
            publish[from] = true;
        }
        for worker in &plan.workers {
            let mut previous = None;
            for &index in &worker.occurrences {
                if base_of[index].is_some() && previous_phase[index] != previous {
                    return Err(CuError::from(format!(
                        "Worker executor: refine step of task '{}' (CL {}) must directly follow its previous phase on worker '{}'",
                        steps[occurrences[index].step].node.get_id(),
                        occurrences[index].copperlist,
                        worker.id
                    )));
                }
                previous = Some(index);
            }
        }

        let mut progress = vec![0; occurrences.len()];
        let mut worker_lengths = vec![0; plan.workers.len()];
        let mut completion = vec![Vec::new(); plan.copperlists_per_cycle as usize];
        for (worker_id, worker) in plan.workers.iter().enumerate() {
            for &index in &worker.occurrences {
                if base_of[index].is_none() {
                    worker_lengths[worker_id] += 1;
                    progress[index] = worker_lengths[worker_id];
                    if finish_counts[index] != 0 {
                        completion[occurrences[index].copperlist as usize]
                            .push((worker_id, progress[index]));
                    }
                }
            }
        }
        Ok(Self {
            storage_slots: plan.max_in_flight as usize,
            async_output: false,
            plan: plan.clone(),
            stage_of_step,
            stages: next_stage,
            base_of,
            previous_phase,
            dependencies,
            reference_dependencies,
            owner,
            finish_counts,
            publish,
            progress,
            worker_lengths,
            completion,
        })
    }
}
