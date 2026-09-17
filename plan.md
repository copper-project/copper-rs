 # Replace parallel-rt Through PlannerKind::Pipeline, Then Generalize

  ## Summary

  Start with behavioral parity: Pipeline mechanically places each
  process stage on its own ordered lane and replaces the current
  implicit parallel-rt executor. It performs no optimization or PGO.
  Once that path is stable, generalize the same lane executor for
  stateless overlap and explicit schedules.

  ## PR Series

  1. [x] feat(runtime): replace implicit parallel-rt with Pipeline planning
      - Add PlannerKind::{Serial, TaskOrder, Pipeline}; omitted planner
        remains serial.

      - Select threaded execution only with:

        runtime: (
            planner: (
                kind: Pipeline,
                config: { "max_in_flight": 2 },
            ),
        )

      - Keep parallel-rt as the Cargo capability gate. A Pipeline config
        without it fails at compile time; enabling the feature alone no
        longer changes execution.

      - Pipeline creates one lane per process stage in graph order,
        keeps anytime refinements with their base stage, and preserves a
        total within-CopperList order.

      - Default max_in_flight to logging.copperlist_count; reject zero
        or excess capacity.

      - Derive lane affinity and policy from the rt thread-pool entry,
        distributing stages round-robin across configured CPUs.

      - Replace the old queue-chain executor with fixed lane
        coordination while retaining current CopperList storage and
        keyframe locking for this parity PR.

      - Remove the string-selected CuPlanner -> StepOrder/emit_plan
        extension path. Convert the custom-planner example into an
        offline tool that writes a TaskOrder config.

      - Migrate existing parallel-rt examples to conditional Pipeline
        configuration and prove their output matches the old executor.

  2. [ ] feat(runtime): add stateless transform tasks
      - Add CuStatelessTask and kind: stateless_task.
      - Use immutable per-CopperList callbacks with exclusive mutable
        lifecycle calls and require Send + Sync.

      - Execute stateless tasks normally in Serial and Pipeline plans;
        initially keep one component on one lane.

      - Reject background and anytime combinations.

  3. [ ] refactor(runtime): preallocate concurrent CopperList and keyframe
     storage
      - Introduce a fixed CopperList arena with move-only slot leases.
      - Give each slot preallocated component keyframe regions and
        encode them directly in restore order.

      - Return slots only after CopperList and keyframe output complete.
      - Preserve wire/log/replay formats and add no hot-path allocation
        or payload copying.

      - Remove the global keyframe capture bottleneck so different
        CopperLists can safely progress concurrently.

  4. [ ] feat(planner): add portable exact schedules
      - Introduce version-1 CuPlan and the replacement
        CuPlanner::plan(&PlanningInput) -> CuPlan.

      - Add PlannerKind::ExplicitSchedule; make Pipeline emit the same
        public plan representation.

      - Generalize lane assignments from one-stage workers to arbitrary
        ordered workers and cross-worker dependencies.

      - Validate complete assignment, data and anytime precedence, task/
        bridge/resource recurrence, capacity, placement, cycles,
        missions, and graph identity.

      - Permit overlapping invocations only for CuStatelessTask or
        resources explicitly declared concurrent.

      - Preserve sampled background behavior and deterministic lag: 1;
        reject unsupported background concurrency.

      - Keep simulation, replay, and run_one_iteration() sequential over
        the validated schedule.

  5. [ ] feat(planner): add exact-plan tooling and documentation
      - Add plan-export, plan-validate, and plan-import with matching
        just recipes.

      - Author standalone plan.ron, then validate and embed it into a
        generated Copper config for compile-time code generation.

      - Extend cu_runtime_matrix with Pipeline, serial, and explicit-
        schedule fixtures.

      - Document Pipeline as the direct parallel-rt migration path and
        ExplicitSchedule as the advanced path.

      - Keep PGO, automatic candidate search, AUR, benchmark datasets,
        and sibling wiki/book changes outside this series.

  ## Behavioral Contract

  - parallel-rt compiles threaded support; PlannerKind::Pipeline selects
    it.

  - Pipeline performs deterministic mechanical placement, with no cost
    model or runtime scheduling.

  - Serial remains the default and retains current behavior.
  - Pipeline and the equivalent exported ExplicitSchedule must compile
    to identical lane schedules.

  - CopperLists commit and log strictly by id; failures never publish
    partial lists.

  - All planning, validation, and indexing happen at compile time.

  ## Verification

  - Compare the new Pipeline executor against the current parallel-rt
    path for payload digests, CopperList ids, logs, keyframes, replay,
    errors, shutdown, and slot wraparound.

  - Cover one/multiple stages, one/multiple in-flight lists, bridges,
    joins, anytime/background tasks, task errors, worker panics, and
    stop requests.

  - Assert no post-start worker, dispatcher, or keyframe allocation.
  - Run focused tests and just fmt per PR; run just lint, just api-
    check, and just nostd-ci for shared API changes.

  - Before the final PR, run just pr-check and the Scofield coordination
    harness externally, requiring semantic equivalence and no material
    performance regression.

  ## Assumptions

  - Reimplement from current master; do not transplant prototype
    commits.

  - PR 1 is the first usable milestone and directly replaces current
    parallel-rt behavior.

  - The PRs are sequential and rebased after dependencies merge.
  - Breaking the recent planner API/config is accepted.
