//! Render explicit worker schedules as dependency columns without inventing timings.

use super::LABEL_WIDTH;
use super::MAX_VISIBLE_COPPERLISTS;
use super::RenderedSection;
use super::WAVE_CELL_GAP;
use super::WAVE_CELL_HEIGHT;
use super::WAVE_CELL_WIDTH;
use super::WAVE_COLUMNS;
use super::format_policy;
use super::truncate;
use super::xml;
use cu29_runtime::planner::CuMissionPlan;
use cu29_runtime::planner::CuPlanPlacement;
use cu29_traits::CuError;
use cu29_traits::CuResult;
use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::collections::VecDeque;
use std::fmt::Write;

struct Event {
    step: usize,
    copperlist: usize,
    worker: usize,
    ordinal: usize,
}

struct Layout {
    events: Vec<Event>,
    columns: Vec<usize>,
    copperlists: usize,
}

fn layout(plan: &CuMissionPlan) -> CuResult<Layout> {
    let copperlists = (plan.max_in_flight as usize).min(MAX_VISIBLE_COPPERLISTS);
    let per_cycle = plan.copperlists_per_cycle as usize;
    if per_cycle == 0 || copperlists == 0 {
        return Err(CuError::from(
            "Explicit schedule display requires a positive cycle and depth",
        ));
    }
    let cycles = copperlists.div_ceil(per_cycle);
    let mut events = Vec::new();
    let mut indices = BTreeMap::new();
    let mut edges = BTreeSet::new();
    for (worker_index, worker) in plan.workers.iter().enumerate() {
        let mut previous = None;
        let mut ordinal = 0;
        for cycle in 0..cycles {
            for &step in &worker.steps {
                let step = step as usize;
                let copperlist = cycle * per_cycle + plan.steps[step].copperlist as usize;
                if copperlist >= copperlists {
                    continue;
                }
                let index = events.len();
                indices.insert((cycle, step), index);
                events.push(Event {
                    step,
                    copperlist,
                    worker: worker_index,
                    ordinal,
                });
                if let Some(previous) = previous {
                    edges.insert((previous, index));
                }
                previous = Some(index);
                ordinal += 1;
            }
        }
    }
    for cycle in 0..cycles {
        for edge in &plan.dependencies {
            if let (Some(&from), Some(&to)) = (
                indices.get(&(cycle, edge.from as usize)),
                indices.get(&(cycle + edge.cycle_lag as usize, edge.to as usize)),
            ) {
                edges.insert((from, to));
            }
        }
    }
    let mut incoming = vec![0; events.len()];
    let mut successors = vec![Vec::new(); events.len()];
    for (from, to) in edges {
        incoming[to] += 1;
        successors[from].push(to);
    }
    let mut ready: VecDeque<_> = incoming
        .iter()
        .enumerate()
        .filter_map(|(index, &count)| (count == 0).then_some(index))
        .collect();
    let mut columns = vec![0; events.len()];
    let mut visited = 0;
    while let Some(from) = ready.pop_front() {
        visited += 1;
        for &to in &successors[from] {
            columns[to] = columns[to].max(columns[from] + 1);
            incoming[to] -= 1;
            if incoming[to] == 0 {
                ready.push_back(to);
            }
        }
    }
    if visited != events.len() {
        return Err(CuError::from(
            "Explicit schedule display contains a precedence cycle",
        ));
    }
    Ok(Layout {
        events,
        columns,
        copperlists,
    })
}

pub(super) fn render(
    plan: &CuMissionPlan,
    labels: &BTreeMap<String, (String, String)>,
    top: f64,
) -> CuResult<RenderedSection> {
    let layout = layout(plan)?;
    let mut svg = String::new();
    let mut y = top;
    writeln!(
        svg,
        r#"<text class="subtitle" x="0" y="{y}">Schedule</text>"#
    )
    .unwrap();
    y += 20.0;
    writeln!(
        svg,
        r#"<text class="meta" x="0" y="{y}">{} / {} CL · {} CL/cycle</text>"#,
        layout.copperlists, plan.max_in_flight, plan.copperlists_per_cycle
    )
    .unwrap();
    y += 20.0;
    for background in &plan.background {
        writeln!(
            svg,
            r#"<text class="meta" x="0" y="{y}">{} → {} · {} jobs · {:?}</text>"#,
            xml(&background.task),
            xml(&background.pool),
            background.max_running,
            background.result
        )
        .unwrap();
        y += 18.0;
    }
    let column_count = layout.columns.iter().max().map_or(0, |column| column + 1);
    for page in 0..column_count.div_ceil(WAVE_COLUMNS) {
        for column in 0..WAVE_COLUMNS.min(column_count - page * WAVE_COLUMNS) {
            let x = LABEL_WIDTH + column as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
            writeln!(
                svg,
                r#"<text class="ordinal mono" x="{x}" y="{y}">Order {:02}</text>"#,
                page * WAVE_COLUMNS + column + 1
            )
            .unwrap();
        }
        y += 12.0;
        for (worker_index, worker) in plan.workers.iter().enumerate() {
            let row_y = y + worker_index as f64 * (WAVE_CELL_HEIGHT + WAVE_CELL_GAP);
            let placement = match worker.placement {
                CuPlanPlacement::Main => "main thread".to_string(),
                CuPlanPlacement::Thread { cpu, policy } => format!(
                    "{} · {}",
                    cpu.map_or_else(|| "OS scheduled".into(), |cpu| format!("CPU {cpu}")),
                    format_policy(policy)
                ),
            };
            writeln!(svg, r#"<g><title>{}: {}</title><text class="lane mono" x="0" y="{}">{}</text><text class="card-line" x="0" y="{}">{}</text></g>"#, xml(&worker.id), xml(&placement), row_y + 24.0, xml(&truncate(&worker.id, 16)), row_y + 42.0, xml(&truncate(&placement, 21))).unwrap();
            for (index, event) in layout.events.iter().enumerate() {
                let column = layout.columns[index];
                if event.worker != worker_index || column / WAVE_COLUMNS != page {
                    continue;
                }
                let x = LABEL_WIDTH
                    + (column % WAVE_COLUMNS) as f64 * (WAVE_CELL_WIDTH + WAVE_CELL_GAP);
                let step = &plan.steps[event.step];
                let (label, fill) = labels.get(&step.key).ok_or_else(|| {
                    CuError::from(format!(
                        "Explicit schedule step '{}' has no display label",
                        step.key
                    ))
                })?;
                let cl = if event.copperlist == 0 {
                    "CL n".to_string()
                } else {
                    format!("CL n+{}", event.copperlist)
                };
                writeln!(svg, r##"<g class="light-card" data-worker="{}" data-cl-offset="{}" data-order="{}"><title>{} · {} · operation {}\n{}</title><rect x="{x}" y="{row_y}" width="{WAVE_CELL_WIDTH}" height="{WAVE_CELL_HEIGHT}" rx="6" fill="{fill}" stroke="#6c7086"/><text class="ordinal mono" x="{}" y="{}">{} · op {}</text><text class="card-title" x="{}" y="{}">{}</text></g>"##, xml(&worker.id), event.copperlist, column, xml(&worker.id), xml(&cl), event.ordinal + 1, xml(&step.key), x + 8.0, row_y + 19.0, xml(&cl), event.ordinal + 1, x + 8.0, row_y + 42.0, xml(&truncate(label, 17))).unwrap();
            }
        }
        y += plan.workers.len() as f64 * (WAVE_CELL_HEIGHT + WAVE_CELL_GAP) + 18.0;
    }
    Ok(RenderedSection {
        svg,
        height: y - top,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_runtime::planner::{CuPlanDependency, CuPlanStep, CuPlanWorker};

    fn plan() -> CuMissionPlan {
        CuMissionPlan {
            copperlists_per_cycle: 1,
            max_in_flight: 3,
            steps: ["source", "left", "right", "sink"]
                .into_iter()
                .map(|key| CuPlanStep {
                    key: key.into(),
                    copperlist: 0,
                })
                .collect(),
            workers: vec![
                CuPlanWorker {
                    id: "sensor_display".into(),
                    placement: CuPlanPlacement::Main,
                    steps: vec![0, 3],
                },
                CuPlanWorker {
                    id: "inference".into(),
                    placement: CuPlanPlacement::Thread {
                        cpu: Some(2),
                        policy: cu29_runtime::config::SchedulingPolicy::Fair,
                    },
                    steps: vec![2, 1],
                },
            ],
            dispatcher: None,
            background: Vec::new(),
            dependencies: [(0, 1), (0, 2), (1, 3), (2, 3)]
                .into_iter()
                .map(|(from, to)| CuPlanDependency {
                    from,
                    to,
                    cycle_lag: 0,
                })
                .collect(),
        }
    }

    #[test]
    fn worker_order_and_dependencies_determine_columns() {
        let layout = layout(&plan()).unwrap();
        let column = |step, cl| {
            let index = layout
                .events
                .iter()
                .position(|event| event.step == step && event.copperlist == cl)
                .unwrap();
            layout.columns[index]
        };
        assert!(column(2, 0) < column(1, 0));
        assert!(column(0, 0) < column(2, 0));
        assert!(column(1, 0) < column(3, 0));
        assert!(column(3, 0) < column(0, 1));
        assert!(column(1, 0) < column(2, 1));
    }

    #[test]
    fn cycle_lag_orders_workers_across_repetitions() {
        let mut plan = plan();
        plan.dependencies.push(CuPlanDependency {
            from: 3,
            to: 2,
            cycle_lag: 1,
        });
        let layout = layout(&plan).unwrap();
        let column = |step, cl| {
            let index = layout
                .events
                .iter()
                .position(|event| event.step == step && event.copperlist == cl)
                .unwrap();
            layout.columns[index]
        };
        assert!(column(3, 0) < column(2, 1));
    }

    #[test]
    fn pages_keep_worker_labels_cpu_placement_and_copperlists() {
        let plan = plan();
        let labels = plan
            .steps
            .iter()
            .map(|step| (step.key.clone(), (step.key.clone(), "#89b4fa".into())))
            .collect();
        let svg = render(&plan, &labels, 0.0).unwrap().svg;
        assert_eq!(svg, render(&plan, &labels, 0.0).unwrap().svg);
        assert!(svg.contains("CPU 2 · Fair"));
        assert!(svg.contains("CL n+2"));
        assert!(svg.contains("Order 09"));
        assert!(svg.matches("sensor_display:").count() > 1);
        assert!(svg.contains("data-worker=\"inference\""));
        assert!(!svg.contains("Precedence columns, not elapsed time"));
    }
}
