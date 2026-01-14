use horus::prelude::*;
use horus::TopicMetadata;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::any::type_name;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

const CHAIN_LEN: usize = 8;
const CHAIN_TOPICS: [&str; CHAIN_LEN] = [
    "ct-0", "ct-1", "ct-2", "ct-3", "ct-4", "ct-5", "ct-6", "ct-7",
];
const GPIO_TOPICS: [&str; CHAIN_LEN] = [
    "gpio-0", "gpio-1", "gpio-2", "gpio-3", "gpio-4", "gpio-5", "gpio-6", "gpio-7",
];
const TASK_COUNT: usize = 1 + CHAIN_LEN + CHAIN_LEN;
const CONNECTION_COUNT: usize = CHAIN_LEN + CHAIN_LEN;

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CaterpillarMsg {
    seq: u64,
    state: bool,
    origin_ns: u64,
}

impl LogSummary for CaterpillarMsg {
    fn log_summary(&self) -> String {
        format!(
            "seq={} state={} origin_ns={}",
            self.seq, self.state, self.origin_ns
        )
    }
}

static START: OnceLock<Instant> = OnceLock::new();

fn now_ns() -> u64 {
    let start = START.get_or_init(Instant::now);
    start.elapsed().as_nanos() as u64
}

struct CaterpillarSource {
    output: Hub<CaterpillarMsg>,
    output_topic: &'static str,
    state: bool,
    seq: u64,
    stats: Arc<Mutex<StatsStore>>,
}

impl CaterpillarSource {
    fn new(output_topic: &'static str, stats: Arc<Mutex<StatsStore>>) -> Result<Self> {
        Ok(Self {
            output: Hub::new(output_topic)?,
            output_topic,
            state: false,
            seq: 0,
            stats,
        })
    }
}

impl Node for CaterpillarSource {
    fn name(&self) -> &'static str {
        "src"
    }

    fn get_publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: self.output_topic.to_string(),
            type_name: type_name::<CaterpillarMsg>().to_string(),
        }]
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let tick_start = Instant::now();
        self.state = !self.state;
        self.seq += 1;
        let msg = CaterpillarMsg {
            seq: self.seq,
            state: self.state,
            origin_ns: now_ns(),
        };
        let _ = self.output.send(msg, &mut ctx);
        let tick_ns = tick_start.elapsed().as_nanos() as u64;
        self.stats.lock().unwrap().record("src", tick_ns);
    }
}

struct CaterpillarTask {
    name: &'static str,
    input: Hub<CaterpillarMsg>,
    output: Option<Hub<CaterpillarMsg>>,
    gpio: Hub<CaterpillarMsg>,
    input_topic: &'static str,
    output_topic: Option<&'static str>,
    gpio_topic: &'static str,
    stats: Arc<Mutex<StatsStore>>,
}

impl CaterpillarTask {
    fn new(
        name: &'static str,
        input_topic: &'static str,
        output_topic: Option<&'static str>,
        gpio_topic: &'static str,
        stats: Arc<Mutex<StatsStore>>,
    ) -> Result<Self> {
        Ok(Self {
            name,
            input: Hub::new(input_topic)?,
            output: match output_topic {
                Some(topic) => Some(Hub::new(topic)?),
                None => None,
            },
            gpio: Hub::new(gpio_topic)?,
            input_topic,
            output_topic,
            gpio_topic,
            stats,
        })
    }
}

impl Node for CaterpillarTask {
    fn name(&self) -> &'static str {
        self.name
    }

    fn get_publishers(&self) -> Vec<TopicMetadata> {
        let mut pubs = Vec::new();
        if let Some(topic) = self.output_topic {
            pubs.push(TopicMetadata {
                topic_name: topic.to_string(),
                type_name: type_name::<CaterpillarMsg>().to_string(),
            });
        }
        pubs.push(TopicMetadata {
            topic_name: self.gpio_topic.to_string(),
            type_name: type_name::<CaterpillarMsg>().to_string(),
        });
        pubs
    }

    fn get_subscribers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: self.input_topic.to_string(),
            type_name: type_name::<CaterpillarMsg>().to_string(),
        }]
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let tick_start = Instant::now();
        if let Some(msg) = self.input.recv(&mut ctx) {
            if let Some(ref output) = self.output {
                let _ = output.send(msg.clone(), &mut ctx);
            }
            let _ = self.gpio.send(msg, &mut ctx);
            let tick_ns = tick_start.elapsed().as_nanos() as u64;
            self.stats.lock().unwrap().record(self.name, tick_ns);
        }
    }
}

struct GpioSink {
    name: &'static str,
    input: Hub<CaterpillarMsg>,
    input_topic: &'static str,
    record_end_to_end: bool,
    stats: Arc<Mutex<StatsStore>>,
}

impl GpioSink {
    fn new(
        name: &'static str,
        input_topic: &'static str,
        record_end_to_end: bool,
        stats: Arc<Mutex<StatsStore>>,
    ) -> Result<Self> {
        Ok(Self {
            name,
            input: Hub::new(input_topic)?,
            input_topic,
            record_end_to_end,
            stats,
        })
    }
}

impl Node for GpioSink {
    fn name(&self) -> &'static str {
        self.name
    }

    fn get_subscribers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: self.input_topic.to_string(),
            type_name: type_name::<CaterpillarMsg>().to_string(),
        }]
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let tick_start = Instant::now();
        if let Some(msg) = self.input.recv(&mut ctx) {
            let tick_ns = tick_start.elapsed().as_nanos() as u64;
            let mut stats = self.stats.lock().unwrap();
            stats.record(self.name, tick_ns);

            if self.record_end_to_end {
                let end_to_end_ns = now_ns().saturating_sub(msg.origin_ns);
                stats.record("End2End", end_to_end_ns);
            }
        }
    }
}

fn main() -> Result<()> {
    let _ = START.set(Instant::now());
    let stats = Arc::new(Mutex::new(StatsStore::new(Duration::from_secs(1))));

    println!(
        "horus caterpillar: tasks={} connections={} (chain={}, gpio={})",
        TASK_COUNT, CONNECTION_COUNT, CHAIN_LEN, CHAIN_LEN
    );

    let mut scheduler = Scheduler::new()
        .enable_determinism()
        .with_capacity(TASK_COUNT);

    scheduler.add(
        Box::new(CaterpillarSource::new(
            CHAIN_TOPICS[0],
            stats.clone(),
        )?),
        0,
        Some(false),
    );

    for i in 0..CHAIN_LEN {
        let output_topic = if i + 1 < CHAIN_LEN {
            Some(CHAIN_TOPICS[i + 1])
        } else {
            None
        };
        let task = CaterpillarTask::new(
            CHAIN_TOPICS[i],
            CHAIN_TOPICS[i],
            output_topic,
            GPIO_TOPICS[i],
            stats.clone(),
        )?;
        scheduler.add(Box::new(task), (1 + i) as u32, Some(false));
    }

    for i in 0..CHAIN_LEN {
        let gpio = GpioSink::new(
            GPIO_TOPICS[i],
            GPIO_TOPICS[i],
            i + 1 == CHAIN_LEN,
            stats.clone(),
        )?;
        scheduler.add(
            Box::new(gpio),
            (1 + CHAIN_LEN + i) as u32,
            Some(false),
        );
    }

    scheduler.run()?;
    Ok(())
}

struct StatsStore {
    stats: BTreeMap<&'static str, Stat>,
    last_report: Instant,
    report_interval: Duration,
}

impl StatsStore {
    fn new(report_interval: Duration) -> Self {
        Self {
            stats: BTreeMap::new(),
            last_report: Instant::now(),
            report_interval,
        }
    }

    fn record(&mut self, name: &'static str, sample_ns: u64) {
        self.stats
            .entry(name)
            .or_insert_with(Stat::default)
            .record(sample_ns);

        if self.last_report.elapsed() >= self.report_interval {
            self.last_report = Instant::now();
            println!("\n{}", self.format_table());
        }
    }

    fn format_table(&self) -> String {
        let mut rows: Vec<[String; 7]> = Vec::new();
        rows.push([
            "Task".to_string(),
            "Min".to_string(),
            "Max".to_string(),
            "Mean".to_string(),
            "Stddev".to_string(),
            "Jitter".to_string(),
            "Max Jitter".to_string(),
        ]);

        for name in TASK_ORDER {
            if let Some(stat) = self.stats.get(name) {
                if stat.count > 0 {
                    let mean = stat.mean();
                    let stddev = stat.stddev();
                    let jitter_avg = stat.jitter_avg();
                    rows.push([
                        name.to_string(),
                        format_ns(stat.min_ns),
                        format_ns(stat.max_ns),
                        format_ns(mean),
                        format_ns(stddev),
                        format_ns(jitter_avg),
                        format_ns(stat.jitter_max),
                    ]);
                    continue;
                }
            }

            rows.push([
                name.to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
            ]);
        }

        let mut widths = [0usize; 7];
        for row in &rows {
            for (idx, cell) in row.iter().enumerate() {
                widths[idx] = widths[idx].max(cell.len());
            }
        }

        let mut out = String::new();
        for (row_idx, row) in rows.iter().enumerate() {
            let line = format!(
                "{:<w0$}  {:>w1$}  {:>w2$}  {:>w3$}  {:>w4$}  {:>w5$}  {:>w6$}",
                row[0],
                row[1],
                row[2],
                row[3],
                row[4],
                row[5],
                row[6],
                w0 = widths[0],
                w1 = widths[1],
                w2 = widths[2],
                w3 = widths[3],
                w4 = widths[4],
                w5 = widths[5],
                w6 = widths[6],
            );
            out.push_str(&line);
            if row_idx + 1 < rows.len() {
                out.push('\n');
            }
        }
        out
    }
}

#[derive(Default)]
struct Stat {
    count: u64,
    min_ns: u64,
    max_ns: u64,
    mean_ns: f64,
    m2: f64,
    last_ns: Option<u64>,
    jitter_sum: u128,
    jitter_max: u64,
    jitter_count: u64,
}

impl Stat {
    fn record(&mut self, sample_ns: u64) {
        self.count += 1;
        if self.count == 1 {
            self.min_ns = sample_ns;
            self.max_ns = sample_ns;
            self.mean_ns = sample_ns as f64;
            self.m2 = 0.0;
        } else {
            self.min_ns = self.min_ns.min(sample_ns);
            self.max_ns = self.max_ns.max(sample_ns);
            let delta = sample_ns as f64 - self.mean_ns;
            self.mean_ns += delta / self.count as f64;
            let delta2 = sample_ns as f64 - self.mean_ns;
            self.m2 += delta * delta2;
        }

        if let Some(prev) = self.last_ns {
            let jitter = sample_ns.abs_diff(prev);
            self.jitter_sum += jitter as u128;
            self.jitter_max = self.jitter_max.max(jitter);
            self.jitter_count += 1;
        }
        self.last_ns = Some(sample_ns);
    }

    fn mean(&self) -> u64 {
        if self.count == 0 {
            0
        } else {
            self.mean_ns.round() as u64
        }
    }

    fn stddev(&self) -> u64 {
        if self.count == 0 {
            0
        } else {
            (self.m2 / self.count as f64).sqrt().round() as u64
        }
    }

    fn jitter_avg(&self) -> u64 {
        if self.jitter_count == 0 {
            0
        } else {
            (self.jitter_sum / self.jitter_count as u128) as u64
        }
    }
}

fn format_ns(ns: u64) -> String {
    if ns >= 1_000_000 {
        format!("{:.3} ms", ns as f64 / 1_000_000.0)
    } else if ns >= 1_000 {
        format!("{:.3} us", ns as f64 / 1_000.0)
    } else {
        format!("{} ns", ns)
    }
}

const TASK_ORDER: [&str; TASK_COUNT + 1] = [
    "src",
    "ct-0",
    "gpio-0",
    "ct-1",
    "gpio-1",
    "ct-2",
    "gpio-2",
    "ct-3",
    "gpio-3",
    "ct-4",
    "gpio-4",
    "ct-5",
    "gpio-5",
    "ct-6",
    "gpio-6",
    "ct-7",
    "gpio-7",
    "End2End",
];
