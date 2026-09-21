//! Candidates measured: their profiles beside their predictions, ranked by
//! the constraint-ranked score computed from what was measured.

use super::CuContract;
use super::CuPrediction;
use super::CuProfile;
use super::propose::rate_deficit;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;
use cu29_traits::CuError;
use cu29_traits::CuResult;
use serde::Deserialize;
use serde::Serialize;

/// One candidate's measurement beside its prediction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuScoreRow {
    pub candidate: String,
    /// The constraint-ranked tiers from measured p99 latencies and delivered
    /// rates, smallest first is better.
    pub score: Vec<f64>,
    pub chains: BTreeMap<String, CuChainScore>,
    /// Delivered rate per contract source.
    pub sources: BTreeMap<String, f64>,
    /// The proposer's prediction, when the candidate has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub predicted: Option<CuPrediction>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuChainScore {
    pub deadline_ms: u32,
    pub samples: u64,
    pub p50_ns: u64,
    pub p99_ns: u64,
    pub max_ns: u64,
    pub misses: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub predicted_ns: Option<u64>,
}

/// Every measured candidate, best first.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuScoreTable {
    pub rows: Vec<CuScoreRow>,
}

impl CuScoreTable {
    /// Ranks `measured` (candidate name to its profile) by the contract's
    /// constraint-ranked score. `predictions` supplies the model's numbers
    /// for comparison, keyed by the candidate's name up to a `/`.
    pub fn new(
        contract: &CuContract,
        predictions: &BTreeMap<String, CuPrediction>,
        measured: &BTreeMap<String, CuProfile>,
    ) -> CuResult<Self> {
        if let Some((first, reference)) = measured.iter().next() {
            for (candidate, profile) in measured {
                if profile.config_signature != reference.config_signature
                    || profile.mission != reference.mission
                {
                    return Err(CuError::from(format!(
                        "Profiles of '{first}' and '{candidate}' were recorded on different graphs or missions"
                    )));
                }
            }
        }
        let mut rows = Vec::with_capacity(measured.len());
        for (candidate, profile) in measured {
            // Rounds of one candidate are named `plan-1/r2`.
            let predicted = predictions
                .get(candidate)
                .or_else(|| predictions.get(candidate.split('/').next().unwrap_or(candidate)));
            let mut chains = BTreeMap::new();
            let mut ratios = Vec::with_capacity(contract.chains.len());
            for chain in &contract.chains {
                let stats = profile.chains.get(&chain.id).ok_or_else(|| {
                    CuError::from(format!(
                        "Profile of '{candidate}' has no chain '{}'",
                        chain.id
                    ))
                })?;
                if stats.latency.samples == 0 {
                    return Err(CuError::from(format!(
                        "Profile of '{candidate}' has no samples for chain '{}'",
                        chain.id
                    )));
                }
                let deadline_ns = u64::from(chain.deadline_ms) * 1_000_000;
                ratios.push(stats.latency.p99_ns as f64 / deadline_ns as f64);
                chains.insert(
                    chain.id.clone(),
                    CuChainScore {
                        deadline_ms: chain.deadline_ms,
                        samples: stats.latency.samples,
                        p50_ns: stats.latency.p50_ns,
                        p99_ns: stats.latency.p99_ns,
                        max_ns: stats.latency.max_ns,
                        misses: stats.misses,
                        predicted_ns: predicted
                            .and_then(|p| p.chains.get(&chain.id))
                            .map(|c| c.latency_ns),
                    },
                );
            }
            let mut sources = BTreeMap::new();
            let mut rate_deficit = 0.0f64;
            for source in &contract.sources {
                let rate = profile
                    .sources
                    .get(&source.task)
                    .ok_or_else(|| {
                        CuError::from(format!(
                            "Profile of '{candidate}' has no source rate for '{}'",
                            source.task
                        ))
                    })?
                    .delivered_rate;
                rate_deficit += self::rate_deficit(rate);
                sources.insert(source.task.clone(), rate);
            }
            let sum: f64 = ratios.iter().sum();
            let score = vec![
                round6(rate_deficit),
                ratios.iter().filter(|&&r| r > 1.0).count() as f64,
                ratios
                    .iter()
                    .filter(|&&r| r > 1.0 - contract.headroom)
                    .count() as f64,
                round6(sum),
            ];
            rows.push(CuScoreRow {
                candidate: candidate.clone(),
                score,
                chains,
                sources,
                predicted: predicted.cloned(),
            });
        }
        rows.sort_by(|a, b| {
            a.score
                .iter()
                .zip(&b.score)
                .map(|(x, y)| x.total_cmp(y))
                .find(|o| o.is_ne())
                .unwrap_or_else(|| a.candidate.cmp(&b.candidate))
        });
        Ok(Self { rows })
    }

    /// The best measured candidate.
    pub fn best(&self) -> Option<&CuScoreRow> {
        self.rows.first()
    }

    pub fn serialize_ron(&self) -> CuResult<String> {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
            .map_err(|e| CuError::new_with_cause("Could not serialize the score table", e))
    }
}

fn round6(value: f64) -> f64 {
    (value * 1e6 + 0.5) as u64 as f64 / 1e6
}

fn ms(ns: u64) -> f64 {
    ns as f64 / 1e6
}

impl fmt::Display for CuScoreTable {
    /// One block per candidate, best first: the score, then every chain's
    /// predicted and measured latency against its deadline.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (rank, row) in self.rows.iter().enumerate() {
            writeln!(f, "{}. {} score {:?}", rank + 1, row.candidate, row.score)?;
            for (id, chain) in &row.chains {
                let predicted = chain
                    .predicted_ns
                    .map(|ns| format!("{:.2}", ms(ns)))
                    .unwrap_or_else(|| "-".into());
                writeln!(
                    f,
                    "   {id:<32} predicted {predicted:>8} | p50 {:>8.2} p99 {:>8.2} max {:>8.2} ms | {} misses / {} of {} ms",
                    ms(chain.p50_ns),
                    ms(chain.p99_ns),
                    ms(chain.max_ns),
                    chain.misses,
                    chain.samples,
                    chain.deadline_ms
                )?;
            }
            for (task, rate) in &row.sources {
                let predicted = row
                    .predicted
                    .as_ref()
                    .and_then(|p| p.sources.get(task))
                    .map(|r| format!("{r:.3}"))
                    .unwrap_or_else(|| "-".into());
                writeln!(
                    f,
                    "   {task:<32} rate predicted {predicted:>8} | delivered {rate:.3}"
                )?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner::CuChain;
    use crate::planner::CuChainProfile;
    use crate::planner::CuCostStats;
    use crate::planner::CuSourceProfile;
    use crate::planner::CuSourceRate;

    fn contract() -> CuContract {
        CuContract::deserialize_ron(
            r#"(
            chains: [(id: "hot", source: "src", sink: "sink", deadline_ms: 10)],
            sources: [(task: "src", period_ms: 5)],
            cpus: [0],
            max_in_flight: 1,
        )"#,
        )
        .unwrap()
    }

    fn profile(p99_ms: u64, rate: f64) -> CuProfile {
        let mut profile = CuProfile::new("sig".into(), "default".into());
        let mut samples = vec![p99_ms * 1_000_000; 4];
        samples[0] /= 2;
        profile.chains.insert(
            "hot".into(),
            CuChainProfile {
                deadline_ms: 10,
                latency: CuCostStats::from_samples(&mut samples),
                misses: u64::from(p99_ms > 10),
            },
        );
        profile.sources.insert(
            "src".into(),
            CuSourceProfile {
                period_ms: 5,
                fired: 10,
                expected: 10.0,
                delivered_rate: rate,
            },
        );
        profile
    }

    #[test]
    fn ranks_measured_candidates_by_deadlines_and_keeps_predictions() {
        let contract = contract();
        let _ = (CuChain::clone, CuSourceRate::clone);
        let measured: BTreeMap<String, CuProfile> = [
            ("late".to_string(), profile(12, 1.0)),
            ("tight".to_string(), profile(9, 1.0)),
            ("dropping".to_string(), profile(4, 0.5)),
        ]
        .into_iter()
        .collect();
        let table = CuScoreTable::new(&contract, &BTreeMap::new(), &measured).unwrap();
        let order: Vec<&str> = table.rows.iter().map(|r| r.candidate.as_str()).collect();
        assert_eq!(order, ["tight", "late", "dropping"]);
        assert_eq!(table.best().unwrap().score, vec![0.0, 0.0, 1.0, 0.9]);
        assert_eq!(table.rows[2].score[0], 0.5);
        assert!(table.to_string().contains("tight score"));
        let text = table.serialize_ron().unwrap();
        assert!(text.contains("dropping"));
    }
}
