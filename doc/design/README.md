# Architecture notes

These notes describe the checked-in implementation, its constraints, and why the
design took this shape. Source links identify the code and regression coverage;
remaining work is labeled separately from implemented behavior.

| Area | Read for |
| --- | --- |
| [Log storage](log-storage.md) | Byte addressing, static metadata, run identity, append, rollover, lifecycle |
| [Concurrent logging](concurrent-logging.md) | Format v3 ownership, reclamation, output handoff, failures, performance evidence |
| [Offline telemetry plan](offline-telemetry.md) | Proposed blocking TCP delivery for replay, bounded buffering, and receiver completion |
| [Self-describing logs](self-describing-logs.md) | Native wire recipes, startup catalog, offline decoding, feature boundaries |
| [Clock synchronization](clock-sync.md) | Raw/execution timelines, reference discipline, uncertainty, replay |
| [Profile-Guided Scheduling](profile-guided-scheduling.md) | Recorded profiles, offline search, compile-time plans, measured selection |
| [Isaac Sim](isaac-sim.md) | Simulation stepping, shared-buffer ownership, task overrides, replay gaps |
| [V1 API contract](v1-api-surface.md) | Stability labels, supported application APIs, experimental and internal surfaces |
| [Workspace](workspace.md) | In-tree codec/camera/inference dependencies and import provenance |

Start with [log storage](log-storage.md) before reading the other logging notes.
The [V1 API contract](v1-api-surface.md) defines stability separately from
these implementation notes. Application setup belongs in crate/example READMEs;
maintainer checks belong in [CONTRIBUTING.md](../../CONTRIBUTING.md).
