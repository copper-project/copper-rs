# Copper agent guidance

Copper is a Rust robotics runtime/SDK with compile-time task graphs, deterministic
replay, unified logging, and `std`/`no_std` targets.

## Skills

Before relevant work, read the applicable `SKILL.md`:

| Work | Skill |
|---|---|
| Architecture / reading apps | `copper-arch` |
| Rust / documentation / console or UI style | `copper-coding-style` |
| Core / runtime changes | `copper-core-dev` |
| Public traits, adapters, config policies | `copper-api-flavor` |
| Sources, tasks, sinks, bridges | `copper-component-design` |
| RON config | `copper-ron-config` |
| Build, checks, justfiles, releases | `copper-workflow` |
| Proc-macro failures | `copper-macro-debug` |
| Runtime debugging / logs / replay | `copper-debug-replay` |

Use discovered skills; otherwise read `<skill>/SKILL.md` from
[copper-project/copper-rs-skills](https://github.com/copper-project/copper-rs-skills).
If the skill cannot be accessed, report it before dependent work. Setup is
documented in the repository's [README](https://github.com/copper-project/copper-rs-skills#readme);
do not silently install skills.

## Always apply

- Source authority: code > rustdoc/API > wiki/book. Start with `README.md`,
  `justfile`, and `CONTRIBUTING.md`; narrative docs live in `../copper-rs.wiki`
  and `../copper-rs-book/book/src`.
- Before changing a feature, ask whether backward compatibility is required and
  which released APIs, formats, or user data must be supported.
- Preserve static typed wiring, deterministic replay, and `no_std`. Require
  explicit user approval for added hot-path copies, allocation, serialization,
  or latency; keep offline/debug costs outside the real-time path.
- Before a work pass, remove generated `Cargo.lock` files unless instructed
  otherwise; do not remove them as final cleanup.
- Run `just fmt` after each edit pass. Before pushing, run the narrowest relevant
  root verification target (default `just pr-check`; details in `copper-workflow`).
