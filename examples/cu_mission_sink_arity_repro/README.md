# cu_mission_sink_arity_repro

This is a standalone repro for a mission-specific sink input arity mismatch.

Mission `A` wires `sink` with one input:

- `src_int -> sink`

Mission `B` wires the same sink task type with two inputs:

- `src_int -> sink`
- `src_bool -> sink`

The sink implementation declares:

```rust
type Input<'m> = input_msg!('m, i32, bool);
```

That means mission `B` matches the task signature, but mission `A` does not.

Run the repro from the repo root:

```bash
cargo check --manifest-path examples/cu_mission_sink_arity_repro/Cargo.toml
```

The expected failure is a type mismatch in generated runtime code where one mission passes a
single `CuStampedData<_>` reference while the sink task expects a tuple of two input references.
