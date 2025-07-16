# Future Enhancements for cu_transform

## Discrete Time of Validity (Tov) Support

Currently, the `Tov` enum in `cu29_clock` supports:
- `None`: No specific time of validity
- `Time(CuTime)`: A single point in time
- `Range(CuTimeRange)`: A continuous time range

### Proposed Enhancement: Discrete Variant

Add a new variant to support discrete timestamps:
```rust
pub enum Tov {
    None,
    Time(CuTime),
    Range(CuTimeRange),
    Discrete(ArrayVec<CuTime, 8>), // Or similar fixed-size array
}
```

### Use Cases

1. **Irregular Transform Sampling**: When transforms are captured at non-uniform intervals
2. **Multi-Frame Synchronization**: When a transform is only valid at specific synchronized timestamps across multiple sensors
3. **Sparse Transform Data**: When transforms are only defined at specific keyframes

### Implementation Considerations

1. **Core Change Required**: This requires modifying the `cu29_clock` crate
2. **Lookup Semantics**: Need to define behavior for transform lookups:
   - Exact match only?
   - Nearest neighbor?
   - Interpolation between discrete points?
3. **Memory Efficiency**: Use fixed-size arrays to maintain stack allocation
4. **Backward Compatibility**: Ensure existing code continues to work

### Current Workaround

For now, users needing discrete timestamp support can:
1. Use multiple `CuMsg<TransformMsg>` with individual `Tov::Time` values
2. Implement custom timestamp tracking in their application logic