# cu29-value-types

Allocation-free, `no_std` types shared by Copper's native codec and offline value
reader: scalar kinds, record shapes, and typed quantity metadata.

`QuantityMetadata::coherent(Quantity::Velocity)` describes coherent velocity
storage. `QuantityMetadata::time(TimeStorageUnit::Nanosecond)` describes Copper
clock storage. Constructors associate each quantity with a compatible storage
unit. `StorageUnit::symbol()` provides the presentation spelling.

Copper owns the metadata vocabulary and its permanent numeric IDs. New catalogue
entries receive new IDs; existing IDs retain their meaning. The `serde` feature
enables serialization for the shared types.

Coherent storage symbols use conventional Unicode SI notation, such as `V`, `T`,
`N·m`, and `m·s⁻¹`, from the shared `cu29-value-types` catalogue.
