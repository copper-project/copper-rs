# cu29-helpers

This crate is intentionally retired.

It exists as a compatibility stub so downstream users who still depend on
`cu29-helpers` get a compile-time migration message that points them at the new
generated app builder API:

```rust,ignore
let mut app = MyApp::builder()
    .with_log_path(log_path, slab_size)?
    .build()?;
```

If the app needs a custom clock, logger, config override, or resource factory,
set those through the builder before calling `.build()`.
