# cu29-reflect-derive

No-op fallback `Reflect` derive macro for Copper.

This crate is used when reflection is disabled. It accepts `#[derive(Reflect)]`
and `#[reflect(...)]` helper attributes, but intentionally emits no generated
code. This keeps type definitions portable across feature configurations.

Most users do not depend on this crate directly and get it through Copper
feature wiring.
