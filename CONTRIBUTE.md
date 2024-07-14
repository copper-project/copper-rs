TODO: make that a proper document
* use Machete to remote unused dependencies
    * cargo install cargo-machete
    * cargo machete --with-metadata
    * cargo machete fix
    
The tool is not perfect, so in case of false positive, add an entry to the Cargo.toml crate:
```toml
[package.metadata.cargo-machete]
ignored = ["copper-log-runtime", "copper-log"]  # EXPLAIN WHY
```