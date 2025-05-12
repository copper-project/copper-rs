# Copper Logging (cu29-log)

The `cu29-log` crate provides a structured logging system for the Copper framework. It allows for efficient, compile-time optimized logging with different severity levels and structured log entry storage.

## Log Levels

Copper logging supports five levels of severity:

- **Debug**: Detailed information useful during development
- **Info**: General information about system operation
- **Warning**: Indication of potential issues that don't prevent normal operation
- **Error**: Issues that might disrupt normal operation but don't cause system failure
- **Critical**: Critical errors requiring immediate attention, usually resulting in system failure

## Compile-Time Log Level Selection

A key feature of Copper's logging system is the ability to select the minimum log level at compile time. This means that log messages below the selected level are completely eliminated from the compiled code, resulting in zero runtime overhead for disabled log levels.

### How to Configure Log Levels

To set the minimum log level for your application, add one of the following feature flags to your `Cargo.toml` dependencies:

```toml
[dependencies]
cu29-log = { version = "0.7.0" }
cu29-log-derive = { version = "0.7.0", features = ["log-level-info"]  }
```

or

```toml
[dependencies]
cu29 = { version = "0.7.0", features = ["log-level-info"]}
```

Available feature flags:

- `log-level-debug`: Includes all logs (Debug, Info, Warning, Error, Critical)
- `log-level-info`: Includes Info, Warning, Error, and Critical logs
- `log-level-warning`: Includes Warning, Error, and Critical logs
- `log-level-error`: Includes Error and Critical logs
- `log-level-critical`: Includes only Critical logs

By default, no feature flag is enabled. If two feature flags are enabled, the higher level feature flag will be used as the max log level. For example, if `log-level-info` and `log-level-error` are enabled, `info!`, `warning!`, `error!` and `critical!` will be logged.

## Using the Logging Macros

The following macros are available for logging at different levels through the `cu29_log_derive` crate:

```rust
use cu29_log_derive::{debug, info, warning, error, critical};

// Log examples at different levels
debug!("Detailed debugging information: {}", value);
info!("Normal operational information");
warning!("Warning: {}", message);
error!("Error occurred: {}", err);
critical!("Critical failure: {}", critical_error);

debug!("User {} performed action {}", user_id = 123, action = "login");
```

Note that these macros will be automatically compiled out when the log level is set higher than the corresponding message level, resulting in zero runtime overhead.

### Parameters

All logging macros support both named and unnamed parameters:

- Unnamed parameters: `debug!("Value: {}", value)`
- Named parameters: `debug!("Value: {}", name = value)`

## Behavior in Debug vs Release Mode

- In **debug** mode, logs are both sent to the structured log sink and printed to the console for immediate visibility.
- In **release** mode, logs are only sent to the structured log sink, optimizing for performance.

## Advanced Usage

### CuLogLevel Enum

You can use the `CuLogLevel` enum directly in your code if needed:

```rust
use cu29_log::CuLogLevel;

let level = CuLogLevel::Warning;
```

The enum implements `PartialOrd` and `Ord`, so you can compare levels:

```rust
assert!(CuLogLevel::Error > CuLogLevel::Warning);
```

### Structured Log Entries

The `CuLogEntry` struct is the core of the structured logging system:

```rust
let entry = CuLogEntry::new(msg_index, CuLogLevel::Warning);
entry.add_param(param_name_index, param_value);
```

Each log entry contains:

- A timestamp (`CuTime`)
- Log level (`CuLogLevel`)
- Message index (interned string reference)
- Parameter names and values (stored efficiently using `SmallVec`)

### Log Storage and Retrieval

Log messages and parameter names are interned at compile time and stored in an index for efficient retrieval. The system uses the `LOG_INDEX_DIR` environment variable to locate the log index directory.

```rust
// Get the default log index directory path
let index_dir = cu29_log::default_log_index_dir();
```

### Log Formatting

The library provides utilities to format log entries into human-readable text:

```rust
let formatted = cu29_log::rebuild_logline(&interned_strings, &log_entry)?;
```
