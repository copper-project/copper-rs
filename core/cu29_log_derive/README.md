## Structured Logging Macros

The `cu29_log_derive` crate provides macros like `!debug` to enable structured logging, allowing you to efficiently log
data in
a binary format.

### Example Usage

```rust
debug!("This string won't be stored nor interpreted on the robot", myvaluename = 42);
```

Instead of storing the complete formatted string in a log file, this macro captures a unique identifier for the format
string and parameter names, then logs the values in a compact bincode format. This approach significantly improves
logging efficiency.

### Integration with Copper

If you are using this crate as part of a Copper project, no additional setup is required. The logs will automatically
integrate with the Unified Logger (`cu29_unifiedlog`), storing logs in the binary format.

### Standalone Usage

For those using this crate independently of Copper, follow the example setup provided
in [cu_standalone_structlog](https://github.com/copper-project/copper-rs/tree/master/examples/cu_standalone_structlog)
to configure your logger.

#### Extracting Logs

You can extract and view your logs using either Rust or Python:

- **Rust:** Refer to the [cu29_export](https://github.com/copper-project/copper-rs/tree/master/core/cu29_export) module.
- **Python:** Use the provided script
  in [readlog.py](https://github.com/copper-project/copper-rs/tree/master/examples/cu_standalone_structlog/readlog.py)
  for Python support.

