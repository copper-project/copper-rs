### This is a Generic Rate limiter for Copper

If you have a source that is going too fast, you can use this simple task to skip over messages.

### Task and Input


```ron
  (
            id: "rl1",
            type: "cu_ratelimit::CuRateLimit<MyPayload>", // Set your type here
            config: {
                "rate": 10.0,  // in Hz
            },
        ),
 [...]
```

### Configuration

- `rate`: the maximum rate in Hz  

### Output

It will match the Input type, set it as a type specialization like ie: replace MyPayload to your actual type in the config.
