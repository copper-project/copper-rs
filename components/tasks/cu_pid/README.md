### This is a Generic PID Controller

Check out cu_rp_balancebot for a full example of how to use it.

### Task and Input

To be able to use it, you need to specialize it before you can reference it in your copper RON config:

```rust
// in mymod.rs
use cu_pid::GenericPIDTask;
pub type MyPID = GenericPIDTask<MyPayload>;

// MyPayload needs to implement an Into<f32> trait to be able to be used as a reference for the PID controller
pub struct MyPayload {
    pub value: f32,
}

impl Into<f32> for MyPayload {
    fn into(self) -> f32 {
        self.value
    }
}

```

Then you can use it in your copper RON config:

```ron
  (
            id: "my_pid",
            type: "mymod::MyPID", // Set your type alias here
            config: {
                "kp": 0.015,  
                "kd": 0.01,
                "ki": 0.00005,
                 "setpoint": 3176.0,
                 "cutoff": 170.0, 
            },
        ),
 [...]
```

### Configuration

- `kp`: Proportional gain
- `ki`: Integral gain
- `kd`: Derivative gain
- `setpoint`: The target value
- `cutoff`: The +/- deviation from the setpoint that is considered acceptable, otherwise the PID will return None (
  safety mode)

### Output

The PID controller will return a full state with the p, i and d contributions in PIDControlOutput struct:

```rust
pub struct PIDControlOutput {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub output: f32,  // output == p+i+d
}
```

