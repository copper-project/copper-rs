# Modular Configuration Example

This directory contains a simple example demonstrating how to use Copper's modular configuration feature. The example creates a basic robot configuration with a source task and two motor tasks, illustrating the use of includes and parameter substitution.

## Configuration Files

- **base.ron**: Contains the basic configuration with a source task
- **motors.ron**: A template for motor tasks with parameters for ID, pin, and direction
- **main_config.ron**: The main configuration that includes the other files with specific parameters

## How It Works

1. **base.ron** defines a `FlippingSource` task that generates signals
2. **motors.ron** is a template for motors with parameterized values:
   - `{{id}}` for the motor identifier
   - `{{pin}}` for the GPIO pin
   - `{{direction}}` for the motor direction

3. **main_config.ron** includes these files with specific parameters:
   - `base.ron` with no parameters
   - `motors.ron` twice with different parameters for left and right motors

## Resulting Configuration

The final configuration will have:

- One source task: `source`
- Two motor tasks: `motor_left` and `motor_right`
- Connections from the source to each motor
- Console monitoring and logging configuration

## Usage

To use this configuration, reference the main configuration file in your Copper application:

```rust
#[copper_runtime(config = "examples/modular_config_example/main_config.ron")]
struct MyRobot {}
```

## Benefits

This example demonstrates several benefits of modular configuration:

1. **Reusability**: The motor template is reused for multiple motors
2. **Maintainability**: Changes to common components only need to be made in one place
3. **Parameterization**: Different instances have different settings without duplicating configuration
4. **Organization**: Configuration is split into logical components