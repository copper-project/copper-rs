# Modular Configuration

Copper's configuration system supports modular composition through file includes and parameter substitution. This document explains how to use these features to create reusable and maintainable configuration files.

## Table of Contents

- [Overview](#overview)
- [Including Configuration Files](#including-configuration-files)
- [Parameter Substitution](#parameter-substitution)
- [Merging Rules](#merging-rules)
- [Nested Includes](#nested-includes)
- [Common Use Cases](#common-use-cases)
- [Best Practices](#best-practices)
- [Limitations](#limitations)

## Overview

As robot configurations grow more complex, it becomes important to organize and reuse configuration components. Copper's modular configuration system allows you to:

- Split large configurations into manageable, reusable chunks
- Create configuration variations without duplicating the entire RON file
- Parameterize configurations to handle different deployments and environments

## Including Configuration Files

You can include other RON configuration files using the `includes` section at the top level of your configuration:

```ron
(
    tasks: [
        // Your main configuration tasks...
    ],
    cnx: [
        // Your main configuration connections...
    ],
    includes: [
        (
            path: "path/to/included_config.ron",
            params: {}, // Optional parameter substitutions
        ),
    ],
)
```

The `path` is relative to the location of the main configuration file. When Copper processes the configuration, it will:

1. Read the main configuration file
2. Process each included file
3. Merge the included configurations according to the [merging rules](#merging-rules)

## Parameter Substitution

You can parameterize your included configurations using template variables that will be replaced at runtime:

```ron
// included_config.ron
(
    tasks: [
        (
            id: "task_{{instance_id}}", // Will be replaced with the provided instance_id
            type: "tasks::Task{{instance_id}}",
            config: {
                "param_value": {{param_value}}, // Will be replaced with the provided param_value
            },
        ),
    ],
    cnx: [],
)
```

Then in your main configuration:

```ron
(
    tasks: [],
    cnx: [],
    includes: [
        (
            path: "included_config.ron",
            params: {
                "instance_id": "42", // Replaces {{instance_id}} with "42"
                "param_value": 100,  // Replaces {{param_value}} with 100
            },
        ),
    ],
)
```

### Parameter Format

Parameters use the `{{parameter_name}}` format and can appear in:

- Task IDs
- Task types
- Connection strings
- Configuration values (both keys and values)

Parameter values can be:

- Strings
- Numbers (integers, floats)
- Booleans
- Null values
- Arrays
- Maps

## Merging Rules

When merging included configurations with the main configuration, Copper follows these rules:

1. **Tasks**:
   - Tasks from included files are added to the main configuration
   - If a task with the same ID already exists in the main configuration, the main configuration's task takes precedence
   - Tasks from later includes override tasks from earlier includes with the same ID

2. **Connections**:
   - Connections from included files are added to the main configuration
   - If a connection with the same source and destination already exists, the main configuration's connection takes precedence
   - Connections from later includes override connections from earlier includes with the same source and destination

3. **Monitor and Logging**:
   - If a monitor or logging configuration is defined in the main file, it takes precedence over included configurations
   - If the main file doesn't define a monitor or logging configuration, the first included file with such configurations is used

## Nested Includes

Configuration files can include other configuration files, allowing for hierarchical composition:

```ron
// main.ron
(
    tasks: [],
    cnx: [],
    includes: [
        (
            path: "middle.ron",
            params: {},
        ),
    ],
)

// middle.ron
(
    tasks: [
        (
            id: "middle_task",
            type: "tasks::MiddleTask",
        ),
    ],
    cnx: [],
    includes: [
        (
            path: "nested.ron",
            params: {},
        ),
    ],
)

// nested.ron
(
    tasks: [
        (
            id: "nested_task",
            type: "tasks::NestedTask",
        ),
    ],
    cnx: [],
)
```

When Copper processes the configuration:
1. It will start with `main.ron`
2. It will include `middle.ron` (which includes its own task)
3. It will then include `nested.ron` (which includes its own task)
4. The final configuration will contain tasks from all three files

### Nested References

Configuration files can include other configuration files, creating a hierarchy of configurations. Be careful when creating deeply nested configurations to avoid unintended recursion or excessive nesting, as this may impact performance.

## Common Use Cases

### 1. Sharing Common Components

Create a base configuration with common components:

```ron
// common_sensors.ron
(
    tasks: [
        (
            id: "imu",
            type: "sensors::IMU",
        ),
        (
            id: "gps",
            type: "sensors::GPS",
        ),
    ],
    cnx: [],
)
```

Include it in multiple robot configurations:

```ron
// robot_config.ron
(
    tasks: [
        // Robot-specific tasks
    ],
    cnx: [
        // Robot-specific connections
    ],
    includes: [
        (
            path: "common_sensors.ron",
            params: {},
        ),
    ],
)
```

### 2. Environment-Specific Configurations

Create configurations for different deployment environments:

```ron
// dev_environment.ron
(
    tasks: [
        (
            id: "camera",
            type: "sensors::MockCamera", // Mock camera for development
        ),
    ],
    cnx: [],
)

// prod_environment.ron
(
    tasks: [
        (
            id: "camera",
            type: "sensors::RealCamera", // Real camera for production
        ),
    ],
    cnx: [],
)
```

Choose the appropriate environment in your main configuration:

```ron
// main_config.ron
(
    tasks: [
        // Common tasks
    ],
    cnx: [
        // Common connections
    ],
    includes: [
        (
            path: "dev_environment.ron", // Change to prod_environment.ron for production
            params: {},
        ),
    ],
)
```

### 3. Reusing Task Templates with Different Parameters

Create a parameterized task template:

```ron
// motor_template.ron
(
    tasks: [
        (
            id: "motor_{{motor_id}}",
            type: "actuators::Motor",
            config: {
                "pin": {{pin}},
                "direction": "{{direction}}",
            },
        ),
    ],
    cnx: [],
)
```

Include it multiple times with different parameters:

```ron
// robot_config.ron
(
    tasks: [],
    cnx: [],
    includes: [
        (
            path: "motor_template.ron",
            params: {
                "motor_id": "left",
                "pin": 4,
                "direction": "forward",
            },
        ),
        (
            path: "motor_template.ron",
            params: {
                "motor_id": "right",
                "pin": 5,
                "direction": "reverse",
            },
        ),
    ],
)
```

This will generate two motor tasks with different IDs, pins, and directions.

## Best Practices

1. **Organize Configuration Files**:
   - Keep related components together in the same include file
   - Use descriptive filenames that indicate the purpose of the configuration
   - Create a directory structure that reflects the organization of your robot

2. **Parameter Naming**:
   - Use clear, descriptive parameter names
   - Follow a consistent naming convention
   - Document parameters with comments in the configuration files

3. **Handling Variations**:
   - Create small, focused configuration files for specific components
   - Use parameters for values that might change between deployments
   - Create environment-specific configuration files for different contexts (e.g., development, testing, production)

4. **Testing**:
   - Test your configuration with different parameter values
   - Verify that the merged configuration matches your expectations
   - Consider creating a test that validates your configuration files

## Limitations

1. **Order Dependency**: The order of includes matters, as later includes can override earlier ones.

2. **Parameter Scope**: Parameters are only applied to the specific include where they're defined. If you want to use the same parameter across multiple includes, you need to specify it for each include.

3. **Error Handling**: If an included file cannot be found, Copper will return an error. Make sure all included files are available at the specified paths.