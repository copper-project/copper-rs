# cu_transform

## Overview

This library provides spatial transformation functionality for the Copper framework, including both pose (position and
orientation) transformations and velocity transformations.

## Interface

Provides Rust APIs consumed by Copper components and applications.

## Configuration

Typically no direct runtime `copperconfig.ron` keys; configure through Rust API usage.

## Usage

Add this crate as a dependency and call its APIs from your component/application code.

## Compatibility

### Features

- Efficient representation of 3D transformations using homogeneous matrices
- Time-stamped transforms for tracking movements through time
- Transform tree for managing parent-child relationships between coordinate frames
- Velocity transformations for motion planning and control
- High-performance caching for both transforms and velocity transforms
- Interpolation between transforms
- allocation free for real time operations

## Links

- Crate path: `components/libs/cu_transform`
- docs.rs: <https://docs.rs/cu-transform>

## Additional Notes

### Transform Tree

The transform tree maintains a hierarchical relationship between coordinate frames:

```rust
use cu_transform::{StampedTransform, TransformTree, Transform3D};
use cu29::clock::CuDuration;

fn tree_lookup() {
    // Create a transform tree
    let mut tree = TransformTree::<f32>::new();

    // Add a transform from "world" to "robot"
    let world_to_robot = StampedTransform {
        transform: Transform3D::default(), // Identity transform
        stamp: CuDuration(1000),
        parent_frame: "world".try_into().expect("invalid name"),
        child_frame: "robot".try_into().expect("invalid name"),
    };
    tree.add_transform(world_to_robot).unwrap();

    // Look up transform
    let transform = tree.lookup_transform("world", "robot", CuDuration(1000)).unwrap();
}
```

### Velocity Transforms

The cu_transform library also supports calculating and transforming velocities:

```rust
use cu_transform::{VelocityTransform, TransformTree};
use cu29::clock::CuDuration;

fn velocity_lookup() {
    // Create and set up a transform tree with moving frames
    // ...

    // Look up velocity between frames at a specific time
    // This uses caching to speed up repeated lookups
    let velocity = tree.lookup_velocity("world", "robot", CuDuration(1500)).unwrap();

    // Access linear and angular components
    let linear_x = velocity.linear[0]; // Linear velocity in x direction (m/s)
    let angular_z = velocity.angular[2]; // Angular velocity around z axis (rad/s)

    // For unit-aware applications, use the unit methods
    let linear_vel = velocity.linear_velocity(); // Returns velocities with proper units
}
```

### Computing Velocity

Velocity is computed by differentiating transformations over time:

```rust
fn compute_velocity() {
    // Assuming we have two transforms at different times
    let velocity = transform2.compute_velocity(&transform1);
}
```

### Frame Transformation

Velocities can be transformed between coordinate frames:

```rust
use cu_transform::{transform_velocity, VelocityTransform, Transform3D};

fn frame_transformation() {
    // Velocity in frame A
    let velocity_a = VelocityTransform {
        linear: [1.0, 0.0, 0.0],  // 1 m/s in x direction
        angular: [0.0, 0.0, 0.5],  // 0.5 rad/s around z axis
    };

    // Transform from frame A to frame B
    let transform_a_to_b = Transform3D { /* ... */ };

    // Position where the velocity is measured
    let position = [0.0, 0.0, 0.0];

    // Transform velocity from frame A to frame B
    let velocity_b = transform_velocity(&velocity_a, &transform_a_to_b, &position);
}
```

### Velocity Transform Implementation

The velocity transformation follows the standard rigid body motion equations:

1. Linear velocity transformation: v_b = R * v_a + ω × (R * p)
2. Angular velocity transformation: ω_b = R * ω_a

Where:

- v_a is the linear velocity in frame A
- ω_a is the angular velocity in frame A
- R is the rotation matrix from A to B
- p is the position vector where velocity is measured
- × is the cross product

### Transform Chain

When looking up velocity across multiple frames, the transform tree handles the chain of transformations properly,
accumulating both linear and angular velocities correctly.

### Caching Mechanism

Both transforms and velocity transforms utilize a high-performance caching system to accelerate repeated lookups:

```rust

fn caching_tree() {
    // Create a transform tree with custom cache settings
    let tree = TransformTree::<f32>::with_cache_settings(
        200,                   // Cache size (max number of entries)
        Duration::from_secs(5) // Cache entry lifetime
    );

    // Lookup operations use the cache automatically
    let velocity = tree.lookup_velocity("world", "robot", time);

    // Cache is automatically invalidated when:
    // - New transforms are added to the tree
    // - Cache entries exceed their age limit
    // - The cache reaches capacity (uses LRU eviction)

    // Clear the cache manually if needed
    tree.clear_cache();

    // The cache is cleaned automatically at regular intervals,
    // but you can trigger cleanup explicitly if needed
    tree.cleanup_cache();
}

```

The velocity cache significantly improves performance for:

- Real-time applications with frequent velocity lookups
- Applications with many coordinate frames
- Complex transform chains that would otherwise require expensive recalculation

The cache keys include frame IDs, timestamp, and a hash of the transform path, ensuring correctness when the transform
tree structure changes.
