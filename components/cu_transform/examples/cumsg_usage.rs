use cu29::clock::{CuDuration, RobotClock, Tov};
use cu29::prelude::CuMsg;
use cu_spatial_payloads::Transform3D;
use cu_transform::{FrameIdString, TransformMsg, TransformTree};

fn main() {
    println!("Cu Transform - CuMsg Pattern Demo");
    println!("=================================");
    println!();

    // Create a transform tree
    let mut tree = TransformTree::<f32>::new();
    let clock = RobotClock::default();

    // Create transforms using the new CuMsg pattern
    println!("Creating transforms with CuMsg...");

    // World to base transform
    let transform1 = Transform3D::from_matrix([
        [1.0, 0.0, 0.0, 0.0], // Column-major: each inner array is a column
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0, 1.0], // Translation (1, 0, 0)
    ]);

    let msg1 = TransformMsg::new(
        transform1,
        FrameIdString::from("world").expect("Frame name too long"),
        FrameIdString::from("base").expect("Frame name too long"),
    );
    let mut cu_msg1 = CuMsg::new(Some(msg1));
    cu_msg1.metadata.tov = Tov::Time(CuDuration(1_000_000_000)); // 1 second

    // Add to tree using the new API
    tree.add_transform_msg(&cu_msg1)
        .expect("Failed to add transform");
    println!("  Added world->base transform at t=1s");

    // Base to arm transform
    let transform2 = Transform3D::from_matrix([
        [0.0, 1.0, 0.0, 0.0], // 90-degree rotation around Z
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.5, 0.0, 0.0, 1.0], // Translation (0.5, 0, 0)
    ]);

    let msg2 = TransformMsg::new(
        transform2,
        FrameIdString::from("base").expect("Frame name too long"),
        FrameIdString::from("arm").expect("Frame name too long"),
    );
    let mut cu_msg2 = CuMsg::new(Some(msg2));
    cu_msg2.metadata.tov = Tov::Time(CuDuration(1_000_000_000)); // 1 second

    tree.add_transform_msg(&cu_msg2)
        .expect("Failed to add transform");
    println!("  Added base->arm transform at t=1s");

    // Demonstrate using time ranges for a broadcast
    println!("\nBroadcasting multiple transforms with time range...");

    // Create multiple transforms at different times
    let times = vec![
        CuDuration(2_000_000_000), // 2 seconds
        CuDuration(2_100_000_000), // 2.1 seconds
        CuDuration(2_200_000_000), // 2.2 seconds
    ];

    for (i, &time) in times.iter().enumerate() {
        let x_translation = 1.0 + (i as f32) * 0.1;
        let transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [x_translation, 0.0, 0.0, 1.0],
        ]);

        let msg = TransformMsg::new(
            transform,
            FrameIdString::from("world").expect("Frame name too long"),
            FrameIdString::from("base").expect("Frame name too long"),
        );
        let mut cu_msg = CuMsg::new(Some(msg));

        // For a broadcast with multiple transforms, use Range
        if i == 0 {
            cu_msg.metadata.tov = Tov::Range(cu29::clock::CuTimeRange {
                start: times[0],
                end: *times.last().unwrap(),
            });
        } else {
            cu_msg.metadata.tov = Tov::Time(time);
        }

        tree.add_transform_msg(&cu_msg)
            .expect("Failed to add transform");
    }

    println!("  Added 3 transforms with time range 2.0s - 2.2s");

    // Query the tree
    println!("\nQuerying transforms...");

    let result = tree.lookup_transform("world", "arm", CuDuration(1_000_000_000), &clock);
    match result {
        Ok(transform) => {
            let mat = transform.to_matrix();
            println!(
                "  World->arm at t=1s: translation=({:.2}, {:.2}, {:.2})",
                mat[3][0], mat[3][1], mat[3][2]
            );
        }
        Err(e) => println!("  Error: {}", e),
    }

    // Query velocity (requires transforms at multiple times)
    let velocity = tree.lookup_velocity("world", "base", CuDuration(2_100_000_000), &clock);
    match velocity {
        Ok(vel) => {
            println!(
                "  World->base velocity at t=2.1s: linear=({:.2}, {:.2}, {:.2}) m/s",
                vel.linear[0], vel.linear[1], vel.linear[2]
            );
        }
        Err(e) => println!("  Error computing velocity: {}", e),
    }

    println!("\nKey advantages of CuMsg pattern:");
    println!("- Timestamps handled by CuMsg metadata (Tov)");
    println!("- Supports time ranges for broadcasts");
    println!("- Integrates with Copper message system");
    println!("- No separate StampedTransform type needed");
}
