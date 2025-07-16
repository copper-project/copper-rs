use cu29::clock::{CuDuration, RobotClock, Tov};
use cu_spatial_payloads::Transform3D;
use cu_transform::transform_payload::StampedFrameTransform;
use cu_transform::{FrameIdString, FrameTransform, TransformTree};

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
        [1.0f32, 0.0, 0.0, 0.0], // Column-major: each inner array is a column
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0, 1.0], // Translation (1, 0, 0)
    ]);

    let frame_transform = FrameTransform::new(
        transform1,
        FrameIdString::from("world").expect("Frame name too long"),
        FrameIdString::from("base").expect("Frame name too long"),
    );
    let mut sft = StampedFrameTransform::new(Some(frame_transform));
    sft.tov = Tov::Time(CuDuration(1_000_000_000)); // 1 second

    // Add to tree using the new API
    tree.add_transform_msg(&sft)
        .expect("Failed to add transform");
    println!("  Added world->base transform at t=1s");

    // Base to arm transform
    let transform2 = Transform3D::from_matrix([
        [0.0, 1.0, 0.0, 0.0], // 90-degree rotation around Z
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.5, 0.0, 0.0, 1.0], // Translation (0.5, 0, 0)
    ]);

    let msg2 = FrameTransform::new(
        transform2,
        FrameIdString::from("base").expect("Frame name too long"),
        FrameIdString::from("arm").expect("Frame name too long"),
    );
    let mut sft = StampedFrameTransform::new(Some(msg2));
    sft.tov = Tov::Time(CuDuration(1_000_000_000)); // 1 second

    tree.add_transform_msg(&sft)
        .expect("Failed to add transform");
    println!("  Added base->arm transform at t=1s");

    // Demonstrate using time ranges for a broadcast
    println!("\nBroadcasting multiple transforms with time range...");

    // Create multiple transforms at different times
    let times = [
        CuDuration(2_000_000_000), // 2 seconds
        CuDuration(2_100_000_000), // 2.1 seconds
        CuDuration(2_200_000_000),
    ];

    for (i, &time) in times.iter().enumerate() {
        let x_translation = 1.0 + (i as f32) * 0.1;
        let transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [x_translation, 0.0, 0.0, 1.0],
        ]);

        let msg = FrameTransform::new(
            transform,
            FrameIdString::from("world").expect("Frame name too long"),
            FrameIdString::from("base").expect("Frame name too long"),
        );
        let mut sft = StampedFrameTransform::new(Some(msg));

        // For a broadcast with multiple transforms, use Range
        if i == 0 {
            sft.tov = Tov::Range(cu29::clock::CuTimeRange {
                start: times[0],
                end: *times.last().unwrap(),
            });
        } else {
            sft.tov = Tov::Time(time);
        }

        tree.add_transform_msg(&sft)
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
        Err(e) => println!("  Error: {e}"),
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
        Err(e) => println!("  Error computing velocity: {e}"),
    }

    println!("\nKey advantages of CuMsg pattern:");
    println!("- Integrates with Copper message system");
    println!("- Timestamps handled by CuMsg metadata (Tov)");
    println!("- Supports time ranges for broadcasts");
}
