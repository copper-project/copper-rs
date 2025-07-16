use cu29::clock::{CuDuration, Tov};
use cu_spatial_payloads::Transform3D;
use cu_transform::transform_payload::StampedFrameTransform;
use cu_transform::{
    ConstTransformBuffer, FrameIdString, FrameTransform, RobotFrame, StampedTransform,
    TransformTree, TypedTransform, TypedTransformBuffer, WorldFrame,
};

fn main() {
    // Example using the typed transform approach
    println!("Cu Transform - New Typed Approach Demo");
    println!("=====================================");

    // Create a buffer for world -> robot transforms
    let mut world_to_robot_buffer: TypedTransformBuffer<f32, WorldFrame, RobotFrame, 10> =
        TypedTransformBuffer::new();

    // Create a transform message
    let transform = Transform3D::from_matrix([
        [1.0, 0.0, 0.0, 1.0], // X translation
        [0.0, 1.0, 0.0, 2.0], // Y translation
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    let world_to_robot_msg = TypedTransform::new(transform, CuDuration(1000));

    println!(
        "Created transform from {} to {}",
        world_to_robot_msg.parent_name(),
        world_to_robot_msg.child_name()
    );
    if let Some(t) = world_to_robot_msg.transform() {
        let mat = t.to_matrix();
        println!(
            "  Translation: [{}, {}, {}]",
            mat[0][3], mat[1][3], mat[2][3]
        );
    }

    // Add to buffer
    world_to_robot_buffer.add_transform(world_to_robot_msg);

    // Create second transform
    let transform2 = Transform3D::from_matrix([
        [1.0, 0.0, 0.0, 2.0], // X translation
        [0.0, 1.0, 0.0, 4.0], // Y translation
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    let world_to_robot_msg2 = TypedTransform::new(transform2, CuDuration(2000));
    world_to_robot_buffer.add_transform(world_to_robot_msg2);

    // Query the buffer
    if let Some(latest) = world_to_robot_buffer.get_latest_transform() {
        println!(
            "\nLatest transform at time {}:",
            latest.timestamp().unwrap().as_nanos()
        );
        if let Some(t) = latest.transform() {
            let mat = t.to_matrix();
            println!(
                "  Translation: [{}, {}, {}]",
                mat[0][3], mat[1][3], mat[2][3]
            );
        }
    }

    // Query closest to a specific time
    if let Some(closest) = world_to_robot_buffer.get_closest_transform(CuDuration(1500)) {
        println!("\nClosest transform to time 1500:");
        println!("  Actual time: {}", closest.timestamp().unwrap().as_nanos());
        if let Some(t) = closest.transform() {
            let mat = t.to_matrix();
            println!(
                "  Translation: [{}, {}, {}]",
                mat[0][3], mat[1][3], mat[2][3]
            );
        }
    }

    // Demonstrate time range
    if let Some(range) = world_to_robot_buffer.get_time_range() {
        println!(
            "\nTime range: {} to {}",
            range.start.as_nanos(),
            range.end.as_nanos()
        );
    }

    // Demonstrate velocity computation
    if let Some(latest) = world_to_robot_buffer.get_latest_transform() {
        if let Some(closest) = world_to_robot_buffer.get_closest_transform(CuDuration(1000)) {
            if let Some(velocity) = latest.compute_velocity(closest) {
                println!("\nVelocity computation:");
                println!(
                    "  Linear velocity: [{}, {}, {}]",
                    velocity.linear[0], velocity.linear[1], velocity.linear[2]
                );
            }
        }
    }

    // Demonstrate the stringly typed version of the API.
    println!("\n\nConstant-Size Buffer Demo");
    println!("===========================================");

    let mut const_buffer: ConstTransformBuffer<f32, 5> = ConstTransformBuffer::new();

    // Add some stamped transforms
    let stamped_transform = StampedTransform {
        transform,
        stamp: CuDuration(1000),
        parent_frame: FrameIdString::from("world").unwrap(),
        child_frame: FrameIdString::from("robot").unwrap(),
    };

    const_buffer.add_transform(stamped_transform);

    if let Some(latest_stamped) = const_buffer.get_latest_transform() {
        println!("Latest transform in constant buffer:");
        println!(
            "  From: {} to: {}",
            latest_stamped.parent_frame, latest_stamped.child_frame
        );
        println!("  Time: {}", latest_stamped.stamp.as_nanos());
        let mat = latest_stamped.transform.to_matrix();
        println!(
            "  Translation: [{}, {}, {}]",
            mat[0][3], mat[1][3], mat[2][3]
        );
    }

    println!("\nThis buffer is stack-allocated with capacity 5 - no heap allocation!");

    // Demonstrate the StampedFrameTransfrom pattern with TransformTree
    println!("\n\nCuMsg<TransformMsg> Pattern Demo");
    println!("================================");

    let mut tree = TransformTree::<f32>::new();

    // Create a CuMsg with TransformMsg
    let frame_transform = FrameTransform::new(
        transform,
        FrameIdString::from("world").expect("Frame name too long"),
        FrameIdString::from("robot").expect("Frame name too long"),
    );

    let mut sft = StampedFrameTransform::new(Some(frame_transform));
    sft.tov = Tov::Time(CuDuration(1_000_000_000)); // 1 second

    // Add using the new API
    tree.add_transform_msg(&sft)
        .expect("Failed to add transform");
    println!("Added transform using CuMsg<TransformMsg> pattern");

    // Query the transform
    let robot_clock = cu29::clock::RobotClock::default();
    let result = tree.lookup_transform("world", "robot", CuDuration(1_000_000_000), &robot_clock);

    match result {
        Ok(transform) => {
            let mat = transform.to_matrix();
            println!(
                "Retrieved transform: translation=({:.2}, {:.2}, {:.2})",
                mat[3][0], mat[3][1], mat[3][2]
            );
        }
        Err(e) => println!("Error: {e}"),
    }
}
