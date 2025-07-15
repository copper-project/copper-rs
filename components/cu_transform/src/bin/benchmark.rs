use cu29::clock::{CuDuration, Tov};
use cu29::prelude::{CuMsg, RobotClock};
use cu_spatial_payloads::Transform3D;
use cu_transform::{TransformPayload, TransformTree};
use std::time::Instant;

fn main() {
    println!("Transform Tree Performance Benchmark");
    println!("-----------------------------------");

    // Parameters
    let num_frames = 100;
    let num_lookups = 10000;
    let num_threads = 8;

    // Create a transform tree
    let mut tree = TransformTree::<f32>::new();

    // Set up a chain of transforms: base -> frame1 -> frame2 -> ... -> frameN
    println!("Setting up {num_frames} frames in a chain");
    for i in 0..num_frames {
        let parent_str = if i == 0 {
            "base".to_string()
        } else {
            format!("frame{i}")
        };

        let child_str = format!("frame{}", i + 1);

        // Create a simple translation
        let transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        let msg = TransformPayload::new(transform, &parent_str, &child_str);
        let mut cu_msg = CuMsg::new(Some(msg));
        cu_msg.tov = Tov::Time(CuDuration(1000));

        tree.add_transform_msg(&cu_msg).unwrap();
    }

    // Create a RobotClock for timing
    let clock = RobotClock::default();

    // Warm up the cache
    println!("Warming up cache...");
    for _ in 0..100 {
        let _ = tree
            .lookup_transform(
                "base",
                &format!("frame{num_frames}"),
                CuDuration(1000),
                &clock,
            )
            .unwrap();
    }

    // Single-threaded benchmark
    println!("\nSingle-threaded lookup benchmark:");
    let start = Instant::now();
    for _ in 0..num_lookups {
        let _ = tree
            .lookup_transform(
                "base",
                &format!("frame{num_frames}"),
                CuDuration(1000),
                &clock,
            )
            .unwrap();
    }
    let elapsed = start.elapsed();
    println!("  {num_lookups} lookups in {elapsed:?}");
    println!("  Avg: {:?} per lookup", elapsed / num_lookups as u32);

    // Multi-threaded benchmark
    println!("\nMulti-threaded lookup benchmark ({num_threads}x threads):");
    let start = Instant::now();

    // Create threads that all access the same transform tree
    let tree_arc = std::sync::Arc::new(tree);
    let clock_arc = std::sync::Arc::new(clock);
    let mut handles = vec![];

    let lookups_per_thread = num_lookups / num_threads;
    for t in 0..num_threads {
        let tree_clone = tree_arc.clone();
        let clock_clone = clock_arc.clone();

        handles.push(std::thread::spawn(move || {
            for i in 0..lookups_per_thread {
                // Mix up the queries across threads to create some cache misses
                let frame_num = if i % 2 == 0 {
                    num_frames - (t % 10)
                } else {
                    num_frames
                };

                let _ = tree_clone
                    .lookup_transform(
                        "base",
                        &format!("frame{frame_num}"),
                        CuDuration(1000),
                        &clock_clone,
                    )
                    .unwrap();
            }
        }));
    }

    // Wait for all threads to complete
    for handle in handles {
        handle.join().unwrap();
    }

    let elapsed = start.elapsed();
    println!("  {num_lookups} lookups in {elapsed:?}");
    println!("  Avg: {:?} per lookup", elapsed / num_lookups as u32);

    // Cache statistics
    println!("\nCleaning up cache...");
    tree_arc.cleanup_cache(&clock_arc);
}
