use cu29::clock::CuDuration;
use cu_spatial_payloads::Transform3D;
use cu_transform::{StampedTransform, TransformTree};

fn main() {
    let mut tree = TransformTree::<f32>::new();

    let world_to_robot = StampedTransform {
        transform: {
            let mut t = Transform3D::default();
            t.mat[0][3] = 1.0; // X translation
            t.mat[1][3] = 2.0; // Y translation
            t
        },
        stamp: CuDuration(1000),
        parent_frame: "world".to_string(),
        child_frame: "robot".to_string(),
    };

    let robot_to_camera = StampedTransform {
        transform: {
            let mut t = Transform3D::default();
            t.mat[0][3] = 0.2; // X translation
            t.mat[1][3] = 0.0; // Y translation
            t.mat[2][3] = 0.3; // Z translation
            t
        },
        stamp: CuDuration(1000),
        parent_frame: "robot".to_string(),
        child_frame: "camera".to_string(),
    };

    tree.add_transform(world_to_robot)
        .expect("Failed to add world_to_robot transform");
    tree.add_transform(robot_to_camera)
        .expect("Failed to add robot_to_camera transform");

    let time = CuDuration(1000);
    let transform = tree.lookup_transform("world", "camera", time);

    match transform {
        Ok(t) => {
            println!("Transform from world to camera:");
            println!(
                "Translation: [{}, {}, {}]",
                t.mat[0][3], t.mat[1][3], t.mat[2][3]
            );
        }
        Err(e) => {
            println!("Error looking up transform: {:?}", e);
        }
    }
}
