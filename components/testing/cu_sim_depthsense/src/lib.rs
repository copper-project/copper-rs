use bevy::prelude::*;
use bevy_mod_raycast::prelude::*;
use cu29::prelude::*;
use cu_sensor_payloads::{PointCloud, PointCloudSoa};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, raycast)
        .run();
}

const RAY_DIST: Vec3 = Vec3::new(0.0, 0.0, -7.0);

const HZ_PTS: usize = 180;
const BEAMS: usize = 32;
const TOTAL_PTS: usize = HZ_PTS * BEAMS;
const VFOV: f32 = 30.0;

pub type VirtPointCloud = PointCloudSoa<TOTAL_PTS>;

pub fn compute_pointcloud(
    sensor_pos: Vec3,
    raycast: &mut Raycast,
    time: CuTime,
    pt: &mut VirtPointCloud,
) {
    let horizontal_step_angle = 360.0 / HZ_PTS as f32;
    let vertical_step_angle = VFOV / (BEAMS - 1) as f32;
    let mut i = 0usize;
    for h in 0..HZ_PTS {
        let horizontal_angle = h as f32 * horizontal_step_angle;
        let horizontal_rotation = Quat::from_rotation_y(horizontal_angle.to_radians());

        for v in 0..BEAMS {
            let vertical_angle = -VFOV / 2.0 + v as f32 * vertical_step_angle;
            let vertical_rotation = Quat::from_rotation_z(vertical_angle.to_radians());

            // Calculate the ray direction
            let dir = horizontal_rotation * vertical_rotation * Vec3::X;

            // Perform the raycast and get the nearest intersection, if any
            if let Some((_, intersection)) = raycast
                .cast_ray(Ray3d::new(sensor_pos, dir.normalize()), &default())
                .first()
            {
                let lidar_pt = intersection.position();
                pt.push(PointCloud {
                    tov: time,
                    x: lidar_pt.x.into(),
                    y: lidar_pt.y.into(),
                    z: lidar_pt.z.into(),
                    i: 100.0.into(),
                    return_order: 0,
                });
            }
        }
    }
}

fn raycast(mut raycast: Raycast, mut gizmos: Gizmos, time: Res<Time>) {
    let t = time.elapsed_seconds();
    let pos = Vec3::new(t.sin(), (t * 1.5).cos(), t.cos()) + RAY_DIST;

    // Define lidar parameters
    let horizontal_steps = 180; // Number of horizontal points
    let vertical_steps = 32; // 32 beams
    let vertical_fov = 30.0; // +/-15 degrees (total 30 degrees vertical FOV)

    let horizontal_step_angle = 360.0 / horizontal_steps as f32;
    let vertical_step_angle = vertical_fov / (vertical_steps - 1) as f32;

    gizmos.sphere(
        pos,
        Quat::IDENTITY, // No rotation needed for a sphere
        0.02,           // Radius of the sphere
        Color::srgb(1.0, 0.0, 0.0),
    );

    for h in 0..horizontal_steps {
        let horizontal_angle = h as f32 * horizontal_step_angle;
        let horizontal_rotation = Quat::from_rotation_y(horizontal_angle.to_radians());

        for v in 0..vertical_steps {
            let vertical_angle = -vertical_fov / 2.0 + v as f32 * vertical_step_angle;
            let vertical_rotation = Quat::from_rotation_z(vertical_angle.to_radians());

            // Calculate the ray direction
            let dir = horizontal_rotation * vertical_rotation * Vec3::X;

            // Perform the raycast and get the nearest intersection, if any
            if let Some((_, intersection)) = raycast
                .cast_ray(Ray3d::new(pos, dir.normalize()), &default())
                .first()
            {
                // Draw a point at the intersection
                gizmos.sphere(
                    intersection.position(),
                    Quat::IDENTITY, // No rotation needed for a sphere
                    0.005,          // Radius of the sphere
                    Color::srgb(0.0, 0.0, 1.0),
                );
            }
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(Camera3dBundle::default());
    commands.spawn(PointLightBundle::default());
    commands.spawn(PbrBundle {
        mesh: meshes.add(Capsule3d::default()),
        material: materials.add(Color::srgb(1.0, 1.0, 1.0)),
        transform: Transform::from_translation(RAY_DIST),
        ..default()
    });
}
