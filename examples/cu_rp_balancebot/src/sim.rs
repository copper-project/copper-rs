mod motor_model;
mod sim_driver;
pub mod tasks;
mod world;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotSim {}

use bevy::asset::{AssetApp, UnapprovedPathMode};
use bevy::prelude::{
    App, AssetPlugin, DefaultPlugins, FixedUpdate, Font, ImagePlugin, Mesh, MinimalPlugins,
    PluginGroup, PostUpdate, SceneSpawner, StandardMaterial, Startup, Window, WindowPlugin,
    default,
};
use bevy::render::RenderPlugin;
use bevy::scene::ScenePlugin;
use cu29::prelude::*;

pub fn make_world(headless: bool) -> App {
    let mut app = App::new();
    #[cfg(target_os = "macos")]
    let render_plugin = RenderPlugin::default();

    #[cfg(not(target_os = "macos"))]
    let render_plugin = RenderPlugin {
        render_creation: bevy::render::settings::WgpuSettings {
            backends: Some(bevy::render::settings::Backends::VULKAN),
            ..Default::default()
        }
        .into(),
        ..Default::default()
    };

    if headless {
        app.add_plugins((
            MinimalPlugins,
            AssetPlugin {
                unapproved_path_mode: UnapprovedPathMode::Allow,
                ..default()
            },
            ScenePlugin,
            ImagePlugin::default(),
        ));

        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();
        app.init_asset::<Font>();
        app.init_resource::<SceneSpawner>();
    } else {
        app.add_plugins(
            DefaultPlugins
                .set(render_plugin)
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Copper Simulator".into(),
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    unapproved_path_mode: UnapprovedPathMode::Allow,
                    ..default()
                }),
        );
    };

    world::build_world(&mut app, headless, false);
    app.add_systems(Startup, sim_driver::setup_native_copper);
    app.add_systems(FixedUpdate, sim_driver::run_copper_callback);
    app.add_systems(PostUpdate, sim_driver::stop_copper_on_exit);
    app
}

fn main() {
    let mut world = make_world(false);
    world.run();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_balancebot_runs() {
        let mut world = make_world(true);
        world.update();
    }
}
