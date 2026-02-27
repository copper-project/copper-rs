#![cfg(feature = "sim")]

extern crate alloc;

mod messages;
mod sim_support;
mod tasks;

use bevy::app::AppExit;
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::prelude::{
    App, Commands, DefaultPlugins, MessageReader, MessageWriter, MinimalPlugins, PluginGroup,
    PostUpdate, ResMut, Resource, Startup, Update, Window, WindowPlugin, default,
};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "copperconfig.ron", sim_mode = true, ignore_resources = true)]
struct FlightControllerSim {}

#[derive(Resource)]
struct CopperState {
    _ctx: CopperContext,
    app: FlightControllerSim,
}

fn default_callback(_step: default::SimStep) -> SimOverride {
    SimOverride::ExecuteByRuntime
}

fn setup_copper(mut commands: Commands) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);
    let logger_path = "logs/flight_controller_sim.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("failed to create logs directory");
    }

    let ctx = basic_copper_setup(&PathBuf::from(logger_path), LOG_SLAB_SIZE, true, None)
        .expect("failed to setup logger");

    let mut app = FlightControllerSimBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("failed to create runtime");

    app.start_all_tasks(&mut default_callback)
        .expect("failed to start tasks");

    commands.insert_resource(CopperState { _ctx: ctx, app });
}

fn run_copper(mut copper: ResMut<CopperState>, mut exit_writer: MessageWriter<AppExit>) {
    if let Err(err) = copper.app.run_one_iteration(&mut default_callback) {
        error!("sim loop stopped: {}", err);
        exit_writer.write(AppExit::Success);
    }
}

fn track_sim_led_state() {
    let _on = sim_support::sim_activity_led_is_on();
    // TODO: drive an in-world 3D LED entity/material from `_on`.
}

fn stop_copper_on_exit(mut exit_events: MessageReader<AppExit>, mut copper: ResMut<CopperState>) {
    for _ in exit_events.read() {
        let _ = copper.app.stop_all_tasks(&mut default_callback);
        let _ = copper.app.log_shutdown_completed();
    }
}

pub fn make_world(headless: bool) -> App {
    let mut app = App::new();
    if headless {
        app.add_plugins(MinimalPlugins);
    } else {
        app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Copper Flight Controller Sim".into(),
                ..default()
            }),
            ..default()
        }));
    }

    app.add_systems(Startup, setup_copper);
    app.add_systems(Update, (run_copper, track_sim_led_state).chain());
    app.add_systems(PostUpdate, stop_copper_on_exit);
    app
}

fn main() {
    let mut app = make_world(false);
    app.run();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_world_starts() {
        let mut app = make_world(true);
        app.update();
    }
}
