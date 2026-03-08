mod motor_model;
mod sim_driver;
pub mod tasks;
mod world;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotSim {}

use bevy::asset::UnapprovedPathMode;
use bevy::prelude::{
    App, AssetPlugin, BackgroundColor, BorderColor, ClearColor, Color, Commands, DefaultPlugins,
    FixedUpdate, ImageNode, Pickable, PluginGroup, PostUpdate, Res, ResMut, Resource, Update, Val,
    Window, WindowPlugin, default,
};
use bevy::render::RenderPlugin;
use bevy::ui::{Node as UiNode, PositionType, UiRect};
use cu_bevymon::{
    CuBevyMonFocusBorder, CuBevyMonPanel, CuBevyMonPlugin, CuBevyMonSurface, CuBevyMonSurfaceNode,
    CuBevyMonTexture, MonitorUiOptions,
};
use cu29::prelude::*;

const SIM_PANEL_PERCENT: f32 = 67.0;
const MONITOR_PANEL_PERCENT: f32 = 33.0;
const MONITOR_PANEL_INSET_PX: f32 = 4.0;

#[derive(Resource, Default)]
struct LayoutSpawned(bool);

fn main() {
    let (monitor_model, copper) = sim_driver::build_bevymon_copper();

    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .set(render_plugin())
            .set(WindowPlugin {
                primary_window: Some(primary_window()),
                ..default()
            })
            .set(AssetPlugin {
                unapproved_path_mode: UnapprovedPathMode::Allow,
                ..default()
            }),
    )
    .add_plugins(
        CuBevyMonPlugin::new(monitor_model)
            .with_initial_focus(CuBevyMonSurface::Sim)
            .with_options(MonitorUiOptions {
                show_quit_hint: false,
            }),
    )
    .insert_resource(ClearColor(Color::srgb(0.03, 0.04, 0.06)))
    .insert_resource(copper)
    .insert_resource(sim_driver::LastCopperTick::default())
    .init_resource::<LayoutSpawned>()
    .add_systems(Update, spawn_balancebot_layout);

    world::build_world(&mut app, false, true);
    app.add_systems(
        FixedUpdate,
        sim_driver::run_copper_callback::<LoggerRuntime>,
    );
    app.add_systems(PostUpdate, sim_driver::stop_copper_on_exit::<LoggerRuntime>);
    app.run();
}

#[cfg(target_os = "macos")]
fn render_plugin() -> RenderPlugin {
    RenderPlugin::default()
}

#[cfg(all(not(target_os = "macos"), not(target_arch = "wasm32")))]
fn render_plugin() -> RenderPlugin {
    RenderPlugin {
        render_creation: bevy::render::settings::WgpuSettings {
            backends: Some(bevy::render::settings::Backends::VULKAN),
            ..default()
        }
        .into(),
        ..default()
    }
}

#[cfg(target_arch = "wasm32")]
fn render_plugin() -> RenderPlugin {
    RenderPlugin::default()
}

#[cfg(not(target_arch = "wasm32"))]
fn primary_window() -> Window {
    Window {
        title: "Copper BalanceBot BevyMon".into(),
        resolution: (1680, 960).into(),
        ..default()
    }
}

#[cfg(target_arch = "wasm32")]
fn primary_window() -> Window {
    Window {
        title: "Copper BalanceBot BevyMon".into(),
        resolution: (1680, 960).into(),
        canvas: Some("#bevy".into()),
        fit_canvas_to_parent: true,
        ..default()
    }
}

fn spawn_balancebot_layout(
    mut commands: Commands,
    texture: Option<Res<CuBevyMonTexture>>,
    mut spawned: ResMut<LayoutSpawned>,
) {
    if spawned.0 {
        return;
    }
    let Some(texture) = texture else {
        return;
    };

    commands
        .spawn((
            UiNode {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                padding: UiRect::all(Val::Px(12.0)),
                column_gap: Val::Px(12.0),
                ..default()
            },
            Pickable::IGNORE,
            BackgroundColor(Color::NONE),
        ))
        .with_children(|parent| {
            parent.spawn((
                UiNode {
                    width: Val::Percent(SIM_PANEL_PERCENT),
                    height: Val::Percent(100.0),
                    position_type: PositionType::Relative,
                    border: UiRect::all(Val::Px(1.0)),
                    ..default()
                },
                Pickable::IGNORE,
                BackgroundColor(Color::NONE),
                BorderColor::all(Color::srgb(0.23, 0.28, 0.33)),
                CuBevyMonSurfaceNode(CuBevyMonSurface::Sim),
                CuBevyMonFocusBorder::new(CuBevyMonSurface::Sim),
            ));

            parent
                .spawn((
                    UiNode {
                        width: Val::Percent(MONITOR_PANEL_PERCENT),
                        height: Val::Percent(100.0),
                        border: UiRect::all(Val::Px(1.0)),
                        padding: UiRect::all(Val::Px(MONITOR_PANEL_INSET_PX)),
                        ..default()
                    },
                    Pickable::IGNORE,
                    BackgroundColor(Color::srgba(0.04, 0.05, 0.08, 0.98)),
                    BorderColor::all(Color::srgb(0.23, 0.28, 0.33)),
                    CuBevyMonSurfaceNode(CuBevyMonSurface::Monitor),
                    CuBevyMonFocusBorder::new(CuBevyMonSurface::Monitor),
                ))
                .with_children(|panel| {
                    panel.spawn((
                        ImageNode::new(texture.0.clone()),
                        UiNode {
                            width: Val::Percent(100.0),
                            height: Val::Percent(100.0),
                            ..default()
                        },
                        Pickable::IGNORE,
                        BackgroundColor(Color::BLACK),
                        CuBevyMonPanel,
                    ));
                });
        });

    spawned.0 = true;
}
