mod motor_model;
mod sim_driver;
pub mod tasks;
mod windowing;
mod world;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotSim {}

#[cfg(target_arch = "wasm32")]
use bevy::asset::AssetMetaCheck;
use bevy::asset::UnapprovedPathMode;
use bevy::camera::ClearColorConfig;
use bevy::prelude::{
    App, AssetPlugin, BackgroundColor, BorderColor, Camera, Camera2d, ClearColor, Color, Commands,
    Component, DefaultPlugins, Entity, FixedUpdate, Pickable, PluginGroup, PostUpdate, Query, Res,
    ResMut, Resource, Startup, Text, TextColor, TextFont, Update, Val, Visibility, Window,
    WindowPlugin, With, default,
};
use bevy::render::RenderPlugin;
use bevy::ui::{Node as UiNode, PositionType, UiRect};
use cu_bevymon::{
    CuBevyMonPlugin, CuBevyMonSplitLayoutConfig, CuBevyMonSplitStyle, CuBevyMonSurface,
    CuBevyMonTexture, MonitorUiOptions, spawn_split_layout,
};
use cu29::prelude::*;

const SIM_PANEL_PERCENT: f32 = 67.0;
const MONITOR_PANEL_PERCENT: f32 = 33.0;
const MONITOR_PANEL_INSET_PX: f32 = 4.0;

#[derive(Resource, Default)]
struct LayoutSpawned(bool);

#[derive(Resource)]
struct SplitUiCamera(Entity);

#[derive(Component)]
struct SplitLoadingOverlay;

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
            .set(asset_plugin()),
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
    .add_systems(Startup, setup_ui_camera)
    .add_systems(Update, windowing::set_copper_window_icon)
    .add_systems(Update, spawn_balancebot_layout)
    .add_systems(Update, sync_split_loading_overlay);

    world::build_world(&mut app, false, true);
    app.add_systems(FixedUpdate, sim_driver::run_copper_callback);
    app.add_systems(PostUpdate, sim_driver::stop_copper_on_exit);
    app.run();
}

fn asset_plugin() -> AssetPlugin {
    #[cfg(target_arch = "wasm32")]
    {
        return AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            unapproved_path_mode: UnapprovedPathMode::Allow,
            ..default()
        };
    }

    #[cfg(not(target_arch = "wasm32"))]
    {
        AssetPlugin {
            unapproved_path_mode: UnapprovedPathMode::Allow,
            ..default()
        }
    }
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
        name: Some("io.github.copper-project.balancebot-bevymon".into()),
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

fn setup_ui_camera(mut commands: Commands) {
    let camera = commands
        .spawn((
            Camera2d,
            Camera {
                order: 1,
                clear_color: ClearColorConfig::None,
                ..default()
            },
        ))
        .id();
    commands.insert_resource(SplitUiCamera(camera));
}

fn spawn_balancebot_layout(
    mut commands: Commands,
    texture: Option<Res<CuBevyMonTexture>>,
    ui_camera: Option<Res<SplitUiCamera>>,
    scene_cameras: Query<Entity, With<world::SplitSceneCamera>>,
    mut spawned: ResMut<LayoutSpawned>,
) {
    if spawned.0 {
        return;
    }
    let Some(texture) = texture else {
        return;
    };
    let Some(ui_camera) = ui_camera else {
        return;
    };
    let Ok(scene_camera) = scene_cameras.single() else {
        return;
    };
    #[cfg(target_os = "macos")]
    let instructions = "WASD / QE\nControl-Click + Drag\nClick + Drag\nScrolling\nSpace\nR\nF";
    #[cfg(not(target_os = "macos"))]
    let instructions = "WASD / QE\nMiddle-Click + Drag\nClick + Drag\nScroll Wheel\nSpace\nR\nF";

    let layout = spawn_split_layout(
        &mut commands,
        texture.0.clone(),
        CuBevyMonSplitLayoutConfig::new(scene_camera)
            .with_ui_camera(ui_camera.0)
            .with_style(CuBevyMonSplitStyle {
                sim_panel_percent: SIM_PANEL_PERCENT,
                monitor_panel_percent: MONITOR_PANEL_PERCENT,
                monitor_panel_inset_px: MONITOR_PANEL_INSET_PX,
                ..default()
            }),
    );

    commands.entity(layout.sim_panel).with_children(|panel| {
        panel
            .spawn((
                UiNode {
                    position_type: PositionType::Absolute,
                    top: Val::Px(18.0),
                    left: Val::Px(0.0),
                    right: Val::Px(0.0),
                    justify_content: bevy::ui::JustifyContent::Center,
                    ..default()
                },
                Pickable::IGNORE,
                BackgroundColor(Color::NONE),
            ))
            .with_children(|loading| {
                loading
                    .spawn((
                        UiNode {
                            padding: UiRect::new(
                                Val::Px(18.0),
                                Val::Px(18.0),
                                Val::Px(10.0),
                                Val::Px(10.0),
                            ),
                            border: UiRect::all(Val::Px(2.0)),
                            border_radius: bevy::ui::BorderRadius::all(Val::Px(12.0)),
                            ..default()
                        },
                        Pickable::IGNORE,
                        SplitLoadingOverlay,
                        BackgroundColor(Color::srgba(0.03, 0.05, 0.09, 0.92)),
                        BorderColor::all(Color::srgba(0.58, 0.74, 0.96, 0.95)),
                    ))
                    .with_children(|cartouche| {
                        cartouche.spawn((
                            Pickable::IGNORE,
                            Text::new("Assets loading..."),
                            TextFont {
                                font_size: 18.0,
                                ..default()
                            },
                            TextColor(Color::srgb(0.93, 0.96, 1.0)),
                        ));
                    });
            });

        panel
            .spawn((
                UiNode {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(5.0),
                    right: Val::Px(5.0),
                    padding: UiRect::new(
                        Val::Px(15.0),
                        Val::Px(15.0),
                        Val::Px(10.0),
                        Val::Px(10.0),
                    ),
                    column_gap: Val::Px(10.0),
                    flex_direction: bevy::ui::FlexDirection::Row,
                    justify_content: bevy::ui::JustifyContent::SpaceBetween,
                    border: UiRect::all(Val::Px(2.0)),
                    border_radius: bevy::ui::BorderRadius::all(Val::Px(10.0)),
                    ..default()
                },
                Pickable::IGNORE,
                BackgroundColor(Color::srgba(0.25, 0.41, 0.88, 0.7)),
                BorderColor::all(Color::srgba(0.8, 0.8, 0.8, 0.7)),
            ))
            .with_children(|help| {
                help.spawn((
                    Pickable::IGNORE,
                    Text::new("Move\nNavigation\nInteract\nZoom\nPause/Resume\nReset\nShow Forces"),
                    TextFont {
                        font_size: 12.0,
                        ..default()
                    },
                    TextColor(Color::srgba(0.25, 0.25, 0.75, 1.0)),
                ));
                help.spawn((
                    Pickable::IGNORE,
                    Text::new(instructions),
                    TextFont {
                        font_size: 12.0,
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    });

    spawned.0 = true;
}

fn sync_split_loading_overlay(
    load_state: Res<world::SceneLoadState>,
    mut overlays: Query<&mut Visibility, With<SplitLoadingOverlay>>,
) {
    if !load_state.ready {
        return;
    }

    for mut visibility in &mut overlays {
        *visibility = Visibility::Hidden;
    }
}
