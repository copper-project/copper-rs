mod motor_model;
mod sim_driver;
pub mod tasks;
mod world;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotSim {}

#[cfg(target_arch = "wasm32")]
use bevy::asset::AssetMetaCheck;
use bevy::asset::UnapprovedPathMode;
use bevy::camera::ClearColorConfig;
use bevy::prelude::{
    App, AssetPlugin, BackgroundColor, BorderColor, Camera, Camera2d, ClearColor, Color, Commands,
    DefaultPlugins, Entity, FixedUpdate, ImageNode, Pickable, PluginGroup, PostUpdate, Query, Res,
    ResMut, Resource, Startup, Text, TextColor, TextFont, Update, Val, Window, WindowPlugin, With,
    default,
};
use bevy::render::RenderPlugin;
use bevy::ui::{Node as UiNode, PositionType, UiRect, UiTargetCamera, widget::ViewportNode};
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

#[derive(Resource)]
struct SplitUiCamera(Entity);

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
    .add_systems(Update, spawn_balancebot_layout);

    world::build_world(&mut app, false, true);
    app.add_systems(
        FixedUpdate,
        sim_driver::run_copper_callback::<LoggerRuntime>,
    );
    app.add_systems(PostUpdate, sim_driver::stop_copper_on_exit::<LoggerRuntime>);
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

    commands
        .spawn((
            UiNode {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                padding: UiRect::all(Val::Px(12.0)),
                column_gap: Val::Px(12.0),
                ..default()
            },
            UiTargetCamera(ui_camera.0),
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
                ViewportNode::new(scene_camera),
                BackgroundColor(Color::NONE),
                BorderColor::all(Color::srgb(0.23, 0.28, 0.33)),
                CuBevyMonSurfaceNode(CuBevyMonSurface::Sim),
                CuBevyMonFocusBorder::new(CuBevyMonSurface::Sim),
            ))
            .with_children(|panel| {
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
                            Text::new(
                                "Move\nNavigation\nInteract\nZoom\nPause/Resume\nReset\nShow Forces",
                            ),
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
