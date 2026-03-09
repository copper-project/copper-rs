use bevy::prelude::*;
use bevy::ui::{Node as UiNode, UiRect, UiTargetCamera, widget::ViewportNode};

use crate::{CuBevyMonFocusBorder, CuBevyMonPanel, CuBevyMonSurface, CuBevyMonSurfaceNode};

#[derive(Component)]
pub struct CuBevyMonSplitRoot;

#[derive(Component)]
pub struct CuBevyMonSplitSimPanel;

#[derive(Component)]
pub struct CuBevyMonSplitMonitorPanel;

#[derive(Clone, Copy, Debug)]
pub struct CuBevyMonSplitStyle {
    pub outer_padding_px: f32,
    pub panel_gap_px: f32,
    pub sim_panel_percent: f32,
    pub monitor_panel_percent: f32,
    pub monitor_panel_inset_px: f32,
    pub focused_border_color: Color,
    pub unfocused_border_color: Color,
    pub focused_border_width_px: f32,
    pub unfocused_border_width_px: f32,
    pub monitor_background: Color,
}

impl Default for CuBevyMonSplitStyle {
    fn default() -> Self {
        Self {
            outer_padding_px: 12.0,
            panel_gap_px: 12.0,
            sim_panel_percent: 67.0,
            monitor_panel_percent: 33.0,
            monitor_panel_inset_px: 4.0,
            focused_border_color: Color::srgb(0.38, 0.40, 0.43),
            unfocused_border_color: Color::BLACK,
            focused_border_width_px: 1.0,
            unfocused_border_width_px: 1.0,
            monitor_background: Color::srgba(0.04, 0.05, 0.08, 0.98),
        }
    }
}

pub struct CuBevyMonSplitLayoutConfig {
    pub scene_camera: Entity,
    pub ui_camera: Option<Entity>,
    pub style: CuBevyMonSplitStyle,
}

impl CuBevyMonSplitLayoutConfig {
    pub fn new(scene_camera: Entity) -> Self {
        Self {
            scene_camera,
            ui_camera: None,
            style: CuBevyMonSplitStyle::default(),
        }
    }

    pub fn with_ui_camera(mut self, ui_camera: Entity) -> Self {
        self.ui_camera = Some(ui_camera);
        self
    }

    pub fn with_style(mut self, style: CuBevyMonSplitStyle) -> Self {
        self.style = style;
        self
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CuBevyMonSplitLayoutEntities {
    pub root: Entity,
    pub sim_panel: Entity,
    pub monitor_panel: Entity,
    pub monitor_texture: Entity,
}

pub fn spawn_split_layout(
    commands: &mut Commands,
    monitor_texture: Handle<Image>,
    config: CuBevyMonSplitLayoutConfig,
) -> CuBevyMonSplitLayoutEntities {
    let style = config.style;
    let sim_border = CuBevyMonFocusBorder::new(CuBevyMonSurface::Sim)
        .with_colors(style.focused_border_color, style.unfocused_border_color)
        .with_widths(
            style.focused_border_width_px,
            style.unfocused_border_width_px,
        );
    let monitor_border = CuBevyMonFocusBorder::new(CuBevyMonSurface::Monitor)
        .with_colors(style.focused_border_color, style.unfocused_border_color)
        .with_widths(
            style.focused_border_width_px,
            style.unfocused_border_width_px,
        );

    let mut root_builder = commands.spawn((
        UiNode {
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            flex_direction: FlexDirection::Row,
            padding: UiRect::all(Val::Px(style.outer_padding_px)),
            column_gap: Val::Px(style.panel_gap_px),
            ..default()
        },
        BackgroundColor(Color::NONE),
        CuBevyMonSplitRoot,
    ));
    if let Some(ui_camera) = config.ui_camera {
        root_builder.insert(UiTargetCamera(ui_camera));
    }

    let mut sim_panel = None;
    let mut monitor_panel = None;
    let mut monitor_texture_node = None;
    let root = root_builder
        .with_children(|parent| {
            sim_panel = Some(
                parent
                    .spawn((
                        UiNode {
                            height: Val::Percent(100.0),
                            min_width: Val::Px(0.0),
                            flex_basis: Val::Px(0.0),
                            flex_grow: style.sim_panel_percent,
                            position_type: PositionType::Relative,
                            border: UiRect::all(Val::Px(style.unfocused_border_width_px)),
                            ..default()
                        },
                        ViewportNode::new(config.scene_camera),
                        BackgroundColor(Color::NONE),
                        BorderColor::all(style.unfocused_border_color),
                        CuBevyMonSurfaceNode(CuBevyMonSurface::Sim),
                        sim_border,
                        CuBevyMonSplitSimPanel,
                    ))
                    .id(),
            );

            monitor_panel = Some(
                parent
                    .spawn((
                        UiNode {
                            height: Val::Percent(100.0),
                            min_width: Val::Px(0.0),
                            flex_basis: Val::Px(0.0),
                            flex_grow: style.monitor_panel_percent,
                            border: UiRect::all(Val::Px(style.unfocused_border_width_px)),
                            padding: UiRect::all(Val::Px(style.monitor_panel_inset_px)),
                            ..default()
                        },
                        BackgroundColor(style.monitor_background),
                        BorderColor::all(style.unfocused_border_color),
                        CuBevyMonSurfaceNode(CuBevyMonSurface::Monitor),
                        monitor_border,
                        CuBevyMonSplitMonitorPanel,
                    ))
                    .with_children(|panel| {
                        monitor_texture_node = Some(
                            panel
                                .spawn((
                                    ImageNode::new(monitor_texture.clone()),
                                    UiNode {
                                        width: Val::Percent(100.0),
                                        height: Val::Percent(100.0),
                                        ..default()
                                    },
                                    BackgroundColor(Color::BLACK),
                                    CuBevyMonPanel,
                                ))
                                .id(),
                        );
                    })
                    .id(),
            );
        })
        .id();

    CuBevyMonSplitLayoutEntities {
        root,
        sim_panel: sim_panel.expect("split layout missing sim panel"),
        monitor_panel: monitor_panel.expect("split layout missing monitor panel"),
        monitor_texture: monitor_texture_node.expect("split layout missing monitor texture"),
    }
}
