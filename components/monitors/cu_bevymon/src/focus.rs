use bevy::math::Rect;
use bevy::prelude::*;
use bevy::ui::UiGlobalTransform;
use bevy::window::PrimaryWindow;

const DEFAULT_FOCUSED_BORDER: Color = Color::srgb(0.38, 0.40, 0.43);
const DEFAULT_UNFOCUSED_BORDER: Color = Color::BLACK;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CuBevyMonSurface {
    Sim,
    #[default]
    Monitor,
}

#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CuBevyMonFocus(pub CuBevyMonSurface);

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CuBevyMonSurfaceNode(pub CuBevyMonSurface);

#[derive(Component, Clone, Copy, Debug)]
pub struct CuBevyMonFocusBorder {
    pub surface: CuBevyMonSurface,
    pub focused_color: Color,
    pub unfocused_color: Color,
    pub focused_width_px: f32,
    pub unfocused_width_px: f32,
}

impl CuBevyMonFocusBorder {
    pub fn new(surface: CuBevyMonSurface) -> Self {
        Self {
            surface,
            focused_color: DEFAULT_FOCUSED_BORDER,
            unfocused_color: DEFAULT_UNFOCUSED_BORDER,
            focused_width_px: 1.0,
            unfocused_width_px: 1.0,
        }
    }

    pub fn with_colors(mut self, focused_color: Color, unfocused_color: Color) -> Self {
        self.focused_color = focused_color;
        self.unfocused_color = unfocused_color;
        self
    }

    pub fn with_widths(mut self, focused_width_px: f32, unfocused_width_px: f32) -> Self {
        self.focused_width_px = focused_width_px;
        self.unfocused_width_px = unfocused_width_px;
        self
    }
}

pub(crate) fn update_surface_focus_from_click(
    window: Single<&Window, With<PrimaryWindow>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut focus: ResMut<CuBevyMonFocus>,
    surfaces: Query<(&ComputedNode, &UiGlobalTransform, &CuBevyMonSurfaceNode)>,
) {
    if !mouse_buttons.just_pressed(MouseButton::Left) {
        return;
    }

    let Some(cursor) = window.physical_cursor_position() else {
        return;
    };

    let next_focus = surfaces
        .iter()
        .filter(|(node, transform, _)| node.contains_point(**transform, cursor))
        .max_by_key(|(node, _, _)| node.stack_index())
        .map(|(_, _, surface)| surface.0);

    if let Some(surface) = next_focus {
        focus.0 = surface;
    }
}

pub(crate) fn update_surface_focus_borders(
    focus: Res<CuBevyMonFocus>,
    mut nodes: Query<(&CuBevyMonFocusBorder, &mut BorderColor, &mut Node)>,
) {
    for (border, mut color, mut node) in &mut nodes {
        let is_focused = focus.0 == border.surface;
        let border_color = if is_focused {
            border.focused_color
        } else {
            border.unfocused_color
        };
        let border_width = if is_focused {
            border.focused_width_px
        } else {
            border.unfocused_width_px
        };

        *color = BorderColor::all(border_color);
        node.border = UiRect::all(Val::Px(border_width));
    }
}

pub(crate) fn local_cursor_position(
    window: &Window,
    node: &ComputedNode,
    transform: &UiGlobalTransform,
) -> Option<Vec2> {
    let cursor = window.physical_cursor_position()?;
    if !node.contains_point(*transform, cursor) {
        return None;
    }

    transform
        .try_inverse()
        .map(|transform| transform.transform_point2(cursor) + 0.5 * node.size())
}

pub(crate) fn surface_local_rect(node_rect: Rect, transform: &UiGlobalTransform) -> Rect {
    let (_, _, translation) = transform.to_scale_angle_translation();
    Rect::from_corners(translation + node_rect.min, translation + node_rect.max)
}
