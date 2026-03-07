use bevy::camera::Viewport;
use bevy::prelude::*;
use bevy::ui::UiGlobalTransform;
use bevy::window::PrimaryWindow;

use crate::focus::{CuBevyMonSurface, CuBevyMonSurfaceNode, surface_local_rect};

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CuBevyMonViewportSurface(pub CuBevyMonSurface);

pub(crate) fn sync_camera_viewports_to_surfaces(
    window: Single<&Window, With<PrimaryWindow>>,
    surfaces: Query<(&ComputedNode, &UiGlobalTransform, &CuBevyMonSurfaceNode)>,
    mut cameras: Query<(&CuBevyMonViewportSurface, &mut Camera)>,
) {
    let window_size = Vec2::new(
        window.physical_width() as f32,
        window.physical_height() as f32,
    );

    for (target_surface, mut camera) in &mut cameras {
        let Some((node, transform, _)) = surfaces
            .iter()
            .find(|(_, _, surface)| surface.0 == target_surface.0)
        else {
            camera.viewport = None;
            continue;
        };

        let rect = surface_local_rect(node.content_box(), transform);
        let min = rect.min.max(Vec2::ZERO);
        let max = rect.max.min(window_size);
        let size = max - min;

        if size.x <= 1.0 || size.y <= 1.0 {
            camera.viewport = None;
            continue;
        }

        camera.viewport = Some(Viewport {
            physical_position: UVec2::new(min.x.floor() as u32, min.y.floor() as u32),
            physical_size: UVec2::new(size.x.floor() as u32, size.y.floor() as u32),
            ..default()
        });
    }
}
