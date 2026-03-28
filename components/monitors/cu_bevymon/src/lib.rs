mod focus;
mod split;
mod terminal;
mod viewport;

use bevy::asset::RenderAssetUsages;
use bevy::input::mouse::MouseWheel;
use bevy::input::{ButtonState, keyboard::KeyboardInput};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::window::PrimaryWindow;
use cu_tuimon::{MonitorLogCapture, MonitorUi, MonitorUiAction, MonitorUiEvent, MonitorUiKey};
use cu29::context::CuContext;
use cu29::monitoring::{
    ComponentId, CopperListIoStats, CopperListView, CuComponentState, CuMonitor,
    CuMonitoringMetadata, CuMonitoringRuntime, Decision,
};
use cu29::{CuError, CuResult};

pub use cu_tuimon::{MonitorModel, MonitorScreen, MonitorUiOptions, ScrollDirection};
pub use focus::{CuBevyMonFocus, CuBevyMonFocusBorder, CuBevyMonSurface, CuBevyMonSurfaceNode};
pub use split::{
    CuBevyMonSplitLayoutConfig, CuBevyMonSplitLayoutEntities, CuBevyMonSplitMonitorPanel,
    CuBevyMonSplitRoot, CuBevyMonSplitSimPanel, CuBevyMonSplitStyle, spawn_split_layout,
};
pub use terminal::CuBevyMonFontOptions;
use terminal::{CuBevyMonTerminal, sync_terminal_to_panel};
pub use viewport::CuBevyMonViewportSurface;

pub struct CuBevyMon {
    model: MonitorModel,
    log_capture: Option<std::sync::Mutex<MonitorLogCapture>>,
}

impl CuBevyMon {
    pub fn model(&self) -> MonitorModel {
        self.model.clone()
    }
}

impl CuMonitor for CuBevyMon {
    fn new(metadata: CuMonitoringMetadata, _runtime: CuMonitoringRuntime) -> CuResult<Self> {
        Ok(Self {
            model: MonitorModel::from_metadata(&metadata),
            log_capture: None,
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.model.set_instance_id(ctx.instance_id());
        self.log_capture = Some(std::sync::Mutex::new(MonitorLogCapture::to_model(
            self.model.clone(),
        )));
        Ok(())
    }

    fn process_copperlist(&self, ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
        if let Some(log_capture) = &self.log_capture {
            let mut log_capture = log_capture.lock().unwrap_or_else(|err| err.into_inner());
            log_capture.poll();
        }
        self.model.process_copperlist(ctx.cl_id(), view);
        Ok(())
    }

    fn observe_copperlist_io(&self, stats: CopperListIoStats) {
        self.model.observe_copperlist_io(stats);
    }

    fn process_error(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        error: &CuError,
    ) -> Decision {
        self.model
            .set_component_error(component_id, error.to_string());
        match step {
            CuComponentState::Start => Decision::Shutdown,
            CuComponentState::Preprocess => Decision::Abort,
            CuComponentState::Process => Decision::Ignore,
            CuComponentState::Postprocess => Decision::Ignore,
            CuComponentState::Stop => Decision::Shutdown,
        }
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.log_capture = None;
        self.model.reset_latency();
        Ok(())
    }
}

#[derive(Resource, Clone)]
pub struct CuBevyMonModel(pub MonitorModel);

#[derive(Resource)]
pub struct CuBevyMonUiState(pub MonitorUi);

#[derive(Resource, Clone)]
pub struct CuBevyMonTexture(pub Handle<Image>);

#[derive(Component)]
pub struct CuBevyMonPanel;

pub struct CuBevyMonPlugin {
    model: MonitorModel,
    options: MonitorUiOptions,
    font_options: CuBevyMonFontOptions,
    initial_focus: CuBevyMonSurface,
}

impl CuBevyMonPlugin {
    pub fn new(model: MonitorModel) -> Self {
        Self {
            model,
            options: MonitorUiOptions::default(),
            font_options: CuBevyMonFontOptions::default(),
            initial_focus: CuBevyMonSurface::Monitor,
        }
    }

    pub fn with_options(mut self, options: MonitorUiOptions) -> Self {
        self.options = options;
        self
    }

    pub fn with_initial_focus(mut self, initial_focus: CuBevyMonSurface) -> Self {
        self.initial_focus = initial_focus;
        self
    }

    pub fn with_font_options(mut self, font_options: CuBevyMonFontOptions) -> Self {
        self.font_options = font_options;
        self
    }

    pub fn with_font_size(mut self, size_px: u32) -> Self {
        self.font_options = CuBevyMonFontOptions::new(size_px);
        self
    }
}

impl Plugin for CuBevyMonPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(CuBevyMonModel(self.model.clone()))
            .insert_resource(CuBevyMonUiState(MonitorUi::new(
                self.model.clone(),
                self.options.clone(),
            )))
            .insert_resource(self.font_options.clone())
            .insert_resource(CuBevyMonFocus(self.initial_focus))
            .add_systems(Startup, setup_terminal_context)
            .add_systems(PostStartup, setup_terminal_texture)
            .add_systems(
                Update,
                (
                    focus::update_surface_focus_from_click,
                    handle_monitor_pointer_input.after(focus::update_surface_focus_from_click),
                    handle_monitor_scroll_input,
                    handle_monitor_keyboard_input,
                    focus::update_surface_focus_borders,
                    draw_bevymon,
                    render_terminal_to_handle,
                ),
            )
            .add_systems(
                PostUpdate,
                (
                    resize_terminal_to_panel,
                    viewport::sync_camera_viewports_to_surfaces,
                ),
            );
    }
}

fn setup_terminal_context(
    mut commands: Commands,
    font_options: Res<CuBevyMonFontOptions>,
) -> Result {
    commands.insert_resource(CuBevyMonTerminal::from_options(&font_options)?);
    Ok(())
}

fn setup_terminal_texture(
    mut commands: Commands,
    context: ResMut<CuBevyMonTerminal>,
    mut images: ResMut<Assets<Image>>,
) -> Result {
    let width = context.backend().get_pixmap_width() as u32;
    let height = context.backend().get_pixmap_height() as u32;
    let data = context.backend().get_pixmap_data_as_rgba();

    let image = Image::new(
        Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    );
    let handle = images.add(image);
    commands.insert_resource(CuBevyMonTexture(handle));
    Ok(())
}

fn draw_bevymon(
    mut context: ResMut<CuBevyMonTerminal>,
    mut ui_state: ResMut<CuBevyMonUiState>,
) -> Result {
    context.draw(|frame| {
        ui_state.0.draw(frame);
    })?;
    Ok(())
}

fn render_terminal_to_handle(
    context: ResMut<CuBevyMonTerminal>,
    texture: Option<Res<CuBevyMonTexture>>,
    mut images: ResMut<Assets<Image>>,
) {
    let Some(texture) = texture else {
        return;
    };

    let width = context.backend().get_pixmap_width() as u32;
    let height = context.backend().get_pixmap_height() as u32;
    let Some(image) = images.get_mut(&texture.0) else {
        return;
    };

    if image.width() != width || image.height() != height {
        image.resize(Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        });
        image.data = Some(context.backend().get_pixmap_data_as_rgba());
        return;
    }

    let data_in = context.backend().get_pixmap_data();
    let data_out = image.data.as_mut().expect("image data missing");
    let (pixels_in, _) = data_in.as_chunks::<3>();
    let (pixels_out, _) = data_out.as_chunks_mut::<4>();
    for (px_in, px_out) in pixels_in.iter().zip(pixels_out.iter_mut()) {
        px_out[0] = px_in[0];
        px_out[1] = px_in[1];
        px_out[2] = px_in[2];
        px_out[3] = 255;
    }
}

fn handle_monitor_pointer_input(
    window: Single<&Window, With<PrimaryWindow>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    context: Res<CuBevyMonTerminal>,
    mut ui_state: ResMut<CuBevyMonUiState>,
    panels: Query<(&ComputedNode, &bevy::ui::UiGlobalTransform), With<CuBevyMonPanel>>,
) {
    if !mouse_buttons.just_pressed(MouseButton::Left)
        && !mouse_buttons.just_released(MouseButton::Left)
    {
        return;
    }

    let Some((node, transform)) = panels.iter().next() else {
        return;
    };
    let Some(local_point) = focus::local_cursor_position(&window, node, transform) else {
        return;
    };

    let char_width = context.backend().char_width.max(1) as f32;
    let char_height = context.backend().char_height.max(1) as f32;
    let col = (local_point.x / char_width).floor().max(0.0) as u16;
    let row = (local_point.y / char_height).floor().max(0.0) as u16;
    let event = if mouse_buttons.just_pressed(MouseButton::Left) {
        MonitorUiEvent::MouseDown { col, row }
    } else {
        MonitorUiEvent::MouseUp { col, row }
    };
    let _ = ui_state.0.handle_event(event);
}

fn handle_monitor_scroll_input(
    focus: Res<CuBevyMonFocus>,
    mut wheel_events: MessageReader<MouseWheel>,
    mut ui_state: ResMut<CuBevyMonUiState>,
) {
    if focus.0 != CuBevyMonSurface::Monitor {
        return;
    }

    for event in wheel_events.read() {
        if event.y > 0.0 {
            let _ = ui_state.0.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Up,
                steps: 1,
            });
        } else if event.y < 0.0 {
            let _ = ui_state.0.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Down,
                steps: 1,
            });
        }

        if event.x > 0.0 {
            let _ = ui_state.0.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Right,
                steps: 5,
            });
        } else if event.x < 0.0 {
            let _ = ui_state.0.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Left,
                steps: 5,
            });
        }
    }
}

fn handle_monitor_keyboard_input(
    focus: Res<CuBevyMonFocus>,
    mut keyboard_inputs: MessageReader<KeyboardInput>,
    mut exit: MessageWriter<AppExit>,
    mut ui_state: ResMut<CuBevyMonUiState>,
) {
    if focus.0 != CuBevyMonSurface::Monitor {
        return;
    }

    for event in keyboard_inputs.read() {
        if event.state != ButtonState::Pressed {
            continue;
        }

        if let Some(key) = monitor_navigation_key(event.key_code) {
            dispatch_monitor_event(&mut ui_state.0, &mut exit, MonitorUiEvent::Key(key));
        }

        if let Some(text) = &event.text {
            for ch in text.chars().filter(|ch| !ch.is_control()) {
                dispatch_monitor_event(
                    &mut ui_state.0,
                    &mut exit,
                    MonitorUiEvent::Key(MonitorUiKey::Char(ch.to_ascii_lowercase())),
                );
            }
        }
    }
}

fn resize_terminal_to_panel(
    mut context: ResMut<CuBevyMonTerminal>,
    panels: Query<&ComputedNode, With<CuBevyMonPanel>>,
) {
    let Some(panel) = panels.iter().next() else {
        return;
    };
    sync_terminal_to_panel(&mut context, panel.size());
}

fn dispatch_monitor_event(
    ui_state: &mut MonitorUi,
    exit: &mut MessageWriter<AppExit>,
    event: MonitorUiEvent,
) {
    match ui_state.handle_event(event) {
        MonitorUiAction::QuitRequested => {
            exit.write(AppExit::Success);
        }
        MonitorUiAction::None => {}
        MonitorUiAction::CopyLogSelection(_) => {}
    }
}

fn monitor_navigation_key(key_code: KeyCode) -> Option<MonitorUiKey> {
    match key_code {
        KeyCode::ArrowLeft => Some(MonitorUiKey::Left),
        KeyCode::ArrowRight => Some(MonitorUiKey::Right),
        KeyCode::ArrowUp => Some(MonitorUiKey::Up),
        KeyCode::ArrowDown => Some(MonitorUiKey::Down),
        _ => None,
    }
}
