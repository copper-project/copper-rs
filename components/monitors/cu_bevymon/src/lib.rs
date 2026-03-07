use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy_ratatui::RatatuiContext;
use cu_tuimon::MonitorUi;
use cu29::context::CuContext;
use cu29::monitoring::{
    ComponentId, CopperListIoStats, CopperListView, CuComponentState, CuMonitor,
    CuMonitoringMetadata, CuMonitoringRuntime, Decision,
};
use cu29::{CuError, CuResult};

pub use cu_tuimon::{MonitorModel, MonitorScreen, MonitorUiOptions, ScrollDirection};

pub struct CuBevyMon {
    model: MonitorModel,
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
        })
    }

    fn process_copperlist(&self, ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
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
}

impl CuBevyMonPlugin {
    pub fn new(model: MonitorModel) -> Self {
        Self {
            model,
            options: MonitorUiOptions::default(),
        }
    }

    pub fn with_options(mut self, options: MonitorUiOptions) -> Self {
        self.options = options;
        self
    }
}

impl Plugin for CuBevyMonPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(bevy_ratatui::context::ContextPlugin)
            .insert_resource(CuBevyMonModel(self.model.clone()))
            .insert_resource(CuBevyMonUiState(MonitorUi::new(
                self.model.clone(),
                self.options.clone(),
            )))
            .add_systems(PostStartup, setup_terminal_texture)
            .add_systems(Update, (draw_bevymon, render_terminal_to_handle))
            .add_systems(PostUpdate, resize_terminal_to_panel);
    }
}

fn setup_terminal_texture(
    mut commands: Commands,
    context: ResMut<RatatuiContext>,
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
    mut context: ResMut<RatatuiContext>,
    mut ui_state: ResMut<CuBevyMonUiState>,
) -> Result {
    context.draw(|frame| {
        ui_state.0.draw(frame);
    })?;
    Ok(())
}

fn render_terminal_to_handle(
    context: ResMut<RatatuiContext>,
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
    let data_out = image.data.as_mut().expect("Image data missing");
    let (pixels_in, _) = data_in.as_chunks::<3>();
    let (pixels_out, _) = data_out.as_chunks_mut::<4>();
    for idx in 0..(width * height) as usize {
        let px_out = &mut pixels_out[idx];
        let px_in = pixels_in[idx];
        px_out[0] = px_in[0];
        px_out[1] = px_in[1];
        px_out[2] = px_in[2];
    }
}

fn resize_terminal_to_panel(
    mut context: ResMut<RatatuiContext>,
    panels: Query<&ComputedNode, With<CuBevyMonPanel>>,
) {
    let Some(panel) = panels.iter().next() else {
        return;
    };
    let size = panel.size();
    if size.x <= 1.0 || size.y <= 1.0 {
        return;
    }

    let char_width = context.backend().char_width.max(1) as f32;
    let char_height = context.backend().char_height.max(1) as f32;
    let cols = (size.x / char_width).floor().max(1.0) as u16;
    let rows = (size.y / char_height).floor().max(1.0) as u16;
    context.backend_mut().resize(cols, rows);
}
