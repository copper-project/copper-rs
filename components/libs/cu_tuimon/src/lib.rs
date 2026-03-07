#[cfg(feature = "log_pane")]
mod logpane;
mod model;
mod system_info;
mod tui_nodes;
mod ui;

#[cfg(feature = "log_pane")]
pub use logpane::{MonitorLogCapture, StyledLine, StyledRun};
pub use model::MonitorModel;
pub use ui::{
    MonitorScreen, MonitorUi, MonitorUiAction, MonitorUiEvent, MonitorUiKey, MonitorUiOptions,
    ScrollDirection,
};
