mod model;
mod palette;
mod tui_nodes;
mod ui;

#[cfg(feature = "sysinfo_pane")]
mod system_info;

#[cfg(feature = "log_pane")]
mod logpane;

pub use model::MonitorModel;
pub use ui::{
    MonitorScreen, MonitorUi, MonitorUiAction, MonitorUiEvent, MonitorUiKey, MonitorUiOptions,
    ScrollDirection,
};

#[cfg(feature = "log_pane")]
pub use logpane::{MonitorLogCapture, StyledLine, StyledRun};
