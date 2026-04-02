mod model;
mod palette;
mod tui_nodes;
mod ui;

#[cfg(feature = "sysinfo_pane")]
mod sysinfo_pane;

#[cfg(feature = "log_pane")]
mod log_pane;

pub use model::MonitorModel;
pub use ui::{
    MonitorScreen, MonitorUi, MonitorUiAction, MonitorUiEvent, MonitorUiKey, MonitorUiOptions,
    ScrollDirection,
};

#[cfg(feature = "log_pane")]
pub use log_pane::{MonitorLogCapture, StyledLine, StyledRun};
