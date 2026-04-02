#[cfg(browser)]
mod browser;
#[cfg(browser)]
pub(crate) use browser::{SystemInfo, default_system_info};

#[cfg(native)]
mod native;
#[cfg(native)]
pub(crate) use native::{SystemInfo, default_system_info};
