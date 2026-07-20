#[cfg(feature = "log")]
pub use log;

#[cfg(feature = "log")]
#[macro_export]
macro_rules! trace {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::log::trace!($s $(, $arg)*)
    };
}

#[cfg(feature = "log")]
#[macro_export]
macro_rules! debug {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::log::debug!($s $(, $arg)*)
    };
}

#[cfg(feature = "log")]
#[macro_export]
macro_rules! info {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::log::info!($s $(, $arg)*)
    };
}

#[cfg(feature = "log")]
#[macro_export]
macro_rules! warn {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::log::warn!($s $(, $arg)*)
    };
}

#[cfg(feature = "log")]
#[macro_export]
macro_rules! error {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::log::error!($s $(, $arg)*)
    };
}

#[cfg(feature = "defmt")]
pub use defmt;

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! trace {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        use $crate::logging::defmt as defmt;
        defmt::trace!($s $(, $arg)*)
    }};
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! debug {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        use $crate::logging::defmt as defmt;
        defmt::debug!($s $(, $arg)*)
    }};
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! info {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        use $crate::logging::defmt as defmt;
        defmt::info!($s $(, $arg)*)
    }};
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! warn {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        use $crate::logging::defmt as defmt;
        defmt::warn!($s $(, $arg)*)
    }};
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! error {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        use $crate::logging::defmt as defmt;
        defmt::error!($s $(, $arg)*)
    }};
}

#[cfg(feature = "web_console")]
pub use web_sys::console;

#[cfg(feature = "web_console")]
#[macro_export]
macro_rules! trace {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::console::trace_1(&format!($s $(, $arg)*).into())
    };
}

#[cfg(feature = "web_console")]
#[macro_export]
macro_rules! debug {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::console::debug_1(&format!($s $(, $arg)*).into())
    };
}

#[cfg(feature = "web_console")]
#[macro_export]
macro_rules! info {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::console::info_1(&format!($s $(, $arg)*).into())
    };
}

#[cfg(feature = "web_console")]
#[macro_export]
macro_rules! warn {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::console::warn_1(&format!($s $(, $arg)*).into())
    };
}

#[cfg(feature = "web_console")]
#[macro_export]
macro_rules! error {
    ($s:literal $(, $arg:expr)* $(,)?) => {
        $crate::logging::console::error_1(&format!($s $(, $arg)*).into())
    };
}

#[cfg(not(any(feature = "log", feature = "defmt", feature = "web_console")))]
#[macro_export]
macro_rules! trace {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        let _ = format_args!($s $(, $arg)*);
    }};
}

#[cfg(not(any(feature = "log", feature = "defmt", feature = "web_console")))]
#[macro_export]
macro_rules! debug {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        let _ = format_args!($s $(, $arg)*);
    }};
}

#[cfg(not(any(feature = "log", feature = "defmt", feature = "web_console")))]
#[macro_export]
macro_rules! info {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        let _ = format_args!($s $(, $arg)*);
    }};
}

#[cfg(not(any(feature = "log", feature = "defmt", feature = "web_console")))]
#[macro_export]
macro_rules! warn {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        let _ = format_args!($s $(, $arg)*);
    }};
}

#[cfg(not(any(feature = "log", feature = "defmt", feature = "web_console")))]
#[macro_export]
macro_rules! error {
    ($s:literal $(, $arg:expr)* $(,)?) => {{
        let _ = format_args!($s $(, $arg)*);
    }};
}
