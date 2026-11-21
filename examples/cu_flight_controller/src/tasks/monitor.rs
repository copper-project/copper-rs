#[cfg(all(feature = "memmon", not(feature = "firmware")))]
pub type FlightMonitor = cu_memmon::CuMemMon;

#[cfg(all(
    feature = "bevymon",
    not(feature = "memmon"),
    not(feature = "firmware")
))]
pub type FlightMonitor = cu_bevymon::CuBevyMon;

#[cfg(all(
    feature = "sim",
    not(feature = "memmon"),
    not(feature = "bevymon"),
    not(feature = "firmware")
))]
pub type FlightMonitor = cu_consolemon::CuConsoleMon;

#[cfg(any(
    all(feature = "firmware", feature = "logmon"),
    all(
        not(feature = "sim"),
        not(feature = "firmware"),
        not(feature = "bevymon"),
        not(feature = "memmon")
    )
))]
pub type FlightMonitor = cu_logmon::CuLogMon;
