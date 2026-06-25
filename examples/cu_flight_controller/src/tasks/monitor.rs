#[cfg(all(feature = "bevymon", not(feature = "firmware")))]
pub type FlightMonitor = cu_bevymon::CuBevyMon;

#[cfg(all(feature = "sim", not(feature = "bevymon"), not(feature = "firmware")))]
pub type FlightMonitor = cu_consolemon::CuConsoleMon;

#[cfg(any(
    feature = "firmware",
    all(not(feature = "sim"), not(feature = "bevymon"))
))]
pub type FlightMonitor = cu_logmon::CuLogMon;
