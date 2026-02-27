#[cfg(all(feature = "sim", not(feature = "firmware")))]
pub type FlightMonitor = cu_consolemon::CuConsoleMon;

#[cfg(any(feature = "firmware", not(feature = "sim")))]
pub type FlightMonitor = cu_logmon::CuLogMon;
