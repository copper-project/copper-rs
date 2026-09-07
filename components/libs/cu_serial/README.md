# Nonblocking serial I/O

`SerialIo` provides `try_read` and `try_write` over caller-owned byte slices.
Calls do one bounded attempt without waiting, locking or allocating. Zero means
no progress; disconnected devices report an error. Writes may accept a prefix,
whose length is returned. Implementations must return a count within the slice.

`ReadySerial<S>` adapts embedded-io `Read + Write + ReadReady + WriteReady` HALs.
It tests readiness and makes at most one read/write call, preserving the HAL's
nonblocking guarantee. HALs with DMA or different APIs can implement `SerialIo`
directly. Linux bundles can export `LinuxNonblockingSerialPort` using the explicit
`serialN_nonblocking` RON setting.
