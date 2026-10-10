# Copper PTP reference clocks

`cu-ptp` connects an existing PTP reference to Copper's experimental shared
execution clock. Enable `cu29/clock-sync` in the application and `linux-phc` on
this crate for Linux. Select an owned resource with `runtime.clock.parent`.

```ron
resources: [(
    id: "ptp",
    provider: "cu_ptp::LinuxPtpBundle",
    config: {
        "device": "/dev/ptp0",
        "management": "/var/run/ptp4lro",
        "reference_error_ns": 10000,
    },
)],
runtime: (clock: (parent: "ptp.reference", max_error_ns: 100000, sample_interval_ns: 100000000)),
```

The PHC must belong to the selected ptp4l port and already be synchronized.
Copper opens it for reading and queries ptp4l's management socket using GET
requests. The external PTP service owns clock adjustment. Acquisition completes
before task start hooks; reference polling runs on a worker, and bounded
maintenance runs before each Serial iteration. If uncertainty or sample age
exceeds the configured budget, that iteration fails before consumers execute.

`reference_error_ns` is your bound on upstream reference error, including network
asymmetry and timestamping quality. Copper adds the measured ptp4l offset,
capture latency, extrapolation error and remaining slew. Choose this bound from
your setup's measurements; a small configured value cannot improve its accuracy.
Optional resource fields are `domain` (0), `port` (1),
`max_reference_age_ns` (5000000000), and `allow_local_master` (false). Selecting a
local grandmaster explicitly makes its clock the root of the experiment.

`LinuxSystemPtpBundle` reads `CLOCK_REALTIME` from a software-timestamped ptp4l
setup. Configure `management`, `reference_error_ns` and the known
`utc_offset_seconds` to convert UTC into the common TAI epoch. It accepts the
same optional fields. Resource construction opens no I/O; start opens handles,
and stop joins the worker before closing them.

On bare metal, wrap BSP service functions with `BoardPtp<HZ>` and
`BoardPtpHooks`, and export it through a bundle implementing
`ClockReferenceBundle`. Supply the known counter frequency in Hz, initialize the
architecture timer before building the application, extend rollovers in the BSP,
and bound each service poll. Keep counter reads in the foreground on Cortex-M;
interrupt handlers can enqueue captures for the foreground PTP service.

Try the [Linux setup and runnable examples](../../../examples/cu_clock_sync/README.md).
