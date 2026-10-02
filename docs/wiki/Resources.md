# Resources

Resources are typed runtime objects that tasks and bridges can access without going through the message graph. They are provided by resource bundles and wired by name in `copperconfig.ron`. Today we primarily use them to expose hardware as a HAL for embedded systems, but the same mechanism works well for "engine services" like memory arenas, thread pools, or reserved subsystems dedicated to specific tasks or bridges.

## Concept

- Bundles own the concrete resources and register them into the runtime at startup.
- Tasks and bridges declare the bindings they need by name, and the config maps those names to `bundle_id.resource_id`.
- Resources can be exclusive (owned by one task) or shared (borrowed or `Arc`).
- Bundles can be mission-scoped so the same task code can run against different hardware per mission.

## Using resources (most users)

1) Declare the bundles you want in the config.
2) Bind task and bridge resource names to concrete `bundle.resource` paths.
3) Use the bindings in your task/bridge constructor.

Example `copperconfig.ron` snippet:

```ron
(
    resources: [
        (id: "board", provider: "board_crate::BoardBundle"),
    ],
    tasks: [
        (
            id: "telemetry",
            type: "app::TelemetryTask",
            resources: { bus: "board.telemetry_bus" },
        ),
    ],
    bridges: [
        (
            id: "crsf",
            type: "cu_crsf::CrsfBridge<SerialPort, SerialError>",
            resources: { serial: "board.uart0" },
        ),
    ],
)
```

The bundle provider defines the resource ids (like `uart0` or `telemetry_bus`), so check the board/component crate docs or the bundle source for the list.

In code, either implement `ResourceBindings` manually or use the `resources!` macro for a compact declaration:

```rust
use cu29::prelude::*;

resources!({
    bus => Shared<TelemetryBus>,
});

impl CuTask for TelemetryTask {
    type Resources<'r> = Resources;
    // ...
    fn new_with(_cfg: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { bus: resources.bus })
    }
}
```

Notes:

- `Owned<T>` means the task takes exclusive ownership (common for peripherals).
- `Shared<T>` means the task receives an owned `Arc<T>` cloned from a shared resource registered by the bundle.
- `Borrowed<T>` means the task borrows `&T` from the `ResourceManager` for the lifetime of its resource binding.
- Use `missions: [...]` on resource entries to scope bundles to specific missions.
- See [Copper Configuration file Reference](Copper-RON-Configuration-Reference) for the full schema.

## Creating or adding resources

1) Define the resource types you want to expose (drivers, pools, arenas, handles).
2) Declare bundle IDs with `bundle_resources!`.
3) Implement `ResourceBundle` to construct resources and register them in the `ResourceManager`.
4) Point your config `resources` entries at the bundle provider path.

Minimal bundle example:

```rust
use cu29::resource::{ResourceBundle, ResourceManager};
use cu29::{CuResult, bundle_resources};
use std::sync::Arc;

pub struct BoardBundle;
bundle_resources!(BoardBundle: Uart0, TelemetryBus);

impl ResourceBundle for BoardBundle {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        _config: Option<&cu29::config::ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let uart_key = bundle.key(BoardBundleId::Uart0);
        let bus_key = bundle.key(BoardBundleId::TelemetryBus);
        manager.add_owned(uart_key, MyUart::new())?;
        manager.add_shared(bus_key, Arc::new(MyBus::new()))?;
        Ok(())
    }
}
```

Embedded note: if you need to build resources from handles created in `main` (for example, HAL peripherals), use `ResourceManager::add_bundle_prebuilt` to insert them during runtime setup.

## CUDA pools

**Since v1.1:** On supported host platforms, enable `cu29`'s `cuda` feature to expose
`cu29::pool::cuda::{CuCudaPool, CudaSliceWrapper}`. `CuCudaPool::new` returns a
registered `Arc` pool whose `acquire` method produces
`CuHandle<CudaSliceWrapper<T>>`. Tasks can forward clones of that handle through
the graph and use `as_cuda_slice` or `as_cuda_slice_mut` when calling CUDA-aware
libraries.

A resource bundle is the preferred place to create the shared `CudaContext` and
device pool. Give each task its own stream unless stream sharing is an explicit
part of the component contract. Cudarc tracks accesses to a `CudaSlice` across
streams, so consumers should obtain pointers through its `DevicePtr` and
`DevicePtrMut` APIs rather than caching raw pointers across task boundaries.

CUDA allocations are process-local and do not currently have a replayable wire
representation. Configure nodes producing CUDA handles with
`logging: (handle_content: none)` so Copper records message metadata without
trying to serialize device contents. Use `CuHandle::storage_id` as a diagnostic
when validating that adjacent tasks forwarded the same allocation.

## Examples

- `https://github.com/copper-project/copper-rs/tree/master/examples/cu_resources_test` - End-to-end example with owned vs shared resources, mission-specific bundles, and bridge bindings.
- `https://github.com/copper-project/copper-rs/tree/master/examples/cu_elrs_bdshot_demo` - Embedded demo that wires a board resource bundle into tasks and bridges.
- `https://github.com/copper-project/copper-rs/tree/master/examples/cu_flight_controller` - Uses the `resources!` macro to bind board peripherals to tasks.
