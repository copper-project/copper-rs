# Workspace dependencies and import provenance

Copper builds its codec, ZED camera components, and ViTFly inference task from
this workspace. A checkout contains their Rust code, tests, examples, and the
open-source ZED C wrapper sources.

| Crate | Location |
| --- | --- |
| `cu-bincode` | [core/cu_bincode](../../core/cu_bincode) |
| `cu-bincode-derive` | [core/cu_bincode_derive](../../core/cu_bincode_derive) |
| `cu-zed` | [components/sources/cu_zed](../../components/sources/cu_zed) |
| `zed-sdk` | [components/libs/zed_sdk](../../components/libs/zed_sdk) |
| `zed-sdk-sys` | [components/libs/zed_sdk_sys](../../components/libs/zed_sdk_sys) |
| `cu-vitfly` | [components/tasks/cu_vitfly](../../components/tasks/cu_vitfly) |

Keeping codec, payload, camera, and inference code in one dependency graph lets
Cargo unify native wire-description features and lets generated apps use the same
codec as the runtime. Generated payload derives use `cu29::bincode` for crates.io,
Git, and local builds. The workspace patches/dependencies in
[`Cargo.toml`](../../Cargo.toml) define that resolution.

ViTFly defaults to CPU inference; parity tests cache downloaded weights under the
component. ZED's portable payload/projection code is separate from native SDK
wrappers: the wrapper crates are excluded from macOS workspace CI through
`package.metadata.copper`. Native camera demos still require the Stereolabs SDK;
vendoring the C wrapper does not supply that proprietary runtime.

The codec crates retain their independent `2.2.0` versions and MIT licenses.
Their Cargo Workspaces metadata keeps their versions independent during Copper
version bumps. The ZED crates inherit the Copper workspace version.

## Import provenance

The original in-tree import used these source revisions:

| Source | Revision |
| --- | --- |
| `copper-project/cu-bincode` | `97a6ab79889e64b71345e5f520234eac69136524` |
| `copper-project/cu-vitfly` | `3520dbed8c64c114f669c3a835bf4192d35e071b` |
| `copper-project/zed` | `606d0bfdb955a67a33c885578e35258889897a76` |
| `stereolabs/zed-c-api` | `a684088d4f2bc8e67c409fbc648cea9673ad74bf` |

The ZED C API is included under
`components/libs/zed_sdk_sys/vendor/zed-c-api`, with its upstream license.

## Pointers

- [Codec and feature contract](../../core/cu_bincode/README.md),
  [wire/catalog design](self-describing-logs.md).
- [Portable ZED payloads](../../components/sources/cu_zed/README.md),
  [SDK wrapper](../../components/libs/zed_sdk/README.md),
  [native bindings](../../components/libs/zed_sdk_sys/README.md).
- [ViTFly backends and model setup](../../components/tasks/cu_vitfly/README.md).
- [Workspace CI exclusions](../../support/ci/workspace_excludes.py),
  [template generator](../../support/cargo_cunew),
  [maintainer checks](../../CONTRIBUTING.md).
