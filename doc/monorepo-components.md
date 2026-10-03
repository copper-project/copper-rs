# In-tree codec, camera, and inference crates

Copper builds its codec, ZED camera components, and ViTFly inference task from
this workspace. A checkout contains their Rust code, tests, examples, and the
open-source ZED C wrapper sources.

| Crate | Location |
| --- | --- |
| `cu-bincode` | `core/cu_bincode` |
| `cu-bincode-derive` | `core/cu_bincode_derive` |
| `cu-zed` | `components/sources/cu_zed` |
| `zed-sdk` | `components/libs/zed_sdk` |
| `zed-sdk-sys` | `components/libs/zed_sdk_sys` |
| `cu-vitfly` | `components/tasks/cu_vitfly` |

Run `just monorepo-crates-check` from the repository root to check dependency
resolution, clippy, tests, codec doctests, and codec `no_std` builds. ViTFly uses
the CPU backend by default. Its parity tests download and cache the pretrained
weights inside `components/tasks/cu_vitfly/weights` on first use.

Run `just cuda-test` in `components/tasks/cu_vitfly` to test CUDA inference.
Camera demos require the native Stereolabs SDK; the crate READMEs describe
installation and demo commands.

The codec crates retain their independent `2.2.0-beta` versions and MIT licenses.
Their Cargo Workspaces metadata keeps their versions independent during Copper
version bumps. The ZED crates inherit the Copper workspace version.

## Import provenance

The migration imports tracked source files from these revisions:

| Source | Revision |
| --- | --- |
| `copper-project/cu-bincode` | `97a6ab79889e64b71345e5f520234eac69136524` |
| `copper-project/cu-vitfly` | `3520dbed8c64c114f669c3a835bf4192d35e071b` |
| `copper-project/zed` | `606d0bfdb955a67a33c885578e35258889897a76` |
| `stereolabs/zed-c-api` | `a684088d4f2bc8e67c409fbc648cea9673ad74bf` |

The ZED C API is included under
`components/libs/zed_sdk_sys/vendor/zed-c-api`, with its upstream license.
