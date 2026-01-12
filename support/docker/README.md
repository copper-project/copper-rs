## Docker helpers

Build and run the Copper development container images.

### Justfile commands

- `just docker-build-env` — build the standard dev image from `Dockerfile.ubuntu`.
- `just docker-build-cuda-env` — build the CUDA-enabled dev image from `Dockerfile.ubuntu-cuda`.
- `just docker-run-env` — run the standard image, binding the repo into `/home/copper-rs`.
- `just docker-run-cuda-env` — run the CUDA image, binding the repo into `/home/copper-rs`.
