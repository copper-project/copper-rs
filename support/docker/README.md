## Docker helpers

Build and run the Copper development container images.

### Justfile commands

- `just docker-build-env` — build the standard dev image from `Dockerfile.ubuntu`.
- `just docker-build-cuda-env` — build the CUDA-enabled dev image from `Dockerfile.ubuntu-cuda`.
- `just docker-build-ci-ubuntu26` — build the Ubuntu 26.04 Linux CI image from `Dockerfile.ci-ubuntu26`.
- `just docker-run-env` — run the standard image, binding the repo into `/home/copper-rs`.
- `just docker-run-cuda-env` — run the CUDA image, binding the repo into `/home/copper-rs`.

The Linux CI image is published to `ghcr.io/copper-project/copper-rs-ci:ubuntu-26.04`
by `.github/workflows/build-ci-images.yml`.
