## Docker helpers

Build and run the Copper development container images.

### Justfile commands

- `just docker-build-env` — build the standard dev image from `Dockerfile.ubuntu`.
- `just docker-build-cuda-env` — build the CUDA-enabled dev image from `Dockerfile.ubuntu-cuda`.
- `just docker-build-ci-ubuntu26` — build the Ubuntu 26.04 Linux CI image from `Dockerfile.ci-ubuntu26`.
- `just docker-build-ci-cuda-ubuntu24` — build the CUDA Linux CI image from `Dockerfile.ci-cuda-ubuntu24`.
- `just docker-run-env` — run the standard image, binding the repo into `/home/copper-rs`.
- `just docker-run-cuda-env` — run the CUDA image, binding the repo into `/home/copper-rs`.

The Linux CI image is published to `ghcr.io/copper-project/copper-rs-ci:ubuntu-26.04`
by `.github/workflows/build-ci-images.yml`, which refreshes it nightly and on
changes under `support/docker/`. It includes ROS 2 Lyrical plus `rmw_zenoh_cpp` so
ROS 2 interop tests can run in the normal Linux test job.
The CUDA Linux CI image is published to
`ghcr.io/copper-project/copper-rs-ci:cuda-12.6.1-ubuntu-24.04`
by `.github/workflows/build-ci-images.yml`, which refreshes it nightly and on
changes under `support/docker/`.
