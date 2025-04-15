#!/bin/bash
# Run the cooper-rs-env container and mount this directory in the container
# Should make this a volume mount eventually
# Should be run from within the "docker" directory for the mounting to work properly
docker run -it -entrypoint="" --mount type=bind,source="$(pwd)"/../..,target=/home/copper-rs copper-rs-cuda-env
