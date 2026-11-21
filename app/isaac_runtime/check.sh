#!/usr/bin/env bash
# Installation checks are read-only and never open the camera or start a node.
set -euo pipefail
cd -- "$(dirname -- "$0")"
# Also used by pack.sh and the container entrypoint. Never fall through to the
# base image's plugin if the release ELF is missing, stale or not yet approved.
sha256sum --check --strict --quiet SHA256SUMS || {
    echo 'CUDA plugin verification failed; rebuild in the pinned Jetson image (see isaac_runtime/README.md).' >&2
    exit 1
}
[[ ${1:-} != --plugin-only ]] || exit 0
[[ "$(uname -m)" == aarch64 ]] || { echo 'This build targets Jetson Orin aarch64.' >&2; exit 1; }
docker image inspect airborne/isaac-ros:5.0-imx219-preview-20261022 --format 'Isaac ROS base image {{.Id}}'
[[ -r /dev/nvhost-gpu ]] || { echo 'Missing access to Jetson GPU.' >&2; exit 1; }
