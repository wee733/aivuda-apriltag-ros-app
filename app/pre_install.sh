#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
sha256sum --check --strict --quiet tag_scale_filter/SHA256SUMS || {
    echo 'AprilTag component verification failed. Rebuild the component in the pinned Jetson image.' >&2
    exit 1
}
exec ./isaac_runtime/check.sh "$@"
