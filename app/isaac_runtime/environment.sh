#!/usr/bin/env bash
set -eo pipefail
# The cached image sets DISPLAY=:1. Argus uses a headless EGL display when unset.
unset DISPLAY
/app/pre_install.sh --plugin-only
source /opt/ros/lyrical/setup.bash
export AMENT_PREFIX_PATH="/opt/aivuda-isaac/prefix:${AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="/app/lib:/opt/aivuda-isaac/prefix/lib:${LD_LIBRARY_PATH}"
exec /opt/aivuda-isaac/prefix/lib/tag_scale_filter/watch_detector "$@"
