#!/usr/bin/env bash
set -eo pipefail
root="${AIVUDA_APP_INSTALL_PATH:-$(cd -- "$(dirname -- "$0")" && pwd)}"
runtime="$root/isaac_runtime"
[[ -d "$runtime" ]] || runtime="$root/../isaac_runtime"
config="${AIVUDA_APP_CONFIG_PATH:-$root/config/default_config.yaml}"
exec /usr/bin/python3 "$root/scripts/tag_config.py" "$config" "$runtime/run.py" "$root"
