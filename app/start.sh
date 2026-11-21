#!/usr/bin/env bash
set -eo pipefail
root="${AIVUDA_APP_INSTALL_PATH:-$(cd -- "$(dirname -- "$0")" && pwd)}"
runtime="$root/isaac_runtime"
[[ -d "$runtime" ]] || runtime="$root/../isaac_runtime"
config="${AIVUDA_APP_CONFIG_PATH:-$root/config/default_config.yaml}"
data_dir="${AIVUDA_APP_RUNTIME_DATA_PATH:-/tmp/aivuda-isaac-apriltag}"
mkdir -p "$data_dir"
export AIVUDA_APP_RUNTIME_DATA_PATH="$data_dir"

domain=$(/usr/bin/python3 - "$config" "$data_dir/app_config.json" <<'PY'
import json
import math
from pathlib import Path
import sys
import yaml

cfg_path, out_path = Path(sys.argv[1]), Path(sys.argv[2])
raw = yaml.safe_load(cfg_path.read_text(encoding='utf-8')) or {}
ap = raw.get('apriltag') or {}
domain = ap.get('ros_domain_id', 0)
if type(domain) is not int or not 0 <= domain <= 232:
    raise SystemExit('apriltag.ros_domain_id must be an integer in [0, 232]')

backend = str(ap.get('backend', 'CUDA'))
if backend not in ('CUDA', 'CPU', 'PVA'):
    raise SystemExit(f'Unsupported AprilTag backend: {backend}')

family = str(ap.get('tag_family', 'tag36h11'))
if backend == 'CUDA' and family != 'tag36h11':
    raise SystemExit('Isaac ROS CUDA backend only supports tag_family=tag36h11 (switch backend to CPU/PVA for other families)')

default_size_m = float(ap.get('default_size_m', 0.289))
if not math.isfinite(default_size_m) or not 0 < default_size_m <= 10.0:
    raise SystemExit('apriltag.default_size_m must be in (0, 10]')

max_tags = int(ap.get('max_tags', 64))
tile_size = int(ap.get('tile_size', 32))
if not 1 <= max_tags <= 256 or not 2 <= tile_size <= 64:
    raise SystemExit('apriltag.max_tags must be in [1, 256] and tile_size in [2, 64]')

filter_mode = str(ap.get('filter_mode', 'configured_only'))
if filter_mode not in ('configured_only', 'all_detected'):
    raise SystemExit('apriltag.filter_mode must be configured_only or all_detected')

raw_tags = ap.get('tags')
if raw_tags is None:
    raw_tags = [
        {'id': 0, 'size_m': 0.034, 'label': 'landing_small'},
        {'id': 77, 'size_m': 0.289, 'label': 'landing_large'},
    ]
seen = set()
tags = []
for entry in raw_tags:
    tid = int(entry['id'])
    size_m = float(entry['size_m'])
    label = str(entry.get('label', '') or '').strip()[:64]
    if tid < 0 or tid in seen:
        raise SystemExit(f'Duplicate or invalid tag id: {tid}')
    if not math.isfinite(size_m) or not 0 < size_m <= 10.0:
        raise SystemExit(f'Invalid size_m for tag {tid}: {size_m}')
    seen.add(tid)
    tags.append({'id': tid, 'size_m': size_m, 'label': label})

if filter_mode == 'configured_only' and not tags:
    raise SystemExit('configured_only mode requires at least one entry in apriltag.tags')

payload = {
    'ros_domain_id': domain,
    'backend': backend,
    'tag_family': family,
    'default_size_m': default_size_m,
    'max_tags': max_tags,
    'tile_size': tile_size,
    'filter_mode': filter_mode,
    'tags': tags,
    'ui_port': 19310,
}
tmp = out_path.with_suffix('.tmp')
tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
tmp.replace(out_path)
print(domain)
PY
)

exec /usr/bin/python3 "$runtime/run.py" apriltag "$root" "$domain" \
    /app/scripts/in_container.sh /data/app_config.json
