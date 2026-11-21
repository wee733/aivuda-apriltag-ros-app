"""Validate AprilTag configuration on the host and in the live rule editor."""
import json
import os
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile


DEFAULT_TAGS = [
    {'id': 0, 'size_m': 0.034, 'label': 'landing_small'},
    {'id': 77, 'size_m': 0.289, 'label': 'landing_large'},
]


def bounded_integer(name, value, minimum, maximum):
    # JSON Schema integers include numbers such as 32.0, but not booleans.
    integral = type(value) is int or (type(value) is float and value.is_integer())
    if not integral or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be an integer in [{minimum}, {maximum}]; got {value!r}')
    return int(value)


def tag_size(field, value, numeric_only):
    message = f'{field} must be a number in (0, 10]'
    if isinstance(value, bool) or (numeric_only and type(value) not in (int, float)):
        raise ValueError(message)
    try:
        size = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(message) from exc
    if not 0 < size <= 10:
        raise ValueError(message)
    return size


def normalize_rules(tags, filter_mode, default_size_m, *, numeric_only=False):
    if filter_mode not in ('configured_only', 'all_detected'):
        raise ValueError('filter_mode must be configured_only or all_detected')
    if not isinstance(tags, list):
        raise ValueError('tags must be a list')
    if filter_mode == 'configured_only' and not tags:
        raise ValueError('configured_only mode requires at least one tag entry')
    default_size = tag_size('default_size_m', default_size_m, numeric_only)
    by_id = {}
    for index, entry in enumerate(tags):
        if not isinstance(entry, dict):
            raise ValueError('each tag entry must be an object')
        tid = bounded_integer('tag id', entry.get('id'), 0, 65535)
        if tid in by_id:
            raise ValueError(f'Duplicate tag id: {tid}')
        size = tag_size(f'tags[{index}].size_m', entry.get('size_m'), numeric_only)
        label = str(entry.get('label', '') or '').strip()[:64]
        by_id[tid] = {'id': tid, 'size_m': size, 'label': label}
    return {'tags': list(by_id.values()), 'filter_mode': filter_mode, 'default_size_m': default_size}


def write_config(path, text):
    """Publish the complete UTF-8 configuration with one same-directory rename."""
    stream = NamedTemporaryFile(mode='w', dir=path.parent, encoding='utf-8', delete=False)
    tmp = Path(stream.name)
    try:
        with stream:
            stream.write(text + '\n')
        tmp.replace(path)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise


def resolve_domain(value, cfg_path):
    import yaml

    value = bounded_integer('apriltag.ros_domain_id', value, -1, 232)
    if value != -1:
        return value

    # aivudaOS passes config/apps/<id>/<version>/<version>.yaml even to
    # systemd units that don't inherit the service's custom workspace env.
    if cfg_path.match('config/apps/*/*/*.yaml'):
        sys_path = cfg_path.parents[3] / 'sys.yaml'
    else:
        ws = os.environ.get('AIVUDAOS_WS_ROOT', '').strip()
        sys_path = (Path(ws).expanduser() if ws else Path.home() / 'aivudaOS_ws') / 'config/sys.yaml'
    try:
        settings = yaml.safe_load(sys_path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        settings = {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise SystemExit(f'Cannot read aivudaOS shared config {sys_path}: {exc}')
    if settings is None:  # Empty or comment-only sys.yaml has no shared settings.
        settings = {}
    if not isinstance(settings, dict) or not isinstance(settings.get('sys', {}), dict):
        raise SystemExit(f'Expected a mapping of shared settings in {sys_path}')
    shared = settings.get('sys', {})
    if 'ros2_default_domain_id' in shared:
        return bounded_integer('sys.ros2_default_domain_id', shared['ros2_default_domain_id'], 0, 232)

    # Older installations may only supply a ROS_DOMAIN_ID launch environment.
    env_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if env_domain.isascii() and env_domain.isdecimal():
        digits = env_domain.lstrip('0') or '0'
        if len(digits) <= 3 and int(digits) <= 232:
            return int(digits)
    raise SystemExit(f'ROS_DOMAIN_ID must be an integer in [0, 232]; got {env_domain!r}')


def launch(cfg_path, runner, root):
    import yaml

    raw = yaml.safe_load(cfg_path.read_text(encoding='utf-8')) or {}
    ap = raw.get('apriltag') or {}
    domain = resolve_domain(ap.get('ros_domain_id', -1), cfg_path)
    backend = str(ap.get('backend', 'CUDA'))
    if backend not in ('CUDA', 'CPU', 'PVA'):
        raise SystemExit(f'Unsupported AprilTag backend: {backend}')

    family = str(ap.get('tag_family', 'tag36h11'))
    if backend == 'CUDA' and family != 'tag36h11':
        raise SystemExit('Isaac ROS CUDA backend only supports tag_family=tag36h11 (switch backend to CPU/PVA for other families)')

    tags = ap.get('tags')
    if tags is None:
        tags = DEFAULT_TAGS
    if not isinstance(tags, list) or len(tags) > 128:
        raise ValueError('apriltag.tags must be a list with at most 128 entries')
    rules = normalize_rules(tags, ap.get('filter_mode', 'configured_only'),
                            ap.get('default_size_m', 0.289), numeric_only=True)
    max_tags = bounded_integer('apriltag.max_tags', ap.get('max_tags', 64), 1, 256)
    tile_size = bounded_integer('apriltag.tile_size', ap.get('tile_size', 32), 2, 64)

    payload = {
        'ros_domain_id': domain,
        'backend': backend,
        'tag_family': family,
        'max_tags': max_tags,
        'tile_size': tile_size,
        **rules,
        'ui_port': 19310,
    }
    # Carry this launch's private configuration into the container without touching
    # the live instance's /data/app_config.json, even if Docker rejects this run.
    os.execv(sys.executable, [sys.executable, runner, 'apriltag', root, str(domain),
                              '/app/scripts/in_container.sh', '/data/app_config.json',
                              json.dumps(payload, ensure_ascii=False)])


if __name__ == '__main__':
    try:
        launch(Path(sys.argv[1]), *sys.argv[2:])
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
