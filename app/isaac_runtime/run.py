#!/usr/bin/python3
"""Run one local Isaac ROS 5 workload in a foreground, Aivuda-owned container."""
import fcntl
import grp
import json
import os
from pathlib import Path
import subprocess
import sys



IMAGE = 'airborne/isaac-ros:5.0-imx219-preview-20261022'
ENTRY_SCRIPT = '/opt/aivuda-isaac/environment.sh'
DEVICE_PATTERNS = ('nvhost-gpu', 'nvhost-as-gpu', 'nvhost-ctrl-gpu', 'nvhost-power-gpu',
                   'nvhost-tsg-gpu', 'nvgpu/igpu0/as', 'nvgpu/igpu0/channel',
                   'nvgpu/igpu0/ctrl', 'nvgpu/igpu0/power', 'nvgpu/igpu0/tsg',
                   'nvmap', 'nvsciipc', 'dri/renderD*', 'dri/card*')


def process_start_ticks(pid):
    # comm can contain spaces, ')' and non-UTF8 bytes. starttime is field 22.
    stat = Path(f'/proc/{pid}/stat').read_bytes()
    return stat.rsplit(b')', 1)[1].split()[19].decode('ascii')


def resolve_discovery_env():
    """Mirror AivudaOS global runtime_environment into the ROS 2 Lyrical container."""
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '').strip()
    explicit_range = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '').strip()
    static_peers = os.environ.get('ROS_STATIC_PEERS', '').strip()
    if localhost_only == '1':
        discovery_range = 'LOCALHOST'
    elif explicit_range:
        discovery_range = explicit_range
    elif localhost_only == '0':
        discovery_range = 'SUBNET'
    else:
        discovery_range = 'LOCALHOST'
    env = ['--env', f'ROS_AUTOMATIC_DISCOVERY_RANGE={discovery_range}']
    if static_peers:
        env.extend(['--env', f'ROS_STATIC_PEERS={static_peers}'])
    return env


def owner_gone(cmd):
    # docker run stores the supervising client's PID and start time in Config.Cmd
    # at creation, even if the client never reaches the start request.
    if not isinstance(cmd, list) or len(cmd) != 3 or cmd[0] != ENTRY_SCRIPT:
        return False
    pid, ticks = cmd[1:3]
    if not all(isinstance(n, str) and n.isascii() and n.isdecimal() for n in (pid, ticks)):
        return False
    if pid.startswith('0') or (ticks.startswith('0') and ticks != '0'):
        return False
    try:
        return process_start_ticks(pid) != ticks
    except FileNotFoundError:
        # hidepid can hide another user's live process from /proc as ENOENT.
        try:
            os.kill(int(pid), 0)
        except ProcessLookupError:
            return True
        except (OSError, OverflowError, ValueError):
            pass
        return False
    except (OSError, IndexError, UnicodeError):
        return False  # Unreadable owner identity is not evidence of a stale launch.


def cleanup_stale_container(container_name):
    """Remove a leftover stopped container with the same name after an unclean shutdown."""
    # Only a created container needs an owner check. Omit the workload payload.
    inspect_format = ('{{.Id}} {{.State.Status}} '
                      '{{if and (eq .State.Status "created") .Config.Cmd (ge (len .Config.Cmd) 4)}}'
                      '{{json (slice .Config.Cmd 0 3)}}{{else}}null{{end}}')
    inspect = subprocess.run(
        ['docker', 'container', 'inspect', '--format', inspect_format, container_name],
        capture_output=True, text=True, check=False,
    )
    if inspect.returncode != 0:
        return
    container_id, status, cmd = inspect.stdout.strip().split(maxsplit=2)
    if status not in ('exited', 'dead') and not (status == 'created' and owner_gone(json.loads(cmd))):
        raise SystemExit(f'Container {container_name} ({container_id}) is already in use ({status})')
    # Concurrent recoveries may inspect the same orphan. Never force removal or
    # resolve the name a second time: it may already refer to the winning launch.
    subprocess.run(['docker', 'rm', container_id],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def supplementary_groups(runtime_data):
    gids = {grp.getgrnam(name).gr_gid for name in ('video', 'render')}
    # --user UID:GID drops the host's supplementary groups. Preserve the data
    # directory's group if needed to create files, not unrelated host groups.
    data = runtime_data.stat()
    if (data.st_uid != os.getuid() and data.st_mode & 0o030 == 0o030
            and data.st_mode & 0o003 != 0o003 and data.st_gid in os.getgroups()):
        gids.add(data.st_gid)
    return sorted(gids - {os.getgid()})


def main():
    mode, root, domain, *command = sys.argv[1:]
    if mode not in ('camera', 'apriltag') or not command or not 0 <= int(domain) <= 232:
        raise ValueError('Expected camera|apriltag APP_ROOT ROS_DOMAIN_ID COMMAND ...')
    root = Path(root).resolve()
    runtime = Path(__file__).resolve().parent
    # The old airborne worker uses the same inode for its per-service flock.
    legacy = Path.home() / 'workspaces/airborne/control/run' / f'{mode}.lock'
    if legacy.parent.is_dir():
        guard = os.open(legacy, os.O_RDWR | os.O_CREAT, 0o660)
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit(f'{mode} already has an airborne owner ({legacy})')
        os.set_inheritable(guard, True)

    container_name = f'aivuda_isaac_{mode}'
    cleanup_stale_container(container_name)

    runtime_data_env = os.environ.get('AIVUDA_APP_RUNTIME_DATA_PATH', '').strip()
    runtime_data = (Path(runtime_data_env).resolve() if runtime_data_env
                    else Path(f'/tmp/aivuda-isaac-{mode}'))
    runtime_data.mkdir(parents=True, exist_ok=True)

    # The same PID becomes the foreground Docker client at exec. The worker
    # watches that process even if AivudaOS ultimately sends it SIGKILL.
    owner = os.getpid()
    start_ticks = process_start_ticks('self')
    args = ['docker', 'run', '--rm', '--pull=never', '--init', '--name', container_name,
            '--network=host', '--ipc=host', '--pid=host', '--runtime=nvidia',
            '--user', f'{os.getuid()}:{os.getgid()}', '--cap-drop=ALL',
            '--tmpfs', '/home/app:rw,mode=1777', '--workdir', '/home/app',
            '--env', 'HOME=/home/app', '--env', 'ROS_HOME=/home/app/.ros',
            '--env', 'AIVUDA_DATA_DIR=/data',
            '--env', f'ROS_DOMAIN_ID={int(domain)}', *resolve_discovery_env(),
            '--env', 'NVIDIA_VISIBLE_DEVICES=all', '--env', 'NVIDIA_DRIVER_CAPABILITIES=all',
            '--volume', '/tmp:/tmp', '--volume', f'{root}:/app:ro',
            '--volume', f'{runtime}:/opt/aivuda-isaac:ro',
            '--volume', f'{runtime_data}:/data:rw',
            '--entrypoint', '/bin/bash']
    for gid in supplementary_groups(runtime_data):
        args.extend(['--group-add', str(gid)])
    for pattern in DEVICE_PATTERNS:
        for path in sorted(Path('/dev').glob(pattern)):
            if path.is_char_device():
                args.extend(['--device', str(path)])
    args += [IMAGE, ENTRY_SCRIPT, str(owner), start_ticks, *command]
    os.execvp(args[0], args)


if __name__ == '__main__':
    main()
