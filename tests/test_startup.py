"""Check host validation and the configuration handoff without running Docker or ROS."""
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest


APP = Path(__file__).resolve().parents[1] / 'app'


@pytest.fixture
def launch(tmp_path, launch_app):
    data = tmp_path / 'data'
    data.mkdir()

    def start(**overrides):
        return launch_app(tmp_path / 'settings.yaml', {'ros_domain_id': 42, **overrides})

    return start, data / 'app_config.json'


@pytest.mark.parametrize(('field', 'value'), [
    ('id', -1), ('id', 65536), ('id', 77.9), ('id', True), ('id', '77'),
    ('max_tags', 64.5), ('max_tags', True), ('max_tags', 0), ('max_tags', 257),
    ('tile_size', 32.5), ('tile_size', '32'), ('tile_size', 1), ('tile_size', 65),
])
def test_invalid_integer_never_reaches_runner(launch, field, value):
    start, live_config = launch
    config = {'tags': [{'id': value, 'size_m': 0.034}]} if field == 'id' else {field: value}
    result = start(**config)
    assert result.returncode != 0 and not result.stdout
    assert field in result.stderr and 'integer' in result.stderr
    assert not live_config.exists()


def test_integral_numbers_are_normalized_before_launch(launch):
    start, _ = launch
    result = start(ros_domain_id=42.0, tags=[{'id': 77.0, 'size_m': 1}], max_tags=64.0, tile_size=32.0)
    assert result.returncode == 0, result.stderr
    _, _, domain, _, _, payload = json.loads(result.stdout)
    cfg = json.loads(payload)
    integers = [cfg['ros_domain_id'], cfg['tags'][0]['id'], cfg['max_tags'], cfg['tile_size']]
    assert domain == '42' and integers == [42, 77, 64, 32]
    assert all(type(n) is int for n in integers)


@pytest.mark.parametrize('field', ['default_size_m', 'size_m'])
@pytest.mark.parametrize('value', [True, False, '1', None, 0, 10.1])
def test_invalid_size_never_reaches_runner(launch, field, value):
    start, _ = launch
    config = {'tags': [{'id': 0, 'size_m': value}]} if field == 'size_m' else {field: value}
    result = start(**config)
    assert result.returncode != 0 and not result.stdout
    assert field in result.stderr


@pytest.mark.parametrize(('tid', 'max_tags', 'tile_size'), [(0, 1, 2), (65535, 256, 64)])
def test_only_container_owner_publishes_valid_config(launch, tmp_path, tid, max_tags, tile_size,
                                                    load_module, monkeypatch):
    start, live_config = launch
    live_config.write_text('{"previous": true}\n')
    # Prepare the losing launch last: it must not change the old or winning config.
    winner = start(tags=[{'id': tid, 'size_m': 0.289, 'label': '降落标记'}],
                   max_tags=max_tags, tile_size=tile_size)
    loser = start(tags=[{'id': tid, 'size_m': 0.034}])
    assert winner.returncode == loser.returncode == 0, (winner.stderr, loser.stderr)
    assert live_config.read_text() == '{"previous": true}\n'
    mode, _, domain, script, path, payload = json.loads(winner.stdout)
    assert (mode, domain, script, path) == ('apriltag', '42', '/app/scripts/in_container.sh',
                                          '/data/app_config.json')
    # Capture the real shell handoff, then run its Python entry point up to ROS startup.
    fake_python = tmp_path / 'python3'
    fake_python.write_text('#!/usr/bin/python3\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n')
    fake_python.chmod(0o755)
    args = subprocess.check_output([str(APP / 'scripts/in_container.sh'), str(live_config), payload],
                                   env={**os.environ, 'PATH': str(tmp_path) + ':' + os.environ['PATH']}, text=True)
    entry, *detector_args = json.loads(args)
    assert entry == '/app/scripts/start_detector.py'
    detector = load_module(APP / Path(entry).relative_to('/app'))
    launched = []
    monkeypatch.setattr(detector, 'start_ros', lambda path, backend: launched.append((path, backend)))
    detector.main(*detector_args)
    assert launched == [(live_config, 'CUDA')]
    cfg = json.loads(live_config.read_text())
    assert cfg['ros_domain_id'] == 42 and (cfg['max_tags'], cfg['tile_size']) == (max_tags, tile_size)
    assert cfg['tags'] == [{'id': tid, 'size_m': 0.289, 'label': '降落标记'}]


@pytest.mark.parametrize(('status', 'owner', 'removable'), [
    ('created', 'live', False), ('created', 'missing', True), ('created', 'reused', True),
    ('created', 'unreadable', False), ('created', 'hidden', False), ('created', 'foreign', False),
    ('running', 'missing', False),
    ('paused', 'missing', False), ('restarting', 'missing', False), ('removing', 'missing', False),
    ('exited', 'missing', True), ('dead', 'missing', True),
])
def test_cleanup_removes_only_stale_container_by_id(monkeypatch, runner, status, owner, removable):
    cmd = [runner.ENTRY_SCRIPT, str(os.getpid()), runner.process_start_ticks('self'), '/app/start']
    if owner == 'foreign':
        cmd[0] = '/some/other/entrypoint'
    elif owner == 'reused':
        cmd[2] = str(int(cmd[2]) + 1)
    elif owner in ('missing', 'unreadable', 'hidden'):
        def stat(pid):
            raise PermissionError if owner == 'unreadable' else FileNotFoundError

        def probe(pid, sig):
            assert sig == 0
            raise ProcessLookupError if owner == 'missing' else PermissionError

        monkeypatch.setattr(runner, 'process_start_ticks', stat)
        monkeypatch.setattr(runner.os, 'kill', probe)
    calls = []

    def docker(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(returncode=0, stdout=f'old_container_id {status} {json.dumps(cmd[:3])}\n')

    monkeypatch.setattr(runner.subprocess, 'run', docker)
    if removable:
        runner.cleanup_stale_container('name')
        assert calls[1:] == [['docker', 'rm', 'old_container_id']]
    else:
        with pytest.raises(SystemExit, match='already in use'):
            runner.cleanup_stale_container('name')
        assert len(calls) == 1


def test_competing_launch_can_recover_after_owner_exits(runner, monkeypatch):
    # A real child process stands in for the foreground Docker client between
    # create and start. Keep its container in 'created' across both launches.
    with subprocess.Popen(['sh', '-c', 'read line'], stdin=subprocess.PIPE) as owner:
        cmd = [runner.ENTRY_SCRIPT, str(owner.pid), runner.process_start_ticks(owner.pid), '/app/start']
        inspect = SimpleNamespace(returncode=0, stdout=f'original created {json.dumps(cmd[:3])}\n')
        removed = []

        def docker(args, **kwargs):
            if args[:3] == ['docker', 'container', 'inspect']:
                return inspect
            removed.append(args)
            return SimpleNamespace(returncode=0)

        monkeypatch.setattr(runner.subprocess, 'run', docker)
        with pytest.raises(SystemExit, match='already in use'):
            runner.cleanup_stale_container('name')
        assert not removed
        owner.stdin.close()
        owner.wait(timeout=5)
        runner.cleanup_stale_container('name')
        assert removed == [['docker', 'rm', 'original']]


@pytest.mark.parametrize(('owner_uid', 'data_gid', 'mode', 'expected'), [
    (0, 4200, 0o2770, [44, 109, 4200]),  # Group write/search is required.
    (0, 4201, 0o2770, [44, 109]),        # Caller isn't a member.
    (0, 44, 0o2770, [44, 109]),          # Already needed for devices.
    (0, 1001, 0o2770, [44, 109]),        # Primary GID is already passed by --user.
    (1001, 4200, 0o770, [44, 109]),      # Caller owns the directory.
    (0, 4200, 0o777, [44, 109]),         # Writable without group membership.
])
def test_only_required_data_directory_group_is_added(monkeypatch, runner, owner_uid, data_gid, mode, expected):
    monkeypatch.setattr(runner.os, 'getuid', lambda: 1001)
    monkeypatch.setattr(runner.os, 'getgid', lambda: 1001)
    monkeypatch.setattr(runner.os, 'getgroups', lambda: [1001, 4200, 999])  # includes unrelated docker GID
    monkeypatch.setattr(runner.grp, 'getgrnam', lambda n: SimpleNamespace(gr_gid={'video': 44, 'render': 109}[n]))
    path = SimpleNamespace(stat=lambda: SimpleNamespace(st_uid=owner_uid, st_gid=data_gid, st_mode=mode))
    assert runner.supplementary_groups(path) == expected
