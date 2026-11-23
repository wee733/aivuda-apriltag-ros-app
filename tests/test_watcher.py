"""Check host-owner lifetime and foreground failure propagation in the native watcher."""
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from conftest import APP


@pytest.fixture(scope='module')
def watcher(tmp_path_factory):
    executable = tmp_path_factory.mktemp('watcher') / 'watch_detector'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    str(APP / 'tag_scale_filter/src/watch_detector.c'), '-o', str(executable)], check=True)
    return str(executable)


def ticks(pid):
    stat = Path('/proc/{}/stat'.format(pid)).read_bytes()
    return stat.rsplit(b')', 1)[1].split()[19].decode('ascii')


def test_foreground_failure_propagates(watcher):
    pid = os.getpid()
    result = subprocess.run([watcher, str(pid), ticks(pid), 'sh', '-c', 'exit 23'])
    assert result.returncode == 23


def test_stale_owner_cannot_start_workload(watcher, tmp_path):
    marker = tmp_path / 'started'
    result = subprocess.run([watcher, str(os.getpid()), '0', 'touch', str(marker)])
    assert result.returncode != 0 and not marker.exists()


def test_owner_exit_stops_workload(watcher, tmp_path):
    ready = tmp_path / 'child'
    with subprocess.Popen(['sleep', '10']) as owner:
        command = 'import os,sys,time; open(sys.argv[1],"w").write(str(os.getpid())); time.sleep(10)'
        args = [watcher, str(owner.pid), ticks(owner.pid), sys.executable, '-c', command, str(ready)]
        with subprocess.Popen(args) as supervisor:
            deadline = time.monotonic() + 3
            while not ready.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert ready.exists()
            owner.terminate()
            assert supervisor.wait(timeout=3) == 0
            assert not Path('/proc/' + ready.read_text()).exists()
