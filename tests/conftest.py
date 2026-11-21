"""Host-side fixtures for running the launch scripts without Docker or ROS."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest


APP = Path(__file__).resolve().parents[1] / 'app'


@pytest.fixture
def load_module(monkeypatch):
    def load(path):
        monkeypatch.syspath_prepend(str(path.parent))
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    return load


@pytest.fixture
def runner(load_module):
    return load_module(APP / 'isaac_runtime/run.py')


@pytest.fixture
def launch_app(tmp_path, monkeypatch):
    root = tmp_path / 'app'
    runtime = root / 'isaac_runtime'
    runtime.mkdir(parents=True)
    (root / 'scripts').symlink_to(APP / 'scripts', target_is_directory=True)
    (runtime / 'run.py').write_text('import json, sys\nprint(json.dumps(sys.argv[1:]))\n')
    monkeypatch.setenv('AIVUDA_APP_INSTALL_PATH', str(root))
    monkeypatch.setenv('AIVUDA_APP_RUNTIME_DATA_PATH', str(tmp_path / 'data'))

    def launch(config, settings):
        config.write_text(json.dumps({'apriltag': settings}))
        return subprocess.run([str(APP / 'start.sh')], text=True, capture_output=True,
                              env={**os.environ, 'AIVUDA_APP_CONFIG_PATH': str(config)})

    return launch
