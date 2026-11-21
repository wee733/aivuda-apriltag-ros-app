"""Exercise start.sh with a capture-only runner (no Docker, ROS or device needed)."""
import json

import pytest


@pytest.fixture
def launch(tmp_path, monkeypatch, launch_app):
    def start(*, app=None, system=None, env_domain=None, workspace='managed'):
        home = tmp_path / 'home'
        ws = home / 'aivudaOS_ws' if workspace == 'default' else tmp_path / 'custom-ws'
        cfg_dir = ws / 'config'
        cfg_dir.mkdir(parents=True)
        if system is not None:
            (cfg_dir / 'sys.yaml').write_text(system)
        cfg = cfg_dir / 'apps/apriltag/v1/v1.yaml' if workspace == 'managed' else tmp_path / 'app.yaml'
        cfg.parent.mkdir(parents=True, exist_ok=True)
        monkeypatch.setenv('HOME', str(home))
        monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
        monkeypatch.delenv('AIVUDAOS_WS_ROOT', raising=False)
        if env_domain is not None:
            monkeypatch.setenv('ROS_DOMAIN_ID', env_domain)
        if workspace == 'custom':
            monkeypatch.setenv('AIVUDAOS_WS_ROOT', str(ws))
        return launch_app(cfg, app or {})

    return start


def assert_domain(result, domain):
    assert result.returncode == 0, result.stderr
    args = json.loads(result.stdout)
    assert args[2] == str(domain)
    assert json.loads(args[-1])['ros_domain_id'] == domain


@pytest.mark.parametrize('workspace', ['managed', 'custom', 'default'])
def test_inherit_platform_before_environment(launch, workspace):
    assert_domain(launch(app={'ros_domain_id': -1.0}, env_domain='6',
                         system='_version: 3\nsys: {ros2_default_domain_id: 232.0}',
                         workspace=workspace), 232)


def test_explicit_domain_bypasses_inheritance(launch):
    assert_domain(launch(app={'ros_domain_id': 0}, system='[', env_domain='invalid'), 0)


@pytest.mark.parametrize(('system', 'env_domain', 'expected'), [
    (None, None, 0), (None, '232', 232), ('sys: {role: {id: 1}}', '007', 7),
    ('', '8', 8), ('# no shared settings yet\n', None, 0),
    ('sys: {ros2_default_domain_id: 0}', 'invalid', 0),
])
def test_missing_app_domain_uses_shared_or_legacy_default(launch, system, env_domain, expected):
    assert_domain(launch(system=system, env_domain=env_domain), expected)


@pytest.mark.parametrize('value', [None, True, '5', 3.5, -2, 233])
def test_invalid_app_domain_does_not_launch(launch, value):
    result = launch(app={'ros_domain_id': value}, env_domain='5')
    assert result.returncode != 0 and not result.stdout
    assert 'apriltag.ros_domain_id' in result.stderr


@pytest.mark.parametrize('value', ['null', 'true', '"5"', '-1', '233', '3.5'])
def test_invalid_global_domain_is_not_replaced_with_environment(launch, value):
    result = launch(system='sys: {ros2_default_domain_id: ' + value + '}', env_domain='7')
    assert result.returncode != 0 and not result.stdout
    assert 'sys.ros2_default_domain_id' in result.stderr


@pytest.mark.parametrize('settings', ['sys: [', 'sys: []', '[]'])
def test_malformed_global_config_is_not_treated_as_absent(launch, settings):
    result = launch(system=settings)
    assert result.returncode != 0 and not result.stdout
    assert 'sys.yaml' in result.stderr


@pytest.mark.parametrize('value', ['', '-1', '233', '1.5', 'invalid'])
def test_invalid_launch_environment_does_not_select_zero(launch, value):
    result = launch(env_domain=value)
    assert result.returncode != 0 and not result.stdout
    assert 'ROS_DOMAIN_ID' in result.stderr
