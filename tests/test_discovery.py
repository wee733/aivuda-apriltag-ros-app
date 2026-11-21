"""Check the DDS discovery settings passed from aivudaOS to the Isaac ROS container."""

def test_isaac_runtime_mirrors_aivuda_discovery_env(monkeypatch, runner):
    for key in ('ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS'):
        monkeypatch.delenv(key, raising=False)
    assert runner.resolve_discovery_env() == ['--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST']

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    assert runner.resolve_discovery_env() == ['--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST']

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.delenv('ROS_AUTOMATIC_DISCOVERY_RANGE', raising=False)
    assert runner.resolve_discovery_env() == ['--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET']

    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    monkeypatch.setenv('ROS_STATIC_PEERS', '192.168.1.10')
    assert runner.resolve_discovery_env() == [
        '--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET',
        '--env', 'ROS_STATIC_PEERS=192.168.1.10',
    ]
