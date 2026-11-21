"""Exercise configurable tag filtering, per-ID pose scaling, telemetry and TF output."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

APP_DIR = Path(__file__).resolve().parents[1] / 'app'


@pytest.fixture
def mux_mod(monkeypatch, load_module):
    # Stub ROS message modules so TagScaleRouter can be unit-tested without Lyrical rclpy.
    class FakeArray:
        def __init__(self):
            self.header = None
            self.detections = []

    class FakeTF:
        def __init__(self):
            self.transforms = []

    monkeypatch.setitem(sys.modules, 'isaac_ros_apriltag_interfaces', SimpleNamespace())
    monkeypatch.setitem(
        sys.modules, 'isaac_ros_apriltag_interfaces.msg',
        SimpleNamespace(AprilTagDetectionArray=FakeArray),
    )
    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'rclpy.executors',
                        SimpleNamespace(ExternalShutdownException=RuntimeError))
    class FakeNode:
        def __init__(self, name):
            self.logger = Mock()

        def get_logger(self):
            return self.logger

        def declare_parameter(self, name, value):
            return SimpleNamespace(value=value)

        def create_publisher(self, *args):
            pass

        def create_subscription(self, *args):
            pass

    monkeypatch.setitem(sys.modules, 'rclpy.node', SimpleNamespace(Node=FakeNode))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(qos_profile_sensor_data=None))
    monkeypatch.setitem(sys.modules, 'sensor_msgs', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(CameraInfo=object))
    monkeypatch.setitem(sys.modules, 'tf2_msgs', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'tf2_msgs.msg', SimpleNamespace(TFMessage=FakeTF))

    return load_module(APP_DIR / 'scripts/tag_result_mux.py')


@pytest.fixture
def make_mux(mux_mod, tmp_path, monkeypatch):
    cfg = mux_mod.TagResultMux._load_config(tmp_path / 'missing-config.json')
    cfg['ui_port'] = 0
    monkeypatch.setenv('AIVUDA_DATA_DIR', str(tmp_path))
    monkeypatch.setattr(mux_mod.TagResultMux, '_load_config', staticmethod(lambda path: cfg))
    return mux_mod.TagResultMux


def test_tag_scale_router_filters_and_rescales_without_ros(mux_mod):
    router = mux_mod.TagScaleRouter(
        [{'id': 0, 'size_m': 0.034, 'label': 'small'}, {'id': 77, 'size_m': 0.289, 'label': 'large'}],
        0.289, 'tag36h11', filter_mode='configured_only', default_size_m=0.289,
    )

    def make_tag(tid, pos, family='tag36h11'):
        return SimpleNamespace(
            family=family,
            id=tid,
            center=SimpleNamespace(x=640.0, y=360.0),
            corners=[SimpleNamespace(x=0.0, y=0.0), SimpleNamespace(x=50.0, y=0.0),
                     SimpleNamespace(x=50.0, y=50.0), SimpleNamespace(x=0.0, y=50.0)],
            pose=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=pos[0], y=pos[1], z=pos[2]),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.6, w=0.8),
            ))),
        )

    msg = mux_mod.AprilTagDetectionArray()
    msg.header = 'hdr'
    msg.detections = [
        make_tag(77, (1.0, 2.0, 4.0)),
        make_tag(0, (2.89, -5.78, 8.67)),
        make_tag(5, (0.0, 0.0, 2.89)),  # unconfigured ID -> filtered out in configured_only
    ]
    out, telem = router.filter_detections(msg, return_telemetry=True)
    assert out.header == 'hdr'
    assert [d.id for d in out.detections] == [77, 0]
    p77 = out.detections[0].pose.pose.pose.position
    p0 = out.detections[1].pose.pose.pose.position
    assert (p77.x, p77.y, p77.z) == pytest.approx((1.0, 2.0, 4.0))
    scale = 0.034 / 0.289
    assert (p0.x, p0.y, p0.z) == pytest.approx((2.89 * scale, -5.78 * scale, 8.67 * scale))
    assert out.detections[1].corners == msg.detections[1].corners
    assert out.detections[1].pose.pose.pose.orientation == SimpleNamespace(x=0.0, y=0.0, z=0.6, w=0.8)
    assert msg.detections[1].pose.pose.pose.position == SimpleNamespace(x=2.89, y=-5.78, z=8.67)
    assert telem[1]['label'] == 'small' and telem[1]['configured'] is True
    assert telem[1]['edge_px'] == pytest.approx(50.0)

    # Invalid hot updates must not silently assign a size to a different tag,
    # or disturb the rules already in use by the detection and TF paths.
    before = router.rules_snapshot()
    for invalid_id in (77.9, '77', True, -1, 65536):
        with pytest.raises(ValueError):
            router.update_rules([{'id': invalid_id, 'size_m': 0.034}])
        assert router.rules_snapshot() == before

    # Hot-update to all_detected with custom default_size_m=0.1445 (half of base_size=0.289)
    router.update_rules(
        [{'id': 12, 'size_m': 0.10, 'label': 'custom'}],
        filter_mode='all_detected', default_size_m=0.1445,
    )
    out_all, telem_all = router.filter_detections(msg, return_telemetry=True)
    assert [d.id for d in out_all.detections] == [77, 0, 5]
    p5 = out_all.detections[2].pose.pose.pose.position
    assert p5.z == pytest.approx(1.445)
    assert telem_all[2]['configured'] is False and telem_all[2]['size_m'] == pytest.approx(0.1445)

    # Reset back to configured_only and verify TF scaling
    router.update_rules(
        [{'id': 0, 'size_m': 0.034, 'label': 'small'}, {'id': 77, 'size_m': 0.289, 'label': 'large'}],
        filter_mode='configured_only', default_size_m=0.289,
    )
    tf_msg = mux_mod.TFMessage()
    tf_msg.transforms = [
        SimpleNamespace(child_frame_id='tag36h11:77',
                        transform=SimpleNamespace(translation=SimpleNamespace(x=1.0, y=2.0, z=3.0))),
        SimpleNamespace(child_frame_id='tag36h11:0',
                        transform=SimpleNamespace(translation=SimpleNamespace(x=2.89, y=0.0, z=5.78))),
        SimpleNamespace(child_frame_id='tag36h11:99',
                        transform=SimpleNamespace(translation=SimpleNamespace(x=9.0, y=9.0, z=9.0))),
    ]
    tf_out = router.filter_tf(tf_msg)
    assert [t.child_frame_id for t in tf_out.transforms] == ['tag36h11:77', 'tag36h11:0']
    t0 = tf_out.transforms[1].transform.translation
    assert (t0.x, t0.y, t0.z) == pytest.approx((2.89 * scale, 0.0, 5.78 * scale))


@pytest.mark.parametrize('payload', [
    {'tags': [{'id': 1, 'size_m': True}]},
    {'default_size_m': True},
    {'tags': [{'id': 1.0, 'size_m': 0.1}, {'id': 1, 'size_m': 0.2}]},
])
def test_invalid_save_keeps_previous_override(make_mux, tmp_path, payload):
    node = make_mux()
    override = tmp_path / 'tags_override.json'
    applied = node.apply_runtime_override({'tags': [{'id': 65535.0, 'size_m': 0.2}]})
    saved = json.loads(override.read_text())
    assert saved == applied['rules']
    assert type(saved['tags'][0]['id']) is int and saved['tags'][0]['id'] == 65535
    with pytest.raises(ValueError):
        node.apply_runtime_override(payload)
    assert node.status_payload()['rules'] == saved
    assert json.loads(override.read_text()) == saved
    assert make_mux().status_payload()['rules'] == saved


@pytest.mark.parametrize('contents', ['{', '[]', '{"tags":[]}',
                                      '{"tags":[{"id":3,"size_m":true}]}',
                                      '{"tags":[],"filter_mode":"unknown"}'])
def test_invalid_saved_override_uses_defaults(make_mux, tmp_path, contents):
    (tmp_path / 'tags_override.json').write_text(contents)
    node = make_mux()
    status = node.status_payload()
    assert status['override_active'] is False
    assert status['rules'] == status['config_defaults']
    node.get_logger().warning.assert_called_once()


@pytest.mark.parametrize('operation', ['save', 'reset'])
def test_persistence_failure_preserves_rules(make_mux, tmp_path, monkeypatch, operation):
    node = make_mux()
    saved = node.apply_runtime_override({'default_size_m': 0.1})['rules']

    def fail(*args, **kwargs):
        raise PermissionError('read-only directory')

    with pytest.raises(PermissionError):
        if operation == 'save':
            monkeypatch.setattr(Path, 'replace', fail)
            node.apply_runtime_override({'default_size_m': 0.2})
        else:
            monkeypatch.setattr(Path, 'unlink', fail)
            node.reset_runtime_override()
    status = node.status_payload()
    assert status['rules'] == saved and status['override_active'] is True
    override = tmp_path / 'tags_override.json'
    assert list(tmp_path.iterdir()) == [override]
    assert json.loads(override.read_text()) == saved


@pytest.mark.parametrize('second_action', ['save', 'reset'])
def test_concurrent_override_requests(make_mux, tmp_path, monkeypatch, second_action):
    node = make_mux()
    defaults = node.router.rules_snapshot()
    replacing = threading.Event()
    release = threading.Event()
    second_started = threading.Event()
    replace = Path.replace

    def pause_replace(source, target):
        replacing.set()
        assert release.wait(5)
        return replace(source, target)

    def second_request():
        second_started.set()
        if second_action == 'reset':
            return node.reset_runtime_override()
        return node.apply_runtime_override({'default_size_m': 0.2})

    monkeypatch.setattr(Path, 'replace', pause_replace)
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(node.apply_runtime_override, {'filter_mode': 'all_detected'})
        try:
            assert replacing.wait(5)
            second = pool.submit(second_request)
            assert second_started.wait(5)
            status = node.status_payload()
            assert status['rules'] == defaults and not status['override_active']
        finally:
            release.set()
        assert first.result()['rules']['filter_mode'] == 'all_detected'
        final = second.result()
    override = tmp_path / 'tags_override.json'
    if second_action == 'reset':
        assert final == {'rules': defaults, 'override_active': False}
        assert not override.exists()
    else:
        expected = dict(defaults, filter_mode='all_detected', default_size_m=0.2)
        assert final == {'rules': expected, 'override_active': True}
        assert json.loads(override.read_text()) == expected
    for instance in (node, make_mux()):
        status = instance.status_payload()
        assert {key: status[key] for key in final} == final
