"""Exercise configurable tag filtering, per-ID pose scaling, telemetry and TF output."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

APP_DIR = Path(__file__).resolve().parents[1] / 'app'


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tag_scale_router_filters_and_rescales_without_ros(monkeypatch):
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
    monkeypatch.setitem(sys.modules, 'rclpy.node', SimpleNamespace(Node=object))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(qos_profile_sensor_data=None))
    monkeypatch.setitem(sys.modules, 'sensor_msgs', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(CameraInfo=object))
    monkeypatch.setitem(sys.modules, 'tf2_msgs', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'tf2_msgs.msg', SimpleNamespace(TFMessage=FakeTF))

    mux_mod = _load_module('tag_result_mux', APP_DIR / 'scripts' / 'tag_result_mux.py')
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
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ))),
        )

    msg = FakeArray()
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
    assert telem[1]['label'] == 'small' and telem[1]['configured'] is True
    assert telem[1]['edge_px'] == pytest.approx(50.0)

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
    tf_msg = FakeTF()
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
