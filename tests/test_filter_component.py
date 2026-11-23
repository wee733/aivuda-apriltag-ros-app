"""Run in an isolated ROS domain with the packaged component on AMENT_PREFIX_PATH."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

APP = Path(__file__).resolve().parents[1] / 'app'


def wait_for(node, condition, timeout=10):
    import rclpy

    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    pytest.fail('Timed out waiting for component or test output')


@pytest.mark.skipif(not os.environ.get('AIVUDA_TEST_TAG_COMPONENT'), reason='Requires isolated Lyrical container')
@pytest.mark.parametrize(('mode', 'tags', 'expected_ids'), [
    ('configured_only', [{'id': 77, 'size_m': 0.289}, {'id': 0, 'size_m': 0.034}], [77, 0]),
    ('all_detected', [{'id': 77, 'size_m': 0.289}, {'id': 0, 'size_m': 0.034}], [77, 43, 0]),
    ('all_detected', [], [77, 43, 0]),
])
def test_native_filter(load_module, mode, tags, expected_ids):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from composition_interfaces.srv import LoadNode
    from geometry_msgs.msg import TransformStamped
    from isaac_ros_apriltag_interfaces.msg import AprilTagDetection, AprilTagDetectionArray
    from launch import LaunchContext
    from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
    from tf2_msgs.msg import TFMessage

    graph = load_module(APP / 'scripts/detector.launch.py')
    nodes = graph.components(1280, 720, dict(backend='CUDA', tag_family='tag36h11',
                                            filter_mode=mode, tags=tags, default_size_m=0.1445))
    assert len(nodes) == 3
    request = get_composable_node_load_request(nodes[-1], LaunchContext())
    assert any(p.name == 'use_intra_process_comms' and p.value.bool_value for p in request.extra_arguments)
    exe = Path(get_package_prefix('rclcpp_components')) / 'lib/rclcpp_components/component_container'
    rclpy.init(args=[])
    node = rclpy.create_node('test_tag_scale')
    with subprocess.Popen([str(exe), '--ros-args', '-r', '__node:=test_filter_manager']) as manager:
        try:
            client = node.create_client(LoadNode, '/test_filter_manager/_container/load_node')
            assert client.wait_for_service(timeout_sec=10)
            future = client.call_async(request)
            wait_for(node, future.done)
            assert future.result().success, future.result().error_message
            pub = node.create_publisher(AprilTagDetectionArray, '/apriltag/internal/detections', 10)
            pub_tf = node.create_publisher(TFMessage, '/apriltag/internal/tf', 10)
            received, received_tf = [], []
            sub = node.create_subscription(AprilTagDetectionArray, '/tag_detections', received.append, 10)
            sub_tf = node.create_subscription(TFMessage, '/tf', received_tf.append, 10)
            wait_for(node, lambda: pub.get_subscription_count() and pub_tf.get_subscription_count()
                     and sub.get_publisher_count() and sub_tf.get_publisher_count())
            detections, transforms = AprilTagDetectionArray(), TFMessage()
            detections.header.frame_id = 'imx219_optical_frame'
            detections.header.stamp.sec = 12
            for tid, family in ((77, 'tag36h11'), (43, 'tag36h11'), (0, 'tag36h11'), (4, 'tag16h5')):
                det = AprilTagDetection(id=tid, family=family)
                det.pose.header = detections.header
                pos = det.pose.pose.pose.position
                pos.x, pos.y, pos.z = 2.89, -5.78, 8.67
                det.pose.pose.pose.orientation.x = 0.5
                det.corners[0].x = 640.0
                detections.detections.append(det)
                tf = TransformStamped(child_frame_id=f'{family}:{tid}', header=detections.header)
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos.x, pos.y, pos.z
                tf.transform.rotation = det.pose.pose.pose.orientation
                transforms.transforms.append(tf)
            for frame in ('tag36h11:', 'tag36h11:-1', 'tag36h11:1x', 'tag36h11:999999999999999999'):
                transforms.transforms.append(TransformStamped(child_frame_id=frame))
            pub.publish(detections)
            pub_tf.publish(transforms)
            wait_for(node, lambda: received and received_tf)
            assert received[0].header == detections.header
            assert [d.id for d in received[0].detections] == expected_ids
            assert [t.child_frame_id for t in received_tf[0].transforms] == [f'tag36h11:{i}' for i in expected_ids]
            sizes = {t['id']: t['size_m'] for t in tags}
            base = max([0.1445] + list(sizes.values()))
            for det, tf in zip(received[0].detections, received_tf[0].transforms):
                factor = sizes.get(det.id, 0.1445) / base
                for vec in (det.pose.pose.pose.position, tf.transform.translation):
                    assert (vec.x, vec.y, vec.z) == pytest.approx((2.89 * factor, -5.78 * factor, 8.67 * factor))
                assert det.corners[0].x == 640.0 and tf.header == det.pose.header == detections.header
                assert det.pose.pose.pose.orientation == tf.transform.rotation
                assert tf.transform.rotation.x == 0.5
            received.clear()
            pub.publish(AprilTagDetectionArray(header=detections.header))
            wait_for(node, lambda: received)
            assert not received[0].detections and received[0].header == detections.header
        finally:
            manager.send_signal(signal.SIGINT)
            try:
                manager.wait(timeout=5)
            except subprocess.TimeoutExpired:
                manager.kill()
            node.destroy_node()
            rclpy.shutdown()
