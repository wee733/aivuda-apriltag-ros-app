"""Calibrated rectification and official NVIDIA Isaac ROS AprilTagNode with per-tag metric sizing."""

import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def stop_if_child_exits(event, context):
    if not context.is_shutdown:
        raise RuntimeError(f'{event.process_name} exited unexpectedly (code {event.returncode})')


def _load_runtime_config(config_file, scripts_dir, backend_fallback):
    path = Path(config_file)
    if path.is_file():
        return json.loads(path.read_text(encoding='utf-8'))
    cfg = yaml.safe_load((scripts_dir.parent / 'config/default_config.yaml').read_text())['apriltag']
    return dict(cfg, backend=backend_fallback)


def components(width, height, cfg):
    backend = cfg['backend']
    family = str(cfg.get('tag_family', 'tag36h11'))
    max_tags = int(cfg.get('max_tags', 64))
    tile_size = int(cfg.get('tile_size', 32))
    default_size_m = float(cfg.get('default_size_m', 0.289))
    tags = cfg.get('tags') or []
    # Official Isaac ROS AprilTagNode takes one global scalar `size`.
    # Pick the largest configured size (or default_size_m) as the single base scale;
    # pinhole PnP translation scales linearly with tag_size / base_size while R and corners are invariant.
    candidate_sizes = [float(t['size_m']) for t in tags if t.get('size_m')] + [default_size_m]
    base_size = float(max(candidate_sizes))

    intra_process = [{'use_intra_process_comms': True}]
    filter_params = {
        'base_size': base_size, 'tag_family': family, 'default_size_m': default_size_m,
        'filter_mode': cfg['filter_mode'],
    }
    # Empty arrays have no element type in launch_ros. The C++ defaults are typed.
    if tags:
        filter_params.update(tag_ids=[t['id'] for t in tags],
                             tag_sizes=[float(t['size_m']) for t in tags])
    return [
        ComposableNode(
            package='isaac_ros_image_proc', plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='rectify', namespace='camera', extra_arguments=intra_process,
            parameters=[{'output_width': width, 'output_height': height, 'input_qos': 'SENSOR_DATA',
                         'output_qos': 'SENSOR_DATA', 'input_queue_size': 3, 'output_queue_size': 3}],
        ),
        ComposableNode(
            package='isaac_ros_apriltag', plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
            name='apriltag', extra_arguments=intra_process,
            parameters=[{
                'size': base_size,
                'tag_family': family,
                'backends': backend,
                'max_tags': max_tags,
                'tile_size': tile_size,
            }],
            remappings=[('image', '/camera/image_rect'), ('camera_info', '/camera/camera_info_rect'),
                        ('tag_detections', '/apriltag/internal/detections'),
                        ('/tf', '/apriltag/internal/tf')],
        ),
        ComposableNode(
            package='tag_scale_filter', plugin='aivuda::TagScaleFilterNode',
            name='tag_scale_filter', extra_arguments=intra_process, parameters=[filter_params],
        ),
    ]


def detection_graph(context):
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    backend = LaunchConfiguration('backend').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    cfg = _load_runtime_config(config_file, Path(__file__).resolve().parent, backend)
    container = ComposableNodeContainer(
        name='imx219_apriltag_container', namespace='', package='rclcpp_components',
        executable='component_container', arguments=['--executor-type', 'multi-threaded'],
        ros_arguments=['--log-level', 'serialize_buffer_with_endpoint:=warn',
                       '--log-level', 'deserialize_buffer_with_endpoint:=warn'],
        composable_node_descriptions=components(width, height, cfg), output='screen', on_exit=stop_if_child_exits,
    )
    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width'),
        DeclareLaunchArgument('height'),
        DeclareLaunchArgument('backend', default_value='CUDA', choices=['CUDA', 'CPU', 'PVA']),
        DeclareLaunchArgument('config_file', default_value='/data/app_config.json'),
        OpaqueFunction(function=detection_graph),
    ])


def load_components(width, height, config_file, backend, manager_pid):
    """Apply the same launch descriptions, then exit once the native container owns them."""
    import os
    import signal

    from composition_interfaces.srv import LoadNode
    from launch import LaunchContext
    from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
    import rclpy

    try:
        rclpy.init(args=[])
        with rclpy.create_node('apriltag_component_loader') as node:
            client = node.create_client(
                LoadNode, '/imx219_apriltag_container/_container/load_node')
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError('AprilTag component container is unavailable')
            graph = components(width, height,
                               _load_runtime_config(config_file, Path(__file__).parent, backend))
            for description in graph:
                request = get_composable_node_load_request(description, LaunchContext())
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=120)
                if not future.done():
                    raise RuntimeError(f'Timed out loading {request.plugin_name}')
                result = future.result()
                if not result.success:
                    raise RuntimeError(f'Failed to load {request.plugin_name}: {result.error_message}')
                node.get_logger().info(f'Loaded {result.full_node_name}')
    except Exception:
        # A failed component graph must also fail the foreground container process.
        if os.getpgrp() == manager_pid:
            signal.signal(signal.SIGUSR1, signal.SIG_IGN)
            os.killpg(manager_pid, signal.SIGUSR1)
        raise
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    import sys
    load_components(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3], sys.argv[4], int(sys.argv[5]))
