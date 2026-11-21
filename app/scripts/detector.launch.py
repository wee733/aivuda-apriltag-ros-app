"""Calibrated rectification and official NVIDIA Isaac ROS AprilTagNode with dynamic per-tag sizing."""

import json
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
    profile = yaml.safe_load((scripts_dir.parent / 'config/business_tags.yaml').read_text())
    tags = [{'id': int(t['id']), 'size_m': float(t['size']), 'label': ''} for t in profile['tags']]
    return {
        'backend': backend_fallback,
        'tag_family': profile['tag_family'],
        'default_size_m': max(t['size_m'] for t in tags),
        'max_tags': 64,
        'tile_size': 32,
        'filter_mode': 'configured_only',
        'tags': tags,
        'ui_port': 19310,
    }


def detection_graph(context):
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    backend_arg = LaunchConfiguration('backend').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    scripts = Path(__file__).resolve().parent

    cfg = _load_runtime_config(config_file, scripts, backend_arg)
    backend = str(cfg.get('backend', backend_arg))
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
    nodes = [
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
    ]
    container = ComposableNodeContainer(
        name='imx219_apriltag_container', namespace='', package='rclcpp_components',
        executable='component_container', arguments=['--executor-type', 'multi-threaded'],
        ros_arguments=['--log-level', 'serialize_buffer_with_endpoint:=warn',
                       '--log-level', 'deserialize_buffer_with_endpoint:=warn'],
        composable_node_descriptions=nodes, output='screen', on_exit=stop_if_child_exits,
    )
    mux = ExecuteProcess(
        cmd=[
            sys.executable, str(scripts / 'tag_result_mux.py'), '--ros-args',
            '-p', f'config_file:={config_file}',
            '-p', f'base_size:={base_size}',
            '-p', f'image_width:={width}',
            '-p', f'image_height:={height}',
        ],
        output='screen', on_exit=stop_if_child_exits,
    )
    return [container, mux]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width'),
        DeclareLaunchArgument('height'),
        DeclareLaunchArgument('backend', default_value='CUDA', choices=['CUDA', 'CPU', 'PVA']),
        DeclareLaunchArgument('config_file', default_value='/data/app_config.json'),
        OpaqueFunction(function=detection_graph),
    ])
