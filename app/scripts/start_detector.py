#!/usr/bin/env python3
"""Size rectification from the live calibrated stream, then launch official Isaac ROS AprilTag."""

import json
import math
import os
from pathlib import Path
import sys
import subprocess

from tag_config import write_config


def calibrated_dimensions(info):
    if info.width <= 0 or info.height <= 0 or not info.header.frame_id:
        raise ValueError('CameraInfo has invalid dimensions or an empty optical frame')
    expected = {'plumb_bob': 5, 'equidistant': 4, 'rational_polynomial': 8}
    if expected.get(info.distortion_model) != len(info.d):
        raise ValueError('CameraInfo has an unsupported or uncalibrated distortion model')
    if not all(math.isfinite(x) for row in (info.k, info.d, info.r, info.p) for x in row):
        raise ValueError('CameraInfo contains non-finite coefficients')
    if min(info.k[0], info.k[4], info.p[0], info.p[5]) <= 0:
        raise ValueError('CameraInfo has no valid focal lengths for metric pose')
    return info.width, info.height


def start_ros(config_path, backend):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo

    rclpy.init(args=[])
    node = Node('apriltag_camera_info_check')
    info = []
    width = height = None
    try:
        node.create_subscription(CameraInfo, '/camera/camera_info', info.append,
                                 qos_profile_sensor_data)
        node.get_logger().info('Waiting for calibrated /camera/camera_info')
        while rclpy.ok() and not info:
            rclpy.spin_once(node, timeout_sec=1.0)
        if not info:
            return
        width, height = calibrated_dimensions(info[0])
        node.get_logger().info(f'Rectifying {width}x{height} from {info[0].header.frame_id}')
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    except RuntimeError:
        if rclpy.ok():
            raise
        return
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    launch_file = Path(__file__).with_name('detector.launch.py')
    # The short-lived loader uses the same descriptions as ros2 launch. exec leaves
    # the stock C++ container as the foreground workload after startup.
    from ament_index_python.packages import get_package_prefix
    executable = Path(get_package_prefix('rclcpp_components')) / 'lib/rclcpp_components/component_container'
    # Docker's init reaps the short-lived loader. It shares the detector group
    # so the owner watcher also terminates it if stopped during component loading.
    subprocess.run(['sh', '-c', '"$@" &', 'load-apriltag', sys.executable,
                    str(launch_file), str(width), str(height), str(config_path),
                    backend, str(os.getpid())], check=True)
    os.execv(str(executable), [str(executable), '--executor-type', 'multi-threaded',
                               '--ros-args', '-r', '__node:=imx219_apriltag_container',
                               '--log-level', 'serialize_buffer_with_endpoint:=warn',
                               '--log-level', 'deserialize_buffer_with_endpoint:=warn'])


def main(config_arg, payload=None):
    config_path = Path(config_arg)
    if payload is not None:
        # Only the process that acquired the container name publishes its config.
        write_config(config_path, payload)
        backend = json.loads(payload).get('backend', 'CUDA')
    elif config_path.is_file():
        cfg = json.loads(config_path.read_text(encoding='utf-8'))
        backend = cfg.get('backend', 'CUDA')
    else:
        # Backward compatibility if invoked with bare backend string.
        backend = config_arg
        config_path = Path('/data/app_config.json')

    start_ros(config_path, backend)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/data/app_config.json',
         sys.argv[2] if len(sys.argv) == 3 else None)
