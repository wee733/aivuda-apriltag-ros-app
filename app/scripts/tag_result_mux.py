#!/usr/bin/env python3
# Copyright 2026 isaac_ros_imx219 contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Filter, rescale and serve live telemetry/config UI API for official Isaac ROS AprilTagNode."""

from collections import deque
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage


def _quat_to_rpy_deg(x, y, z, w):
    """Convert xyzw quaternion to intrinsic XYZ roll/pitch/yaw in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [round(math.degrees(roll), 2), round(math.degrees(pitch), 2), round(math.degrees(yaw), 2)]


def _normalize_tags(tags):
    if not isinstance(tags, list):
        raise ValueError('tags must be a list')
    seen = set()
    cleaned = []
    for entry in tags:
        if not isinstance(entry, dict):
            raise ValueError('each tag entry must be an object')
        tid = int(entry['id'])
        size_m = float(entry['size_m'])
        label = str(entry.get('label', '') or '').strip()[:64]
        if tid < 0 or tid > 65535 or tid in seen:
            raise ValueError(f'duplicate or out-of-range tag id: {tid}')
        if not math.isfinite(size_m) or not 0.0 < size_m <= 10.0:
            raise ValueError(f'size_m for tag {tid} must be in (0, 10]')
        seen.add(tid)
        cleaned.append({'id': tid, 'size_m': size_m, 'label': label})
    return cleaned


class TagScaleRouter:
    """Filter configured tags and apply exact per-tag metric translation scaling.

    In pinhole projection of a planar fiducial, 2D pixel corners and 3D rotation R
    are strictly invariant to physical tag side length, while 3D translation t
    scales linearly with tag_size / base_size.
    """

    def __init__(self, tags, base_size, family, *,
                 filter_mode='configured_only', default_size_m=None):
        if not math.isfinite(base_size) or base_size <= 0.0:
            raise ValueError('base_size must be finite and positive')
        self.family = str(family)
        self.base_size = float(base_size)
        self._lock = threading.RLock()
        self.tags = []
        self.filter_mode = 'configured_only'
        self.default_size_m = float(default_size_m if default_size_m is not None else base_size)
        self._size_by_id = {}
        self._label_by_id = {}
        self._scale_by_id = {}
        self._default_scale = 1.0
        self.update_rules(tags, filter_mode=filter_mode, default_size_m=self.default_size_m)

    def update_rules(self, tags, *, filter_mode=None, default_size_m=None):
        cleaned = _normalize_tags(tags)
        mode = self.filter_mode if filter_mode is None else str(filter_mode)
        if mode not in ('configured_only', 'all_detected'):
            raise ValueError('filter_mode must be configured_only or all_detected')
        if mode == 'configured_only' and not cleaned:
            raise ValueError('configured_only mode requires at least one tag entry')
        default_size = self.default_size_m if default_size_m is None else float(default_size_m)
        if not math.isfinite(default_size) or not 0.0 < default_size <= 10.0:
            raise ValueError('default_size_m must be in (0, 10]')
        size_by_id = {t['id']: t['size_m'] for t in cleaned}
        label_by_id = {t['id']: t['label'] for t in cleaned}
        scale_by_id = {tid: size / self.base_size for tid, size in size_by_id.items()}
        with self._lock:
            self.tags = cleaned
            self.filter_mode = mode
            self.default_size_m = default_size
            self._size_by_id = size_by_id
            self._label_by_id = label_by_id
            self._scale_by_id = scale_by_id
            self._default_scale = default_size / self.base_size

    def rules_snapshot(self):
        with self._lock:
            return {
                'tags': [dict(t) for t in self.tags],
                'filter_mode': self.filter_mode,
                'default_size_m': self.default_size_m,
            }

    def _resolve_id(self, tag_id):
        """Return (scale, size_m, label, configured) or None if filtered out."""
        scale = self._scale_by_id.get(tag_id)
        if scale is not None:
            return scale, self._size_by_id[tag_id], self._label_by_id[tag_id], True
        if self.filter_mode == 'all_detected':
            return self._default_scale, self.default_size_m, '', False
        return None

    def filter_detections(self, message, *, return_telemetry=False):
        """Return an AprilTagDetectionArray with tags rescaled to exact metric sizes."""
        result = AprilTagDetectionArray()
        result.header = message.header
        telemetry = [] if return_telemetry else None
        with self._lock:
            for tag in message.detections:
                if tag.family != self.family:
                    continue
                resolved = self._resolve_id(tag.id)
                if resolved is None:
                    continue
                scale, size_m, label, configured = resolved
                if scale == 1.0:
                    out_tag = tag
                else:
                    out_tag = copy.deepcopy(tag)
                    pos = out_tag.pose.pose.pose.position
                    pos.x *= scale
                    pos.y *= scale
                    pos.z *= scale
                result.detections.append(out_tag)
                if return_telemetry:
                    p = out_tag.pose.pose.pose.position
                    q = out_tag.pose.pose.pose.orientation
                    corners = [[round(float(c.x), 2), round(float(c.y), 2)] for c in out_tag.corners]
                    edge_px = 0.0
                    if len(corners) == 4:
                        edges = [
                            math.hypot(corners[i][0] - corners[(i + 1) % 4][0],
                                       corners[i][1] - corners[(i + 1) % 4][1])
                            for i in range(4)
                        ]
                        edge_px = round(min(edges), 1)
                    center = [
                        round(float(getattr(out_tag.center, 'x', 0.0)), 1),
                        round(float(getattr(out_tag.center, 'y', 0.0)), 1),
                    ]
                    dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
                    telemetry.append({
                        'id': int(out_tag.id),
                        'family': str(out_tag.family),
                        'label': label,
                        'configured': configured,
                        'size_m': round(size_m, 4),
                        'distance_m': round(dist, 4),
                        'position_m': [round(float(p.x), 4), round(float(p.y), 4), round(float(p.z), 4)],
                        'rpy_deg': _quat_to_rpy_deg(q.x, q.y, q.z, q.w),
                        'edge_px': edge_px,
                        'center_px': center,
                        'corners_px': corners,
                    })
        if return_telemetry:
            return result, telemetry
        return result

    def filter_tf(self, message):
        """Return a TFMessage with tag transforms rescaled to exact metric sizes."""
        result = TFMessage()
        prefix = f'{self.family}:'
        with self._lock:
            for tf in message.transforms:
                if not tf.child_frame_id.startswith(prefix):
                    continue
                suffix = tf.child_frame_id[len(prefix):]
                if not suffix.isdecimal():
                    continue
                resolved = self._resolve_id(int(suffix))
                if resolved is None:
                    continue
                scale = resolved[0]
                if scale == 1.0:
                    result.transforms.append(tf)
                else:
                    scaled = copy.deepcopy(tf)
                    t = scaled.transform.translation
                    t.x *= scale
                    t.y *= scale
                    t.z *= scale
                    result.transforms.append(scaled)
        return result


class _ApiServer:
    """Lightweight localhost HTTP API backing the built-in AprilTag Web UI."""

    def __init__(self, node, port):
        self.node = node
        self.port = int(port)

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass  # Avoid spamming ROS stdout on 5 Hz UI polls.

            def _send_json(self, status, payload):
                raw = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                path = self.path.split('?', 1)[0].rstrip('/') or '/'
                if path in ('/', '/status', '/health'):
                    self._send_json(200, node.status_payload())
                    return
                self._send_json(404, {'error': f'not found: {path}'})

            def do_POST(self):
                path = self.path.split('?', 1)[0].rstrip('/')
                length = int(self.headers.get('Content-Length', '0') or '0')
                body = self.rfile.read(min(length, 262144)) if length > 0 else b'{}'
                try:
                    data = json.loads(body.decode('utf-8') or '{}')
                except ValueError as exc:
                    self._send_json(400, {'error': f'invalid JSON: {exc}'})
                    return
                if path == '/tags':
                    try:
                        applied = node.apply_runtime_override(data)
                    except (KeyError, TypeError, ValueError) as exc:
                        self._send_json(400, {'error': str(exc)})
                        return
                    self._send_json(200, {'ok': True, **applied})
                    return
                if path == '/tags/reset':
                    applied = node.reset_runtime_override()
                    self._send_json(200, {'ok': True, **applied})
                    return
                self._send_json(404, {'error': f'not found: {path}'})

        self._server = ThreadingHTTPServer(('127.0.0.1', self.port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        name='apriltag-ui-api', daemon=True)
        self._thread.start()

    def close(self):
        try:
            self._server.shutdown()
            self._server.server_close()
        except OSError:
            pass


class TagResultMux(Node):
    """Route official Isaac ROS AprilTag detections with configurable per-ID metric sizing and UI API."""

    def __init__(self):
        super().__init__('tag_result_mux')
        config_file = str(self.declare_parameter('config_file', '/data/app_config.json').value)
        base_size = float(self.declare_parameter('base_size', 0.289).value)
        img_w = int(self.declare_parameter('image_width', 1280).value)
        img_h = int(self.declare_parameter('image_height', 720).value)

        cfg = self._load_config(config_file)
        self._base_cfg = cfg
        self._data_dir = Path(os.environ.get('AIVUDA_DATA_DIR', '/data'))
        self._override_file = self._data_dir / 'tags_override.json'
        self._override_active = False

        initial_tags = cfg['tags']
        initial_filter = cfg['filter_mode']
        initial_default_size = cfg['default_size_m']
        if self._override_file.is_file():
            try:
                ov = json.loads(self._override_file.read_text(encoding='utf-8'))
                initial_tags = ov['tags']
                initial_filter = ov.get('filter_mode', initial_filter)
                initial_default_size = ov.get('default_size_m', initial_default_size)
                self._override_active = True
            except Exception as exc:
                self.get_logger().warning(f'Ignoring invalid tags_override.json: {exc}')

        self.router = TagScaleRouter(
            initial_tags, base_size, cfg['tag_family'],
            filter_mode=initial_filter, default_size_m=initial_default_size,
        )
        self._state_lock = threading.Lock()
        self._frame_times = deque(maxlen=60)
        self._frames_total = 0
        self._last_frame_mono = None
        self._last_stamp_s = None
        self._optical_frame = ''
        self._detections = []
        self._camera_info = {'width': img_w, 'height': img_h, 'fx': None, 'fy': None}

        self.output = self.create_publisher(AprilTagDetectionArray, '/tag_detections', 1)
        self.tf_output = self.create_publisher(TFMessage, '/tf', 100)
        self.det_sub = self.create_subscription(
            AprilTagDetectionArray, '/apriltag/internal/detections', self.on_detections, 10)
        self.tf_sub = self.create_subscription(
            TFMessage, '/apriltag/internal/tf', self.on_tf, 100)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info_rect', self.on_camera_info, qos_profile_sensor_data)

        ui_port = int(cfg.get('ui_port', 19310))
        self._api = None
        if ui_port > 0:
            try:
                self._api = _ApiServer(self, ui_port)
                self.get_logger().info(f'AprilTag Web UI API listening on http://127.0.0.1:{ui_port}')
            except OSError as exc:
                self.get_logger().warning(f'AprilTag Web UI API disabled (port {ui_port}): {exc}')

        rules = self.router.rules_snapshot()
        self.get_logger().info(
            f'Routing Isaac ROS AprilTag ({cfg["backend"]}, {cfg["tag_family"]}, '
            f'mode={rules["filter_mode"]}, base_size={base_size:.3f} m, '
            f'configured_ids={[t["id"] for t in rules["tags"]]})'
        )

    @staticmethod
    def _load_config(config_file):
        path = Path(config_file)
        if path.is_file():
            return json.loads(path.read_text(encoding='utf-8'))
        return {
            'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', '0')),
            'backend': 'CUDA',
            'tag_family': 'tag36h11',
            'default_size_m': 0.289,
            'max_tags': 64,
            'tile_size': 32,
            'filter_mode': 'configured_only',
            'tags': [
                {'id': 0, 'size_m': 0.034, 'label': 'landing_small'},
                {'id': 77, 'size_m': 0.289, 'label': 'landing_large'},
            ],
            'ui_port': 19310,
        }

    def on_camera_info(self, msg):
        with self._state_lock:
            self._camera_info = {
                'width': int(msg.width),
                'height': int(msg.height),
                'fx': round(float(msg.p[0]), 2) if len(msg.p) >= 6 else None,
                'fy': round(float(msg.p[5]), 2) if len(msg.p) >= 6 else None,
            }
            if msg.header.frame_id:
                self._optical_frame = msg.header.frame_id

    def on_detections(self, message):
        result, telemetry = self.router.filter_detections(message, return_telemetry=True)
        self.output.publish(result)
        now_mono = time.monotonic()
        stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        with self._state_lock:
            self._frame_times.append(now_mono)
            self._frames_total += 1
            self._last_frame_mono = now_mono
            self._last_stamp_s = stamp_s
            if message.header.frame_id:
                self._optical_frame = message.header.frame_id
            self._detections = telemetry

    def on_tf(self, message):
        result = self.router.filter_tf(message)
        if result.transforms:
            self.tf_output.publish(result)

    def apply_runtime_override(self, data):
        tags = data.get('tags', self.router.rules_snapshot()['tags'])
        filter_mode = data.get('filter_mode', self.router.filter_mode)
        default_size_m = data.get('default_size_m', self.router.default_size_m)
        self.router.update_rules(tags, filter_mode=filter_mode, default_size_m=default_size_m)
        snap = self.router.rules_snapshot()
        self._data_dir.mkdir(parents=True, exist_ok=True)
        tmp = self._override_file.with_suffix('.tmp')
        tmp.write_text(json.dumps(snap, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        tmp.replace(self._override_file)
        self._override_active = True
        self.get_logger().info(
            f'Hot-updated AprilTag rules: mode={snap["filter_mode"]}, '
            f'default_size={snap["default_size_m"]} m, ids={[t["id"] for t in snap["tags"]]}'
        )
        return {'rules': snap, 'override_active': True}

    def reset_runtime_override(self):
        self.router.update_rules(
            self._base_cfg['tags'],
            filter_mode=self._base_cfg['filter_mode'],
            default_size_m=self._base_cfg['default_size_m'],
        )
        try:
            self._override_file.unlink()
        except FileNotFoundError:
            pass
        self._override_active = False
        snap = self.router.rules_snapshot()
        self.get_logger().info('Reset AprilTag rules to AivudaOS Config Center defaults')
        return {'rules': snap, 'override_active': False}

    def status_payload(self):
        now_mono = time.monotonic()
        with self._state_lock:
            times = [t for t in self._frame_times if now_mono - t <= 2.0]
            fps = round((len(times) - 1) / (times[-1] - times[0]), 1) if len(times) >= 2 and times[-1] > times[0] else 0.0
            age_ms = round((now_mono - self._last_frame_mono) * 1e3, 1) if self._last_frame_mono is not None else None
            detections = list(self._detections) if (age_ms is not None and age_ms < 2000.0) else []
            cam = dict(self._camera_info)
            frames_total = self._frames_total
            optical_frame = self._optical_frame
            last_stamp_s = self._last_stamp_s
        return {
            'running': True,
            'fps': fps,
            'frames_total': frames_total,
            'last_frame_age_ms': age_ms,
            'last_stamp_s': last_stamp_s,
            'optical_frame': optical_frame,
            'camera': cam,
            'isaac_ros': {
                'backend': self._base_cfg['backend'],
                'tag_family': self._base_cfg['tag_family'],
                'max_tags': self._base_cfg['max_tags'],
                'tile_size': self._base_cfg['tile_size'],
                'base_size_m': self.router.base_size,
                'ros_domain_id': self._base_cfg.get('ros_domain_id', 0),
                'discovery_range': os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', ''),
            },
            'rules': self.router.rules_snapshot(),
            'config_defaults': {
                'tags': self._base_cfg['tags'],
                'filter_mode': self._base_cfg['filter_mode'],
                'default_size_m': self._base_cfg['default_size_m'],
            },
            'override_active': self._override_active,
            'detections': detections,
        }

    def destroy_node(self):
        if self._api is not None:
            self._api.close()
        return super().destroy_node()


def main(args=None):
    """Run the metadata router with clean shutdown handling."""
    rclpy.init(args=args)
    node = None
    try:
        node = TagResultMux()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
