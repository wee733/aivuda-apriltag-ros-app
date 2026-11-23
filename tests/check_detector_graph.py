"""Feed two known tags to a running detector in an isolated ROS domain."""
import time

import cv2
import numpy as np
import rclpy
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


def calibrated_frame():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    canvas = np.full((720, 1280), 255, np.uint8)
    for tid, x in ((0, 240), (77, 820)):
        if hasattr(cv2.aruco, 'generateImageMarker'):
            marker = cv2.aruco.generateImageMarker(dictionary, tid, 260)
        else:
            marker = cv2.aruco.drawMarker(dictionary, tid, 260)
        canvas[230:490, x:x + 260] = marker
    image = Image(height=720, width=1280, encoding='rgb8', step=1280 * 3)
    image.header.frame_id = 'imx219_optical_frame'
    image.data = cv2.cvtColor(canvas, cv2.COLOR_GRAY2RGB).tobytes()
    info = CameraInfo(width=1280, height=720, distortion_model='plumb_bob', d=[0.] * 5)
    info.k = [600., 0., 640., 0., 600., 360., 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [600., 0., 640., 0., 0., 600., 360., 0., 0., 0., 1., 0.]
    return image, info


def main():
    rclpy.init()
    node = rclpy.create_node('apriltag_synthetic_camera')
    try:
        image, info = calibrated_frame()
        image_pub = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        info_pub = node.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        expected = {0: 0.034, 77: 0.289}
        frames, rectified, transforms = [], [], []
        node.create_subscription(AprilTagDetectionArray, '/tag_detections', frames.append, 10)
        node.create_subscription(TFMessage, '/tf', transforms.append, 10)
        node.create_subscription(CameraInfo, '/camera/camera_info_rect', rectified.append, qos_profile_sensor_data)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            image.header.stamp = node.get_clock().now().to_msg()
            info.header = image.header
            image_pub.publish(image)
            info_pub.publish(info)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)
            if not rectified or not transforms or not frames:
                continue
            latest = frames[-1]
            depths = {d.id: d.pose.pose.pose.position.z for d in latest.detections}
            if depths.keys() != expected.keys():
                continue
            if not any(m.header == latest.header for m in rectified):
                continue
            ids = {t.child_frame_id for msg in transforms for t in msg.transforms
                   if t.header == latest.header}
            if ids != {'tag36h11:0', 'tag36h11:77'}:
                continue
            # A fronto-parallel 260 px marker at focal length 600 px.
            for tid, size in expected.items():
                nominal = 600 * size / 260
                assert abs(depths[tid] - nominal) < 0.05 * nominal, depths
            print('Detected both synthetic tags with metric Z (m)', depths)
            return
        raise RuntimeError('Timed out waiting for matching rectified CameraInfo, detections and TF')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
