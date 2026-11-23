# Build and verify the AprilTag detector

The release contains the ARM64 C++ component, its ament entries and the owner watcher. To rebuild these files, mount the checkout read-write at `/repo` in the Jetson's pinned `airborne/isaac-ros:5.0-imx219-preview-20261022` container. Inside the container run the following commands.

```bash
set -e
source /opt/ros/lyrical/setup.bash
cd "$(mktemp -d)"
colcon build --merge-install --paths /repo/app/tag_scale_filter \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
prefix=/repo/app/isaac_runtime/prefix
files=(lib/libtag_scale_filter.so lib/tag_scale_filter/watch_detector
       share/ament_index/resource_index/packages/tag_scale_filter
       share/ament_index/resource_index/rclcpp_components/tag_scale_filter
       share/tag_scale_filter/package.xml)
(cd install && tar cf - "${files[@]}") | tar xf - -C "$prefix"
cd /repo/app
sha256sum tag_scale_filter/{CMakeLists.txt,package.xml,src/*} \
  isaac_runtime/prefix/lib/{libtag_scale_filter.so,tag_scale_filter/watch_detector} \
  isaac_runtime/prefix/share/ament_index/resource_index/{packages,rclcpp_components}/tag_scale_filter \
  isaac_runtime/prefix/share/tag_scale_filter/package.xml > tag_scale_filter/SHA256SUMS
```

On the host run `python3 -m pytest -q tests` and `./pack.sh`. To exercise the packaged component on Jetson, run the next commands inside an isolated container. The test subscribes and publishes its own detection and TF fixtures.

```bash
source /opt/ros/lyrical/setup.bash
export AMENT_PREFIX_PATH=/repo/app/isaac_runtime/prefix:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/repo/app/isaac_runtime/prefix/lib:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=163 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
AIVUDA_TEST_TAG_COMPONENT=1 python3 -m pytest -q /repo/tests/test_filter_component.py
```

For the full graph, run the detector in a dedicated domain with the IMX219 camera, or run `python3 tests/check_detector_graph.py` in that domain to supply calibrated RGB test frames with tags 0 and 77. The synthetic publisher needs OpenCV with ArUco support. It verifies rectified CameraInfo, both detections and TF. Use a separate domain for fixtures so each camera topic has one source.

With the live camera, compare timestamps on `/camera/camera_info_rect` and `/tag_detections`, check the array frequency and verify the three component names with `ros2 component list`. Empty detection arrays are expected when no tag is visible. Once initialization finishes, `docker top aivuda_isaac_apriltag` shows the native watcher and the stock component container. The source and rebuild procedure for the shared CUDA IPC plugin are in the [runtime guide](../app/isaac_runtime/README.md).
