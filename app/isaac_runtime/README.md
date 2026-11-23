# Isaac ROS 5 sensor runtime

The AprilTag archive includes its runtime in this directory. The runtime starts the installed Isaac ROS image with the application user, a read-only app mount, local GPU device access, and shared host networking, IPC, PID namespace and /tmp. CUDA IPC exchanges buffer descriptors between local ROS processes. Startup clears the cached image’s DISPLAY to allow headless EGL on this Jetson. The compiled watch_detector process forwards shutdown and watches the foreground Docker client and the C++ container with Linux pidfds. Calibration discovery and component loading use short-lived Python processes. The component library, resource index and watcher are built together as described in the developer notes.

The camera and detector must use the same DDS domain and CUDA endpoint registry. Their launchers export the effective `ROS_DOMAIN_ID` to both. The backend registers and looks up DDS endpoint GIDs in the shared-memory registry for that domain on the local host. Camera and detector containers use host networking and IPC with matching hostname, UID and GPU device. Upgrade both apps together when using a nonzero domain with an older peer that still registers every endpoint under index 0.

This release packages the same source and Orin plugin ELF as the IMX219 camera app. It validates IPC leases and writer events. The receiver returns an empty CPU buffer for an unusable descriptor. Consumers should verify image buffer size before reading. The plugin retains the `NODELETE` linker flag for deferred cleanup. See `cuda_buffer_backend/UPSTREAM.txt` for provenance.

Packaging, pre-install and container startup check `SHA256SUMS` against the plugin and its local build inputs. After a plugin change, rebuild rather than refreshing the checksum of an old `.so`. On the Jetson, use the cached `airborne/isaac-ros:5.0-imx219-preview-20261022` image with the repository mounted read-write at `/repo`; the image's `cuda_buffer` headers and library must match. Inside that container, run:

```bash
set -e
source /opt/ros/lyrical/setup.bash
runtime=/repo/app/isaac_runtime
cd "$(mktemp -d)"
colcon build --paths "$runtime/cuda_buffer_backend" --merge-install \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
readelf -dW install/lib/libcuda_buffer_backend.so | grep NODELETE
install -m 755 install/lib/libcuda_buffer_backend.so "$runtime/prefix/lib/"
cd "$runtime"
sha256sum prefix/lib/libcuda_buffer_backend.so \
  cuda_buffer_backend/src/cuda_buffer_backend_plugin.cpp \
  cuda_buffer_backend/include/cuda_buffer_backend/cuda_buffer_backend.hpp \
  cuda_buffer_backend/CMakeLists.txt > SHA256SUMS
./check.sh --plugin-only
```

Re-run `./pack.sh` from the repository root. Keep the camera and detector in separate processes for the inter-process GPU smoke test; a checksum check alone does not exercise CUDA IPC.
