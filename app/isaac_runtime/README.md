# Isaac ROS 5 sensor runtime

The camera and AprilTag archives each include this directory. The runtime starts the installed Isaac ROS image with the application user, a read-only app mount, local GPU device access, and shared host networking, IPC, PID namespace and /tmp. CUDA IPC exchanges buffer descriptors between local ROS processes. Startup clears the cached image’s DISPLAY to allow headless EGL on this Jetson. The in-container supervisor forwards shutdown and watches the foreground Docker client with a Linux pidfd.

The camera and detector must use the same `ROS_DOMAIN_ID` for DDS discovery. This does **not** select the CUDA endpoint registry. When Fast DDS creates a local endpoint, it calls the loaded backend's `on_creating_endpoint`. The packaged IMX219 backend writes the endpoint GID to registry index **0**, including on nonzero DDS domains. The detector therefore uses index 0 for both registration and lookup. Discovery queries this registry by the GID reported by DDS; it does not use the registry to discover topics across DDS domains. Switching only the detector to a DDS-indexed registry would break CUDA negotiation with this camera.

The pinned `cuda_buffer` manager names the POSIX shared-memory object `/ros2_hem_d0_h<hostname hash>` using `gethostname()`. The two containers need the same hostname and IPC namespace. Both launchers use Docker host networking (which defaults to the host's hostname) and `--ipc=host`. Inter-process CUDA negotiation also checks UID and GPU device. A future producer that uses per-DDS-domain registries requires a coordinated plugin change on both sides, not just a different environment setting.

The bundled `cuda_buffer_backend` is based on ros2/rosidl_buffer_backends 0.1.2. It keeps deferred deleters mapped (`NODELETE`) and drops descriptors whose required CUDA writer event cannot be imported. For same-process or unknown peers, a writer waits for its event before sending a descriptor without a fence. The descriptor layout and the inter-process camera protocol are unchanged.

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
