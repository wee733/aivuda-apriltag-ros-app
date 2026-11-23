# Detector development

The [ROS launch graph](app/scripts/detector.launch.py) describes all three components. The production entry point waits for valid CameraInfo, starts the stock multithreaded component container, and uses a short-lived loader to submit the same component descriptions. Each load must succeed. The native watcher terminates the group if the host owner exits. The detector has no HTTP endpoints.

The [C++ component](app/tag_scale_filter/src/tag_scale_filter.cpp) accepts unique ownership of detection and TF messages on the two internal topics. It preserves frame metadata, corners and orientation. It multiplies translation by the configured edge length divided by the detector's base edge length. The base is the maximum of all per-ID lengths and the default length. Unknown IDs are omitted in configured-only mode. An empty detections message still signals a processed frame. TF publishes only when at least one tag is retained.

See [target build and verification](docs/apriltag.md) for build commands, the packaged ARM64 files and hardware tests. Stage new app files before running `./pack.sh`.
