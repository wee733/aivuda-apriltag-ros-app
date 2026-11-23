# AprilTag ROS Detector

The headless `apriltag_ros` app rectifies calibrated camera frames and publishes metric AprilTag detections and TF using NVIDIA Isaac ROS. The default configuration measures ID 0 at 0.034 m and ID 77 at 0.289 m. Edit tag rules in the AivudaOS configuration center and restart the detector.

Current package version is `isaacros5-imx219-1.2.0`. Run `./pack.sh` to create the release archive in `dist/`. The archive contains the app files tracked by Git, including the ARM64 component and CUDA buffer plugin.

See the [operator guide](app/README.md) and [developer notes](README_dev.md).
