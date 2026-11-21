# AprilTag ROS Detector

This app rectifies the calibrated camera stream, detects AprilTags with NVIDIA Isaac ROS, and publishes detections and transforms. The Web UI shows detections and supports changes to per-ID sizing and filtering. The default tag36h11 configuration defines ID 0 at 0.034 m and ID 77 at 0.289 m.

The [launch graph](app/scripts/detector.launch.py) runs a single detector. The [result router](app/scripts/tag_result_mux.py) applies metric sizes and filters results. The app includes the [Isaac ROS runtime adapter](app/isaac_runtime/) and uses the provisioned Isaac ROS 5 Lyrical container on the Jetson. See the [configuration schema](app/config/config_schema.yaml).

## Build the aivudaOS package

App ID `apriltag_ros` · Version `isaacros5-imx219-1.1.2`

From a Git checkout, run the following command with Bash, Git, GNU tar, gzip and Python 3 with PyYAML installed.

```bash
./pack.sh
```

The archive in `dist/` has the app manifest at its root. The script uses the working copies of files tracked under `app/`. Stage new application files with Git before packaging. Upload the archive through the aivudaOS app installer.
