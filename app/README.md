# AprilTag ROS Detector

Detects landing tags 0 and 77 and publishes their poses.

机载着陆标志检测 app。安装使用设备缓存的 Isaac ROS 5.0 / ROS 2 Lyrical 镜像
`airborne/isaac-ros:5.0-imx219-preview-20261022` 和随包提供的 Jetson Orin
CUDA buffer 插件。安装时检查 Docker、GPU 和镜像，运行时只读挂载 app。

在 aivudaOS 启动 IMX219 相机和本 app。两者的 ROS 域默认均为 0。
检测器可先启动，它会等待 `/camera/camera_info`，核对实际内参、畸变模型和焦距后
按图像尺寸配置去畸变。机上默认输入为模式 2、1280×720、旋转 180° 下的实测标定。
更换相机模式或标定后重启检测 app。停止检测器会释放它管理的 ROS 节点与容器。

- 官方 RectifyNode 订阅 `/camera/image_raw` 与 `/camera/camera_info`。两个官方
  AprilTagNode 在同一多线程组件容器内共享去畸变图像，输入队列深度沿用 3。
- 标志族为 tag36h11。两次位姿求解分别使用 ID 0 的 0.034 m 和 ID 77 的 0.289 m
  黑框边长，详见 `config/business_tags.yaml`。
- `/tag_detections` 为 `isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray`。
  合并器按采集时间戳和光学坐标系对齐，保留对应标签的角点、位姿和原图时间戳。
  两路都报告未检出时发布空数组。`/tf` 只转发对应黑框尺寸的原始变换。
- 默认后端 `CUDA` 与机载服务 2026-11-01 配置一致。旧台架记录显示 cuAprilTags
  对远处小标签直接求出的深度和倾角存在误差，接收方沿用自己的角点位姿解算。
  可在配置中心选择 `CPU` 复测原有 VPI 路径，修改后重启。

检测图来自 airborne 提交 `c896f25` 中的 `apriltag.launch.py` 和当前
`control/config/camera_profiles.yaml`。原始几何结果合并代码和标签尺寸表随包保留。
主机要查看结果时需加载与容器匹配的 Jazzy AprilTag 消息接口。在有标志的视场内，
核对实际检测 ID、角点、时间戳和话题频率；空数组只证明该帧完成了检测。
