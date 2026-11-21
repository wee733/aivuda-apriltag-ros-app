# AprilTag ROS Detector

Detects landing tags 0 and 77 and publishes their poses.

机载着陆标志检测 app。安装使用设备缓存的 Isaac ROS 5.0 / ROS 2 Lyrical 镜像
`airborne/isaac-ros:5.0-imx219-preview-20261022` 和随包提供的 Jetson Orin
CUDA buffer 插件。安装时检查 Docker、GPU 和镜像，运行时只读挂载 app。

在 aivudaOS 启动 IMX219 相机和本 app。检测器的 `apriltag.ros_domain_id` 默认为 `-1`：优先使用系统配置 `sys.ros2_default_domain_id`，未配置时使用启动环境的 `ROS_DOMAIN_ID`，两者都不存在时使用 `0`。显式填写 `0` 到 `232` 可覆盖继承。用于启动的域值无效时会报错，不会自动切换到域 `0`。

相机、检测器和结果订阅者必须使用同一个实际域。例如，系统域为 `42` 而旧版相机仅从环境变量继承时，请在相机配置中显式填写 `42`，或者给相机启动环境设置 `ROS_DOMAIN_ID=42`。升级后若保留了显式配置 `0`，需要继承时请改为 `-1` 并重启。
检测器可先启动，它会等待 `/camera/camera_info`，核对实际内参、畸变模型和焦距后
按图像尺寸配置去畸变。机上默认输入为模式 2、1280×720、旋转 180° 下的实测标定。
更换相机模式或标定后重启检测 app。停止检测器会释放它管理的 ROS 节点与容器。

- 官方 RectifyNode 订阅 `/camera/image_raw` 与 `/camera/camera_info`。同一多线程组件容器内的一个 AprilTagNode 处理去畸变图像。
- 默认标志族为 `tag36h11`。检测节点以配置中的最大边长（含 `default_size_m`）为基准求解位姿；结果路由按各 ID 的实际黑框边长与基准边长之比缩放平移，默认 ID 0 为 0.034 m、ID 77 为 0.289 m。
- `/tag_detections` 为 `isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray`。缩放不改变图像角点、旋转或消息头；`/tf` 使用相同的标签筛选规则与平移比例。
- 默认后端 `CUDA` 与机载服务 2026-11-01 配置一致。旧台架记录显示 cuAprilTags
  对远处小标签直接求出的深度和倾角存在误差，接收方沿用自己的角点位姿解算。
  可在配置中心选择 `CPU` 复测原有 VPI 路径，修改后重启。

检测图来自 airborne 提交 `c896f25` 中的 `apriltag.launch.py` 和当前
`control/config/camera_profiles.yaml`。原始几何结果合并代码和标签尺寸表随包保留。
主机要查看结果时需加载与容器匹配的 Jazzy AprilTag 消息接口。在有标志的视场内，
核对实际检测 ID、角点、时间戳和话题频率；空数组只证明该帧完成了检测。
