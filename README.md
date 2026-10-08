# 桌面级机械臂颜色分拣系统（ROS + OpenCV + Dobot Magician）

> 浙江工业大学 · 信息工程学院 · 自动化 · 机器人控制课程设计
> 作者：高宇辰（302023510072）　指导老师：禹鑫燚
> 完成时间：2026 年 6–7 月

基于 ROS 构建一套视觉引导的机械臂分拣系统，实现 USB 相机采集 → HSV 颜色识别 → 相机内外参标定 → ArUco 位姿估计 → 机械臂 PTP 运动抓取搬运的完整流程。

---

## 运行效果

**RViz 中的坐标系关系与机械臂位姿可视化**
![RViz TF](docs/images/01-rviz-tf.jpg)

**六色物块实时识别（左）与 HSV 二值化掩膜（右）**
![Color Detection](docs/images/02-color-detect.jpg)

**DobotServer 串口连接与运动指令执行日志**
![Dobot Connect](docs/images/03-dobot-connect.jpg)

---

## 开发环境

| 项目 | 配置 |
|---|---|
| 操作系统 | Ubuntu 18.04 LTS（VMware Workstation 虚拟机） |
| ROS 版本 | **ROS Melodic**（`rosversion -d` → melodic 1.14.x） |
| 图像处理 | OpenCV 4 |
| 相机 | Logitech C270i USB 单目相机 |
| 机械臂 | Dobot Magician（气动吸盘末端执行器） |
| 标定板 | 8×6 内部角点棋盘格，方格边长 0.024 m |
| ArUco 码 | `DICT_5X5_100` 字典，标记边长 0.10 m |
| 构建工具 | catkin_make / CMake（C++11） |

> **版本说明**：本项目实际运行在 **ROS Melodic + Ubuntu 18.04** 上（见 `docs/images/03-dobot-connect.jpg` 终端截图与 `src/axif_tf/CMakeLists.txt` 中的 `tf2_ros` 依赖）。
> 课程实验报告中填写的 "Ubuntu 20.04 + ROS Noetic" 为笔误，实际环境以本 README 为准。

---

## 仓库内容

```
src/
├── opencvtest/          六色物块识别节点（自行开发）
│   ├── src/sorting.cpp          394 行，颜色识别核心
│   └── msg/pixel_point0.msg     自定义消息：6 色 × (u,v) 坐标数组
├── axif_tf/             ArUco Marker 检测与相机位姿估计（自行开发）
│   └── src/getmarker.cpp        105 行
└── dobot/               Dobot 机械臂 ROS 接口
    └── src/DobotClient_PTP.cpp  PTP 运动控制客户端

docs/
├── 机器人控制课程设计实验报告.docx   完整实验报告（含标定截图、10 项调试记录）
└── images/                          运行截图 3 张
```

**编译产物**（已通过 `catkin_make` 构建成功）：
`devel/lib/opencvtest/sorting` · `devel/lib/axif_tf/getmarker` · `devel/lib/dobot/DobotClient_PTP` · `devel/lib/dobot/DobotServer` · `devel/lib/usb_cam/usb_cam_node`

### 关于完整性（如实说明）

本仓库为工作区**恢复后的产物**。第三方依赖（`usb_cam` 驱动包、Dobot 官方 SDK 与 Qt/ICU 二进制库）未纳入——它们属于厂商代码，且体积达 227 MB。

**坐标变换与分拣动作序列部分的源码未能恢复。** 本仓库包含的自行开发代码为：

| 文件 | 状态 |
|---|---|
| `src/opencvtest/src/sorting.cpp` | ✅ **完整**，六色识别 + HSV 取色工具 + 自定义消息发布 |
| `src/axif_tf/src/getmarker.cpp` | ⚠️ ArUco 检测与位姿估计**已完成**；**不含 TF 变换广播**（`sendTransform`）部分 |
| `src/dobot/src/DobotClient_PTP.cpp` | ⚠️ 基于 Dobot 官方例程改造，PTP 参数配置完整；**不含按颜色排序的分拣动作序列** |

实验报告（`docs/`）中记录了坐标变换链 `pixel → camera → world → dobot_base` 的原理推导、变换公式与手动测量参数，以及全部 10 项调试问题的排查过程，可作复现参考。

---

## 核心实现

### 一、六色物块识别（`src/opencvtest/src/sorting.cpp`，394 行）

ROS 节点名 `color_distinguish`，订阅 `/usb_cam/image_raw`，发布自定义话题 `pixel_center_axis`。

**处理流程：**
```
BGR 图像 → cvtColor 转 HSV → inRange 阈值二值化 → medianBlur(25×25) 中值滤波
        → findContours 查找外轮廓 → boundingRect 面积筛选(1000~15000 px²)
        → 计算中心坐标 → 按颜色分类存入消息数组 → 绘制标注
```

**中心坐标计算：**
```cpp
double center_u = 0.5 * (rect.tl().x + rect.br().x);
double center_v = 0.5 * (rect.tl().y + rect.br().y);
```

**六色 HSV 阈值**（实测调参结果，代码中定义为 `ColorConfig` 结构体）：

| 颜色 | H_min | H_max | S_min | S_max | V_min | V_max |
|---|---|---|---|---|---|---|
| 红色区间1 | 0 | 10 | 43 | 255 | 46 | 255 |
| 红色区间2 | 156 | 180 | 43 | 255 | 46 | 255 |
| 橙色 | 11 | 25 | 43 | 255 | 46 | 255 |
| 黄色 | 26 | 34 | 43 | 255 | 46 | 255 |
| 绿色 | 35 | 77 | 43 | 255 | 46 | 255 |
| 蓝色 | 100 | 124 | 43 | 255 | 46 | 255 |
| 紫色 | 125 | 155 | 43 | 255 | 46 | 255 |

**关键技术点：红色双区间合并**

HSV 色环上红色跨越 0° 边界，单一阈值区间无法覆盖。代码中单独实现 `processRed()`：

```cpp
inRange(hsv, Scalar(0,   43, 46), Scalar(10,  255, 255), mask1);   // 区间1
inRange(hsv, Scalar(156, 43, 46), Scalar(180, 255, 255), mask2);   // 区间2
bitwise_or(mask1, mask2, mask_combined);                            // 合并
medianBlur(mask_combined, mask_combined, 25);
```

**HSV 取色工具**：为便于现场调参，在识别窗口实现了鼠标回调——点击画面任意位置即打印该点 BGR 与 HSV 值并在图像上标注。这是解决"HSV 阈值与实际光照不匹配导致识别数为 0"问题的实用手段。

**自定义消息 `pixel_point0.msg`：**
```
string name
float64[] red_u, red_v
float64[] orange_u, orange_v
float64[] yellow_u, yellow_v
float64[] green_u, green_v
float64[] blue_u, blue_v
float64[] purple_u, purple_v
```

---

### 二、ArUco 检测与相机位姿估计（`src/axif_tf/src/getmarker.cpp`）

ROS 节点名 `axif_tf`，订阅 `/usb_cam/image_raw`。

```cpp
dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
cv::aruco::detectMarkers(marker_image, dictionary, corners, ids);
cv::aruco::estimatePoseSingleMarkers(corners, 0.10, camera_matrix, dist_coeffs, rvecs, tvecs);
cv::aruco::drawAxis(marker_image, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1);
Zc = tvecs[0][2];   // 提取深度，供像素坐标→相机坐标换算使用
```

原理：检测 ArUco 标记的四个角点，用 **PnP（Perspective-n-Point）算法**求解相机相对标记的位姿，得到旋转向量 `rvec` 与平移向量 `tvec`；`rvec` 经 **Rodrigues 变换**可转为 3×3 旋转矩阵。`tvec` 的 Z 分量即物块所在平面的深度 `Zc`。

**相机内参标定结果**（张正友棋盘格法实测，硬编码于源码）：

```cpp
// 内参矩阵 K
camera_matrix = [ 833.5051,   0,       330.5683;
                  0,          833.8074, 255.7232;
                  0,          0,        1       ]

// 畸变系数 [k1, k2, p1, p2, k3]
dist_coeffs = [0.04509, 0.22342, 0.004863, 0.004637, 0]
```

标定命令：
```bash
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 --square 0.024 \
    image:=/usb_cam/image_raw camera:=/usb_cam
```

**像素坐标 → 相机坐标**（需已知深度 `Zc`）：
```
Xc = (u - cx) * Zc / fx
Yc = (v - cy) * Zc / fy
```

**相机坐标 → 机械臂基座坐标**：实验报告中通过 TF 树查询实现：
```cpp
transformPoint("dobot_base", point_in_camera, point_in_base);
// 自动完成 camera → world → dobot_base 变换链
// world → dobot_base 平移量为卷尺实测：(-0.143, 0.258, 0.138) 米，旋转按单位矩阵处理
```
> ⚠️ 该变换节点（`transform_base` / `sendDobotTf`）的源码本次未恢复，仅存于实验报告。

---

### 三、Dobot 机械臂控制（`src/dobot/src/DobotClient_PTP.cpp`）

通过 ROS Service 调用 `DobotServer` 提供的接口：

| 服务名 | 功能 |
|---|---|
| `SetPTPCmd` | PTP 点位运动控制 |
| `SetEndEffectorParams` | 设置末端执行器偏移参数 |
| `SetEndEffectorSuctionCup` | 吸盘控制（suck=1 吸取，suck=0 释放） |
| `SetHOMECmd` | 机械臂回零 |
| `SetPTPJumpParams` | 设置门型运动抬升高度与 Z 轴限位 |
| `SetPTPCommonParams` | 设置速度 / 加速度比例 |

**实测调参值**（源码中的配置）：
```cpp
srv5.request.xBias = 70;              // 吸盘相对末端的 x 向物理偏移 (mm)
srv.request.velocity = 100;           // 四轴速度
srv.request.acceleration = 100;       // 四轴加速度
srv.request.xyzVelocity = 100;        // 笛卡尔速度
srv.request.jumpHeight = 20;          // 门型运动抬升高度 (mm)
srv.request.zLimit = 200;             // Z 轴限位 (mm)
srv.request.velocityRatio = 50;       // 速度比例 (%)
srv.request.accelerationRatio = 50;   // 加速度比例 (%)
```

> **关于 `xBias` 补偿**：机械臂末端执行器与理论模型存在物理偏移，直接按运动学计算的坐标抓取会产生偏差。通过实测补偿吸盘物理偏移量解决，这是把抓取误差压到 ±1cm 以内的关键动作之一。

---

## 遇到的问题与解决（10 个真实调试记录）

> 以下问题均来自实际调试过程，完整版见 `docs/机器人控制课程设计实验报告.docx` 第 4 章。

### 1. VMware 虚拟机无法识别 USB 摄像头
- **现象**：`ls /dev/video*` 无输出
- **原因**：VMware USB 服务未启动 / 设备未手动挂载到虚拟机
- **解决**：虚拟机菜单 → 可移动设备 → 手动"连接"摄像头；USB 兼容性设为 3.1；重启 VMware USB Arbitration Service

### 2. 摄像头无法打开 `/dev/video0`
- **现象**：`v4l2-ctl -d /dev/video0 --all` 报 `Cannot open device`
- **原因**：设备权限不足，当前用户不在 `video` 用户组
- **解决**：临时 `sudo chmod 666 /dev/video0`；永久 `sudo usermod -a -G video $USER`

### 3. usb_cam 报 `frame mapping timeout`
- **现象**：`Video4linux: frame mapping timeout (11)`，无法采集图像
- **原因**：VMware USB 带宽不足；像素格式与硬件不匹配
- **解决**：USB 控制器升至 3.1；`pixel_format` 由 mjpeg 改 yuyv；降低分辨率；帧率 30 → 10

### 4. MJPG 格式解码失败
- **现象**：`No JPEG data found in image` / `FFMPEG: error passing frame to decoder context`
- **原因**：虚拟机 USB 传输不稳定导致 JPEG 数据包损坏
- **解决**：改用 YUYV 原始格式，由 usb_cam 软件转换而非硬件解码；帧率降至 5–10 fps
- **经验**：**在虚拟机环境中，选择最稳定的格式比选择最高效的格式更重要**

### 5. 相机标定时 CALIBRATE 按钮始终灰色
- **现象**：不断移动标定板仍无法标定
- **原因**：标定板移动范围未覆盖图像全区域，未满足标定算法的姿态多样性要求
- **解决**：缓慢移动覆盖四角与中心，使 X / Y / Size / Skew 四个维度均有变化；改善光照避免反光和阴影

### 6. 物块识别结果始终为 0
- **现象**：`color_distinguish` 节点运行正常但各颜色计数均为 0
- **原因**：HSV 阈值与实际物块颜色不匹配；面积阈值区间设置不当
- **解决**：编写 HSV 鼠标取色工具实测物块真实 HSV 值重新标定阈值；面积阈值由 6000~10000 调整为 **1000~15000**
- **经验**：**HSV 阈值高度依赖环境光照，更换环境必须重新标定**

### 7. 编译时找不到 `tf` 包
- **现象**：`Could not find a package configuration file provided by "tf"`
- **原因**：新版 ROS 中 `tf` 已被 `tf2_ros` 和 `tf2_geometry_msgs` 取代
- **解决**：`CMakeLists.txt` 中依赖改为 `tf2_ros` / `tf2_geometry_msgs`，头文件引用同步修改
- **经验**：**不同 ROS 版本 API 存在差异，需关注版本兼容性**

### 8. OpenCV 版本不兼容
- **现象**：`Could not find a configuration file for package "OpenCV" compatible with requested version "3"`
- **原因**：系统安装的是 OpenCV 4，而 `CMakeLists.txt` 指定了版本 3
- **解决**：`find_package(OpenCV 3 REQUIRED)` → `find_package(OpenCV REQUIRED)`，不指定版本自动适配

### 9. Dobot 机械臂连接失败
- **现象**：`rosrun dobot DobotServer ttyUSB0` 无法连接
- **原因**：串口设备权限不足；实际设备号不一定是 `ttyUSB0`
- **解决**：`ls /dev/ttyUSB*` 确认真实设备号；`sudo chmod 666 /dev/ttyUSB*`；确保机械臂上电
- **经验**：**串口设备每次重新插拔后都需重设权限**

### 10. 机械臂 PTP 运动偏移过大
- **现象**：实际到达位置与预期偏差超过 1cm
- **原因**：末端执行器偏移参数未设置；坐标变换参数不准；机械臂未回零
- **解决**：
  1. 设置吸盘末端偏移 `xBias`（源码中为 70 mm），补偿物理偏移
  2. 在坐标变换中对实测偏差做补偿修正
  3. PTP 运动前先执行 `SetHOMECmd` 回零
- **经验**：**末端执行器与理论模型存在物理偏移，必须在软件中补偿**

---

## 项目收获

1. **ROS 系统理解**：实际搭建包含 7 个以上节点的系统，理解话题 / 服务的分布式通信机制、模块化开发与松耦合设计，掌握 rviz 可视化、rqt_graph 节点关系查看、rostopic 调试等工具链。
2. **视觉处理能力**：掌握 HSV 颜色空间、中值滤波、轮廓检测、张正友标定、ArUco 位姿估计的**原理与实际调参方法**，认识到视觉算法效果高度依赖环境光照条件。
3. **工程实践能力**：涵盖硬件调试（USB 挂载 / 串口权限 / 虚拟机配置）、软件编译（CMake 依赖管理 / catkin 构建）、算法调参（HSV 阈值 / 面积阈值 / 末端偏移补偿）、系统集成（多节点启动顺序与联调）。
4. **问题定位方法**：学会通过分析日志、查阅文档、逐层排除来定位问题——例如摄像头采集失败，依次排查 USB 挂载、用户组权限、USB 带宽、像素格式四个方向才最终定位。

---

## 后续改进方向

- 用深度学习模型替代 HSV 阈值法，提升不同光照条件下的识别鲁棒性
- 补全 TF 变换节点，将坐标变换链完整纳入版本管理
- 增加避障算法，优化机械臂运动规划
- 增加异常检测与自动恢复机制
- 迁移至 ROS2

---

## 参考

- ROS Wiki: [usb_cam](http://wiki.ros.org/usb_cam) · [camera_calibration](http://wiki.ros.org/camera_calibration) · [tf2](http://wiki.ros.org/tf2)
- OpenCV: ArUco marker detection · Rodrigues transform · PnP
- Dobot ROS Demo（官方例程）
