# 桌面级机械臂颜色分拣系统（ROS + OpenCV + Dobot Magician）

> 浙江工业大学 · 信息工程学院 · 自动化 · 机器人控制课程设计
> 作者：高宇辰（302023510072）
> 完成时间：2026 年 6–7 月

---

## 项目简介

基于 ROS 构建一套完整的**视觉引导机械臂分拣系统**，实现从图像采集、颜色识别、相机标定、坐标变换到机械臂抓取搬运的全自动闭环。

系统完整覆盖机器人开发的 **感知 → 决策 → 执行** 三层链路：

| 层级 | 实现 |
|---|---|
| **感知层** | USB 相机采集图像 → HSV 颜色空间分割 → 六色物块识别与像素中心坐标计算 |
| **决策层** | 相机内参标定 → ArUco 外参标定 → TF 树坐标变换 → 像素坐标映射到机械臂基座坐标 |
| **执行层** | ROS Service 调用 Dobot API → PTP 点位运动 + 气动吸盘 → 按颜色分类放置 |

---

## 运行效果

**RViz 中的 TF 坐标变换与机械臂位姿可视化**
![RViz TF](docs/images/01-rviz-tf.jpg)

**六色物块实时识别与 HSV 二值化掩膜**
![Color Detection](docs/images/02-color-detect.jpg)

**DobotServer 串口连接与运动指令执行日志**
![Dobot Connect](docs/images/03-dobot-connect.jpg)

---

## 开发环境

| 项目 | 配置 |
|---|---|
| 操作系统 | Ubuntu 18.04 LTS（VMware Workstation 虚拟机） |
| ROS 版本 | **ROS Melodic**（`rosversion -d` 确认为 melodic 1.14.x） |
| 图像处理 | OpenCV（系统自带版本） |
| 相机 | Logitech C270i USB 单目相机 |
| 机械臂 | Dobot Magician（气动吸盘末端执行器） |
| 标定板 | 8×6 内部角点棋盘格，方格边长 0.024 m |
| ArUco 码 | DICT_5X5_100 字典，标记边长 0.10 m |
| 构建工具 | catkin_make / CMake |

> **版本说明**：本项目实际运行在 **ROS Melodic + Ubuntu 18.04** 上（见 `docs/images/03-dobot-connect.jpg` 终端截图）。
> 课程实验报告中填写的 "Ubuntu 20.04 + ROS Noetic" 为笔误，实际环境以本 README 为准。

---

## 系统架构

```
                    ┌─────────────────┐
                    │  Logitech C270i │
                    │    USB Camera   │
                    └────────┬────────┘
                             │ V4L2 + FFmpeg
                             ▼
                    ┌─────────────────┐
                    │     usb_cam     │  /usb_cam/image_raw
                    │      node       │
                    └────────┬────────┘
                             │ 640×480 YUYV
                             ▼
        ┌────────────────────────────────────────┐
        │         color_distinguish 节点          │
        │  BGR→HSV → inRange 二值化 → 中值滤波     │
        │  → findContours → 面积筛选 → 中心坐标    │
        └────────┬───────────────────┬───────────┘
                 │                   │
                 │ 六色物块像素坐标     │ Aruco 检测 → PnP 位姿解算
                 ▼                   ▼
        ┌────────────────────────────────────────┐
        │              TF 坐标变换树               │
        │  pixel → camera → world → dobot_base   │
        │        transformPoint() 自动查链         │
        └────────────────┬───────────────────────┘
                         │ 基座坐标系下的目标点
                         ▼
        ┌────────────────────────────────────────┐
        │         DobotClient_PTP 节点            │
        │  SetPTPCmd / SetEndEffectorSuctionCup  │
        │  / SetHOMECmd                          │
        └────────────────┬───────────────────────┘
                         ▼
                 DobotServer ←→ Dobot Magician
                         │
                         ▼
              抓取 → 抬升 → 搬运 → 释放（按颜色分拣）
```

---

## 核心功能模块

| 模块 | 具体实现 |
|---|---|
| 图像采集 | `usb_cam` 驱动 USB 相机，640×480 @ 30fps，`/dev/video0`，yuyv 格式 |
| 相机内参标定 | 张正友棋盘格法，求解内参矩阵 (fx, fy, cx, cy) 与畸变系数 (k1, k2, p1, p2, k3) |
| 物块识别 | HSV 颜色空间分割，识别红、橙、黄、绿、蓝、紫六色物块 |
| 坐标计算 | 计算物块在像素坐标系、相机坐标系、机械臂基座坐标系下的坐标 |
| TF 变换 | 发布 pixel→camera→world→dobot_base 完整变换链 |
| 机械臂控制 | PTP 运动、吸盘吸取/释放、回零 |

---

## 关键技术点

### 1. HSV 颜色空间识别

相比 RGB，HSV 将**色相（H）与亮度（V）解耦**，H 分量基本决定颜色类型，对光照变化鲁棒性更强。

```
输入图像(BGR) → cvtColor 转 HSV → inRange 阈值二值化
              → medianBlur(25×25) 去噪 → findContours 查轮廓
              → boundingRect 面积筛选 → 计算中心坐标
```

中心坐标计算：
```
u_center = (u_tl + u_br) / 2
v_center = (v_tl + v_br) / 2
```

**六色 HSV 阈值表**（实测调参结果）：

| 颜色 | H_min | H_max | S_min | S_max | V_min | V_max |
|---|---|---|---|---|---|---|
| 红色 | 0 | 10 | 43 | 255 | 46 | 255 |
| 红色（补） | 156 | 180 | 43 | 255 | 46 | 255 |
| 橙色 | 11 | 25 | 43 | 255 | 46 | 255 |
| 黄色 | 26 | 34 | 43 | 255 | 46 | 255 |
| 绿色 | 35 | 77 | 43 | 255 | 46 | 255 |
| 蓝色 | 100 | 124 | 43 | 255 | 46 | 255 |
| 紫色 | 125 | 155 | 43 | 255 | 46 | 255 |

> **红颜色需要双区间**：HSV 色环中红色跨越 0° 边界，必须同时取 H∈[0,10] 和 H∈[156,180] 两段并合并，否则红色物块会被漏检或只识别到一半。

### 2. 相机内参标定（张正友平面标定法）

利用平面棋盘格在**不同姿态**下的多幅图像，通过单应性矩阵求解相机内参。

```bash
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 --square 0.024 \
    image:=/usb_cam/image_raw camera:=/usb_cam
```

投影关系：

```
     [u]   [fx  0  cx] [R T] [Xw]
 s · [v] = [ 0  fy cy] [   ] [Yw]
     [1]   [ 0  0  1] [   ] [Zw]
                             [ 1]
```

**踩坑记录**：CALIBRATE 按钮长时间不亮 —— 原因是标定板移动范围不足，X/Y/Size/Skew 四个维度未充分覆盖。解决方法是让标定板缓慢移动并覆盖画面**四个角和中心**，同时改变倾斜角度。

### 3. 相机外参标定（ArUco + PnP）

ArUco 是一种二进制方形标记（黑边 + 内部编码矩阵），ID 唯一。通过检测四个角点，利用 **PnP（Perspective-n-Point）算法**求解相机相对标记的位姿，得到外参矩阵：

```
[R  T]     R: 3×3 旋转矩阵
[0  1]     T: 3×1 平移向量
```

旋转向量通过 **Rodrigues 变换**转为旋转矩阵后发布为 TF：

```cpp
cv::Rodrigues(marker_rvecs[0], rotated_matrix);
tf::Matrix3x3 tf_rotated_matrix(...);
tf::Transform transform(tf_rotated_matrix, tf_tvec);
pointer_marker_position_broadcaster->sendTransform(
    tf::StampedTransform(transform, ros::Time::now(), "logitech", "world"));
```

### 4. TF 坐标变换链（本项目最核心的部分）

| 坐标系 | 说明 |
|---|---|
| `pixel` | 图像像素坐标 (u, v)，原点在图像左上角 |
| `logitech`（camera） | 相机坐标系，以光心为原点，Zc 沿光轴 |
| `world` | 世界坐标系，以 ArUco Marker 中心为原点 |
| `dobot_base` | 机械臂基座坐标系，以 Dobot 底座中心为原点 |

**手动测量的 world → dobot_base 变换**（平移量由卷尺实测）：
```cpp
tf::Vector3 tf_tvecs(-0.143, 0.258, 0.138);  // 单位：米
```

**像素坐标 → 相机坐标**（需已知深度 Zc，由 ArUco 检测提供）：
```
Xc = (u - cx) * Zc / fx
Yc = (v - cy) * Zc / fy
Zc = 深度值（ArUco 检测得到）
```

**相机坐标 → 机械臂基座坐标**（TF 树自动完成变换链）：
```cpp
transformPoint("dobot_base", point_in_camera, point_in_base);
// 自动查询 camera → world → dobot_base 完整变换链
```

### 5. 机械臂控制（ROS Service）

| 服务名 | 功能 |
|---|---|
| `SetPTPCmd` | PTP 点位运动控制 |
| `SetEndEffectorSuctionCup` | 吸盘控制（suck=1 吸取，suck=0 释放） |
| `SetHOMECmd` | 机械臂回零 |
| `GetPose` | 获取机械臂当前姿态 |
| `SetEndEffectorParams` | 设置末端执行器偏移参数 |

**分拣动作序列**（按红→橙→黄→绿→蓝→紫顺序）：
```
移动到物块正上方（安全高度）→ 下降到物块表面 → 开启吸盘吸取
→ 上升到安全高度 → 移动到目标摆放位置 → 下降到摆放位置
→ 关闭吸盘释放 → 上升到安全高度
```

---

## 遇到的问题与解决（10 个真实调试记录）

这一节是项目的**核心价值所在**——真实工程问题的定位与解决过程。

### 4.1 VMware 虚拟机无法识别 USB 摄像头
- **现象**：`ls /dev/video*` 无输出
- **原因**：VMware USB 服务未启动 / 设备未手动挂载到虚拟机
- **解决**：虚拟机菜单 → 可移动设备 → 手动"连接"摄像头；USB 兼容性设为 3.1；重启 VMware USB Arbitration Service

### 4.2 摄像头无法打开 /dev/video0
- **现象**：`v4l2-ctl -d /dev/video0 --all` 报 `Cannot open device`
- **原因**：设备权限不足，当前用户不在 `video` 用户组
- **解决**：临时 `sudo chmod 666 /dev/video0`；永久 `sudo usermod -a -G video $USER`

### 4.3 usb_cam 报 `frame mapping timeout`
- **现象**：`Video4linux: frame mapping timeout (11)`，无法采集图像
- **原因**：VMware USB 带宽不足；像素格式与硬件不匹配
- **解决**：USB 控制器升至 3.1；`pixel_format` 由 mjpeg 改 yuyv；分辨率 640×480 → 320×240；帧率 30 → 10

### 4.4 MJPG 格式解码失败
- **现象**：`No JPEG data found in image` / `FFMPEG: error passing frame to decoder context`
- **原因**：虚拟机 USB 传输不稳定导致 JPEG 数据包损坏
- **解决**：改用 YUYV 原始格式，由 usb_cam 软件转换而非硬件解码；帧率降至 5–10fps
- **经验**：**在虚拟机环境中，选择最稳定的格式比选择最高效的格式更重要**

### 4.5 相机标定时 CALIBRATE 按钮始终灰色
- **现象**：不断移动标定板仍无法标定
- **原因**：标定板移动范围未覆盖图像全区域；角点检测失败
- **解决**：缓慢移动覆盖四角与中心；让 X/Y/Size/Skew 四维均有变化；改善光照避免反光阴影

### 4.6 物块识别结果始终为 0
- **现象**：`color_distinguish` 节点运行正常但各颜色计数均为 0
- **原因**：HSV 阈值与实际物块不匹配；面积阈值设置不当
- **解决**：用 HSV 取色工具实测物块真实 HSV 值重新标定阈值；面积阈值由 6000~10000 改为 1000~15000
- **经验**：**HSV 阈值高度依赖环境光照，换环境必须重新标定**

### 4.7 编译时找不到 tf 包
- **现象**：`Could not find a package configuration file provided by "tf"`
- **原因**：ROS 新版本中 `tf` 已被 `tf2_ros` 和 `tf2_geometry_msgs` 取代
- **解决**：`CMakeLists.txt` 中 tf → tf2_ros / tf2_geometry_msgs；头文件引用同步修改
- **经验**：**不同 ROS 版本 API 存在差异，需关注版本兼容性**

### 4.8 OpenCV 版本不兼容
- **现象**：`Could not find a configuration file for package "OpenCV" compatible with requested version "3"`
- **原因**：系统安装的是 OpenCV 4，而 CMakeLists.txt 指定了版本 3
- **解决**：`find_package(OpenCV 3 REQUIRED)` → `find_package(OpenCV REQUIRED)`，不指定版本自动适配

### 4.9 Dobot 机械臂连接失败
- **现象**：`rosrun dobot DobotServer ttyUSB0` 无法连接
- **原因**：串口设备权限不足；设备号不一定是 ttyUSB0
- **解决**：`ls /dev/ttyUSB*` 确认真实设备号；`sudo chmod 666 /dev/ttyUSB*`；确保机械臂上电
- **经验**：**串口设备每次重新插拔后都需重设权限**

### 4.10 机械臂 PTP 运动偏移过大
- **现象**：实际到达位置与预期偏差超过 1cm
- **原因**：末端执行器偏移参数未设置；坐标变换调参不准；机械臂未回零
- **解决**：
  1. 设置吸盘末端偏移：`srv5.request.xBias = 61;`（吸盘相对末端的 x 向偏移，单位 mm）
  2. 坐标变换实测补偿：`output.x1.push_back(result_out.point.x + 0.004);`
  3. PTP 运动前先执行 `SetHOMECmd` 回零
- **经验**：**末端执行器与理论模型存在物理偏移，必须在软件中补偿**

---

## 项目收获

1. **ROS 系统理解**：实际搭建包含 7 个以上节点的完整系统，理解话题/服务的分布式通信机制、模块化开发与松耦合优势，掌握 rviz 可视化、rqt_graph 节点关系查看、rostopic 调试等工具链。
2. **视觉处理能力**：掌握 HSV 颜色空间、中值滤波、轮廓检测、张正友标定、ArUco 位姿估计的**原理与实际调参方法**，认识到视觉算法效果高度依赖环境条件。
3. **坐标变换理解**：通过亲手搭建 pixel→camera→world→dobot_base 完整变换链，深入理解内参矩阵、外参矩阵、TF 树与 `transformPoint` 的作用。
4. **工程实践能力**：涵盖硬件调试（USB/串口/虚拟机配置）、软件编译（CMake 依赖管理/catkin）、算法调参（HSV/面积/偏移补偿）、系统集成（多节点启动顺序与联调）。
5. **系统思维**：建立"感知→决策→执行"的完整机器人系统开发框架，理解**任何环节的微小偏差都会影响最终结果**。

---

## 后续改进方向

- 用深度学习模型替代 HSV 阈值法，提升不同光照条件下的识别鲁棒性
- 增加避障算法，优化机械臂运动规划
- 优化图像处理流程提升实时性
- 增加异常检测与自动恢复机制
- 迁移至 ROS2

---

## 关于源代码

本项目源代码因存储介质问题已丢失。本仓库保存**完整的技术实现文档、系统架构设计、核心算法原理与代码片段、以及全部调试记录**，可用作技术复现参考。

如需了解任何技术细节，欢迎通过 Issue 交流。

---

## 参考

- 课程实验报告原文（完整版含标定截图与运行日志）
- ROS Wiki: [usb_cam](http://wiki.ros.org/usb_cam) · [camera_calibration](http://wiki.ros.org/camera_calibration) · [tf](http://wiki.ros.org/tf)
- OpenCV: ArUco marker detection · Rodrigues transform
- Dobot ROS Demo
