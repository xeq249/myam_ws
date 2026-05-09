# 无人机–机械臂协同控制（毕业设计代码仓库）

本目录为 **ROS catkin 工作空间 `test_ws`**，面向 **多旋翼无人机 + RoArm M3 机械臂** 的协同控制与抓取相关研究：包含 **整机 URDF / Gazebo 仿真、PX4 SITL、MAVROS、MoveIt 运动规划、RealSense D435 与 AprilTag 感知**，以及 **独立机械臂在真机侧的 HTTP 示例节点**。

> **说明**：仿真与整机 MoveIt 配置以 **`myam_sys`**（挂载机械臂与 D435 的无人机模型）为主；单独机械臂的 MoveIt 配置见 **`roarm_moveit`**。真机舵机/USB 串口需在固件侧或自研驱动与 ROS 控制器之间做桥接，本仓库侧重仿真与 ROS 上层逻辑。

---

## 环境依赖（概要）

以下为典型开发环境，请按实验室机器实际安装的发行版对齐。

| 组件 | 用途 |
|------|------|
| **ROS Noetic**（推荐） | 工作空间与各 `package.xml` 依赖均以 Noetic 系为主 |
| **catkin_tools** 或 **catkin_make** | 编译 |
| **Gazebo Classic**、`gazebo_ros`、`gazebo_ros_control` | 机身 + 臂 + 场景的物理与控制仿真 |
| **MoveIt**、`trac_ik`（`myam_sys_moveit` 已声明） | 机械臂（及可选整机）运动规划 |
| **MAVROS** | 与 PX4 飞控通信（SITL / 真机视配置而定） |
| **PX4-Autopilot**（SITL） | `myam_simulation` 中一键脚本默认使用 `$(HOME)/PX4-Autopilot` |
| **Intel RealSense SDK + `realsense2_camera`** | 真机或仿真中 D435 图像/深度（本仓库含 `realsense-ros` 子树） |
| **`apriltag_ros`** | AprilTag 检测（仿真栈） |
| **`realsense_ros_gazebo`** | Gazebo 中 D435 插件（`myam_description` 中 `exec_depend`） |

其他 Python 依赖以各包脚本为准（如 `cv_bridge`、`moveit_commander` 等）。

---

## 工作空间内主要功能包

| 包名 | 作用 |
|------|------|
| **`myam_description`** | 整机 `myam_sys` 的 URDF/xacro、网格、Gazebo 插件与传感器相关描述；通过 xacro 挂载 **RoArm M3**（网格与关节命名与 `roarm_description` 对齐） |
| **`myam_sys_moveit`** | 整机（含 `roarm_base_link` → `hand_tcp` 规划链等）的 **MoveIt** 配置、RViz、与 Gazebo/`ros_control` 协同的 launch |
| **`myam_simulation`** | **PX4 SITL + Gazebo + MAVROS + D435 + AprilTag** 等组合启动脚本，以及离机抓取、场景同步等实验用 launch |
| **`roarm_description`** | **RoArm M3** 单机模型：RViz 显示、Gazebo 与 `gazebo_ros_control` 仿真用 URDF |
| **`roarm_moveit`** | **仅机械臂** 的 MoveIt 配置（基于 `roarm_description`） |
| **`roarm_blue_cube_http`** | D435 检测蓝色立方体，经 **HTTP** 发送 JSON 控制 **真机 RoArm-M3**（与 MoveIt 解耦的示例路径） |
| **`realsense-ros`** | Intel 官方 RealSense ROS1 封装（作为子目录引入） |
| **`apriltag_ros`** | AprilTag ROS 封装 |

---

## 编译

```bash
cd /path/to/myam_ws
source /opt/ros/noetic/setup.bash
catkin_make   # 或 catkin build
source devel/setup.bash
```

若首次编译缺少依赖，请根据报错用 `rosdep` 或 apt 补全（如 `mavros`、`moveit` 元包等）。

---

## 典型使用方式

### 1. 仿真：PX4 SITL + Gazebo + D435 + AprilTag

一键启动（需本机已编译 PX4，且路径与 launch 中 `px4_dir` 一致）：

```bash
roslaunch myam_simulation sitl_moveit.launch
```

可按 launch 内注释调整 `camera_ns`、`sitl_world` 等参数。

### 2. 仿真：整机 + MoveIt + 抓取相关任务

与 `sitl_moveit` 同栈，默认 **不启 RViz**（减轻负载），需要可视化时：

```bash
roslaunch myam_simulation sitl_task.launch use_rviz:=true
```

可选配合场景中的台面/碰撞体同步、地面蓝块等（见 `myam_simulation/launch/` 内各文件头注释，如 `myam_grasping_scene_sync.launch`、`sitl_grasp_blue_ground.launch`）。

**MoveIt 单机演示**（不启整套 SITL 时，常以 fake 控制器为主）：

```bash
roslaunch myam_sys_moveit demo.launch
```

真机或 Gazebo 联合控制时，将 `moveit_controller_manager` 等为 `ros_control` / `simple` 等，并保证 **`/joint_states`** 与控制器接口一致（见 `demo.launch` 内参数说明）。

### 3. 单机 RoArm M3：模型与 MoveIt

- RViz / Gazebo 模型：`roarm_description` 中 `display.launch`、`gazebo.launch`
- 仅臂 MoveIt：`roarm_moveit` 中相应 launch（需先 `roslaunch` 加载 `robot_description` 与关节状态，与官方 setup 一致）

### 4. 真机：蓝块检测 + HTTP 控制机械臂

见包内说明与 launch：

```bash
roslaunch roarm_blue_cube_http blue_cube_to_roarm_http.launch
```

需同时运行 RealSense 节点，并在参数中配置 HTTP 服务端点与相机话题等（详见该包 `launch` 与节点源码）。

---

## 更多

- **协同控制**：飞控层由 **PX4 + MAVROS** 提供状态与 offboard 等接口；机械臂由 **MoveIt** 或独立节点（如 HTTP/串口）执行；两者通过 ROS 话题/服务/时间同步（仿真中常用 `/use_sim_time`）耦合。
- **坐标系**：整机规划常以 **`roarm_base_link`**、末端 **`hand_tcp`** 等与 SRDF/MoveIt 一致；仿真与真机务必统一 TF 与外参。
- **真机 MoveIt**：需满足 **关节状态上报 + 轨迹执行**（`FollowJointTrajectory` 或与 `moveit_simple_controller_manager`/`ros_control` 等价的接口）；纯 USB JSON 直连需自研适配层后方可与 MoveIt 无缝衔接。

---

## 许可与子项目

各子目录多为 **BSD** 或与上游一致的开源许可证；`roarm_blue_cube_http` 为 **MIT**。**`realsense-ros`、`apriltag_ros`** 等请遵循其原作者许可与引用要求。

