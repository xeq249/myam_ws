#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offboard：巡航悬停 → 机械臂 home → 原地悬停若干秒 → 前飞至前向点 → 下降至更低悬停 → 彩色图识别蓝块
（检测阶段可按像素误差微调前飞点对准目标，长时间无分割则自动再下降）→
深度+TF 得质心后 MoveIt 末端接近（姿态约束 pose 目标 + 分段；再失败则姿态两步回退，最后才固定关节备份）→ 回 home → 爬升返航 → AUTO.LAND。

用法：先启动 sitl_ground_arm / MAVROS / move_group，再 roslaunch myam_simulation offboard_grasp_blue_cube.launch
默认彩色图话题为 Gazebo realsense 插件的 /camera_realsense/color/image_raw（勿与 ROS realsense 驱动的 /camera/ 混淆）。

注意：MoveIt 的 go(wait=True) 会长时间占用线程；若主线程无法处理 joint_states，
轨迹执行会永远不结束（表现为卡在「机械臂 home」）。本脚本在 main 中启动后台回调：
优先 AsyncSpinner（若 rospy 提供），否则使用 daemon 线程 rospy.spin()。

PX4 Offboard 须**持续**收到位置设定点（通常 ≥2 Hz）。若主循环在 arm.go()/抓取 等 MoveIt
调用上阻塞，会长时间不发布 →「No offboard signal」/Failsafe 切 Position；MoveIt 结束后又恢复。
因此另起**独立线程**按固定频率发布 setpoint_raw/local，与主循环解耦。

重要：同一进程内对 moveit_commander 应只 roscpp_initialize 一次，且不要反复 roscpp_shutdown，
否则抓取阶段会无法再连接 move_group action。本脚本全程复用一组 MoveGroupCommander。
"""

from __future__ import print_function

import copy
import math
import sys
import threading
import time

import cv2
import moveit_commander
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from moveit_commander import MoveGroupCommander
from moveit_commander.exception import MoveItCommanderException
from sensor_msgs.msg import CameraInfo, Image

try:
    from tf2_geometry_msgs import do_transform_point
    from tf2_geometry_msgs import do_transform_vector3
except ImportError:
    do_transform_point = None
    do_transform_vector3 = None


def start_rospy_callbacks_for_moveit(num_threads=4):
    """在 MoveIt go(wait=True) 阻塞主线程时，仍须处理 joint_states / Action 回调。"""
    try:
        from rospy.timer import AsyncSpinner as _AsyncSpinner  # type: ignore
    except ImportError:
        _AsyncSpinner = None
    if _AsyncSpinner is not None:
        spin = _AsyncSpinner(num_threads)
        spin.start()
        rospy.loginfo(
            "已启动 rospy.timer.AsyncSpinner(threads=%d)，供 MoveIt 执行", num_threads
        )
        return spin
    if hasattr(rospy, "AsyncSpinner"):
        spin = rospy.AsyncSpinner(num_threads)
        spin.start()
        rospy.loginfo(
            "已启动 rospy.AsyncSpinner(threads=%d)，供 MoveIt 执行", num_threads
        )
        return spin
    thr = threading.Thread(name="rospy_spin_callbacks", target=rospy.spin)
    thr.daemon = True
    thr.start()
    rospy.loginfo(
        "已启动后台 rospy.spin() 线程（当前 rospy 无 AsyncSpinner，用于 joint_states）"
    )
    return thr


def start_offboard_setpoint_stream(mission, hz):
    """
    独立线程持续发布 MAVROS Offboard 位置设定点，避免主线程阻塞在 MoveIt 时 PX4 丢失 Offboard。
    """
    hz = max(1.0, float(hz))

    def _loop():
        r = rospy.Rate(hz)
        while not rospy.is_shutdown():
            if mission.phase == mission.DONE:
                r.sleep()
                continue
            tup = mission.get_offboard_sp_tuple()
            if tup is None:
                r.sleep()
                continue
            x, y, z, yaw = tup
            mission.sp_pub.publish(mission.sp(x, y, z, yaw))
            r.sleep()

    th = threading.Thread(name="offboard_setpoint_stream", target=_loop)
    th.daemon = True
    th.start()
    rospy.loginfo(
        "Offboard 设定点独立线程已启动（%.1f Hz），MoveIt 阻塞时仍保持 PX4 Offboard",
        hz,
    )
    return th


def maybe_publish_offboard(m, x, y, z, yaw):
    """未启用独立线程时由主循环发布；启用时由后台线程发布，此处不再发。"""
    if not m.offboard_stream_enable:
        m.sp_pub.publish(m.sp(x, y, z, yaw))


def transform_point(buf, pt, target_frame, source_frame, timeout=2.0):
    if do_transform_point is None:
        rospy.logerr("请安装 ros-$ROS_DISTRO-tf2-geometry-msgs")
        return None
    ps = PointStamped()
    ps.header.frame_id = source_frame
    ps.header.stamp = rospy.Time(0)
    ps.point = pt
    try:
        tfm = buf.lookup_transform(
            target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout)
        )
        out = do_transform_point(ps, tfm)
        return out.point
    except Exception as e:
        rospy.logwarn_throttle(3.0, "TF %s->%s: %s", source_frame, target_frame, e)
        return None


def transform_vector(buf, vec, target_frame, source_frame, timeout=2.0):
    """仅旋转到目标坐标系（平移忽略），用于像素误差推水平修正量。"""
    if do_transform_vector3 is None:
        return None
    vs = Vector3Stamped()
    vs.header.frame_id = source_frame
    vs.header.stamp = rospy.Time(0)
    vs.vector = vec
    try:
        tfm = buf.lookup_transform(
            target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout)
        )
        out = do_transform_vector3(vs, tfm)
        return out.vector
    except Exception as e:
        rospy.logwarn_throttle(5.0, "TF vector %s->%s: %s", source_frame, target_frame, e)
        return None


class Mission(object):
    HOVER = "hover"
    ARM_HOME_1 = "arm_home_1"
    ARM_HOME_SETTLE = "arm_home_settle"  # home 完成后原地悬停，再前飞
    FWD = "forward"
    DESCEND = "descend"  # 前飞点上方下降至更接近台面的悬停高度，再感知/抓取
    DETECT = "detect"
    GRASP = "grasp"
    ARM_HOME_2 = "arm_home_2"
    CLIMB = "climb"
    RTL = "rtl"
    LAND = "land"
    DONE = "done"

    def __init__(self):
        rospy.init_node("offboard_grasp_blue_cube")

        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.warmup_iters = int(rospy.get_param("~warmup_iters", 40))
        self.hover_rel_m = float(rospy.get_param("~hover_rel_m", 1.0))
        # 抓取前相对起飞点降低悬停（需 < hover_rel_m），像素更大、深度更准、臂易够到台面
        self.approach_hover_rel_m = float(
            rospy.get_param("~approach_hover_rel_m", 0.4)
        )
        self.min_approach_hover_rel_m = float(
            rospy.get_param("~min_approach_hover_rel_m", 0.18)
        )
        self.forward_m = float(rospy.get_param("~forward_m", 1.6))
        self.wp_tol_xy = float(rospy.get_param("~wp_tol_xy", 0.15))
        self.wp_tol_z = float(rospy.get_param("~wp_tol_z", 0.12))
        self.hover_settle_sec = float(rospy.get_param("~hover_settle_sec", 2.0))
        # 首段机械臂回 home 后，在起点巡航高度悬停多久再前飞（秒）
        self.arm_home_settle_sec = float(
            rospy.get_param("~arm_home_settle_sec", 2.0)
        )
        self.auto_arm = bool(rospy.get_param("~auto_arm", True))
        self.auto_offboard = bool(rospy.get_param("~auto_offboard", True))
        self.try_pose_from_vision = bool(rospy.get_param("~try_pose_from_vision", True))
        self.async_spinner_threads = int(rospy.get_param("~async_spinner_threads", 4))
        # 首段回 home 是否先调夹爪（易与 go() 一起卡死，默认只动 arm 组）
        self.arm_home_open_gripper = bool(
            rospy.get_param("~arm_home_open_gripper", False)
        )
        self.gripper_goal_tolerance = float(
            rospy.get_param("~gripper_goal_tolerance", 0.2)
        )

        # link2_to_link3 URDF 上限为 2.95，顶格易触发 effort 控制器 CONTROL_FAILED，默认略留余量
        self.arm_home_joints = rospy.get_param(
            "~arm_home_joints", [0.0, -1.57, 2.9, 0.0, 0.0]
        )
        self.arm_goal_joint_tolerance = float(
            rospy.get_param("~arm_goal_joint_tolerance", 0.12)
        )
        self.arm_vel_scaling = float(rospy.get_param("~arm_vel_scaling", 0.12))
        self.arm_accel_scaling = float(rospy.get_param("~arm_accel_scaling", 0.12))
        self.arm_home_vel_scaling = float(
            rospy.get_param("~arm_home_vel_scaling", 0.08)
        )
        self.arm_home_accel_scaling = float(
            rospy.get_param("~arm_home_accel_scaling", 0.08)
        )
        self.arm_planning_time = float(rospy.get_param("~arm_planning_time", 25.0))
        self.move_group_wait_for_servers = float(
            rospy.get_param("~move_group_wait_for_servers", 45.0)
        )
        self.move_group_reinit_sleep = float(
            rospy.get_param("~move_group_reinit_sleep", 1.0)
        )
        self.gripper_open = float(rospy.get_param("~gripper_open", 0.0))
        self.gripper_close = float(rospy.get_param("~gripper_close", 1.0))
        # 更向前、向下伸手，便于够到前飞点下方台面（可按仿真再调）
        self.pregrasp_joints = rospy.get_param(
            "~pregrasp_joints", [0.0, -0.95, 2.35, -0.4, 0.0]
        )
        self.grasp_joints = rospy.get_param(
            "~grasp_joints", [0.0, -1.05, 2.55, -0.5, 0.0]
        )

        # Gazebo librealsense_gazebo_plugin 常见为 /camera_realsense/...（与 ROS realsense 驱动的 /camera/ 不同）
        self.color_topic = rospy.get_param(
            "~color_topic", "/camera_realsense/color/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera_realsense/depth/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera_realsense/color/camera_info"
        )
        self.use_depth = bool(rospy.get_param("~use_depth", True))
        self.assume_depth_m = float(rospy.get_param("~assume_depth_m", 1.15))
        # 在 planning_frame 下对估计的质心位置做微调（米）
        self.vision_target_offset = rospy.get_param(
            "~vision_target_offset", [0.0, 0.0, 0.0]
        )
        # 预接近：在质心上再抬升一段，避免直线撞台面（沿 planning_frame 加）
        # 单段位移过大易 TIMED_OUT；略减小 z 抬升，配合分段插值
        self.vision_pregrasp_delta = rospy.get_param(
            "~vision_pregrasp_delta", [0.0, 0.0, 0.08]
        )
        self.vision_grasp_delta = rospy.get_param(
            "~vision_grasp_delta", [0.0, 0.0, -0.06]
        )
        self.vision_cartesian_vel_scale = float(
            rospy.get_param("~vision_cartesian_vel_scale", 0.4)
        )
        # 固定最小分段数；另按空间步长 vision_cartesian_step_m 自动加段，避免大位移下单段仍超时
        self.vision_cartesian_interp_steps = int(
            rospy.get_param("~vision_cartesian_interp_steps", 12)
        )
        self.vision_cartesian_interp_steps_fin = int(
            rospy.get_param("~vision_cartesian_interp_steps_fin", 8)
        )
        self.vision_cartesian_step_m = float(
            rospy.get_param("~vision_cartesian_step_m", 0.03)
        )
        # 若为 false，跳过笛卡尔直接用关节轨迹（避免仿真里反复 TIMED_OUT）
        self.vision_use_cartesian = bool(
            rospy.get_param("~vision_use_cartesian", True)
        )
        # 仅用 set_position_target 时 IK 常选「大扭腰」解，短位移仍关节行程巨大 → TIMED_OUT；
        # 用当前末端姿态 + 目标位置做 set_pose_target，关节增量小、易在时限内完成。
        self.vision_cartesian_use_pose = bool(
            rospy.get_param("~vision_cartesian_use_pose", True)
        )
        # 分段笛卡尔全失败后：直接以姿态约束走 p_pre、p_fin（仍用视觉三维点）
        self.vision_pose_fallback = bool(
            rospy.get_param("~vision_pose_fallback", True)
        )
        self.vision_goal_position_tolerance = float(
            rospy.get_param("~vision_goal_position_tolerance", 0.065)
        )
        self.vision_goal_orientation_tolerance = float(
            rospy.get_param("~vision_goal_orientation_tolerance", 0.45)
        )
        # MoveIt 执行阶段关节目标容差（与 kinematics.yaml 独立；略松便于 effort 仿真判到达）
        self.vision_relaxed_joint_tolerance = float(
            rospy.get_param("~vision_relaxed_joint_tolerance", 0.22)
        )
        # False：跳过多段笛卡尔（每段一次 go 易反复 TIMED_OUT），直接慢速两步 Pose
        self.vision_segmented_cartesian_first = bool(
            rospy.get_param("~vision_segmented_cartesian_first", False)
        )
        self.vision_pose_step_vel_scale = float(
            rospy.get_param("~vision_pose_step_vel_scale", 0.11)
        )

        # Gazebo 渲染的「蓝」偏青/灰时，过窄 HSV 会得到空掩膜；可按 rqt 取样再收紧
        self.hsv_lower = np.array(rospy.get_param("~hsv_blue_lower", [75, 20, 30]))
        self.hsv_upper = np.array(rospy.get_param("~hsv_blue_upper", [135, 255, 255]))
        # 远距/小块时面积像素少，默认略降；仍可通过 launch 覆盖
        self.min_blob_area = int(rospy.get_param("~min_blob_area", 42))
        self.blue_mask_erode_iters = int(rospy.get_param("~blue_mask_erode_iters", 1))
        self.blue_mask_dilate_iters = int(rospy.get_param("~blue_mask_dilate_iters", 2))
        self.detection_stable = int(rospy.get_param("~detection_stable_frames", 6))
        # 检测阶段：用深度估计尺度，把前飞点 (x_fwd,y_fwd) 微调到光轴对准蓝块（水平对准后再锁稳定帧）
        self.detect_refine_xy_enable = bool(rospy.get_param("~detect_refine_xy_enable", True))
        self.detect_refine_xy_gain = float(rospy.get_param("~detect_refine_xy_gain", 0.22))
        self.detect_refine_xy_max_step = float(rospy.get_param("~detect_refine_xy_max_step", 0.03))
        # 长时间分不出蓝块则再降低悬停高度，增大目标在图像中的面积
        self.detect_refine_descend_enable = bool(
            rospy.get_param("~detect_refine_descend_enable", True)
        )
        self.detect_refine_descend_step = float(
            rospy.get_param("~detect_refine_descend_step", 0.05)
        )
        self.detect_refine_descend_after_sec = float(
            rospy.get_param("~detect_refine_descend_after_sec", 2.2)
        )

        # 与主循环解耦，避免 MoveIt 阻塞时停发 Offboard；设为 false 则仅在主循环里发布（易 Failsafe）
        self.offboard_stream_enable = bool(rospy.get_param("~offboard_stream_enable", True))
        self.offboard_stream_hz = float(rospy.get_param("~offboard_stream_hz", 20.0))

        self.ref_frame = rospy.get_param("~planning_frame", "roarm_base_link")
        self.camera_optical_frame = rospy.get_param(
            "~camera_optical_frame", "camera_color_optical_frame"
        )
        # MAVROS pose 常为 map，若未发布 map↔odom 则与相机树断开；水平微调应用与 MoveIt 相同的机械臂系
        self.tf_refine_target_frame = rospy.get_param("~tf_refine_target_frame", "").strip()

        self.state = State()
        self.pose = PoseStamped()
        self.phase = self.HOVER
        self.bridge = CvBridge()
        self._img = None
        self._depth = None
        self._K = None
        self._stable_cnt = 0
        self._last_uv = None
        self._no_blob_since = None
        self.t_phase0 = 0.0

        self.x0 = self.y0 = self.z0 = 0.0
        self.yaw0 = 0.0
        self.z_h = 0.0
        self.z_grasp = 0.0
        self.x_fwd = self.y_fwd = 0.0

        self._moveit_cpp_inited = False
        self._arm_mgc = None
        self._gripper_mgc = None

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(120.0))
        self.tf_ls = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/mavros/state", State, self._cb_state)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._cb_pose)
        rospy.Subscriber(self.color_topic, Image, self._cb_img, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._cb_depth, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._cb_info, queue_size=1)

        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        rospy.loginfo(
            "订阅相机: color=%s depth=%s camera_info=%s",
            self.color_topic,
            self.depth_topic,
            self.camera_info_topic,
        )

    def _cb_state(self, msg):
        self.state = msg

    def _cb_pose(self, msg):
        self.pose = msg

    def _cb_info(self, msg):
        self._K = np.array(msg.K).reshape(3, 3)

    def _cb_img(self, msg):
        try:
            self._img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, str(e))

    def _cb_depth(self, msg):
        try:
            if msg.encoding == "32FC1":
                self._depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            elif msg.encoding == "16UC1":
                self._depth = (
                    self.bridge.imgmsg_to_cv2(msg, "16UC1").astype(np.float32) / 1000.0
                )
            else:
                self._depth = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, str(e))

    @staticmethod
    def yaw_from_pose(ps):
        q = ps.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def sp(self, x, y, z, yaw):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_YAW_RATE
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
        )
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.yaw = yaw
        return sp

    @staticmethod
    def dist3(ax, ay, az, bx, by, bz):
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)

    def get_offboard_sp_tuple(self):
        """当前任务阶段对应的 (x,y,z,yaw)，供 Offboard 独立发布线程使用。RTL 用 yaw0，其余用实时 yaw。"""
        yaw_live = self.yaw_from_pose(self.pose)
        p = self.phase
        if p == self.DONE:
            return None
        if p in (self.HOVER, self.ARM_HOME_1, self.ARM_HOME_SETTLE):
            return self.x0, self.y0, self.z_h, yaw_live
        if p == self.FWD:
            return self.x_fwd, self.y_fwd, self.z_h, yaw_live
        if p in (self.DESCEND, self.DETECT, self.GRASP, self.ARM_HOME_2):
            return self.x_fwd, self.y_fwd, self.z_grasp, yaw_live
        if p == self.CLIMB:
            return self.x_fwd, self.y_fwd, self.z_h, yaw_live
        if p in (self.RTL, self.LAND):
            return self.x0, self.y0, self.z_h, self.yaw0
        return self.x0, self.y0, self.z_h, yaw_live

    def wait_move_group(self, timeout=120.0):
        """等待 move_group 节点就绪。旧逻辑依赖 get_planning_scene，部分 MoveIt 配置无此服务名。"""
        svc = rospy.get_param("~move_group_ready_service", "/move_group/get_loggers")
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            try:
                rospy.wait_for_service(svc, timeout=1.0)
                rospy.loginfo("move_group 已就绪（检测到 %s）", svc)
                return True
            except rospy.ROSException:
                rospy.loginfo_throttle(
                    5.0,
                    "仍在等待 move_group（%s），请确认已启动 move_group / demo.launch",
                    svc,
                )
            except Exception as e:
                rospy.logwarn_throttle(10.0, "wait move_group: %s", e)
        return False

    def configure_arm_move_group(self, arm, vel_scaling=None, accel_scaling=None):
        """放大关节容差、放慢速度，减轻 Gazebo effort 轨迹执行 ABORTED: CONTROL_FAILED。"""
        vs = self.arm_vel_scaling if vel_scaling is None else vel_scaling
        ac = self.arm_accel_scaling if accel_scaling is None else accel_scaling
        arm.set_planning_time(self.arm_planning_time)
        arm.set_num_planning_attempts(10)
        arm.set_max_velocity_scaling_factor(vs)
        arm.set_max_acceleration_scaling_factor(ac)
        arm.set_goal_joint_tolerance(self.arm_goal_joint_tolerance)

    def ensure_moveit_cpp(self):
        """仅初始化一次；与反复 roscpp_shutdown 不兼容会导致后续无法连接 move_group。"""
        if not self._moveit_cpp_inited:
            moveit_commander.roscpp_initialize(sys.argv)
            self._moveit_cpp_inited = True
            rospy.loginfo("moveit_commander: roscpp_initialize（单例，任务结束不再 shutdown）")

    def _create_move_group(self, group_name):
        last_err = None
        for attempt in range(2):
            try:
                return MoveGroupCommander(
                    group_name, wait_for_servers=self.move_group_wait_for_servers
                )
            except RuntimeError as e:
                last_err = e
                rospy.logwarn(
                    "MoveGroupCommander(%s) 连接失败 (%s)，2s 后重试 …",
                    group_name,
                    e,
                )
                rospy.sleep(2.0)
        raise last_err

    def get_arm_group(self):
        """全程复用同一 arm MoveGroupCommander。"""
        if self._arm_mgc is not None:
            return self._arm_mgc
        if not self.wait_move_group(60.0):
            return None
        self.ensure_moveit_cpp()
        if self.move_group_reinit_sleep > 0.0:
            rospy.sleep(self.move_group_reinit_sleep)
        try:
            self._arm_mgc = self._create_move_group("arm")
        except RuntimeError as e:
            rospy.logerr("无法创建 arm MoveGroupCommander: %s", e)
            return None
        return self._arm_mgc

    def get_gripper_group(self):
        """全程复用同一 gripper MoveGroupCommander。"""
        if self._gripper_mgc is not None:
            return self._gripper_mgc
        if self._arm_mgc is None:
            if not self.wait_move_group(60.0):
                return None
        self.ensure_moveit_cpp()
        if self.move_group_reinit_sleep > 0.0:
            rospy.sleep(self.move_group_reinit_sleep)
        try:
            self._gripper_mgc = self._create_move_group("gripper")
            self._gripper_mgc.set_max_velocity_scaling_factor(0.4)
            self._gripper_mgc.set_goal_joint_tolerance(self.gripper_goal_tolerance)
        except RuntimeError as e:
            rospy.logerr("无法创建 gripper MoveGroupCommander: %s", e)
            return None
        return self._gripper_mgc

    def _blue_mask_processed(self):
        if self._img is None:
            return None
        hsv = cv2.cvtColor(self._img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        if self.blue_mask_erode_iters > 0:
            mask = cv2.erode(mask, None, iterations=self.blue_mask_erode_iters)
        if self.blue_mask_dilate_iters > 0:
            mask = cv2.dilate(mask, None, iterations=self.blue_mask_dilate_iters)
        return mask

    def log_blue_detection_diagnostics(self):
        """人眼可见但 inRange 失败时：掩膜像素≈0 为 HSV；有像素但面积小为 min_blob_area。"""
        mask = self._blue_mask_processed()
        if mask is None:
            return
        nz = int(cv2.countNonZero(mask))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_a = 0.0
        for c in cnts:
            max_a = max(max_a, float(cv2.contourArea(c)))
        rospy.loginfo_throttle(
            5.0,
            "蓝色分割诊断: 掩膜像素=%d, 最大连通域面积=%.0f (需>min_blob_area=%d)；"
            "掩膜≈0 请放宽 hsv_blue_lower/upper；有像素但面积不足请降 min_blob_area",
            nz,
            max_a,
            self.min_blob_area,
        )

    def detect_blue_uv(self):
        mask = self._blue_mask_processed()
        if mask is None:
            return None
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_a = None, 0
        for c in cnts:
            a = cv2.contourArea(c)
            if a > self.min_blob_area and a > best_a:
                best_a, best = a, c
        if best is None:
            return None
        m = cv2.moments(best)
        if m["m00"] < 1e-6:
            return None
        return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])

    def _uv_in_depth(self, u, v):
        """彩色图与深度图分辨率不一致时，将像素坐标映射到 depth 图像素。"""
        if self._img is None or self._depth is None:
            return u, v
        hi, wi = self._img.shape[:2]
        hd, wd = self._depth.shape[:2]
        if hi == hd and wi == wd:
            return u, v
        u2 = int(round(u * wd / float(wi)))
        v2 = int(round(v * hd / float(hi)))
        return max(0, min(wd - 1, u2)), max(0, min(hd - 1, v2))

    def depth_median(self, u, v, r=6):
        if self._depth is None:
            return None
        u, v = self._uv_in_depth(u, v)
        h, w = self._depth.shape[:2]
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))
        patch = self._depth[v - r : v + r + 1, u - r : u + r + 1]
        vals = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 15.0)]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def point_cam(self, u, v):
        z = self.assume_depth_m
        if self.use_depth:
            zd = self.depth_median(u, v)
            if zd is not None:
                z = zd
        if self._K is None:
            return None
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        p = Point()
        p.x = (u - cx) * z / fx
        p.y = (v - cy) * z / fy
        p.z = z
        return p

    def refine_xy_toward_blue(self, u, v):
        """根据质心在彩图相对光轴的偏移，微调 LOCAL 水平面内的前飞点，使机体对准蓝块再读深度/TF。"""
        if not self.detect_refine_xy_enable or self._K is None:
            return
        if do_transform_vector3 is None:
            return
        zd = self.depth_median(u, v)
        if zd is None or zd < 0.08 or zd > 14.0:
            zd = float(self.assume_depth_m)
        fx, fy = float(self._K[0, 0]), float(self._K[1, 1])
        ppx, ppy = float(self._K[0, 2]), float(self._K[1, 2])
        g = self.detect_refine_xy_gain
        vx = g * (float(u) - ppx) / fx * zd
        vy = g * (float(v) - ppy) / fy * zd
        vz = 0.0
        # 勿用 MAVROS pose.header.frame_id（常为 map）：与 Gazebo/robot_state_publisher 树可能断连
        tgt = self.tf_refine_target_frame or self.ref_frame
        out = transform_vector(
            self.tf_buffer, Vector3(vx, vy, vz), tgt, self.camera_optical_frame
        )
        if out is None:
            return
        dx, dy = float(out.x), float(out.y)
        step = math.hypot(dx, dy)
        mxs = self.detect_refine_xy_max_step
        if step > mxs and step > 1e-9:
            s = mxs / step
            dx *= s
            dy *= s
        self.x_fwd += dx
        self.y_fwd += dy
        rospy.loginfo_throttle(
            4.0,
            "检测微调前飞点: Δxy=(%.3f,%.3f) m → 前飞点 (%.2f,%.2f)",
            dx,
            dy,
            self.x_fwd,
            self.y_fwd,
        )

    @staticmethod
    def _interp_xyz(p0, p1, n_seg):
        if n_seg < 1:
            n_seg = 1
        wps = []
        for i in range(1, n_seg + 1):
            t = i / float(n_seg)
            wps.append(
                [
                    p0[0] + (p1[0] - p0[0]) * t,
                    p0[1] + (p1[1] - p0[1]) * t,
                    p0[2] + (p1[2] - p0[2]) * t,
                ]
            )
        return wps

    def _adapt_interp_steps(self, p0, p1, base_n):
        """按欧氏距离细分：每段不超过 vision_cartesian_step_m，与 base_n 取大（上限 48）。"""
        a = np.array(p0, dtype=np.float64)
        b = np.array(p1, dtype=np.float64)
        dist = float(np.linalg.norm(b - a))
        if self.vision_cartesian_step_m > 1e-6:
            auto = int(math.ceil(dist / self.vision_cartesian_step_m))
        else:
            auto = base_n
        n = max(int(base_n), auto)
        n = min(n, 48)
        return max(1, n)

    def _apply_vision_execution_tolerances(self, arm):
        """MoveIt 判定「到达目标」用：与 kinematics 求解容差不同，影响 go()/execute 结束条件。"""
        try:
            arm.set_goal_joint_tolerance(self.vision_relaxed_joint_tolerance)
            arm.set_goal_position_tolerance(self.vision_goal_position_tolerance)
            arm.set_goal_orientation_tolerance(self.vision_goal_orientation_tolerance)
        except Exception as e:
            rospy.logwarn("设置视觉执行容差失败: %s", e)

    def _reset_vision_execution_tolerances(self, arm):
        try:
            arm.set_goal_joint_tolerance(self.arm_goal_joint_tolerance)
            arm.set_goal_position_tolerance(0.001)
            arm.set_goal_orientation_tolerance(0.01)
        except Exception:
            pass

    def _current_ee_pose_at_xyz(self, arm, pt):
        """保持当前末端姿态，仅平移到 pt（planning / pose reference frame）。"""
        cp = arm.get_current_pose()
        pose = copy.deepcopy(cp.pose)
        pose.position.x = float(pt[0])
        pose.position.y = float(pt[1])
        pose.position.z = float(pt[2])
        return pose

    def _go_vision_waypoint(self, arm, pt, label, base_vs):
        """视觉路径上的一步：默认 pose 目标；末次尝试极低速，拉长规划名义时长以利执行时限。"""
        n_att = 3
        for att in range(n_att):
            if att < 2:
                vs = min(0.65, base_vs * (1.45 ** att))
            else:
                vs = max(0.06, min(0.22, base_vs * 0.32))
            arm.set_max_velocity_scaling_factor(vs)
            arm.set_max_acceleration_scaling_factor(vs)
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                if self.vision_cartesian_use_pose:
                    arm.set_pose_target(self._current_ee_pose_at_xyz(arm, pt))
                else:
                    arm.set_position_target(pt)
                if arm.go(wait=True):
                    return True
            except MoveItCommanderException as e:
                rospy.logwarn("%s attempt %d: %s", label, att + 1, e)
            rospy.logwarn(
                "%s 笛卡尔 attempt %d 失败（含超时则加速/末次减速重试）",
                label,
                att + 1,
            )
        return False

    def _vision_pose_two_step_fallback(self, arm, p_pre, p_fin, base_vs):
        """分段笛卡尔全失败后：放宽末端与关节容差，直接两步 Pose（仍用质心三维点）。"""
        rospy.loginfo("视觉：分段笛卡尔失败，尝试姿态约束两步接近（仍用质心三维点）…")
        self._apply_vision_execution_tolerances(arm)
        ok = True
        try:
            vs0 = max(float(base_vs), self.vision_pose_step_vel_scale, 0.18)
            if not self._go_vision_waypoint(arm, p_pre, "Pose预接近", vs0):
                ok = False
            elif not self._go_vision_waypoint(arm, p_fin, "Pose下落", vs0):
                ok = False
        finally:
            self._reset_vision_execution_tolerances(arm)
        return ok

    def _joint_pregrasp_grasp(self, arm, gripper):
        """无视觉或视觉失败时的备用关节轨迹。"""
        arm.clear_pose_targets()
        self.configure_arm_move_group(arm)
        arm.set_joint_value_target([float(x) for x in self.pregrasp_joints])
        if not arm.go(wait=True):
            rospy.logerr("pregrasp（关节）失败")
            return False
        arm.set_joint_value_target([float(x) for x in self.grasp_joints])
        if not arm.go(wait=True):
            rospy.logerr("grasp（关节）失败")
            return False
        return True

    def exec_arm_sequence(self, vision_xyz=None):
        """
        vision_xyz: 蓝色质心在 planning_frame（默认 roarm_base_link）下的估计位置；
        默认先放宽执行容差并两步 Pose 接近（可改 vision_segmented_cartesian_first 恢复高分段）。
        """
        arm = self.get_arm_group()
        gripper = self.get_gripper_group()
        if arm is None or gripper is None:
            rospy.logerr("move_group 不可用")
            return False
        self.configure_arm_move_group(arm)
        gripper.set_max_velocity_scaling_factor(0.4)
        gripper.set_goal_joint_tolerance(self.gripper_goal_tolerance)
        arm.set_pose_reference_frame(self.ref_frame)

        gripper.set_joint_value_target([self.gripper_open])
        rospy.loginfo("MoveIt: 夹爪打开 …")
        gripper.go(wait=True)

        use_vision = (
            self.try_pose_from_vision
            and vision_xyz is not None
            and len(vision_xyz) == 3
        )
        if use_vision and self.vision_use_cartesian:
            base = np.array(vision_xyz, dtype=np.float64)
            off = np.array(self.vision_target_offset, dtype=np.float64)
            d0 = np.array(self.vision_pregrasp_delta, dtype=np.float64)
            d1 = np.array(self.vision_grasp_delta, dtype=np.float64)
            p_pre = (base + off + d0).tolist()
            p_fin = (base + off + d0 + d1).tolist()
            rospy.loginfo(
                "视觉末端对准 %s: 质心 (%.3f,%.3f,%.3f) → 预接近 (%.3f,%.3f,%.3f) → 下落 (%.3f,%.3f,%.3f)",
                self.ref_frame,
                base[0],
                base[1],
                base[2],
                p_pre[0],
                p_pre[1],
                p_pre[2],
                p_fin[0],
                p_fin[1],
                p_fin[2],
            )
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                self._apply_vision_execution_tolerances(arm)
                base_vs = self.vision_cartesian_vel_scale

                def _go_chain(p0, p1, chain_label, n_seg):
                    wps = self._interp_xyz(p0, p1, max(1, n_seg))
                    for i, pt in enumerate(wps):
                        lab = "%s %d/%d" % (chain_label, i + 1, len(wps))
                        if not self._go_vision_waypoint(arm, pt, lab, base_vs):
                            return False
                    return True

                if not self.vision_segmented_cartesian_first:
                    vs0 = max(self.vision_pose_step_vel_scale, base_vs * 0.28)
                    rospy.loginfo(
                        "视觉：跳过高分段笛卡尔，直接两步 Pose（vel_scale≈%.2f，执行容差已放宽）",
                        vs0,
                    )
                    ok_vis = self._go_vision_waypoint(
                        arm, p_pre, "Pose预接近", vs0
                    ) and self._go_vision_waypoint(arm, p_fin, "Pose下落", vs0)
                    if not ok_vis:
                        rospy.logwarn("视觉两步 Pose 失败，改用关节轨迹")
                        if not self._joint_pregrasp_grasp(arm, gripper):
                            return False
                else:
                    try:
                        cp = arm.get_current_pose()
                        p_cur = [
                            cp.pose.position.x,
                            cp.pose.position.y,
                            cp.pose.position.z,
                        ]
                    except Exception as e:
                        rospy.logwarn(
                            "get_current_pose 失败 (%s)，笛卡尔起点用质心近似", e
                        )
                        p_cur = (base + off).tolist()

                    n_pre = self._adapt_interp_steps(
                        p_cur, p_pre, self.vision_cartesian_interp_steps
                    )
                    n_fin = self._adapt_interp_steps(
                        p_pre, p_fin, self.vision_cartesian_interp_steps_fin
                    )
                    rospy.loginfo(
                        "笛卡尔分段: 预接近 %d 段 (步长≤%.3f m)，下落 %d 段",
                        n_pre,
                        self.vision_cartesian_step_m,
                        n_fin,
                    )

                    if not _go_chain(p_cur, p_pre, "预接近", n_pre):
                        rospy.logwarn("视觉预接近（分段）失败")
                        ok_fb = (
                            self._vision_pose_two_step_fallback(
                                arm, p_pre, p_fin, base_vs
                            )
                            if self.vision_pose_fallback
                            else False
                        )
                        if not ok_fb and not self._joint_pregrasp_grasp(
                            arm, gripper
                        ):
                            return False
                    elif not _go_chain(p_pre, p_fin, "下落", n_fin):
                        rospy.logwarn("视觉下落（分段）失败")
                        ok_fb = (
                            self._vision_pose_two_step_fallback(
                                arm, p_pre, p_fin, base_vs
                            )
                            if self.vision_pose_fallback
                            else False
                        )
                        if not ok_fb and not self._joint_pregrasp_grasp(
                            arm, gripper
                        ):
                            return False
            except MoveItCommanderException as e:
                rospy.logwarn("笛卡尔目标异常 %s，改用关节轨迹", e)
                if not self._joint_pregrasp_grasp(arm, gripper):
                    return False
            finally:
                self._reset_vision_execution_tolerances(arm)
        else:
            if self.try_pose_from_vision:
                if vision_xyz is None or len(vision_xyz) != 3:
                    rospy.logwarn("无有效视觉三维点，改用关节 pregrasp/grasp")
                elif not self.vision_use_cartesian:
                    rospy.loginfo("已关闭 vision_use_cartesian，直接用关节轨迹抓取")
            if not self._joint_pregrasp_grasp(arm, gripper):
                return False

        gripper.set_joint_value_target([self.gripper_close])
        rospy.loginfo("MoveIt: 夹爪闭合 …")
        gripper.go(wait=True)
        return True

    def exec_arm_home_only(self):
        arm = self.get_arm_group()
        if arm is None:
            return False
        self.configure_arm_move_group(arm)
        arm.set_joint_value_target([float(x) for x in self.arm_home_joints])
        return arm.go(wait=True)


def main():
    m = Mission()
    start_rospy_callbacks_for_moveit(m.async_spinner_threads)
    rate = rospy.Rate(m.rate_hz)

    while not rospy.is_shutdown() and not m.state.connected:
        rospy.loginfo_throttle(2.0, "等待 MAVROS …")
        rate.sleep()

    yaw = m.yaw_from_pose(m.pose)
    for _ in range(m.warmup_iters):
        p = m.pose.pose.position
        m.sp_pub.publish(m.sp(p.x, p.y, p.z, yaw))
        rate.sleep()

    m.x0 = m.pose.pose.position.x
    m.y0 = m.pose.pose.position.y
    m.z0 = m.pose.pose.position.z
    m.yaw0 = m.yaw_from_pose(m.pose)
    c, s = math.cos(m.yaw0), math.sin(m.yaw0)
    m.z_h = m.z0 + m.hover_rel_m
    m.z_grasp = m.z0 + m.approach_hover_rel_m
    m.z_grasp = max(m.z_grasp, m.z0 + m.min_approach_hover_rel_m)
    m.z_grasp = min(m.z_grasp, m.z_h - 0.08)
    m.x_fwd = m.x0 + m.forward_m * c
    m.y_fwd = m.y0 + m.forward_m * s

    rospy.loginfo(
        "起点 (%.2f,%.2f,%.2f) → 巡航 z=%.2f m；前飞点 (%.2f,%.2f)；抓取悬停 z=%.2f m",
        m.x0,
        m.y0,
        m.z0,
        m.z_h,
        m.x_fwd,
        m.y_fwd,
        m.z_grasp,
    )

    if m.auto_offboard:
        try:
            m.set_mode(0, "OFFBOARD")
        except rospy.ServiceException as e:
            rospy.logwarn("%s", e)
    if m.auto_arm:
        try:
            m.arming(True)
        except rospy.ServiceException as e:
            rospy.logwarn("%s", e)

    if m.offboard_stream_enable:
        start_offboard_setpoint_stream(m, m.offboard_stream_hz)

    m.phase = m.HOVER
    m.t_phase0 = time.time()

    try:
        while not rospy.is_shutdown() and m.phase != m.DONE:
            px = m.pose.pose.position.x
            py = m.pose.pose.position.y
            pz = m.pose.pose.position.z
            yaw = m.yaw_from_pose(m.pose)
    
            if m.phase == m.HOVER:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_h, yaw)
                if m.dist3(px, py, pz, m.x0, m.y0, m.z_h) < max(m.wp_tol_xy, m.wp_tol_z):
                    if time.time() - m.t_phase0 > m.hover_settle_sec:
                        rospy.loginfo("进入机械臂 home")
                        m.phase = m.ARM_HOME_1
                        m.t_phase0 = time.time()
                rate.sleep()
                continue
    
            if m.phase == m.ARM_HOME_1:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_h, yaw)
                rospy.loginfo("MoveIt: 连接 move_group 并回 home …")
                arm = m.get_arm_group()
                if arm is None:
                    rospy.logerr("move_group 不可用")
                    m.phase = m.DONE
                    continue
                m.configure_arm_move_group(
                    arm,
                    vel_scaling=m.arm_home_vel_scaling,
                    accel_scaling=m.arm_home_accel_scaling,
                )
                if m.arm_home_open_gripper:
                    grip = m.get_gripper_group()
                    if grip is None:
                        rospy.logerr("gripper MoveGroup 不可用")
                        m.phase = m.DONE
                        continue
                    grip.set_joint_value_target([m.gripper_open])
                    rospy.loginfo("MoveIt: 先发夹爪打开（首段 home）…")
                    grip.go(wait=True)
                rospy.loginfo("MoveIt: 机械臂关节 home …")
                arm.set_joint_value_target([float(x) for x in m.arm_home_joints])
                if not arm.go(wait=True):
                    rospy.logerr("home 失败")
                rospy.loginfo(
                    "机械臂已回 home，悬停 %.1f s 后前飞 …",
                    m.arm_home_settle_sec,
                )
                m.phase = m.ARM_HOME_SETTLE
                m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.ARM_HOME_SETTLE:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_h, yaw)
                if time.time() - m.t_phase0 >= m.arm_home_settle_sec:
                    rospy.loginfo("home 后悬停结束，开始前飞")
                    m.phase = m.FWD
                    m.t_phase0 = time.time()
                rate.sleep()
                continue
    
            if m.phase == m.FWD:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_h, yaw)
                if m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_h) < m.wp_tol_xy * 1.2:
                    rospy.loginfo("到达前飞点，下降至抓取悬停高度 …")
                    m.phase = m.DESCEND
                    m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.DESCEND:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_grasp, yaw)
                if m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_grasp) < max(
                    m.wp_tol_xy, m.wp_tol_z
                ):
                    if time.time() - m.t_phase0 > m.hover_settle_sec:
                        rospy.loginfo("已降落至抓取高度，开始检测蓝色")
                        m.phase = m.DETECT
                        m._stable_cnt = 0
                        m._no_blob_since = None
                else:
                    m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.DETECT:
                m.sp_pub.publish(m.sp(m.x_fwd, m.y_fwd, m.z_grasp, yaw))
                if m._img is None:
                    rospy.logwarn_throttle(
                        3.0,
                        "未收到彩色图，请检查 ~color_topic（Gazebo 多为 /camera_realsense/color/image_raw）",
                    )
                uv = m.detect_blue_uv()
                if uv is not None:
                    m._no_blob_since = None
                    # 未达稳定帧前持续水平微调，使光轴对准蓝块（深度/TF 与 MoveIt 目标一致）
                    if m._stable_cnt < m.detection_stable:
                        m.refine_xy_toward_blue(uv[0], uv[1])
                    if m._last_uv and abs(uv[0] - m._last_uv[0]) < 10 and abs(uv[1] - m._last_uv[1]) < 10:
                        m._stable_cnt += 1
                    else:
                        m._stable_cnt = 1
                    m._last_uv = uv
                else:
                    m._stable_cnt = 0
                    if m._img is not None:
                        rospy.loginfo_throttle(
                            5.0,
                            "有图像但未分割出足够大的蓝色区域（可调 hsv_blue_lower/upper、min_blob_area）",
                        )
                        m.log_blue_detection_diagnostics()
                        if m.detect_refine_descend_enable:
                            now = time.time()
                            if m._no_blob_since is None:
                                m._no_blob_since = now
                            elif now - m._no_blob_since >= m.detect_refine_descend_after_sec:
                                z_floor = m.z0 + m.min_approach_hover_rel_m - 0.03
                                if m.z_grasp > z_floor + 0.02:
                                    m.z_grasp = max(
                                        z_floor,
                                        m.z_grasp - m.detect_refine_descend_step,
                                    )
                                    rospy.loginfo(
                                        "无蓝块分割: 下降抓取悬停高度 z_grasp=%.2f m（下限≈%.2f）",
                                        m.z_grasp,
                                        z_floor,
                                    )
                                m._no_blob_since = now
                if m._stable_cnt >= m.detection_stable:
                    m.phase = m.GRASP
                rate.sleep()
                continue
    
            if m.phase == m.GRASP:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_grasp, yaw)
                if m._last_uv is None:
                    rospy.logwarn("无有效 uv，回到检测")
                    m.phase = m.DETECT
                    m._stable_cnt = 0
                    rate.sleep()
                    continue
                u, v = m._last_uv
                vision_xyz = None
                if m.try_pose_from_vision:
                    pc = m.point_cam(u, v)
                    if pc is not None:
                        pb = transform_point(
                            m.tf_buffer, pc, m.ref_frame, m.camera_optical_frame
                        )
                        if pb is not None:
                            vision_xyz = (pb.x, pb.y, pb.z)
                            rospy.loginfo(
                                "视觉质心 %s: (%.3f,%.3f,%.3f)",
                                m.ref_frame,
                                pb.x,
                                pb.y,
                                pb.z,
                            )
                ok = m.exec_arm_sequence(vision_xyz=vision_xyz)
                m.phase = m.ARM_HOME_2 if ok else m.DONE
                rate.sleep()
                continue
    
            if m.phase == m.ARM_HOME_2:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_grasp, yaw)
                m.exec_arm_home_only()
                rospy.loginfo("抓取后爬升回巡航高度 (Δz≈%.2f m)", m.hover_rel_m)
                m.phase = m.CLIMB
                m.t_phase0 = time.time()
                rate.sleep()
                continue
    
            if m.phase == m.CLIMB:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_h, yaw)
                if m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_h) < max(
                    m.wp_tol_xy, m.wp_tol_z
                ):
                    if time.time() - m.t_phase0 > m.hover_settle_sec:
                        m.phase = m.RTL
                else:
                    m.t_phase0 = time.time()
                rate.sleep()
                continue
    
            if m.phase == m.RTL:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_h, m.yaw0)
                if m.dist3(px, py, pz, m.x0, m.y0, m.z_h) < m.wp_tol_xy * 1.5:
                    rospy.loginfo("回起点上方，降落")
                    m.phase = m.LAND
                rate.sleep()
                continue
    
            if m.phase == m.LAND:
                try:
                    m.set_mode(0, "AUTO.LAND")
                except rospy.ServiceException as e:
                    rospy.logwarn("%s", e)
                m.phase = m.DONE
                rate.sleep()
                continue
    
            rate.sleep()

        rospy.loginfo("流程结束")
    finally:
        if m._moveit_cpp_inited:
            moveit_commander.roscpp_shutdown()
            rospy.loginfo("moveit_commander: roscpp_shutdown（进程退出）")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
