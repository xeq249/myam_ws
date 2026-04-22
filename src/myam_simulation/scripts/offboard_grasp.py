#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offboard：悬停 → 臂准备 → 前飞/下降 → 视觉蓝块 → MoveIt 抓取 → 返航降落。
依赖 sitl_task + MAVROS + move_group。调参见各 ~ 参数与 launch。
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
from geometry_msgs.msg import Point, Pose, PoseStamped, PointStamped, Vector3, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from moveit_commander import MoveGroupCommander
from moveit_commander.exception import MoveItCommanderException
from sensor_msgs.msg import CameraInfo, Image, JointState

try:
    from gazebo_msgs.msg import ContactsState
except ImportError:
    ContactsState = None

try:
    from tf2_geometry_msgs import do_transform_point
    from tf2_geometry_msgs import do_transform_vector3
except ImportError:
    do_transform_point = None
    do_transform_vector3 = None

try:
    import tf.transformations as _tft
except ImportError:
    _tft = None


def start_rospy_callbacks_for_moveit(num_threads=4):
    try:
        from rospy.timer import AsyncSpinner as _AsyncSpinner  # type: ignore
    except ImportError:
        _AsyncSpinner = None
    if _AsyncSpinner is not None:
        spin = _AsyncSpinner(num_threads)
        spin.start()
        rospy.loginfo(
            "[ROS] AsyncSpinner(timer) threads=%d | EN: async callbacks",
            num_threads,
        )
        return spin
    if hasattr(rospy, "AsyncSpinner"):
        spin = rospy.AsyncSpinner(num_threads)
        spin.start()
        rospy.loginfo(
            "[ROS] AsyncSpinner threads=%d | EN: async callbacks",
            num_threads,
        )
        return spin
    thr = threading.Thread(name="rospy_spin_callbacks", target=rospy.spin)
    thr.daemon = True
    thr.start()
    rospy.loginfo("[ROS] spin 后台线程 | EN: rospy.spin thread")
    return thr


def start_offboard_setpoint_stream(mission, hz, use_wall_clock=True):
    hz = max(1.0, float(hz))
    period = 1.0 / hz

    def _loop():
        r = None if use_wall_clock else rospy.Rate(hz)

        def _sleep_period():
            if use_wall_clock:
                time.sleep(period)
            else:
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    raise

        while not rospy.is_shutdown():
            if mission.phase == mission.PH_DONE:
                try:
                    _sleep_period()
                except rospy.ROSInterruptException:
                    break
                continue
            tup = mission.get_offboard_sp_tuple()
            if tup is None:
                try:
                    _sleep_period()
                except rospy.ROSInterruptException:
                    break
                continue
            x, y, z, yaw = tup
            mission.sp_pub.publish(mission.sp(x, y, z, yaw))
            try:
                _sleep_period()
            except rospy.ROSInterruptException:
                break

    th = threading.Thread(name="offboard_setpoint_stream", target=_loop)
    th.daemon = True
    th.start()
    rospy.loginfo(
        "[OFFBOARD] 设定点流 %.1f Hz | EN: setpoint stream %.1f Hz",
        hz,
        hz,
    )
    return th


def maybe_publish_offboard(m, x, y, z, yaw):
    if not m.offboard_stream_enable:
        m.sp_pub.publish(m.sp(x, y, z, yaw))


def transform_point(buf, pt, target_frame, source_frame, timeout=2.0):
    if do_transform_point is None:
        rospy.logerr(
            "[TF] 请安装 ros-$ROS_DISTRO-tf2-geometry-msgs | EN: install tf2_geometry_msgs"
        )
        return None
    ps = PointStamped()
    ps.header.frame_id = source_frame
    ps.header.stamp = rospy.Time(0)
    ps.point = pt
    try:
        tfm = buf.lookup_transform(
            target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout)
        )
        return do_transform_point(ps, tfm).point
    except Exception as e:
        rospy.logwarn_throttle(
            3.0,
            "[TF] %s→%s 失败: %s | EN: transform failed",
            source_frame,
            target_frame,
            e,
        )
        return None


def transform_vector(buf, vec, target_frame, source_frame, timeout=2.0):
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
        return do_transform_vector3(vs, tfm).vector
    except Exception as e:
        rospy.logwarn_throttle(
            5.0,
            "[TF] 矢量 %s→%s: %s | EN: vector transform failed",
            source_frame,
            target_frame,
            e,
        )
        return None


class GraspMission(object):
    PH_HOVER_1M = "hover_1m"
    PH_ARM_READY = "arm_ready"
    PH_ARM_SETTLE = "arm_settle"
    PH_FWD = "forward"
    PH_FWD_SETTLE = "fwd_settle"
    PH_DESCEND = "descend_slow"
    PH_LOW_HOLD = "low_hold"
    PH_DETECT = "detect"
    PH_GRASP = "grasp"
    PH_ARM_HOME = "arm_home_after"
    PH_CLIMB = "climb"
    PH_CLIMB_SETTLE = "climb_settle"
    PH_RTL = "rtl"
    PH_LAND = "land"
    PH_DONE = "done"

    def __init__(self):
        rospy.init_node("offboard_grasp")

        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.warmup_iters = int(rospy.get_param("~warmup_iters", 40))
        self.hover_rel_m = float(rospy.get_param("~hover_rel_m", 1.0))
        self.low_rel_m = float(rospy.get_param("~low_rel_m", 0.45))
        # 在 low_rel_m 之上再加竖直余量（米），减轻贴台、触底后弹起
        self.low_rel_guard_m = float(rospy.get_param("~low_rel_guard_m", 0.08))
        self.forward_m = float(rospy.get_param("~forward_m", 1.50))
        # 机头方向在 forward_m 上再叠一段前飞（米），修正「落点略远、末端差几厘米够不着」
        self.approach_forward_extra_m = float(
            rospy.get_param("~approach_forward_extra_m", 0.0)
        )
        # 前飞水平目标用时间斜坡（秒），减轻位置阶跃 → 前倾过猛 → 高度环短暂爬升；0=瞬时跳变（旧行为）
        self.forward_ramp_sec = float(rospy.get_param("~forward_ramp_sec", 5.0))
        # 前飞落点相对机头做侧向平移（米），与 forward_m 同时作用在 local 水平面：在 (cos,sin) 上垂向
        # 加 (sin,-cos)。约定：正值 = 从机尾向机头看，落点向**右**平移，可修正「整体偏在机头左侧」
        self.approach_lateral_m = float(rospy.get_param("~approach_lateral_m", 0.0))
        self.wp_tol_xy = float(rospy.get_param("~wp_tol_xy", 0.15))
        self.wp_tol_z = float(rospy.get_param("~wp_tol_z", 0.12))
        self.hold_sec = float(rospy.get_param("~hold_sec", 2.0))
        self.descend_duration = float(rospy.get_param("~descend_duration", 22.0))
        self.low_hold_sec = float(rospy.get_param("~low_hold_sec", 2.0))

        self.auto_arm = bool(rospy.get_param("~auto_arm", True))
        self.auto_offboard = bool(rospy.get_param("~auto_offboard", True))
        self.async_spinner_threads = int(rospy.get_param("~async_spinner_threads", 4))
        self.offboard_stream_enable = bool(rospy.get_param("~offboard_stream_enable", True))
        self.offboard_stream_hz = float(rospy.get_param("~offboard_stream_hz", 20.0))
        # true：offboard 流用墙钟 sleep。use_sim_time 下 rospy.Rate 依赖本进程处理 /clock；
        # MoveIt arm.go 等长时间占 GIL 时 Rate 几乎不走 → 设定点断流 → PX4「No offboard signal」。
        self.offboard_stream_use_wall_clock = bool(
            rospy.get_param("~offboard_stream_use_wall_clock", True)
        )

        self.arm_ready_joints = rospy.get_param(
            "~arm_ready_joints", [0.0, -1.57, 2.93, 0.0, 0.0]
        )
        # 第三关节贴近 URDF 上限时 effort 仿真易 CONTROL_FAILED，作保守备选
        self.arm_ready_joints_fallback = rospy.get_param(
            "~arm_ready_joints_fallback", [0.0, -1.57, 2.90, 0.0, 0.0]
        )
        self.arm_goal_joint_tolerance = float(
            rospy.get_param("~arm_goal_joint_tolerance", 0.12)
        )
        self.arm_vel_scaling = float(rospy.get_param("~arm_vel_scaling", 0.12))
        self.arm_accel_scaling = float(rospy.get_param("~arm_accel_scaling", 0.12))
        self.arm_home_vel_scaling = float(rospy.get_param("~arm_home_vel_scaling", 0.08))
        self.arm_home_accel_scaling = float(
            rospy.get_param("~arm_home_accel_scaling", 0.08)
        )
        # 准备姿态（arm_ready_*）：默认与 arm_home_* 一致；可与 offboard_grasp_blue_cube 对齐单独调
        if rospy.has_param("~arm_ready_vel_scaling"):
            self.arm_ready_vel_scaling = float(rospy.get_param("~arm_ready_vel_scaling"))
        else:
            self.arm_ready_vel_scaling = self.arm_home_vel_scaling
        if rospy.has_param("~arm_ready_accel_scaling"):
            self.arm_ready_accel_scaling = float(rospy.get_param("~arm_ready_accel_scaling"))
        else:
            self.arm_ready_accel_scaling = self.arm_home_accel_scaling
        self.arm_planning_time = float(rospy.get_param("~arm_planning_time", 25.0))
        # 视觉折线段 OMPL：略拉长 allowed_planning_time，减轻「21.6s 内无解」的 TIMED_OUT（见 move_group 日志）
        self.vision_ompl_planning_time = float(
            rospy.get_param("~vision_ompl_planning_time", 55.0)
        )
        self.vision_ompl_planning_attempts = int(
            rospy.get_param("~vision_ompl_planning_attempts", 12)
        )
        self.move_group_wait_for_servers = float(
            rospy.get_param("~move_group_wait_for_servers", 45.0)
        )
        self.move_group_reinit_sleep = float(
            rospy.get_param("~move_group_reinit_sleep", 1.0)
        )
        self.gripper_open = float(rospy.get_param("~gripper_open", 1.5))
        # 预抓取前「尽量张大」：URDF 约 0=闭合、1.5=全开；实际张开角见 gripper_open_max
        self.gripper_open_max = float(
            rospy.get_param("~gripper_open_max", self.gripper_open)
        )
        # 闭合目标角：须接近 0；误设 ~1.0 时 goal_tolerance 下 MoveIt 仍报成功但爪几乎未闭
        self.gripper_close = float(rospy.get_param("~gripper_close", 0.12))
        # 0：直接 _close_gripper_after_approach；否则先 MoveIt 到该角再最终闭合（配开爪近距伪吸附「微夹」）
        self.gripper_soft_pinch = float(rospy.get_param("~gripper_soft_pinch", 0.0))
        self.gripper_soft_pinch_pause_sec = float(
            rospy.get_param("~gripper_soft_pinch_pause_sec", 0.18)
        )
        self.gripper_goal_tolerance = float(
            rospy.get_param("~gripper_goal_tolerance", 0.2)
        )
        self.gripper_joint_name = rospy.get_param(
            "~gripper_joint_name", "link5_to_gripper_link"
        )
        # 夹爪关节角 ≤ 阈值视为已闭合（URDF 约 0=闭、1.5=开）
        self.grasp_joint_closed_threshold = float(
            rospy.get_param("~grasp_joint_closed_threshold", 0.35)
        )
        # 规划系下桌面目标大致高度带；超出则丢弃视觉点（避免 z≈1m 等错误深度导致 TIMED_OUT）
        self.vision_reframe_z_min = float(rospy.get_param("~vision_reframe_z_min", -0.25))
        self.vision_reframe_z_max = float(rospy.get_param("~vision_reframe_z_max", 0.62))
        self.grasp_ee_to_target_max_m = float(
            rospy.get_param("~grasp_ee_to_target_max_m", 0.12)
        )
        # 规划系距离大但画面上仍对准：可将 TCP 投到像素平面与检测 uv 比较（与 grasp_ee_to_target_max_m 逻辑或）
        self.grasp_ee_near_cam_fallback_enable = bool(
            rospy.get_param("~grasp_ee_near_cam_fallback_enable", False)
        )
        self.grasp_ee_near_cam_fallback_max_px = float(
            rospy.get_param("~grasp_ee_near_cam_fallback_max_px", 80.0)
        )
        # 末段 execute 失败（如 Gazebo CONTROL_FAILED）时仍尝试闭爪的放宽判据（0=关闭该项）
        self.force_close_gripper_if_tcp_within_m = float(
            rospy.get_param("~force_close_gripper_if_tcp_within_m", 0.20)
        )
        # 与上项逻辑或：TCP 在画面上距稳定检测 uv 的像素误差 ≤ 此值也允许强制闭爪；0=不用像素
        self.force_close_gripper_if_pixel_below_px = float(
            rospy.get_param("~force_close_gripper_if_pixel_below_px", 0.0)
        )
        # Gazebo myam_grasping.world 蓝块 bumper → ContactsState（须在订阅 bumper 之前赋值）
        self.grasp_contact_trigger_enable = bool(
            rospy.get_param("~grasp_contact_trigger_enable", True)
        )
        self.grasp_contact_topic = rospy.get_param(
            "~grasp_contact_topic", "/grasp_cube_contact_bumper"
        ).strip()
        self._grasp_contact_arm_kws = rospy.get_param(
            "~grasp_contact_arm_name_substrings",
            ["gripper_link", "link5_to_gripper", "hand_tcp"],
        )
        self._grasp_contact_cube_kws = rospy.get_param(
            "~grasp_contact_cube_name_substrings",
            ["cube_link", "grasp_target_cube"],
        )
        # TCP 在相机光学系 z 很小时投影病态；用下限代替直接放弃，避免日志 nan、丢失像素回退
        self.grasp_tcp_proj_min_z_m = float(
            rospy.get_param("~grasp_tcp_proj_min_z_m", 0.02)
        )
        # true：分多步关爪，|joint effort| 超阈值则视为顶到物体后仍执行最后全闭
        self.grasp_incremental_close_enable = bool(
            rospy.get_param("~grasp_incremental_close_enable", False)
        )
        self.grasp_incremental_close_steps = int(
            rospy.get_param("~grasp_incremental_close_steps", 10)
        )
        self.grasp_touch_effort_abs = float(
            rospy.get_param("~grasp_touch_effort_abs", 1.5)
        )
        # 仅加在「下落点」z 上：略增下探（负值），提高指尖够到台面方块的概率
        self.vision_grasp_plunge_extra_z_m = float(
            rospy.get_param("~vision_grasp_plunge_extra_z_m", 0.0)
        )
        # 主视觉链成功后、夹爪闭合前：再采深度并短步 OMPL 修正（有界），减轻开环漂移
        self.grasp_fin_refine_enable = bool(
            rospy.get_param("~grasp_fin_refine_enable", True)
        )
        self.grasp_fin_refine_iters = int(
            rospy.get_param("~grasp_fin_refine_iters", 2)
        )
        self.grasp_fin_refine_settle_sec = float(
            rospy.get_param("~grasp_fin_refine_settle_sec", 0.12)
        )
        self.grasp_fin_refine_min_disp_m = float(
            rospy.get_param("~grasp_fin_refine_min_disp_m", 0.004)
        )
        self.grasp_fin_refine_max_step_m = float(
            rospy.get_param("~grasp_fin_refine_max_step_m", 0.038)
        )
        self.grasp_fin_refine_vel_scale = float(
            rospy.get_param("~grasp_fin_refine_vel_scale", 0.32)
        )
        # 闭合前微调：TCP 已接近本轮刷新目标或像素已对准则提前结束微调→尽快闭爪；0=关闭该项
        self.grasp_fin_refine_early_stop_tcp_m = float(
            rospy.get_param("~grasp_fin_refine_early_stop_tcp_m", 0.0)
        )
        self.grasp_fin_refine_early_stop_pixel_px = float(
            rospy.get_param("~grasp_fin_refine_early_stop_pixel_px", 0.0)
        )
        # GRASP_COMMIT：闭合前仅在「触发区」内连续 N 帧满足（位姿+可选像素）才锁定目标并闭爪；超时则锁定最后有效目标
        self.grasp_commit_enable = bool(rospy.get_param("~grasp_commit_enable", True))
        self.grasp_commit_pos_tol_m = float(
            rospy.get_param("~grasp_commit_pos_tol_m", 0.038)
        )
        # EE z 轴与期望接近方向夹角 ≤ 该值(弧度)；0=不检查姿态
        self.grasp_commit_orient_tol_rad = float(
            rospy.get_param("~grasp_commit_orient_tol_rad", 0.35)
        )
        # >0 时像素偏差须同时满足；0=不把像素并入触发区
        self.grasp_commit_pixel_tol_px = float(
            rospy.get_param("~grasp_commit_pixel_tol_px", 0.0)
        )
        self.grasp_commit_stable_frames = int(
            rospy.get_param("~grasp_commit_stable_frames", 3)
        )
        self.grasp_commit_refine_timeout_sec = float(
            rospy.get_param("~grasp_commit_refine_timeout_sec", 12.0)
        )
        _axis = rospy.get_param(
            "~grasp_commit_desired_approach_axis", [0.0, 0.0, -1.0]
        )
        self._grasp_commit_desired_axis = np.asarray(_axis, dtype=np.float64).reshape(
            3,
        )
        na = float(np.linalg.norm(self._grasp_commit_desired_axis))
        if na > 1e-9:
            self._grasp_commit_desired_axis /= na
        else:
            self._grasp_commit_desired_axis = np.array(
                [0.0, 0.0, -1.0], dtype=np.float64
            )
        # roarm_base_link 原点近似为臂座：球形工作区，限制视觉末端目标，减轻短臂够不着导致的 OMPL 超时
        self.vision_workspace_enable = bool(
            rospy.get_param("~vision_workspace_enable", True)
        )
        self.vision_workspace_radius_max_m = float(
            rospy.get_param("~vision_workspace_radius_max_m", 0.56)
        )
        self.vision_workspace_radius_min_m = float(
            rospy.get_param("~vision_workspace_radius_min_m", 0.0)
        )
        self.vision_workspace_safety_margin_m = float(
            rospy.get_param("~vision_workspace_safety_margin_m", 0.02)
        )
        self.vision_workspace_reject_instead_of_clamp = bool(
            rospy.get_param("~vision_workspace_reject_instead_of_clamp", False)
        )
        self.camera_depth_max_m = float(rospy.get_param("~camera_depth_max_m", 1.8))
        self.depth_median_radius = int(rospy.get_param("~depth_median_radius", 4))

        self.pregrasp_joints = rospy.get_param(
            "~pregrasp_joints", [0.0, -0.95, 2.35, -0.4, 0.0]
        )
        self.grasp_joints = rospy.get_param(
            "~grasp_joints", [0.0, -1.05, 2.55, -0.5, 0.0]
        )

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
        self.vision_target_offset = rospy.get_param(
            "~vision_target_offset", [0.0, 0.0, 0.0]
        )
        self.vision_pregrasp_delta = rospy.get_param(
            "~vision_pregrasp_delta", [0.0, 0.0, 0.038]
        )
        self.vision_grasp_delta = rospy.get_param(
            "~vision_grasp_delta", [0.0, 0.0, -0.048]
        )
        self.vision_cartesian_vel_scale = float(
            rospy.get_param("~vision_cartesian_vel_scale", 0.4)
        )
        # 全 6D 位姿约束时 OMPL 关节路径可能长且扭，effort 仿真易执行超时；仿真抓取默认仅约束位置
        self.vision_cartesian_use_pose = bool(
            rospy.get_param("~vision_cartesian_use_pose", False)
        )
        self.vision_goal_position_tolerance = float(
            rospy.get_param("~vision_goal_position_tolerance", 0.09)
        )
        self.vision_goal_orientation_tolerance = float(
            rospy.get_param("~vision_goal_orientation_tolerance", 0.85)
        )
        self.vision_relaxed_joint_tolerance = float(
            rospy.get_param("~vision_relaxed_joint_tolerance", 0.22)
        )
        self.vision_pose_step_vel_scale = float(
            rospy.get_param("~vision_pose_step_vel_scale", 0.07)
        )
        # 视觉段 arm.go 若速度缩放过低，名义轨迹可达十余秒；MoveIt 默认 duration 监控下约 d*1.1+0.5s 即会 TIMED_OUT
        self.vision_min_velocity_scale = float(
            rospy.get_param("~vision_min_velocity_scale", 0.14)
        )
        # 视觉段每段 arm.go 的速度上限（MoveIt 要求 ≤1）；略提高可缩短名义轨迹时长
        self.vision_waypoint_vel_cap = float(
            rospy.get_param("~vision_waypoint_vel_cap", 0.92)
        )
        # 预伸展关节若把腕部摆到不利构型，compute_cartesian_path 在「保持姿态」下会 fraction=0；默认关，需用时自行调 joint
        self.vision_reach_seed_enable = bool(
            rospy.get_param("~vision_reach_seed_enable", False)
        )
        self.vision_reach_seed_vel_scale = float(
            rospy.get_param("~vision_reach_seed_vel_scale", 0.48)
        )
        self.vision_reach_seed_accel_scale = float(
            rospy.get_param("~vision_reach_seed_accel_scale", 0.48)
        )
        _seed = rospy.get_param("~vision_reach_seed_joints", None)
        self.vision_reach_seed_joints = (
            [float(x) for x in _seed] if _seed is not None and len(_seed) == 5 else None
        )
        # 若预伸展关节把末端摆离预接近点更远（常见于 seed 向下伸而视觉目标在 +z），可回退 arm_ready。
        # 默认关：弦长噪声/小幅变差会触发「刚往下伸又整臂收回」，观感像半途而废。
        self.vision_reach_seed_revert_if_worse = bool(
            rospy.get_param("~vision_reach_seed_revert_if_worse", False)
        )
        self.vision_reach_seed_revert_margin_m = float(
            rospy.get_param("~vision_reach_seed_revert_margin_m", 0.1)
        )
        # 视觉前先关节空间移到 pregrasp_joints（文档：先验证关节路径再末端直线）
        self.vision_joint_staging_enable = bool(
            rospy.get_param("~vision_joint_staging_enable", False)
        )
        # 执行末端 Pose 链之前仅 plan() 预接近/下落，区分无解与执行超时
        self.vision_preflight_plan = bool(rospy.get_param("~vision_preflight_plan", True))
        # 预接近点 plan 失败则跳过 Pose 执行（避免多次 arm.go TIMED_OUT），直接关节备用
        self.vision_preflight_skip_pose_if_pregrasp_fails = bool(
            rospy.get_param("~vision_preflight_skip_pose_if_pregrasp_fails", True)
        )
        # 预检仅作快速探针，勿用 vision_ompl_planning_time(55s) 否则每次预检卡满 55s
        self.vision_preflight_planning_time = float(
            rospy.get_param("~vision_preflight_planning_time", 18.0)
        )
        self.vision_preflight_planning_attempts = int(
            rospy.get_param("~vision_preflight_planning_attempts", 8)
        )
        self.vision_preflight_relaxed_position_tolerance = float(
            rospy.get_param("~vision_preflight_relaxed_position_tolerance", 0.18)
        )
        self.vision_preflight_relaxed_joint_tolerance = float(
            rospy.get_param("~vision_preflight_relaxed_joint_tolerance", 0.28)
        )
        self.vision_preflight_relaxed_orientation_tolerance = float(
            rospy.get_param("~vision_preflight_relaxed_orientation_tolerance", 1.25)
        )
        self.vision_preflight_retry_pose_from_approach = bool(
            rospy.get_param("~vision_preflight_retry_pose_from_approach", True)
        )
        self.vision_preflight_retry_pose_retaining_wrist = bool(
            rospy.get_param("~vision_preflight_retry_pose_retaining_wrist", True)
        )
        # 两步大位移在 effort 仿真里易整段超时；拆成多段短位移，每段 arm.go() 更易跟踪
        self.vision_use_segmented_pose = bool(
            rospy.get_param("~vision_use_segmented_pose", True)
        )
        self.vision_cartesian_step_m = float(
            rospy.get_param("~vision_cartesian_step_m", 0.028)
        )
        self.vision_segment_min_steps = int(
            rospy.get_param("~vision_segment_min_steps", 4)
        )
        self.vision_segment_max_steps = int(
            rospy.get_param("~vision_segment_max_steps", 12)
        )
        # 预接近折线最多 N 段（1=一次 OMPL 到预接近点）；下落仍用 vision_segment_* 与步长
        self.vision_pregrasp_max_segments = int(
            rospy.get_param("~vision_pregrasp_max_segments", 1)
        )
        # cartesian_execute：笛卡尔直线；ompl（及任意非前者）：仅用 OMPL，见 exec_arm_sequence 分支
        self.vision_path_mode = rospy.get_param("~vision_path_mode", "ompl").strip().lower()
        self.vision_cartesian_eef_step = float(
            rospy.get_param("~vision_cartesian_eef_step", 0.04)
        )
        self.vision_cartesian_min_fraction = float(
            rospy.get_param("~vision_cartesian_min_fraction", 0.06)
        )
        self.vision_cartesian_avoid_collision = bool(
            rospy.get_param("~vision_cartesian_avoid_collision", False)
        )
        self.vision_cartesian_chunks = int(rospy.get_param("~vision_cartesian_chunks", 1))
        # False：末端 +Z 指向目标，直线笛卡尔才易有解；True：保持当前姿态（易 fraction=0）
        self.vision_cartesian_keep_orientation = bool(
            rospy.get_param("~vision_cartesian_keep_orientation", False)
        )
        # 工具接近轴：+1 则 +Z 指向目标；-1 则 -Z 指向目标（常见从上向下伸）
        self.vision_cartesian_approach_axis_sign = float(
            rospy.get_param("~vision_cartesian_approach_axis_sign", -1.0)
        )
        self.vision_cartesian_dense_waypoints = int(
            rospy.get_param("~vision_cartesian_dense_waypoints", 14)
        )
        # 单腿欧氏距超过此值则不调笛卡尔（常见 ~0.8m 整段 fraction=0），直接 OMPL 分段
        self.vision_cartesian_skip_if_dist_m = float(
            rospy.get_param("~vision_cartesian_skip_if_dist_m", 0.42)
        )
        # 视觉段 MoveGroup 名义速度（略低于每段内 vision_waypoint 再调的值）
        self.vision_ompl_plan_vel_scale = float(
            rospy.get_param("~vision_ompl_plan_vel_scale", 0.4)
        )
        self.vision_ompl_plan_accel_scale = float(
            rospy.get_param("~vision_ompl_plan_accel_scale", 0.4)
        )
        # 长跨距 OMPL 折线：加大步长、减少段数，降低「每段一次 ~20s 超时」的总次数
        self.vision_polyline_coalesce_dist_m = float(
            rospy.get_param("~vision_polyline_coalesce_dist_m", 0.48)
        )
        self.vision_polyline_step_m_long = float(
            rospy.get_param("~vision_polyline_step_m_long", 0.065)
        )
        # 弦长超过此值时压缩折线段数（略增大每段步长），减轻首段 OMPL 仍 ~20s 超时
        self.vision_polyline_long_leg_compress_m = float(
            rospy.get_param("~vision_polyline_long_leg_compress_m", 0.68)
        )
        self.vision_polyline_long_leg_min_step_m = float(
            rospy.get_param("~vision_polyline_long_leg_min_step_m", 0.105)
        )
        # 长腿 OMPL 首段仍易整段关节路径过长→TIMED_OUT：先沿直线密铺若干毫米级微步再折线
        self.vision_polyline_lead_in_enable = bool(
            rospy.get_param("~vision_polyline_lead_in_enable", True)
        )
        self.vision_polyline_lead_in_min_leg_m = float(
            rospy.get_param("~vision_polyline_lead_in_min_leg_m", 0.5)
        )
        self.vision_polyline_lead_in_step_m = float(
            rospy.get_param("~vision_polyline_lead_in_step_m", 0.02)
        )
        self.vision_polyline_lead_in_cap_m = float(
            rospy.get_param("~vision_polyline_lead_in_cap_m", 0.14)
        )
        self.vision_polyline_lead_in_max_frac = float(
            rospy.get_param("~vision_polyline_lead_in_max_frac", 0.28)
        )
        self.vision_polyline_lead_in_max_points = int(
            rospy.get_param("~vision_polyline_lead_in_max_points", 8)
        )
        # 长腿：先在关节空间向 vision_reach_seed_joints 插一小段（不依赖 reach_seed_enable），再折线
        # 默认关：固定 seed 常使末端离蓝块更远（弦长变大），反而加重 OMPL 目标采样失败
        self.vision_joint_lead_in_enable = bool(
            rospy.get_param("~vision_joint_lead_in_enable", False)
        )
        self.vision_joint_lead_in_min_leg_m = float(
            rospy.get_param("~vision_joint_lead_in_min_leg_m", 0.45)
        )
        self.vision_joint_lead_in_steps = int(
            rospy.get_param("~vision_joint_lead_in_steps", 6)
        )
        self.vision_joint_lead_in_total_fraction = float(
            rospy.get_param("~vision_joint_lead_in_total_fraction", 0.22)
        )
        # OMPL 路点：保持当前腕姿态只改位置，常比纯 set_position_target 关节路径短
        self.vision_ompl_lock_wrist_orientation = bool(
            rospy.get_param("~vision_ompl_lock_wrist_orientation", False)
        )
        # ---------- 5DOF：plan+execute、Z 回退、末点 XY 多试、TCP 指尖余量 ----------
        self.vision_5dof_use_plan_execute = bool(
            rospy.get_param("~vision_5dof_use_plan_execute", True)
        )
        self.vision_5dof_execute_max_retries = int(
            rospy.get_param("~vision_5dof_execute_max_retries", 4)
        )
        self.vision_5dof_execute_scale_decay = float(
            rospy.get_param("~vision_5dof_execute_scale_decay", 0.58)
        )
        self.vision_5dof_z_backoff_steps = int(
            rospy.get_param("~vision_5dof_z_backoff_steps", 6)
        )
        self.vision_5dof_z_backoff_dz = float(
            rospy.get_param("~vision_5dof_z_backoff_dz", 0.032)
        )
        self.vision_grasp_xy_retry_offsets = rospy.get_param(
            "~vision_grasp_xy_retry_offsets",
            [
                [0.0, 0.0],
                [0.007, 0.0],
                [-0.007, 0.0],
                [0.0, 0.007],
                [0.0, -0.007],
            ],
        )
        self.vision_tcp_extra_z_m = float(rospy.get_param("~vision_tcp_extra_z_m", 0.028))
        # 关节前插已成功时跳过笛卡尔微步（微步仍易 TIMED_OUT；主折线段更大、反而常与关节前插衔接更好）
        self.vision_polyline_skip_cartesian_micro_after_joint = bool(
            rospy.get_param("~vision_polyline_skip_cartesian_micro_after_joint", True)
        )
        # 每次抓取前经 dynamic_reconfigure 写回（对抗 RViz 等把 monitoring 改回 true、scaling 钳到 10）
        self.vision_fix_trajectory_execution_via_dynreconf = bool(
            rospy.get_param("~vision_fix_trajectory_execution_via_dynreconf", True)
        )
        self.move_group_trajectory_execution_ns = rospy.get_param(
            "~move_group_trajectory_execution_ns", "/move_group/trajectory_execution"
        ).strip()
        # MoveIt 对轨迹执行速度的倍率（dynreconf 上限 10）。过大→effort 仿真猛跟参考、易大幅晃动；视觉长段可适当大、纯关节 Home 宜小
        self.trajectory_execution_velocity_scaling = float(
            rospy.get_param("~trajectory_execution_velocity_scaling", 3.5)
        )
        # exec_arm_joints（准备/home 等）单独使用，优先抑制晃动
        self.arm_trajectory_execution_velocity_scaling = float(
            rospy.get_param("~arm_trajectory_execution_velocity_scaling", 1.5)
        )
        self.vision_log_plan_on_fail = bool(
            rospy.get_param("~vision_log_plan_on_fail", True)
        )
        # 多次 arm.go 超时后立刻 plan() 会读到动作端残留状态；stop + 短睡再探针
        self.vision_plan_probe_settle_sec = float(
            rospy.get_param("~vision_plan_probe_settle_sec", 0.35)
        )
        self._dynreconf_warned = False
        self._last_traj_dynreconf_wall = 0.0

        self.hsv_lower = np.array(rospy.get_param("~hsv_blue_lower", [72, 18, 25]))
        self.hsv_upper = np.array(rospy.get_param("~hsv_blue_upper", [138, 255, 255]))
        self.min_blob_area = int(rospy.get_param("~min_blob_area", 18))
        self.blue_mask_erode_iters = int(rospy.get_param("~blue_mask_erode_iters", 0))
        self.blue_mask_dilate_iters = int(rospy.get_param("~blue_mask_dilate_iters", 2))
        self.detection_stable = int(rospy.get_param("~detection_stable_frames", 5))

        self.detect_refine_xy_enable = bool(rospy.get_param("~detect_refine_xy_enable", True))
        self.detect_refine_xy_gain = float(rospy.get_param("~detect_refine_xy_gain", 0.11))
        self.detect_refine_xy_max_step = float(
            rospy.get_param("~detect_refine_xy_max_step", 0.014)
        )
        self.detect_refine_min_pixel_error = float(
            rospy.get_param("~detect_refine_min_pixel_error", 22.0)
        )
        self.detect_refine_max_cumulative_m = float(
            rospy.get_param("~detect_refine_max_cumulative_m", 0.05)
        )
        self.detect_refine_xy_stride = max(
            1, int(rospy.get_param("~detect_refine_xy_stride", 2))
        )

        self.ref_frame = rospy.get_param("~planning_frame", "roarm_base_link")
        self.camera_optical_frame = rospy.get_param(
            "~camera_optical_frame", "camera_color_optical_frame"
        )
        self.tf_refine_target_frame = rospy.get_param("~tf_refine_target_frame", "").strip()
        # 识别阶段微调机体位置：增量须与 MAVROS local 位置同系（一般为 map），不可再用臂座系直接加 x_fwd/y_fwd
        self.detect_refine_xy_target_frame = rospy.get_param(
            "~detect_refine_xy_target_frame", "map"
        ).strip()

        self.grasp_max_retries = int(rospy.get_param("~grasp_max_retries", 2))
        self.grasp_retry_forward_m = float(rospy.get_param("~grasp_retry_forward_m", 0.0))
        # True：offboard 偏航锁为起飞后记录的 yaw0（+ 可选 offset），减轻前飞/下降中机头漂移导致视觉与规划系不对齐
        self.lock_mission_yaw = bool(rospy.get_param("~lock_mission_yaw", False))
        self.mission_yaw_offset_rad = float(
            rospy.get_param("~mission_yaw_offset_rad", 0.0)
        )
        if self.grasp_retry_forward_m > 1e-6:
            rospy.logwarn(
                "[CFG] grasp_retry_forward_m=%.3f（机体动则视觉目标在臂座系跳变；可置0）| EN: retry_fwd may shift target in arm frame",
                self.grasp_retry_forward_m,
            )
        self.gripper_joint_upper = float(rospy.get_param("~gripper_joint_upper", 1.52))
        # 与 SRDF arm tip_link / RViz 末端一致，用于 TF 诊断（report_grasp_success）
        self.ee_tcp_frame = rospy.get_param("~ee_tcp_frame", "hand_tcp")

        self.state = State()
        self.pose = PoseStamped()
        self.phase = self.PH_HOVER_1M
        self.bridge = CvBridge()
        self._img = None
        self._depth = None
        self._K = None
        self._stable_cnt = 0
        self._last_uv = None
        self._refine_xy_cumulative_m = 0.0
        self._refine_frame_tick = 0
        self._gripper_joint_pos = None
        self._gripper_joint_effort = None
        # exec_arm_sequence 内最终下落点（工作区夹紧 + vision_tcp_extra_z 后），供 report 与 MoveIt 目标一致
        self._last_exec_p_fin = None
        self.t_phase0 = 0.0
        self._hold_since = None
        self._grasp_attempt = 0

        self.x0 = self.y0 = self.z0 = 0.0
        self.yaw0 = 0.0
        self.z_high = 0.0
        self.z_low = 0.0
        self.z_sp = 0.0
        self.x_fwd = self.y_fwd = 0.0
        self._fwd_ramp_x0 = self._fwd_ramp_y0 = 0.0
        self._fwd_ramp_x1 = self._fwd_ramp_y1 = 0.0

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
        jst = rospy.get_param("~joint_states_topic", "/joint_states_merged")
        rospy.Subscriber(jst, JointState, self._cb_joint_states, queue_size=1)

        if ContactsState is not None and self.grasp_contact_trigger_enable:
            rospy.Subscriber(
                self.grasp_contact_topic,
                ContactsState,
                self._cb_grasp_contact,
                queue_size=10,
            )
            rospy.loginfo(
                "[NODE] Gazebo 爪-块碰撞→收爪 topic=%s | EN: bumper contact",
                self.grasp_contact_topic,
            )

        self.sp_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=10
        )
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        rospy.loginfo(
            "[NODE] 订阅 color=%s depth=%s 规划系=%s | EN: topics + planning frame",
            self.color_topic,
            self.depth_topic,
            self.ref_frame,
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

    def _cb_grasp_contact(self, msg):
        """蓝块 bumper：一对碰撞名含爪与立方体时置位，供跳过微调 / 强制闭爪。"""
        if not self.grasp_contact_trigger_enable:
            return
        try:
            for st in msg.states:
                if self._grasp_contact_pair_is_target(
                    st.collision1_name, st.collision2_name
                ):
                    if not self._grasp_contact_seen:
                        rospy.loginfo(
                            "[CONTACT] 检测到爪↔蓝块碰撞 %s × %s | EN: grasp bump",
                            st.collision1_name,
                            st.collision2_name,
                        )
                    self._grasp_contact_seen = True
                    return
        except Exception:
            pass

    def _grasp_contact_pair_is_target(self, n1, n2):
        n1 = (n1 or "").lower()
        n2 = (n2 or "").lower()

        def has_kw(s, kws):
            return any(k.lower() in s for k in kws)

        arm_cube = has_kw(n1, self._grasp_contact_arm_kws) and has_kw(
            n2, self._grasp_contact_cube_kws
        )
        cube_arm = has_kw(n2, self._grasp_contact_arm_kws) and has_kw(
            n1, self._grasp_contact_cube_kws
        )
        return arm_cube or cube_arm

    def _cb_joint_states(self, msg):
        try:
            i = msg.name.index(self.gripper_joint_name)
            self._gripper_joint_pos = float(msg.position[i])
            if msg.effort and len(msg.effort) == len(msg.name):
                self._gripper_joint_effort = float(msg.effort[i])
            else:
                self._gripper_joint_effort = None
        except ValueError:
            pass

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

    def offboard_yaw_cmd(self):
        """offboard 期望偏航：可锁 yaw0 抑制漂移，否则跟随当前估计。"""
        off = float(self.mission_yaw_offset_rad)
        if self.lock_mission_yaw:
            return float(self.yaw0) + off
        return float(self.yaw_from_pose(self.pose)) + off

    def _forward_offboard_xy(self):
        """前飞段：水平设定点沿 (x0,y0)→(x_fwd,y_fwd) 时间斜坡；近终点则直接锁终点。"""
        if self.phase != self.PH_FWD:
            return self.x_fwd, self.y_fwd
        if self.forward_ramp_sec <= 1e-6:
            return self.x_fwd, self.y_fwd
        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        # 略大锁定半径：避免「机体已接近前飞点，斜坡设定点仍落在后方」→ PX4 往回拽、振荡
        snap_xy = max(self.wp_tol_xy * 2.5, 0.32)
        if math.hypot(px - self.x_fwd, py - self.y_fwd) < snap_xy:
            return self.x_fwd, self.y_fwd
        # 与 t_phase0 一致使用仿真时钟，避免 use_sim_time 下墙钟漂移导致斜坡与 PX4 不同步
        t_a = min(1.0, (rospy.get_time() - self.t_phase0) / self.forward_ramp_sec)
        xr = self._fwd_ramp_x0 + t_a * (self._fwd_ramp_x1 - self._fwd_ramp_x0)
        yr = self._fwd_ramp_y0 + t_a * (self._fwd_ramp_y1 - self._fwd_ramp_y0)
        return xr, yr

    def get_offboard_sp_tuple(self):
        y_cmd = self.offboard_yaw_cmd()
        p = self.phase
        if p == self.PH_DONE:
            return None
        if p in (
            self.PH_HOVER_1M,
            self.PH_ARM_READY,
            self.PH_ARM_SETTLE,
        ):
            return self.x0, self.y0, self.z_high, y_cmd
        if p == self.PH_FWD:
            fx, fy = self._forward_offboard_xy()
            return fx, fy, self.z_high, y_cmd
        if p == self.PH_FWD_SETTLE:
            return self.x_fwd, self.y_fwd, self.z_high, y_cmd
        if p in (
            self.PH_DESCEND,
            self.PH_LOW_HOLD,
            self.PH_DETECT,
            self.PH_GRASP,
            self.PH_ARM_HOME,
        ):
            return self.x_fwd, self.y_fwd, self.z_sp, y_cmd
        if p in (self.PH_CLIMB, self.PH_CLIMB_SETTLE):
            return self.x_fwd, self.y_fwd, self.z_high, y_cmd
        if p in (self.PH_RTL, self.PH_LAND):
            return self.x0, self.y0, self.z_high, float(self.yaw0) + float(
                self.mission_yaw_offset_rad
            )
        return self.x0, self.y0, self.z_high, y_cmd

    def wait_move_group(self, timeout=120.0):
        svc = rospy.get_param("~move_group_ready_service", "/move_group/get_loggers")
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            try:
                rospy.wait_for_service(svc, timeout=1.0)
                rospy.loginfo(
                    "[MOVEIT] move_group 就绪 %s | EN: move_group ready",
                    svc,
                )
                return True
            except rospy.ROSException:
                rospy.loginfo_throttle(
                    5.0,
                    "[MOVEIT] 等待 move_group 服务 | EN: waiting for move_group",
                )
        return False

    def _maybe_fix_trajectory_execution_dynreconf(
        self, reason="", exec_velocity_scaling=None
    ):
        """对抗 RViz / dynreconf 默认值：关闭执行时长监控；execution_velocity_scaling 可控（过大易 effort 振荡）。"""
        if not self.vision_fix_trajectory_execution_via_dynreconf:
            return
        try:
            from dynamic_reconfigure.client import Client
        except ImportError:
            if not self._dynreconf_warned:
                rospy.logwarn(
                    "[MOVEIT] 无 dynamic_reconfigure，跳过轨迹 dyn | EN: skip traj dynreconf"
                )
                self._dynreconf_warned = True
            return
        ev_src = (
            exec_velocity_scaling
            if exec_velocity_scaling is not None
            else self.trajectory_execution_velocity_scaling
        )
        ev = max(0.1, min(10.0, float(ev_src)))
        try:
            cli = Client(self.move_group_trajectory_execution_ns, timeout=4.0)
            cli.update_configuration(
                {
                    "execution_duration_monitoring": False,
                    "allowed_execution_duration_scaling": 10.0,
                    "allowed_goal_duration_margin": 30.0,
                    "execution_velocity_scaling": ev,
                }
            )
            self._last_traj_dynreconf_wall = time.time()
            if reason:
                if reason == "exec_arm_joints":
                    rospy.loginfo_throttle(
                        25.0,
                        "[MOVEIT] dynreconf %s exec_vel=%.2f (%s) | EN: traj exec scaling",
                        self.move_group_trajectory_execution_ns,
                        ev,
                        reason,
                    )
                else:
                    rospy.loginfo(
                        "[MOVEIT] dynreconf %s exec_vel=%.2f (%s) | EN: traj exec scaling",
                        self.move_group_trajectory_execution_ns,
                        ev,
                        reason,
                    )
            else:
                rospy.loginfo_once(
                    "[MOVEIT] dynreconf %s exec_vel=%.2f | EN: traj exec scaling",
                    self.move_group_trajectory_execution_ns,
                    ev,
                )
        except Exception as e:
            rospy.logwarn_throttle(
                30.0,
                "[MOVEIT] dynreconf 失败 %s: %s | EN: dynreconf failed",
                self.move_group_trajectory_execution_ns,
                e,
            )

    @staticmethod
    def _moveit_error_name(val):
        try:
            from moveit_msgs.msg import MoveItErrorCodes as _M

            for name in (
                "SUCCESS",
                "FAILURE",
                "PLANNING_FAILED",
                "INVALID_MOTION_PLAN",
                "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                "CONTROL_FAILED",
                "UNABLE_TO_AQUIRE_SENSOR_DATA",
                "TIMED_OUT",
                "PREEMPTED",
                "START_STATE_IN_COLLISION",
                "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                "GOAL_IN_COLLISION",
                "GOAL_VIOLATES_PATH_CONSTRAINTS",
                "INVALID_GROUP_NAME",
                "INVALID_GOAL_CONSTRAINTS",
                "INVALID_ROBOT_STATE",
                "INVALID_LINK_NAME",
                "INVALID_OBJECT_NAME",
                "FRAME_TRANSFORM_FAILURE",
                "COLLISION_CHECKING_UNAVAILABLE",
                "ROBOT_STATE_STALE",
                "SENSOR_INFO_STALE",
                "NO_IK_SOLUTION",
            ):
                if val == getattr(_M, name):
                    return name
        except Exception:
            pass
        return "UNKNOWN(%s)" % (val,)

    def _log_plan_probe(self, arm, pt, label):
        """仅规划不执行，用于区分不可达/碰撞/无效约束与纯执行超时。"""
        if not self.vision_log_plan_on_fail:
            return
        try:
            from moveit_msgs.msg import MoveItErrorCodes as _M
        except ImportError:
            rospy.logwarn(
                "[PROBE] 无 moveit_msgs，跳过探针 | EN: moveit_msgs missing, skip probe"
            )
            return
        try:
            try:
                arm.stop()
            except Exception:
                pass
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            rospy.sleep(max(0.05, float(self.vision_plan_probe_settle_sec)))
            self.configure_arm_move_group(
                arm,
                vel_scaling=max(self.vision_min_velocity_scale, 0.22),
                accel_scaling=max(self.vision_min_velocity_scale, 0.22),
            )
            self._apply_vision_execution_tolerances(arm)
            if self.vision_cartesian_use_pose:
                arm.set_pose_target(self._current_ee_pose_at_xyz(arm, pt))
            else:
                arm.set_position_target(pt)
            ok, traj, ptime, err = arm.plan()
            npt = 0
            try:
                if traj and traj.joint_trajectory.points:
                    npt = len(traj.joint_trajectory.points)
            except Exception:
                pass
            hint = ""
            if ok:
                hint = "exec_timeout?"
            elif err.val == _M.NO_IK_SOLUTION:
                hint = "NO_IK"
            elif err.val in (_M.START_STATE_IN_COLLISION, _M.GOAL_IN_COLLISION):
                hint = "collision"
            elif err.val == _M.PLANNING_FAILED:
                hint = "PLAN_FAIL"
            elif err.val == _M.INVALID_GOAL_CONSTRAINTS:
                hint = "BAD_CONSTRAINTS"
            elif err.val == _M.TIMED_OUT:
                hint = "TIMED_OUT"
            rospy.logwarn(
                "[PROBE] 「%s」 ok=%s t=%.2fs err=%s wp=%d (%s) | EN: plan probe",
                label,
                ok,
                float(ptime),
                self._moveit_error_name(err.val),
                npt,
                hint,
            )
        except Exception as e:
            rospy.logwarn(
                "[PROBE] 「%s」异常: %s | EN: probe exception",
                label,
                e,
            )
        finally:
            try:
                arm.clear_pose_targets()
            except Exception:
                pass

    def _vision_plan_execute_at_xyz(self, arm, px, py, pz, label, base_vs):
        """5DOF：仅位置 set_position_target → plan/execute，带速度衰减重试。"""
        v0 = max(0.08, min(0.95, float(base_vs)))
        decay = max(0.12, float(self.vision_5dof_execute_scale_decay))
        retries = max(1, int(self.vision_5dof_execute_max_retries))
        for att in range(retries):
            scale = max(0.06, decay ** att)
            vs = max(0.06, min(1.0, v0 * scale))
            arm.set_max_velocity_scaling_factor(vs)
            arm.set_max_acceleration_scaling_factor(min(1.0, v0 * scale))
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                arm.set_start_state_to_current_state()
            except Exception:
                pass
            arm.set_position_target([float(px), float(py), float(pz)])
            err = None
            try:
                ok_plan, traj, ptime, err = arm.plan()
            except MoveItCommanderException as e:
                rospy.logwarn(
                    "[MOVEIT] %s plan 异常: %s | EN: plan exception",
                    label,
                    e,
                )
                ok_plan = False
                traj = None
                ptime = 0.0
            npts = 0
            if traj is not None and hasattr(traj, "joint_trajectory"):
                npts = len(traj.joint_trajectory.points)
            if not ok_plan or traj is None or npts < 1:
                rospy.logwarn(
                    "[MOVEIT] %s 规划失败 att=%d err=%s pts=%d | EN: plan failed",
                    label,
                    att + 1,
                    self._moveit_error_name(getattr(err, "val", -1))
                    if err is not None
                    else "?",
                    npts,
                )
                continue
            try:
                ex_ok = arm.execute(traj, wait=True)
            except MoveItCommanderException as e:
                rospy.logwarn(
                    "[MOVEIT] %s execute 异常: %s | EN: execute exception",
                    label,
                    e,
                )
                ex_ok = False
            if ex_ok:
                return True
            # 执行失败后关节停在半途；不 stop+等待则下一段轨迹首点与真机差过大 → Invalid Trajectory / CONTROL_FAILED
            try:
                arm.stop()
            except Exception:
                pass
            rospy.sleep(max(0.35, float(getattr(self, "vision_plan_probe_settle_sec", 0.35)) * 0.9))
        return False

    def _vision_plan_execute_position_z_backoff(self, arm, pt, label, base_vs):
        px, py = float(pt[0]), float(pt[1])
        pzn = float(pt[2])
        for zb in range(max(1, self.vision_5dof_z_backoff_steps + 1)):
            pz = pzn - zb * float(self.vision_5dof_z_backoff_dz)
            lab = label if zb == 0 else "%s Z=%.3f" % (label, pz)
            if self._vision_plan_execute_at_xyz(arm, px, py, pz, lab, base_vs):
                if zb > 0:
                    rospy.loginfo(
                        "[5DOF] TCP Z 回退成功 z_nom=%.3f→z=%.3f | EN: Z backoff OK",
                        pzn,
                        pz,
                    )
                return True
            if zb < int(self.vision_5dof_z_backoff_steps):
                rospy.loginfo(
                    "%s: 下一尝试将 TCP Z 降低 %.3f m",
                    label,
                    float(self.vision_5dof_z_backoff_dz),
                )
        return False

    def _go_vision_polyline_to_fin_with_xy(self, arm, p_pre, p_fin, vs0):
        offs = self.vision_grasp_xy_retry_offsets or [[0.0, 0.0]]
        for off in offs:
            if len(off) < 2:
                continue
            dx, dy = float(off[0]), float(off[1])
            pf = [float(p_fin[0]) + dx, float(p_fin[1]) + dy, float(p_fin[2])]
            sub = (
                "下落"
                if (abs(dx) < 1e-9 and abs(dy) < 1e-9)
                else "下落 Δxy(%+.4f,%+.4f)" % (dx, dy)
            )
            if self._go_vision_polyline_with_leadin(arm, p_pre, pf, sub, vs0):
                return True
        return False

    def _vision_motion_leg_fin_with_xy(self, arm, p_pre, p_fin, vs0):
        offs = self.vision_grasp_xy_retry_offsets or [[0.0, 0.0]]
        for off in offs:
            if len(off) < 2:
                continue
            dx, dy = float(off[0]), float(off[1])
            pf = [float(p_fin[0]) + dx, float(p_fin[1]) + dy, float(p_fin[2])]
            sub = (
                "下落"
                if (abs(dx) < 1e-9 and abs(dy) < 1e-9)
                else "下落 Δxy(%+.4f,%+.4f)" % (dx, dy)
            )
            if self._vision_motion_leg(arm, p_pre, pf, sub, vs0):
                return True
        return False

    def _go_vision_fin_waypoint_with_xy(self, arm, p_fin, vs0):
        offs = self.vision_grasp_xy_retry_offsets or [[0.0, 0.0]]
        for off in offs:
            if len(off) < 2:
                continue
            dx, dy = float(off[0]), float(off[1])
            pf = [float(p_fin[0]) + dx, float(p_fin[1]) + dy, float(p_fin[2])]
            sub = (
                "下落"
                if (abs(dx) < 1e-9 and abs(dy) < 1e-9)
                else "下落 Δxy(%+.4f,%+.4f)" % (dx, dy)
            )
            if self._go_vision_waypoint(arm, pf, sub, vs0):
                return True
        return False

    def _vision_preflight_plan_ok(self, arm, pt, label):
        """仅 plan() 不 execute；用于预检预接近/下落点在 planning_frame 下是否可得规划。"""
        try:
            try:
                arm.stop()
            except Exception:
                pass
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            rospy.sleep(max(0.04, float(self.vision_plan_probe_settle_sec) * 0.45))
            self.configure_arm_move_group(
                arm,
                vel_scaling=max(
                    self.vision_min_velocity_scale,
                    0.18,
                    float(self.vision_ompl_plan_vel_scale),
                ),
                accel_scaling=max(
                    self.vision_min_velocity_scale,
                    0.18,
                    float(self.vision_ompl_plan_accel_scale),
                ),
            )
            arm.set_planning_time(
                max(3.0, float(self.vision_preflight_planning_time))
            )
            arm.set_num_planning_attempts(
                max(1, int(self.vision_preflight_planning_attempts))
            )
            self._apply_vision_execution_tolerances(arm)
            self._apply_preflight_relaxed_tolerances(arm)
            # 多策略：仅位置 → 接近轴 Pose → 保留腕 Pose（减轻「Unable to sample goal」）
            attempts = []
            if self.vision_cartesian_use_pose:
                attempts.append(
                    ("pose_retained", lambda: arm.set_pose_target(self._current_ee_pose_at_xyz(arm, pt)))
                )
            else:
                attempts.append(
                    ("position", lambda: arm.set_position_target(pt))
                )
                if self.vision_preflight_retry_pose_from_approach:
                    attempts.append(
                        (
                            "pose_approach_axis",
                            lambda: arm.set_pose_target(
                                self._preflight_goal_pose_approach(arm, pt)
                            ),
                        )
                    )
                if self.vision_preflight_retry_pose_retaining_wrist:
                    attempts.append(
                        (
                            "pose_retained",
                            lambda: arm.set_pose_target(
                                self._current_ee_pose_at_xyz(arm, pt)
                            ),
                        )
                    )
            ok = False
            last_err_name = "UNKNOWN"
            last_ptime = 0.0
            last_mode = ""
            for mode_tag, apply_goal in attempts:
                try:
                    arm.clear_pose_targets()
                except MoveItCommanderException:
                    pass
                if (
                    mode_tag == "position"
                    and self.vision_5dof_use_plan_execute
                    and not self.vision_cartesian_use_pose
                ):
                    px, py, pzn = float(pt[0]), float(pt[1]), float(pt[2])
                    plan_ok_any = False
                    for zb in range(max(1, self.vision_5dof_z_backoff_steps + 1)):
                        pz = pzn - zb * float(self.vision_5dof_z_backoff_dz)
                        try:
                            arm.clear_pose_targets()
                        except MoveItCommanderException:
                            pass
                        arm.set_position_target([px, py, pz])
                        err = None
                        try:
                            plan_ok, _traj, ptime, err = arm.plan()
                        except MoveItCommanderException as e:
                            rospy.logwarn(
                                "[PREFLIGHT] 「%s」Z=%.3f plan err: %s | EN: plan exception",
                                label,
                                pz,
                                e,
                            )
                            plan_ok = False
                            ptime = 0.0
                        en = (
                            self._moveit_error_name(err.val)
                            if err is not None
                            else "UNKNOWN"
                        )
                        last_err_name = en
                        last_ptime = float(ptime)
                        last_mode = "position"
                        if plan_ok:
                            plan_ok_any = True
                            ok = True
                            rospy.loginfo(
                                "[PREFLIGHT] 「%s」OK pos z_try=%d t=%.2fs %s | EN: OK",
                                label,
                                zb,
                                last_ptime,
                                en,
                            )
                            break
                        rospy.logwarn(
                            "[PREFLIGHT] 「%s」FAIL pos z_try=%d z=%.3f t=%.2fs %s | EN: fail",
                            label,
                            zb,
                            pz,
                            last_ptime,
                            en,
                        )
                    if plan_ok_any:
                        break
                    continue
                apply_goal()
                plan_ok, _traj, ptime, err = arm.plan()
                en = self._moveit_error_name(err.val)
                last_err_name = en
                last_ptime = float(ptime)
                last_mode = mode_tag
                if plan_ok:
                    ok = True
                    rospy.loginfo(
                        "[PREFLIGHT] 「%s」OK mode=%s t=%.2fs %s | EN: OK",
                        label,
                        mode_tag,
                        last_ptime,
                        en,
                    )
                    break
                rospy.logwarn(
                    "[PREFLIGHT] 「%s」FAIL mode=%s t=%.2fs %s | EN: fail",
                    label,
                    mode_tag,
                    last_ptime,
                    en,
                )
            if not ok:
                rospy.logwarn(
                    "[PREFLIGHT] 「%s」最终失败 err=%s mode=%s | EN: preflight failed",
                    label,
                    last_err_name,
                    last_mode,
                )
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            arm.set_planning_time(self.arm_planning_time)
            arm.set_num_planning_attempts(10)
            success_mode = last_mode if ok else ""
            return bool(ok), success_mode
        except Exception as ex:
            rospy.logwarn(
                "[PREFLIGHT] 「%s」异常: %s | EN: preflight exception",
                label,
                ex,
            )
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                arm.set_planning_time(self.arm_planning_time)
                arm.set_num_planning_attempts(10)
            except Exception:
                pass
            return False, ""

    def configure_arm_move_group(self, arm, vel_scaling=None, accel_scaling=None):
        vs = self.arm_vel_scaling if vel_scaling is None else vel_scaling
        ac = self.arm_accel_scaling if accel_scaling is None else accel_scaling
        arm.set_planning_time(self.arm_planning_time)
        arm.set_num_planning_attempts(10)
        arm.set_max_velocity_scaling_factor(vs)
        arm.set_max_acceleration_scaling_factor(ac)
        arm.set_goal_joint_tolerance(self.arm_goal_joint_tolerance)

    def ensure_moveit_cpp(self):
        if not self._moveit_cpp_inited:
            moveit_commander.roscpp_initialize(sys.argv)
            self._moveit_cpp_inited = True

    def _create_move_group(self, group_name):
        last_err = None
        for _ in range(2):
            try:
                return MoveGroupCommander(
                    group_name, wait_for_servers=self.move_group_wait_for_servers
                )
            except RuntimeError as e:
                last_err = e
                rospy.sleep(2.0)
        raise last_err

    def get_arm_group(self):
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
            rospy.logerr(
                "[MOVEIT] arm MoveGroupCommander: %s | EN: init failed",
                e,
            )
            return None
        try:
            self._arm_mgc.set_pose_reference_frame(self.ref_frame)
        except Exception:
            pass
        try:
            self._arm_mgc.set_end_effector_link(self.ee_tcp_frame)
        except Exception:
            pass
        self._maybe_fix_trajectory_execution_dynreconf(
            reason="after_arm_MGC_ready"
        )
        return self._arm_mgc

    def get_gripper_group(self):
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
            rospy.logerr(
                "[MOVEIT] gripper MoveGroupCommander: %s | EN: init failed",
                e,
            )
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
        if self._img is None or self._depth is None:
            return u, v
        hi, wi = self._img.shape[:2]
        hd, wd = self._depth.shape[:2]
        if hi == hd and wi == wd:
            return u, v
        u2 = int(round(u * wd / float(wi)))
        v2 = int(round(v * hd / float(hi)))
        return max(0, min(wd - 1, u2)), max(0, min(hd - 1, v2))

    def depth_median(self, u, v, r=None):
        if self._depth is None:
            return None
        if r is None:
            r = self.depth_median_radius
        u, v = self._uv_in_depth(u, v)
        h, w = self._depth.shape[:2]
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))
        patch = self._depth[v - r : v + r + 1, u - r : u + r + 1]
        zmax = min(15.0, self.camera_depth_max_m)
        vals = patch[np.isfinite(patch) & (patch > 0.05) & (patch < zmax)]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def point_cam(self, u, v):
        z = min(self.assume_depth_m, self.camera_depth_max_m)
        if self.use_depth:
            zd = self.depth_median(u, v)
            if zd is not None:
                z = min(zd, self.camera_depth_max_m)
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
        if not self.detect_refine_xy_enable or self._K is None:
            return
        if do_transform_vector3 is None:
            return
        zd = self.depth_median(u, v)
        if zd is None or zd < 0.08 or zd > 14.0:
            zd = float(self.assume_depth_m)
        fx, fy = float(self._K[0, 0]), float(self._K[1, 1])
        ppx, ppy = float(self._K[0, 2]), float(self._K[1, 2])
        du = float(u) - ppx
        dv = float(v) - ppy
        epx = math.hypot(du, dv)
        if epx < float(self.detect_refine_min_pixel_error):
            return
        g = self.detect_refine_xy_gain
        vx = g * du / fx * zd
        vy = g * dv / fy * zd
        tgt = (self.detect_refine_xy_target_frame or "map").strip()
        out = transform_vector(
            self.tf_buffer, Vector3(vx, vy, 0.0), tgt, self.camera_optical_frame
        )
        if out is None and self.tf_refine_target_frame:
            out = transform_vector(
                self.tf_buffer,
                Vector3(vx, vy, 0.0),
                self.tf_refine_target_frame,
                self.camera_optical_frame,
            )
        if out is None:
            out = transform_vector(
                self.tf_buffer,
                Vector3(vx, vy, 0.0),
                self.ref_frame,
                self.camera_optical_frame,
            )
        if out is None:
            rospy.logwarn_throttle(
                8.0,
                "refine_xy: TF(%s/roarm_base_link)<-%s 失败，跳过侧向微调",
                tgt,
                self.camera_optical_frame,
            )
            return
        dx, dy = float(out.x), float(out.y)
        step = math.hypot(dx, dy)
        mxs = self.detect_refine_xy_max_step
        if step > mxs and step > 1e-9:
            s = mxs / step
            dx *= s
            dy *= s
            step = mxs
        cap = float(self.detect_refine_max_cumulative_m)
        cum = float(getattr(self, "_refine_xy_cumulative_m", 0.0))
        if cap > 1e-6 and cum + step > cap:
            allow = max(0.0, cap - cum)
            if allow < 1e-6:
                return
            if step > 1e-9:
                fac = allow / step
                dx *= fac
                dy *= fac
                step = allow
        self._refine_xy_cumulative_m = cum + step
        self.x_fwd += dx
        self.y_fwd += dy

    def vision_xyz_plausible(self, xyz):
        """桌面蓝块在 roarm_base_link 下 z 通常远小于 1m；超界多为深度/背景混入。"""
        if xyz is None or len(xyz) != 3:
            return False
        z = float(xyz[2])
        if z < self.vision_reframe_z_min or z > self.vision_reframe_z_max:
            rospy.logerr(
                "[VISION] 蓝块 z=%.3f 超出 [%.2f,%.2f]，丢弃 | EN: blob z out of range, drop",
                z,
                self.vision_reframe_z_min,
                self.vision_reframe_z_max,
            )
            return False
        if self.vision_workspace_enable:
            x, y = float(xyz[0]), float(xyz[1])
            rv = math.sqrt(x * x + y * y + z * z)
            rw = float(self.vision_workspace_radius_max_m)
            if rv > rw + 0.08:
                rospy.logwarn(
                    "[VISION] 质心距臂座 %.3f m > r_max=%.2f，将径向夹紧 | EN: clamp to workspace sphere",
                    rv,
                    rw,
                )
        return True

    def _workspace_adjust_grasp_targets(self, p_pre, p_fin):
        """
        roarm_base_link 原点近似臂座：将预接近/下落点限制在可达球内。
        两段使用同一径向缩放，保留 vision_*_delta 的相对位移，利于短臂分段 OMPL。
        """
        if not self.vision_workspace_enable:
            return True, list(p_pre), list(p_fin), ""
        pp = np.asarray(p_pre, dtype=np.float64).reshape(3)
        pf = np.asarray(p_fin, dtype=np.float64).reshape(3)
        r_eff = float(self.vision_workspace_radius_max_m) - float(
            self.vision_workspace_safety_margin_m
        )
        r_eff = max(0.06, r_eff)
        r_min = max(0.0, float(self.vision_workspace_radius_min_m))
        nf = float(np.linalg.norm(pf))
        np_ = float(np.linalg.norm(pp))
        if r_min > 1e-6 and nf < r_min:
            return (
                False,
                list(p_pre),
                list(p_fin),
                "下落目标距臂座过近 norm=%.3f m < vision_workspace_radius_min_m=%.3f"
                % (nf, r_min),
            )
        rlim = max(np_, nf)
        if rlim <= r_eff + 1e-9:
            return True, pp.tolist(), pf.tolist(), ""
        if self.vision_workspace_reject_instead_of_clamp:
            return (
                False,
                list(p_pre),
                list(p_fin),
                "末端目标超出球形工作区：max(norm)=%.3f m > 有效半径 %.3f m"
                % (rlim, r_eff),
            )
        s = r_eff / max(rlim, 1e-9)
        rospy.logwarn(
            "[WS] 工作区径向缩放 s=%.4f (||p|| %.3f→%.3f m) | EN: radial clamp, keep pre/fin colinear",
            s,
            rlim,
            r_eff,
        )
        return True, (pp * s).tolist(), (pf * s).tolist(), ""

    def _adjust_grasp_targets_from_base(self, base):
        """
        由规划系下的蓝块质心估计 base（长度 3）生成预接近/下落点，与工作区夹紧及 z 余量一致。
        """
        base = np.asarray(base, dtype=np.float64).reshape(3)
        off = np.array(self.vision_target_offset, dtype=np.float64)
        d0 = np.array(self.vision_pregrasp_delta, dtype=np.float64)
        d1 = np.array(self.vision_grasp_delta, dtype=np.float64)
        p_pre = (base + off + d0).tolist()
        p_fin = (base + off + d0 + d1).tolist()
        ws_ok, p_pre, p_fin, ws_note = self._workspace_adjust_grasp_targets(
            p_pre, p_fin
        )
        ez = float(self.vision_tcp_extra_z_m)
        if abs(ez) > 1e-9:
            p_pre[2] = float(p_pre[2]) + ez
            p_fin[2] = float(p_fin[2]) + ez
        pl = float(self.vision_grasp_plunge_extra_z_m)
        if abs(pl) > 1e-9:
            p_fin[2] = float(p_fin[2]) + pl
        return ws_ok, p_pre, p_fin, ws_note

    def _tcp_orient_ok_for_commit(self, arm):
        """抓取触发：末端 EE z 轴（TF）与期望接近方向的夹角判据。"""
        tol = float(self.grasp_commit_orient_tol_rad)
        if tol <= 1e-9:
            return True
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.ref_frame,
                self.ee_tcp_frame,
                rospy.Time(0),
                rospy.Duration(0.35),
            )
            tr = tfm.transform.rotation
            if _tft is not None:
                R = _tft.quaternion_matrix(
                    [tr.x, tr.y, tr.z, tr.w]
                )[:3, :3]
            else:
                x, y, z, w = tr.x, tr.y, tr.z, tr.w
                R = np.array(
                    [
                        [
                            1 - 2 * (y * y + z * z),
                            2 * (x * y - z * w),
                            2 * (x * z + y * w),
                        ],
                        [
                            2 * (x * y + z * w),
                            1 - 2 * (x * x + z * z),
                            2 * (y * z - x * w),
                        ],
                        [
                            2 * (x * z - y * w),
                            2 * (y * z + x * w),
                            1 - 2 * (x * x + y * y),
                        ],
                    ],
                    dtype=np.float64,
                )
            ez = R[:, 2]
            dn = self._grasp_commit_desired_axis
            ez = ez / max(1e-9, float(np.linalg.norm(ez)))
            ang = math.acos(max(-1.0, min(1.0, float(np.dot(ez, dn)))))
            return ang <= tol
        except Exception:
            return True

    def _grasp_commit_gate(self, arm, tgt_xyz):
        """抓取触发区：位置 + 姿态 + 可选像素全部满足。"""
        cur = self.get_tcp_xyz_in_ref_frame(arm)
        if cur is None:
            return False
        d = float(
            np.linalg.norm(
                np.asarray(tgt_xyz, dtype=np.float64)
                - np.asarray(cur, dtype=np.float64)
            )
        )
        if d > float(self.grasp_commit_pos_tol_m):
            return False
        if not self._tcp_orient_ok_for_commit(arm):
            return False
        px_tol = float(self.grasp_commit_pixel_tol_px)
        if px_tol > 1e-9:
            px_err = self._tcp_pixel_error_vs_last_detection(arm)
            if px_err is None or px_err > px_tol:
                return False
        return True

    def _grasp_finishing_refine(self, arm, vs0):
        """
        闭合前短 REFINE（对应流程 SEARCH→…→REFINE→GRASP_COMMIT 中的 REFINE 段）：
        刷新深度并小步修正；由 grasp_commit_* 做触发区+连续帧+超时；碰撞 bumper 可打断。
        """
        if not self.grasp_fin_refine_enable:
            return True
        if (
            self.grasp_contact_trigger_enable
            and getattr(self, "_grasp_contact_seen", False)
        ):
            rospy.loginfo(
                "[REFINE] 爪-蓝块已碰撞→跳过闭合前微调 | EN: skip refine on contact"
            )
            return True
        vmin = max(0.08, float(self.vision_min_velocity_scale))
        vs_ref = max(
            vmin,
            min(float(self.vision_waypoint_vel_cap), float(self.grasp_fin_refine_vel_scale)),
        )
        last_fin = None
        if self._last_exec_p_fin is not None and len(self._last_exec_p_fin) == 3:
            last_fin = np.array(self._last_exec_p_fin, dtype=np.float64)

        refine_t0 = rospy.get_time()
        self._refine_stable_ctr = 0
        self._refine_last_tgt = None

        for k in range(max(0, int(self.grasp_fin_refine_iters))):
            if (
                self.grasp_contact_trigger_enable
                and getattr(self, "_grasp_contact_seen", False)
            ):
                rospy.loginfo(
                    "[REFINE] 微调中检测到碰撞→结束微调 | EN: refine stop on contact"
                )
                return True
            if bool(getattr(self, "grasp_commit_enable", False)):
                tout = float(self.grasp_commit_refine_timeout_sec)
                if tout > 1e-6 and (rospy.get_time() - refine_t0) >= tout:
                    rospy.logwarn(
                        "[GRASP_COMMIT] refine %.1fs 超时→锁定最近目标并闭爪 | EN: timeout commit",
                        tout,
                    )
                    if self._refine_last_tgt is not None:
                        self._last_exec_p_fin = list(self._refine_last_tgt)
                    else:
                        cur_to = self.get_tcp_xyz_in_ref_frame(arm)
                        if cur_to is not None:
                            self._last_exec_p_fin = [
                                float(cur_to[0]),
                                float(cur_to[1]),
                                float(cur_to[2]),
                            ]
                    return True
            rospy.sleep(max(0.0, float(self.grasp_fin_refine_settle_sec)))
            uv = self.detect_blue_uv()
            if uv is None:
                rospy.logwarn_throttle(
                    4.0,
                    "[REFINE] 闭爪前微调 %d: 无蓝块 | EN: no blob in frame",
                    k + 1,
                )
                continue
            pc = self.point_cam(uv[0], uv[1])
            if pc is None:
                continue
            pb = transform_point(
                self.tf_buffer,
                pc,
                self.ref_frame,
                self.camera_optical_frame,
            )
            if pb is None:
                continue
            raw = (pb.x, pb.y, pb.z)
            if not self.vision_xyz_plausible(raw):
                continue
            ws_ok, _p_pre, p_fin, _note = self._adjust_grasp_targets_from_base(raw)
            if not ws_ok:
                continue
            tgt = np.array([float(p_fin[0]), float(p_fin[1]), float(p_fin[2])], dtype=np.float64)
            self._refine_last_tgt = [
                float(tgt[0]),
                float(tgt[1]),
                float(tgt[2]),
            ]

            if bool(getattr(self, "grasp_commit_enable", False)):
                gate = self._grasp_commit_gate(arm, tgt)
                nf = max(0, int(self.grasp_commit_stable_frames))
                need = nf if nf > 0 else 1
                if gate:
                    self._refine_stable_ctr += 1
                    rospy.loginfo_throttle(
                        0.75,
                        "[GRASP_COMMIT] stable %d/%d gate=pos+orient(+px?) | EN: zone",
                        self._refine_stable_ctr,
                        need,
                    )
                else:
                    self._refine_stable_ctr = 0
                if self._refine_stable_ctr >= need:
                    rospy.loginfo(
                        "[GRASP_COMMIT] 冻结目标(%.3f,%.3f,%.3f)→停止微调并闭爪 | EN: freeze",
                        tgt[0],
                        tgt[1],
                        tgt[2],
                    )
                    self._last_exec_p_fin = [
                        float(tgt[0]),
                        float(tgt[1]),
                        float(tgt[2]),
                    ]
                    return True
            else:
                cur_es = self.get_tcp_xyz_in_ref_frame(arm)
                es_m = float(self.grasp_fin_refine_early_stop_tcp_m)
                if cur_es is not None and es_m > 1e-9:
                    d_es = float(
                        np.linalg.norm(
                            tgt - np.asarray(cur_es, dtype=np.float64)
                        )
                    )
                    if d_es <= es_m:
                        rospy.loginfo(
                            "[REFINE] 微调 %d: TCP 距刷新下落点 %.4f m≤%.4f→提前闭爪 | EN: early stop tcp",
                            k + 1,
                            d_es,
                            es_m,
                        )
                        self._last_exec_p_fin = [
                            float(tgt[0]),
                            float(tgt[1]),
                            float(tgt[2]),
                        ]
                        return True
                es_px = float(self.grasp_fin_refine_early_stop_pixel_px)
                if es_px > 1e-9:
                    px_err = self._tcp_pixel_error_vs_last_detection(arm)
                    if px_err is not None and px_err <= es_px:
                        rospy.loginfo(
                            "[REFINE] 微调 %d: 像素偏差 %.1f≤%.1f→提前闭爪 | EN: early stop px",
                            k + 1,
                            px_err,
                            es_px,
                        )
                        self._last_exec_p_fin = [
                            float(tgt[0]),
                            float(tgt[1]),
                            float(tgt[2]),
                        ]
                        return True

            if last_fin is not None:
                delta = tgt - last_fin
                dn = float(np.linalg.norm(delta))
                if dn < float(self.grasp_fin_refine_min_disp_m):
                    rospy.loginfo(
                        "[REFINE] 微调 %d: |Δ|=%.4f 过小跳过 | EN: skip small delta",
                        k + 1,
                        dn,
                    )
                    continue
                mx = float(self.grasp_fin_refine_max_step_m)
                if dn > mx:
                    tgt = last_fin + delta / dn * mx
                    rospy.loginfo(
                        "[REFINE] 微调 %d: 限幅 %.4f→%.4f m | EN: delta clamped",
                        k + 1,
                        dn,
                        mx,
                    )

            move_goal = [float(tgt[0]), float(tgt[1]), float(tgt[2])]
            cur = self.get_tcp_xyz_in_ref_frame(arm)
            if cur is None:
                continue
            if float(np.linalg.norm(np.asarray(move_goal, dtype=np.float64) - cur)) < float(
                self.grasp_fin_refine_min_disp_m
            ):
                rospy.loginfo(
                    "[REFINE] 微调 %d: TCP 已到位 | EN: already in tolerance",
                    k + 1,
                )
                self._last_exec_p_fin = list(move_goal)
                last_fin = np.asarray(move_goal, dtype=np.float64)
                continue

            self._apply_vision_execution_tolerances(arm)
            self.configure_arm_move_group(
                arm,
                vel_scaling=max(
                    float(self.grasp_fin_refine_vel_scale),
                    float(self.vision_ompl_plan_vel_scale) * 0.85,
                ),
                accel_scaling=max(
                    float(self.grasp_fin_refine_vel_scale),
                    float(self.vision_ompl_plan_accel_scale) * 0.85,
                ),
            )
            self._vision_wp_pose_hint = ""

            ok_wp = self._go_vision_waypoint(
                arm,
                move_goal,
                "闭合前微调%d" % (k + 1),
                vs_ref,
            )
            if ok_wp:
                self._last_exec_p_fin = list(move_goal)
                last_fin = np.asarray(move_goal, dtype=np.float64)
                rospy.loginfo(
                    "[REFINE] 微调 %d OK goal(%.3f,%.3f,%.3f) | EN: refine OK",
                    k + 1,
                    move_goal[0],
                    move_goal[1],
                    move_goal[2],
                )
            else:
                rospy.logwarn(
                    "[REFINE] 微调 %d MoveIt 失败 | EN: refine waypoint failed",
                    k + 1,
                )
        return True

    def print_blue_pose(self, u, v):
        """质心在相机光学系与 planning_frame 下的估计（米），供终端查看。"""
        pc = self.point_cam(u, v)
        if pc is None:
            rospy.logwarn(
                "[VISION] 无深度或 camera_info | EN: no depth / camera_info"
            )
            return None
        pb = transform_point(
            self.tf_buffer, pc, self.ref_frame, self.camera_optical_frame
        )
        if pb is not None:
            raw = (pb.x, pb.y, pb.z)
            pl = self.vision_xyz_plausible(raw)
            rospy.loginfo(
                "[VISION] 质心 %s (%.3f,%.3f,%.3f) cam_opt(%.3f,%.3f,%.3f) %s | EN: blob pose %s",
                self.ref_frame,
                pb.x,
                pb.y,
                pb.z,
                pc.x,
                pc.y,
                pc.z,
                "OK" if pl else "reject",
                "valid" if pl else "rejected",
            )
            if pl:
                return raw
            return None
        rospy.logwarn(
            "[VISION] TF %s←optical 失败 | EN: TF camera→%s failed",
            self.ref_frame,
            self.ref_frame,
        )
        return None

    def _apply_vision_execution_tolerances(self, arm):
        try:
            arm.set_goal_joint_tolerance(self.vision_relaxed_joint_tolerance)
            arm.set_goal_position_tolerance(self.vision_goal_position_tolerance)
            arm.set_goal_orientation_tolerance(self.vision_goal_orientation_tolerance)
        except Exception as e:
            rospy.logwarn(
                "[MOVEIT] 设置视觉执行容差失败: %s | EN: goal tolerance failed",
                e,
            )

    def _apply_preflight_relaxed_tolerances(self, arm):
        """预检：在 vision_* 基础上再放宽，便于 OMPL/IK 命中可行目标。"""
        try:
            pj = max(
                float(self.vision_relaxed_joint_tolerance),
                float(self.vision_preflight_relaxed_joint_tolerance),
            )
            pp = max(
                float(self.vision_goal_position_tolerance),
                float(self.vision_preflight_relaxed_position_tolerance),
            )
            po = max(
                float(self.vision_goal_orientation_tolerance),
                float(self.vision_preflight_relaxed_orientation_tolerance),
            )
            arm.set_goal_joint_tolerance(pj)
            arm.set_goal_position_tolerance(pp)
            arm.set_goal_orientation_tolerance(po)
        except Exception as e:
            rospy.logwarn(
                "[PREFLIGHT] 设置宽松容差失败: %s | EN: relaxed tolerance failed",
                e,
            )

    def _preflight_goal_pose_approach(self, arm, pt):
        """末端 +Z 对齐接近轴（与笛卡尔段一致），便于 5 自由度在约束姿态下求 IK。"""
        pose = Pose()
        pose.position.x = float(pt[0])
        pose.position.y = float(pt[1])
        pose.position.z = float(pt[2])
        cp = None
        try:
            cp = arm.get_current_pose().pose
        except Exception:
            cp = None
        fp_xyz = self.get_tcp_xyz_in_ref_frame(arm)
        if fp_xyz is not None:
            fp = [float(fp_xyz[0]), float(fp_xyz[1]), float(fp_xyz[2])]
        else:
            fp = [0.0, 0.0, float(pt[2])]
        qh = self._quat_tool_axis_toward(
            fp,
            [float(pt[0]), float(pt[1]), float(pt[2])],
            float(self.vision_cartesian_approach_axis_sign),
        )
        if qh is not None:
            pose.orientation.x = qh[0]
            pose.orientation.y = qh[1]
            pose.orientation.z = qh[2]
            pose.orientation.w = qh[3]
        elif cp is not None:
            pose.orientation = cp.orientation
        return pose

    def _reset_vision_execution_tolerances(self, arm):
        try:
            arm.set_goal_joint_tolerance(self.arm_goal_joint_tolerance)
            arm.set_goal_position_tolerance(0.001)
            arm.set_goal_orientation_tolerance(0.01)
        except Exception:
            pass

    def _current_ee_pose_at_xyz(self, arm, pt):
        cp = arm.get_current_pose()
        pose = copy.deepcopy(cp.pose)
        pose.position.x = float(pt[0])
        pose.position.y = float(pt[1])
        pose.position.z = float(pt[2])
        return pose

    @staticmethod
    def _interp_line_xyz(p0, p1, n_seg):
        if n_seg < 1:
            n_seg = 1
        a = [float(p0[0]), float(p0[1]), float(p0[2])]
        b = [float(p1[0]), float(p1[1]), float(p1[2])]
        out = []
        for i in range(1, n_seg + 1):
            t = i / float(n_seg)
            out.append(
                [
                    a[0] + (b[0] - a[0]) * t,
                    a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t,
                ]
            )
        return out

    def _vision_num_segments(self, p0, p1):
        a = np.array(p0, dtype=np.float64)
        b = np.array(p1, dtype=np.float64)
        dist = float(np.linalg.norm(b - a))
        step = max(0.008, float(self.vision_cartesian_step_m))
        d_co = float(self.vision_polyline_coalesce_dist_m)
        if d_co > 0.0 and dist > d_co:
            step = max(
                step,
                max(0.012, float(self.vision_polyline_step_m_long)),
            )
        n = max(int(self.vision_segment_min_steps), int(math.ceil(dist / step)))
        n = max(1, min(int(self.vision_segment_max_steps), n))
        d_lg = float(self.vision_polyline_long_leg_compress_m)
        if d_lg > 0.0 and dist > d_lg:
            step_floor = max(step, float(self.vision_polyline_long_leg_min_step_m))
            n_cap = max(
                int(self.vision_segment_min_steps),
                int(math.ceil(dist / step_floor)),
            )
            n = min(n, n_cap)
        return max(1, min(int(self.vision_segment_max_steps), n))

    def _polyline_lead_in_waypoints(self, p0, p1):
        """长跨距时在主折线前插入沿 p0→p1 的短步路点，减轻首段 arm.go 名义时长。"""
        if not self.vision_polyline_lead_in_enable:
            return []
        a = np.array(
            [float(p0[0]), float(p0[1]), float(p0[2])], dtype=np.float64
        )
        b = np.array(
            [float(p1[0]), float(p1[1]), float(p1[2])], dtype=np.float64
        )
        v = b - a
        dist = float(np.linalg.norm(v))
        if dist < float(self.vision_polyline_lead_in_min_leg_m) or dist < 1e-6:
            return []
        step = max(0.008, float(self.vision_polyline_lead_in_step_m))
        cap = min(
            float(self.vision_polyline_lead_in_cap_m),
            dist * max(0.05, min(0.5, float(self.vision_polyline_lead_in_max_frac))),
        )
        u = v / dist
        out = []
        s = step
        mx = max(1, int(self.vision_polyline_lead_in_max_points))
        while len(out) < mx and s < dist - 1e-6 and s <= cap + 1e-9:
            out.append((a + u * s).tolist())
            s += step
        return out

    def _go_vision_joint_lead_in_toward_seed(self, arm, label, vs0):
        """关节空间小步走向 vision_reach_seed_joints（或 pregrasp），缩短与蓝块的欧氏跨度。"""
        if not self.vision_joint_lead_in_enable:
            return True
        seed = self.vision_reach_seed_joints
        if seed is None or len(seed) != 5:
            seed = [float(x) for x in self.pregrasp_joints]
        else:
            seed = [float(x) for x in seed]
        try:
            q0 = arm.get_current_joint_values()
        except Exception as e:
            rospy.logwarn(
                "[ARM] %s 关节前插跳过: %s | EN: joint lead-in skip",
                label,
                e,
            )
            return True
        if len(q0) != 5:
            return True
        n = max(1, min(12, int(self.vision_joint_lead_in_steps)))
        frac = max(0.05, min(0.5, float(self.vision_joint_lead_in_total_fraction)))
        vmin = max(0.08, min(0.95, float(self.vision_min_velocity_scale)))
        vcap = max(0.2, min(1.0, float(self.vision_waypoint_vel_cap)))
        vs = max(vmin, min(vcap, float(vs0)))
        rospy.loginfo(
            "[ARM] %s 关节前插 n=%d →seed %.0f%% | EN: joint lead-in",
            label,
            n,
            frac * 100.0,
        )
        for i in range(1, n + 1):
            t = (i / float(n)) * frac
            qi = [float(q0[j]) + (seed[j] - float(q0[j])) * t for j in range(5)]
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                arm.set_max_velocity_scaling_factor(vs)
                arm.set_max_acceleration_scaling_factor(vs)
            except Exception:
                pass
            arm.set_joint_value_target(qi)
            lab = "%s 关节插%d/%d" % (label, i, n)
            if not arm.go(wait=True):
                rospy.logwarn(
                    "[ARM] %s 失败 | EN: joint waypoint failed",
                    lab,
                )
                return False
        return True

    def _go_vision_polyline_with_leadin(self, arm, p0, p1, label, vs0):
        p0a = np.array(
            [float(p0[0]), float(p0[1]), float(p0[2])], dtype=np.float64
        )
        p1a = np.array(
            [float(p1[0]), float(p1[1]), float(p1[2])], dtype=np.float64
        )
        dtot = float(np.linalg.norm(p1a - p0a))
        p0_list = [float(p0a[0]), float(p0a[1]), float(p0a[2])]
        joint_lead_ok = False
        if self.vision_joint_lead_in_enable and dtot >= float(
            self.vision_joint_lead_in_min_leg_m
        ):
            if not self._go_vision_joint_lead_in_toward_seed(arm, label, vs0):
                rospy.logwarn(
                    "[ARM] %s 关节前插失败，沿用原折线起点 | EN: lead-in fail, keep p0",
                    label,
                )
            else:
                joint_lead_ok = True
                cpv = self.get_tcp_xyz_in_ref_frame(arm)
                if cpv is not None:
                    p0_list = [float(cpv[0]), float(cpv[1]), float(cpv[2])]
                else:
                    rospy.logwarn(
                        "[ARM] %s 关节前插后无 TCP(TF) | EN: no TCP after lead-in",
                        label,
                    )
                    joint_lead_ok = False
        d_after = float(
            np.linalg.norm(
                np.array(p1, dtype=np.float64) - np.array(p0_list, dtype=np.float64)
            )
        )
        if joint_lead_ok:
            rospy.loginfo(
                "[ARM] %s 关节前插弦长 %.3f→%.3f m | EN: chord after lead-in",
                label,
                dtot,
                d_after,
            )
        if joint_lead_ok and self.vision_polyline_skip_cartesian_micro_after_joint:
            rospy.loginfo(
                "[POLY] %s 跳过笛卡尔微步 | EN: skip Cartesian micro-steps",
                label,
            )
            return self._go_vision_polyline(arm, p0_list, p1, label, vs0)
        prefix = self._polyline_lead_in_waypoints(p0_list, p1)
        if not prefix:
            return self._go_vision_polyline(arm, p0_list, p1, label, vs0)
        if self.vision_fix_trajectory_execution_via_dynreconf:
            self._maybe_fix_trajectory_execution_dynreconf(
                reason="vision_polyline_micro %s" % label
            )
        dtot = float(
            np.linalg.norm(
                np.array(p1, dtype=np.float64) - np.array(p0_list, dtype=np.float64)
            )
        )
        rospy.loginfo(
            "[POLY] %s micro n=%d L=%.3f step=%.3f | EN: lead-in micro waypoints",
            label,
            len(prefix),
            dtot,
            float(self.vision_polyline_lead_in_step_m),
        )
        cur = list(p0_list)
        for j, pt in enumerate(prefix):
            lab = "%s 微步%d/%d" % (label, j + 1, len(prefix))
            if not self._go_vision_waypoint(arm, pt, lab, vs0):
                return False
            cur = pt
        return self._go_vision_polyline(arm, cur, p1, label, vs0)

    def _vision_motion_leg(self, arm, p_start, p_end, label, vs0):
        """cartesian_execute：短跨度笛卡尔 chunks，失败或超长则 OMPL 折线/单点。"""
        p0 = [float(p_start[0]), float(p_start[1]), float(p_start[2])]
        p1 = [float(p_end[0]), float(p_end[1]), float(p_end[2])]
        dleg = float(
            np.linalg.norm(
                np.array(p1, dtype=np.float64) - np.array(p0, dtype=np.float64)
            )
        )
        th = float(self.vision_cartesian_skip_if_dist_m)
        want_cart = self.vision_path_mode == "cartesian_execute"
        skip_cs = th > 0.0 and dleg > th
        if want_cart and not skip_cs:
            if self._go_vision_cartesian_chunks(arm, p0, p1, label):
                return True
            rospy.logwarn(
                "[CART] %s 失败→OMPL | EN: cartesian fail, fallback OMPL",
                label,
            )
        elif want_cart and skip_cs:
            rospy.logwarn(
                "[CART] %s L=%.3f>%.3f 跳过笛卡尔 | EN: skip cartesian (span)",
                label,
                dleg,
                th,
            )
        if self.vision_use_segmented_pose:
            return self._go_vision_polyline_with_leadin(arm, p0, p1, label, vs0)
        return self._go_vision_waypoint(arm, p1, label, vs0)

    def _go_vision_reach_seed(self, arm):
        """关节空间先伸到工作区附近，再视觉分段，缩短每段笛卡尔路径与名义执行时间。"""
        if not self.vision_reach_seed_enable:
            return True
        joints = self.vision_reach_seed_joints
        if joints is None or len(joints) != 5:
            joints = [float(x) for x in self.pregrasp_joints]
        try:
            arm.clear_pose_targets()
        except MoveItCommanderException:
            pass
        self.configure_arm_move_group(
            arm,
            vel_scaling=max(0.12, self.vision_reach_seed_vel_scale),
            accel_scaling=max(0.12, self.vision_reach_seed_accel_scale),
        )
        try:
            arm.set_goal_joint_tolerance(max(self.arm_goal_joint_tolerance, 0.14))
        except Exception:
            pass
        arm.set_joint_value_target(joints)
        t0 = time.time()
        ok = arm.go(wait=True)
        dt = time.time() - t0
        ep = self.get_tcp_xyz_in_ref_frame(arm)
        if ep is not None:
            rospy.loginfo(
                "[ARM] reach_seed ok=%s t=%.2fs TCP(%.3f,%.3f,%.3f) | EN: reach seed joints",
                ok,
                dt,
                ep[0],
                ep[1],
                ep[2],
            )
        else:
            rospy.loginfo(
                "[ARM] reach_seed ok=%s t=%.2fs | EN: reach seed joints",
                ok,
                dt,
            )
        if not ok:
            rospy.logwarn(
                "[ARM] reach_seed 未完全到位，继续视觉 | EN: seed incomplete, continue"
            )
        return True

    @staticmethod
    def _quat_tool_axis_toward(from_xyz, to_xyz, axis_sign=1.0, up_hint=None):
        """构造四元数：末端 +Z 轴在规划系中与 axis_sign*(to-from) 单位向量对齐。"""
        if _tft is None:
            return None
        if up_hint is None:
            up_hint = (0.0, 0.0, 1.0)
        d = np.array(to_xyz, dtype=np.float64) - np.array(from_xyz, dtype=np.float64)
        n = np.linalg.norm(d)
        if n < 1e-9:
            return None
        z = (d / n) * float(axis_sign)
        up = np.array(up_hint, dtype=np.float64)
        up = up / (np.linalg.norm(up) + 1e-12)
        if abs(float(np.dot(z, up))) > 0.98:
            up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x = np.cross(up, z)
        nx = np.linalg.norm(x)
        if nx < 1e-9:
            x = np.cross(np.array([0.0, 1.0, 0.0], dtype=np.float64), z)
            nx = np.linalg.norm(x)
        if nx < 1e-9:
            return None
        x = x / nx
        y = np.cross(z, x)
        R = np.eye(4, dtype=np.float64)
        R[0:3, 0] = x
        R[0:3, 1] = y
        R[0:3, 2] = z
        q = _tft.quaternion_from_matrix(R)
        return float(q[0]), float(q[1]), float(q[2]), float(q[3])

    def _build_dense_cartesian_poses(self, fp, tp, ps_template):
        """沿 fp→tp 直线插多个 Pose，姿态使接近轴指向 tp，提高 compute_cartesian_path 成功率。"""
        fp = np.array(fp, dtype=np.float64)
        tp = np.array(tp, dtype=np.float64)
        vec = tp - fp
        dist = float(np.linalg.norm(vec))
        if dist < 1e-6:
            return []
        n = max(4, min(28, int(self.vision_cartesian_dense_waypoints)))
        out = []
        for i in range(1, n + 1):
            t = i / float(n)
            pos = fp + vec * t
            pose = copy.deepcopy(ps_template)
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            if not self.vision_cartesian_keep_orientation:
                qh = self._quat_tool_axis_toward(
                    pos.tolist(),
                    tp.tolist(),
                    self.vision_cartesian_approach_axis_sign,
                )
                if qh is not None:
                    pose.orientation.x = qh[0]
                    pose.orientation.y = qh[1]
                    pose.orientation.z = qh[2]
                    pose.orientation.w = qh[3]
            out.append(pose)
        return out

    def _execute_cartesian_line_to_xyz(self, arm, pt, label):
        """直线笛卡尔到 pt：先尝试密集路点一条轨迹，再尝试单终点。"""
        try:
            arm.clear_pose_targets()
        except MoveItCommanderException:
            pass
        try:
            ps = arm.get_current_pose().pose
            fp = [float(ps.position.x), float(ps.position.y), float(ps.position.z)]
            tgt = copy.deepcopy(ps)
            tgt.position.x = float(pt[0])
            tgt.position.y = float(pt[1])
            tgt.position.z = float(pt[2])
            if not self.vision_cartesian_keep_orientation:
                qh = self._quat_tool_axis_toward(
                    fp, pt, self.vision_cartesian_approach_axis_sign
                )
                if qh is not None:
                    tgt.orientation.x = qh[0]
                    tgt.orientation.y = qh[1]
                    tgt.orientation.z = qh[2]
                    tgt.orientation.w = qh[3]
        except Exception as e:
            rospy.logwarn(
                "[MOVEIT] %s get_current_pose 失败: %s | EN: get pose failed",
                label,
                e,
            )
            return False
        vcap = min(1.0, max(0.15, float(self.vision_waypoint_vel_cap)))
        try:
            arm.set_max_velocity_scaling_factor(vcap)
            arm.set_max_acceleration_scaling_factor(vcap)
        except Exception:
            pass
        step0 = max(0.004, float(self.vision_cartesian_eef_step))
        min_frac = float(self.vision_cartesian_min_fraction)
        best_traj, best_frac = None, 0.0
        best_step = step0
        best_ptag = "none"
        tp_arr = np.array(pt, dtype=np.float64)
        dense = self._build_dense_cartesian_poses(fp, tp_arr, ps)
        path_candidates = []
        if len(dense) >= 2:
            path_candidates.append(("dense", dense))
        path_candidates.append(("single", [tgt]))
        for ptag, wplist in path_candidates:
            for step in (step0, step0 * 0.55, step0 * 1.35, step0 * 0.28):
                s = max(0.003, float(step))
                traj, frac = arm.compute_cartesian_path(
                    wplist,
                    s,
                    avoid_collisions=self.vision_cartesian_avoid_collision,
                )
                fr = float(frac)
                if fr > best_frac:
                    best_traj, best_frac, best_step = traj, fr, s
                    best_ptag = ptag
                if fr >= min_frac:
                    best_traj, best_frac, best_step = traj, fr, s
                    best_ptag = ptag
                    break
            if best_frac >= min_frac:
                break
        if best_ptag == "none":
            nd = len(dense) if dense else 0
            best_ptag = "zero_frac(dense_n=%d)" % nd
        rospy.loginfo(
            "[CART] %s frac=%.3f step=%.4f keep_ori=%s axis=%.1f %s | EN: cartesian path",
            label,
            best_frac,
            best_step,
            self.vision_cartesian_keep_orientation,
            self.vision_cartesian_approach_axis_sign,
            best_ptag,
        )
        if best_traj is None or best_frac < min_frac:
            rospy.logwarn(
                "[CART] %s 路径过短 frac=%.3f < min=%.3f | EN: cartesian too short",
                label,
                best_frac,
                min_frac,
            )
            return False
        try:
            return bool(arm.execute(best_traj, wait=True))
        except MoveItCommanderException as e:
            rospy.logwarn(
                "[CART] %s execute: %s | EN: execute failed",
                label,
                e,
            )
            return False

    def _go_vision_cartesian_chunks(self, arm, p0, p1, label):
        """沿 p0→p1 直线分几段笛卡尔 execute（姿态由 ~vision_cartesian_approach_axis_sign 等决定）。"""
        p0a = np.array(p0, dtype=np.float64)
        p1a = np.array(p1, dtype=np.float64)
        n = max(1, min(10, int(self.vision_cartesian_chunks)))
        for i in range(1, n + 1):
            t = i / float(n)
            pt = (p0a + (p1a - p0a) * t).tolist()
            lab = "%s %d/%d" % (label, i, n)
            if not self._execute_cartesian_line_to_xyz(arm, pt, lab):
                return False
        return True

    def _go_vision_polyline(self, arm, p0, p1, label, vs0):
        if self.vision_fix_trajectory_execution_via_dynreconf:
            self._maybe_fix_trajectory_execution_dynreconf(
                reason="vision_polyline %s" % label
            )
        try:
            arm.set_planning_time(float(self.vision_ompl_planning_time))
            arm.set_num_planning_attempts(
                max(1, int(self.vision_ompl_planning_attempts))
            )
        except Exception:
            pass
        n = self._vision_num_segments(p0, p1)
        if "预接近" in label:
            cap = max(1, int(self.vision_pregrasp_max_segments))
            n = min(n, cap)
        wps = self._interp_line_xyz(p0, p1, n)
        step_len = float(np.linalg.norm(np.array(wps[0]) - np.array(p0))) if wps else 0.0
        rospy.loginfo(
            "[POLY] %s segments=%d chord_L=%.3f step0=%.3f | EN: OMPL polyline",
            label,
            n,
            float(
                np.linalg.norm(
                    np.array(p1, dtype=np.float64) - np.array(p0, dtype=np.float64)
                )
            ),
            step_len,
        )
        for i, pt in enumerate(wps):
            lab = "%s %d/%d" % (label, i + 1, len(wps))
            if not self._go_vision_waypoint(arm, pt, lab, vs0):
                return False
        return True

    def _go_vision_waypoint(self, arm, pt, label, base_vs):
        n_att = 3
        vmin = max(0.08, min(0.95, float(self.vision_min_velocity_scale)))
        vcap = max(0.2, min(1.0, float(self.vision_waypoint_vel_cap)))
        for att in range(n_att):
            if att < 2:
                vs = min(vcap, base_vs * (1.45 ** att))
            else:
                vs = max(0.12, min(0.38, base_vs * 0.48))
            vs = max(vs, vmin)
            arm.set_max_velocity_scaling_factor(vs)
            arm.set_max_acceleration_scaling_factor(vs)
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                hint = str(getattr(self, "_vision_wp_pose_hint", "") or "")
                use_pure_position = (
                    not self.vision_cartesian_use_pose
                    and not self.vision_ompl_lock_wrist_orientation
                    and hint not in ("pose_approach_axis", "pose_retained")
                )
                if use_pure_position and self.vision_5dof_use_plan_execute:
                    try:
                        arm.set_planning_time(float(self.vision_ompl_planning_time))
                        arm.set_num_planning_attempts(
                            max(1, int(self.vision_ompl_planning_attempts))
                        )
                    except Exception:
                        pass
                    if self._vision_plan_execute_position_z_backoff(
                        arm, pt, label, vs
                    ):
                        return True
                else:
                    if self.vision_cartesian_use_pose:
                        arm.set_pose_target(self._current_ee_pose_at_xyz(arm, pt))
                    elif self.vision_ompl_lock_wrist_orientation:
                        arm.set_pose_target(self._current_ee_pose_at_xyz(arm, pt))
                    elif hint == "pose_approach_axis":
                        arm.set_pose_target(self._preflight_goal_pose_approach(arm, pt))
                    elif hint == "pose_retained":
                        arm.set_pose_target(self._current_ee_pose_at_xyz(arm, pt))
                    else:
                        arm.set_position_target(pt)
                    if arm.go(wait=True):
                        return True
            except MoveItCommanderException as e:
                rospy.logwarn(
                    "[MOVEIT] %s 尝试 %d: %s | EN: waypoint attempt",
                    label,
                    att + 1,
                    e,
                )
            try:
                arm.stop()
            except Exception:
                pass
            rospy.sleep(0.22 + 0.2 * float(att))
        try:
            arm.stop()
        except Exception:
            pass
        rospy.sleep(0.12)
        if self.vision_log_plan_on_fail:
            self._log_plan_probe(arm, pt, label)
        return False

    def _open_gripper_wide(self, gripper):
        """预抓取/下探前将夹爪开到目标大开角（默认与 URDF 全开一致）。"""
        if gripper is None:
            return False
        v = float(self.gripper_open_max)
        v = max(0.0, min(v, float(self.gripper_joint_upper)))
        gripper.set_max_velocity_scaling_factor(0.4)
        gripper.set_goal_joint_tolerance(self.gripper_goal_tolerance)
        gripper.set_joint_value_target([v])
        return gripper.go(wait=True)

    def _close_gripper_after_approach(self, gripper):
        """
        默认一次 MoveIt 到 gripper_close。
        可选增量闭合：每小步检查夹爪关节 |effort|，超 grasp_touch_effort_abs 视为已顶到物体，
        触物后仍发一次目标位姿 gripper_close（可略大于 0 留余量，避免全闭挤飞方块）。
        """
        tgt = float(self.gripper_close)
        tgt = max(0.0, min(tgt, float(self.gripper_joint_upper)))
        if not self.grasp_incremental_close_enable:
            gripper.set_joint_value_target([tgt])
            return gripper.go(wait=True)
        p0 = self._gripper_joint_pos
        if p0 is None:
            gripper.set_joint_value_target([tgt])
            return gripper.go(wait=True)
        p0 = float(p0)
        n = max(2, int(self.grasp_incremental_close_steps))
        thr = max(0.0, float(self.grasp_touch_effort_abs))
        for k in range(1, n + 1):
            alpha = k / float(n)
            cmd = p0 + (tgt - p0) * alpha
            cmd = max(0.0, min(cmd, float(self.gripper_joint_upper)))
            gripper.set_joint_value_target([cmd])
            if not gripper.go(wait=True):
                rospy.logwarn(
                    "[GRIP] 增量闭合 %d/%d 失败 | EN: incremental close failed",
                    k,
                    n,
                )
                return False
            rospy.sleep(0.03)
            if thr > 1e-9:
                eff = self._gripper_joint_effort
                if eff is not None and abs(float(eff)) >= thr and cmd > tgt + 0.06:
                    rospy.loginfo(
                        "[GRIP] |eff|=%.3f≥%.3f 触物推测→最终闭合 tgt=%.3f | EN: touch, final close",
                        abs(float(eff)),
                        thr,
                        tgt,
                    )
                    break
        gripper.set_joint_value_target([tgt])
        return gripper.go(wait=True)

    def _joint_pregrasp_grasp(self, arm, gripper):
        if not self._open_gripper_wide(gripper):
            rospy.logerr(
                "[GRIP] 爪全开失败 | EN: gripper open-wide failed"
            )
            return False
        arm.clear_pose_targets()
        self.configure_arm_move_group(arm)
        arm.set_joint_value_target([float(x) for x in self.pregrasp_joints])
        if not arm.go(wait=True):
            return False
        arm.set_joint_value_target([float(x) for x in self.grasp_joints])
        if not arm.go(wait=True):
            return False
        return True

    def exec_arm_sequence(self, vision_xyz):
        arm = self.get_arm_group()
        gripper = self.get_gripper_group()
        if arm is None or gripper is None:
            return False
        self._grasp_contact_seen = False
        self._refine_stable_ctr = 0
        self.configure_arm_move_group(arm)
        gripper.set_max_velocity_scaling_factor(0.4)
        gripper.set_goal_joint_tolerance(self.gripper_goal_tolerance)
        arm.set_pose_reference_frame(self.ref_frame)
        self._vision_wp_pose_hint = ""
        self._last_exec_p_fin = None

        if not self._open_gripper_wide(gripper):
            rospy.logerr(
                "[GRIP] 开爪 MoveIt 失败 | EN: open gripper failed"
            )
            return False

        use_vis = vision_xyz is not None and len(vision_xyz) == 3
        ok_vis = False
        motion_ok = False

        if use_vis:
            base = np.array(vision_xyz, dtype=np.float64)
            off = np.array(self.vision_target_offset, dtype=np.float64)
            ws_ok, p_pre, p_fin, ws_note = self._adjust_grasp_targets_from_base(base)
            rospy.loginfo(
                "[GRASP] 目标 z_pre=%.3f z_fin=%.3f m | EN: approach/grasp heights",
                float(p_pre[2]),
                float(p_fin[2]),
            )
            self._last_exec_p_fin = [
                float(p_fin[0]),
                float(p_fin[1]),
                float(p_fin[2]),
            ]
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                # 每次进入视觉段前再写一次，减轻 RViz 在运行中途改 trajectory_execution
                if time.time() - self._last_traj_dynreconf_wall > 3.0:
                    self._maybe_fix_trajectory_execution_dynreconf(
                        reason="before_vision_grasp"
                    )
                if self.vision_joint_staging_enable:
                    rospy.loginfo(
                        "[ARM] staging→pregrasp | EN: staging to pregrasp joints"
                    )
                    self.configure_arm_move_group(
                        arm,
                        vel_scaling=max(
                            0.14, float(self.vision_ompl_plan_vel_scale)
                        ),
                        accel_scaling=max(
                            0.14, float(self.vision_ompl_plan_accel_scale)
                        ),
                    )
                    try:
                        arm.clear_pose_targets()
                    except MoveItCommanderException:
                        pass
                    arm.set_joint_value_target(
                        [float(x) for x in self.pregrasp_joints]
                    )
                    if not arm.go(wait=True):
                        rospy.logwarn(
                            "[ARM] staging 失败，继续 | EN: staging failed, continue"
                        )
                p_pre_arr = np.asarray(p_pre, dtype=np.float64)
                d_before_seed = None
                if self.vision_reach_seed_enable:
                    pb = self.get_tcp_xyz_in_ref_frame(arm)
                    if pb is not None:
                        d_before_seed = float(np.linalg.norm(p_pre_arr - pb))
                    else:
                        rospy.logwarn_throttle(
                            2.0,
                            "[TF] seed 前无 TCP | EN: no TCP before seed",
                        )
                self._go_vision_reach_seed(arm)
                self._apply_vision_execution_tolerances(arm)
                self.configure_arm_move_group(
                    arm,
                    vel_scaling=self.vision_ompl_plan_vel_scale,
                    accel_scaling=self.vision_ompl_plan_accel_scale,
                )
                skip_pose_execute = not ws_ok
                if not ws_ok:
                    rospy.logerr(
                        "[WS] 工作区: %s →关节备用 | EN: workspace, joint fallback",
                        ws_note,
                    )
                ok_pf_pre, mode_pre = True, "position"
                ok_pf_fin, mode_fin = True, "position"
                if self.vision_preflight_plan and not skip_pose_execute:
                    ok_pf_pre, mode_pre = self._vision_preflight_plan_ok(
                        arm, p_pre, "预接近点"
                    )
                    ok_pf_fin, mode_fin = self._vision_preflight_plan_ok(
                        arm, p_fin, "下落点"
                    )
                    if (
                        self.vision_preflight_skip_pose_if_pregrasp_fails
                        and not ok_pf_pre
                    ):
                        rospy.logwarn(
                            "[PREFLIGHT] 预接近预检失败→跳过末端链 | EN: skip pose chain"
                        )
                        skip_pose_execute = True
                base_vs = self.vision_cartesian_vel_scale
                vs0 = max(
                    self.vision_pose_step_vel_scale,
                    base_vs * 0.32,
                    float(self.vision_ompl_plan_vel_scale) * 0.88,
                )
                ok_vis = False
                if not skip_pose_execute:
                    p_xyz = self.get_tcp_xyz_in_ref_frame(arm)
                    if p_xyz is not None:
                        p_cur = [
                            float(p_xyz[0]),
                            float(p_xyz[1]),
                            float(p_xyz[2]),
                        ]
                    else:
                        rospy.logwarn(
                            "[TF] 无 TCP，用质心+offset 近似起点 | EN: TCP missing, centroid seed"
                        )
                        p_cur = (base + off).tolist()
                    dist_pre = float(
                        np.linalg.norm(
                            p_pre_arr - np.array(p_cur, dtype=np.float64)
                        )
                    )
                    if (
                        self.vision_reach_seed_enable
                        and self.vision_reach_seed_revert_if_worse
                        and d_before_seed is not None
                        and dist_pre
                        > d_before_seed + self.vision_reach_seed_revert_margin_m
                    ):
                        rospy.logwarn(
                            "[ARM] seed 变差 %.3f→%.3f m，回退 arm_ready | EN: seed worse, revert",
                            d_before_seed,
                            dist_pre,
                        )
                        try:
                            arm.clear_pose_targets()
                        except MoveItCommanderException:
                            pass
                        vs_rb = max(
                            0.15, float(self.vision_ompl_plan_vel_scale)
                        )
                        ac_rb = max(
                            0.15, float(self.vision_ompl_plan_accel_scale)
                        )
                        self.exec_arm_joints(
                            self.arm_ready_joints,
                            vel_scale=vs_rb,
                            accel_scale=ac_rb,
                            joint_tol=max(self.arm_goal_joint_tolerance, 0.12),
                        )
                        self._apply_vision_execution_tolerances(arm)
                        self.configure_arm_move_group(
                            arm,
                            vel_scaling=self.vision_ompl_plan_vel_scale,
                            accel_scaling=self.vision_ompl_plan_accel_scale,
                        )
                        p_xyz2 = self.get_tcp_xyz_in_ref_frame(arm)
                        if p_xyz2 is not None:
                            p_cur = [
                                float(p_xyz2[0]),
                                float(p_xyz2[1]),
                                float(p_xyz2[2]),
                            ]
                        dist_pre = float(
                            np.linalg.norm(
                                p_pre_arr - np.array(p_cur, dtype=np.float64)
                            )
                        )
                    rospy.loginfo(
                        "[VISION] 接近 mode=%s d_tcp_pre=%.3f m pre=%s fin=%s | EN: approach",
                        self.vision_path_mode,
                        dist_pre,
                        mode_pre,
                        mode_fin,
                    )
                    self._vision_wp_pose_hint = mode_pre if ok_pf_pre else ""
                    if self.vision_path_mode == "cartesian_execute":
                        leg_a = self._vision_motion_leg(
                            arm, p_cur, p_pre, "预接近", vs0
                        )
                        self._vision_wp_pose_hint = mode_fin if ok_pf_fin else ""
                        ok_vis = leg_a and self._vision_motion_leg_fin_with_xy(
                            arm, p_pre, p_fin, vs0
                        )
                    elif self.vision_use_segmented_pose:
                        leg_a = self._go_vision_polyline_with_leadin(
                            arm, p_cur, p_pre, "预接近", vs0
                        )
                        self._vision_wp_pose_hint = mode_fin if ok_pf_fin else ""
                        ok_vis = leg_a and self._go_vision_polyline_to_fin_with_xy(
                            arm, p_pre, p_fin, vs0
                        )
                    else:
                        leg_a = self._go_vision_waypoint(
                            arm, p_pre, "预接近", vs0
                        )
                        self._vision_wp_pose_hint = mode_fin if ok_pf_fin else ""
                        ok_vis = leg_a and self._go_vision_fin_waypoint_with_xy(
                            arm, p_fin, vs0
                        )
                    if ok_vis:
                        self._grasp_finishing_refine(arm, vs0)

                if not ok_vis:
                    rospy.logwarn(
                        "[VISION] 末端链失败→关节抓取 | EN: pose chain fail, joint grasp"
                    )
                    if not self._joint_pregrasp_grasp(arm, gripper):
                        ok_fc, fc_why, fc_met = self._force_close_grasp_allowed(
                            arm, p_fin
                        )
                        if ok_fc:
                            rospy.logwarn(
                                "[GRASP] 关节备用失败但强制闭爪(%s=%s)→仍闭爪 | EN: force close",
                                fc_why,
                                (
                                    ("%.3f" % fc_met)
                                    if fc_why == "tcp_dist_m"
                                    else (
                                        ("%.1fpx" % fc_met)
                                        if fc_why == "pixel_err"
                                        else "bumper"
                                    )
                                ),
                            )
                            ok_vis = True
                        else:
                            self._reset_vision_execution_tolerances(arm)
                            return False
                    ok_vis = True
                    self._last_exec_p_fin = None
            except MoveItCommanderException:
                if not self._joint_pregrasp_grasp(arm, gripper):
                    ok_fc, fc_why, fc_met = self._force_close_grasp_allowed(
                        arm, p_fin
                    )
                    if ok_fc:
                        rospy.logwarn(
                            "[GRASP] MoveIt 异常后关节备用失败但强制闭爪(%s=%s)→仍闭爪 | EN: force close",
                            fc_why,
                            (
                                ("%.3f" % fc_met)
                                if fc_why == "tcp_dist_m"
                                else (
                                    ("%.1fpx" % fc_met)
                                    if fc_why == "pixel_err"
                                    else "bumper"
                                )
                            ),
                        )
                        ok_vis = True
                    else:
                        self._reset_vision_execution_tolerances(arm)
                        return False
                ok_vis = True
                self._last_exec_p_fin = None
            finally:
                self._reset_vision_execution_tolerances(arm)
            # 末端 Pose 链成功，或关节备用到位，均允许进入夹爪闭合（抓取精度后者依赖关节示教）
            motion_ok = ok_vis
        else:
            motion_ok = self._joint_pregrasp_grasp(arm, gripper)
            if not motion_ok:
                return False

        if not motion_ok:
            rospy.logerr(
                "[GRASP] 手臂未到位，跳过闭爪 | EN: arm not settled, skip close"
            )
            return False

        pinch = float(self.gripper_soft_pinch)
        lo = float(self.gripper_close) + 0.03
        hi = float(self.gripper_open_max) - 0.05
        if pinch > lo and pinch < hi:
            pinch = max(0.0, min(pinch, float(self.gripper_joint_upper)))
            gripper.set_joint_value_target([pinch])
            if not gripper.go(wait=True):
                rospy.logwarn(
                    "[GRIP] 轻合 pinch=%.3f 失败→仍尝试最终闭合 | EN: soft pinch failed",
                    pinch,
                )
            else:
                rospy.loginfo(
                    "[GRIP] 轻合 pinch=%.3f →最终 tgt=%.3f | EN: soft pinch then close",
                    pinch,
                    float(self.gripper_close),
                )
            rospy.sleep(max(0.0, float(self.gripper_soft_pinch_pause_sec)))

        if not self._close_gripper_after_approach(gripper):
            rospy.logerr(
                "[GRIP] MoveIt 闭爪失败 | EN: gripper close (MoveIt) failed"
            )
            return False
        return True

    def exec_arm_joints(self, joints, vel_scale=None, accel_scale=None, joint_tol=None):
        arm = self.get_arm_group()
        if arm is None:
            return False
        # 与视觉段一致但用更小的 execution_velocity_scaling，避免 effort 仿真猛跟参考导致大幅晃动
        if self.vision_fix_trajectory_execution_via_dynreconf:
            self._maybe_fix_trajectory_execution_dynreconf(
                reason="exec_arm_joints",
                exec_velocity_scaling=self.arm_trajectory_execution_velocity_scaling,
            )
        self.configure_arm_move_group(
            arm,
            vel_scaling=vel_scale or self.arm_home_vel_scaling,
            accel_scaling=accel_scale or self.arm_home_accel_scaling,
        )
        if joint_tol is not None:
            try:
                arm.set_goal_joint_tolerance(float(joint_tol))
            except Exception:
                pass
        arm.set_joint_value_target([float(x) for x in joints])
        ok = arm.go(wait=True)
        if joint_tol is not None:
            try:
                arm.set_goal_joint_tolerance(self.arm_goal_joint_tolerance)
            except Exception:
                pass
        return ok

    def _tcp_pixel_error_vs_last_detection(self, arm):
        """
        将 hand_tcp（规划系）变到相机光学系，按 camera_info 投影为像素，与检测稳定时的 uv 比偏差（像素）。
        用于「规划系 3D 目标有偏但画面仍对准」时的辅助判据；与相机到蓝块的深度（米）不是同一量纲，不宜直接相减比较。
        """
        if self._K is None or self._last_uv is None:
            return None
        cpo = self.get_tcp_xyz_in_ref_frame(arm)
        if cpo is None:
            return None
        pt = Point(float(cpo[0]), float(cpo[1]), float(cpo[2]))
        pc = transform_point(
            self.tf_buffer,
            pt,
            self.camera_optical_frame,
            self.ref_frame,
        )
        if pc is None:
            return None
        z = float(pc.z)
        if z <= 0.0:
            return None
        z_floor = max(1e-6, float(self.grasp_tcp_proj_min_z_m))
        z = max(z, z_floor)
        fx, fy = float(self._K[0, 0]), float(self._K[1, 1])
        cx, cy = float(self._K[0, 2]), float(self._K[1, 2])
        u = fx * float(pc.x) / z + cx
        v = fy * float(pc.y) / z + cy
        du = u - float(self._last_uv[0])
        dv = v - float(self._last_uv[1])
        return float(math.hypot(du, dv))

    def get_tcp_xyz_in_ref_frame(self, arm=None):
        """
        hand_tcp 原点在 self.ref_frame（与视觉目标同系）下的 xyz。
        MoveIt Python 的 get_current_pose() 把位姿写在 get_planning_frame()
        （常为 base_link），不能与 roarm_base_link 下的目标直接做差；此处优先 TF。
        """
        p_tf = transform_point(
            self.tf_buffer,
            Point(0.0, 0.0, 0.0),
            self.ref_frame,
            self.ee_tcp_frame,
            timeout=1.5,
        )
        if p_tf is None:
            rospy.sleep(0.05)
            p_tf = transform_point(
                self.tf_buffer,
                Point(0.0, 0.0, 0.0),
                self.ref_frame,
                self.ee_tcp_frame,
                timeout=0.8,
            )
        if p_tf is not None:
            return np.array([p_tf.x, p_tf.y, p_tf.z], dtype=np.float64)
        if arm is None or do_transform_point is None:
            return None
        try:
            ps = arm.get_current_pose()
            fr = ps.header.frame_id
            pp = ps.pose.position
            if not fr or fr == self.ref_frame:
                return np.array([pp.x, pp.y, pp.z], dtype=np.float64)
            pst = PointStamped()
            pst.header.frame_id = fr
            pst.header.stamp = rospy.Time(0)
            pst.point = pp
            tfm = self.tf_buffer.lookup_transform(
                self.ref_frame, fr, rospy.Time(0), rospy.Duration(1.5)
            )
            outp = do_transform_point(pst, tfm)
            return np.array(
                [outp.point.x, outp.point.y, outp.point.z], dtype=np.float64
            )
        except Exception:
            return None

    def _tcp_distance_to_xyz(self, arm, xyz):
        """ref_frame 下 TCP 与目标点欧氏距离（米）；不可用时返回 None。"""
        if xyz is None:
            return None
        try:
            seq = list(xyz)
            if len(seq) < 3:
                return None
            tcp = self.get_tcp_xyz_in_ref_frame(arm)
            if tcp is None:
                return None
            t = np.asarray(tcp, dtype=np.float64)
            p = np.asarray([float(seq[0]), float(seq[1]), float(seq[2])], dtype=np.float64)
            return float(np.linalg.norm(t - p))
        except Exception:
            return None

    def _force_close_grasp_allowed(self, arm, p_fin):
        """
        末段执行失败时是否仍允许 MoveIt 闭爪：TCP 距下落点足够近，或画面像素对准足够好（逻辑或）。
        返回 (ok, reason, metric)；reason 为 'tcp_dist_m'、'pixel_err' 或 'gazebo_contact'。
        """
        if bool(getattr(self, "_grasp_contact_seen", False)):
            return True, "gazebo_contact", 1.0
        thr = float(self.force_close_gripper_if_tcp_within_m)
        dfin = self._tcp_distance_to_xyz(arm, p_fin)
        if thr > 1e-6 and dfin is not None and dfin <= thr:
            return True, "tcp_dist_m", dfin
        px_max = float(self.force_close_gripper_if_pixel_below_px)
        if px_max > 1e-6:
            px_err = self._tcp_pixel_error_vs_last_detection(arm)
            if px_err is not None and px_err <= px_max:
                return True, "pixel_err", px_err
        return False, "", None

    def report_grasp_success(self, moveit_ok, vision_xyz_used):
        """MoveIt 成功 + 爪闭 +（视觉时）TF 下 TCP 与目标距离在阈值内（与视觉同 ref_frame）。"""
        rospy.sleep(0.4)
        jp = self._gripper_joint_pos
        jp_sane = (
            jp is not None
            and 0.0 <= jp <= self.gripper_joint_upper
        )
        if jp is not None and not jp_sane:
            rospy.logwarn(
                "[GRIP] 关节 %.4f 超界 [0,%.2f]，忽略闭合判据 | EN: joint out of range, ignore close check",
                jp,
                self.gripper_joint_upper,
            )
        closed = jp_sane and jp <= self.grasp_joint_closed_threshold

        ee_near = True
        if not moveit_ok:
            ee_near = False
        elif vision_xyz_used is not None and len(vision_xyz_used) == 3:
            ee_near = False
            arm = self.get_arm_group()
            if arm is not None:
                try:
                    off = np.array(self.vision_target_offset, dtype=np.float64)
                    d0 = np.array(self.vision_pregrasp_delta, dtype=np.float64)
                    d1 = np.array(self.vision_grasp_delta, dtype=np.float64)
                    ez = float(self.vision_tcp_extra_z_m)
                    tgt_legacy = np.array(vision_xyz_used, dtype=np.float64) + off + d0 + d1
                    tgt_with_ez = tgt_legacy + np.array(
                        [0.0, 0.0, ez], dtype=np.float64
                    )
                    cpo = self.get_tcp_xyz_in_ref_frame(arm)
                    if cpo is None:
                        ee_near = False
                    else:
                        use_fin = (
                            self._last_exec_p_fin is not None
                            and len(self._last_exec_p_fin) == 3
                        )
                        if use_fin:
                            tgt = np.array(self._last_exec_p_fin, dtype=np.float64)
                            tgt_tag = "fin"
                        else:
                            tgt = tgt_with_ez
                            tgt_tag = "leg"
                        dist = float(np.linalg.norm(tgt - cpo))
                        max_m = float(self.grasp_ee_to_target_max_m)
                        geom_near = dist < max_m
                        px_err = self._tcp_pixel_error_vs_last_detection(arm)
                        cam_fb_ok = bool(
                            self.grasp_ee_near_cam_fallback_enable
                            and px_err is not None
                            and px_err < self.grasp_ee_near_cam_fallback_max_px
                        )
                        ee_near = geom_near or cam_fb_ok
                        px_log = -1.0 if px_err is None else px_err
                        rospy.loginfo(
                            "[EE] tgt=%s dist=%.3f thr=%.3f m px=%.0f geom=%s cam_fb=%s | EN: TCP vs goal",
                            tgt_tag,
                            dist,
                            max_m,
                            px_log,
                            geom_near,
                            cam_fb_ok,
                        )
                except Exception as e:
                    rospy.logwarn(
                        "[EE] 判距异常: %s | EN: EE check exception",
                        e,
                    )
                    ee_near = False

        ok = bool(moveit_ok and closed and ee_near)
        _jp = jp if jp is not None else float("nan")
        _msg = (
            "[RESULT] 抓取成功=%s MoveIt=%s 爪闭=%s 末端近=%s jp=%.3f | EN: grasp ok=%s moveit=%s closed=%s near=%s"
            % (
                ok,
                moveit_ok,
                closed,
                ee_near,
                _jp,
                ok,
                moveit_ok,
                closed,
                ee_near,
            )
        )
        if ok:
            rospy.loginfo(_msg)
        else:
            rospy.logwarn(_msg)
        return ok


def main():
    m = GraspMission()
    start_rospy_callbacks_for_moveit(m.async_spinner_threads)
    rate = rospy.Rate(m.rate_hz)

    while not rospy.is_shutdown() and not m.state.connected:
        rospy.loginfo_throttle(
            2.0,
            "[MAVROS] 等待连接 | EN: waiting for MAVROS state",
        )
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
    m.z_high = m.z0 + m.hover_rel_m
    m.z_low = m.z0 + m.low_rel_m + m.low_rel_guard_m
    m.z_sp = m.z_high
    lat = float(m.approach_lateral_m)
    fwd_total = float(m.forward_m) + float(m.approach_forward_extra_m)
    m.x_fwd = m.x0 + fwd_total * c + lat * s
    m.y_fwd = m.y0 + fwd_total * s - lat * c

    rospy.loginfo(
        "[MISSION] 起点(%.2f,%.2f,%.2f) z_hi=%.2f z_lo=%.2f(rel=%.2f+guard=%.2f) fwd=%.2f+extra=%.2f lat=%.2f →(%.2f,%.2f) | EN: home & approach WP",
        m.x0,
        m.y0,
        m.z0,
        m.z_high,
        m.z_low,
        m.low_rel_m,
        m.low_rel_guard_m,
        m.forward_m,
        m.approach_forward_extra_m,
        lat,
        m.x_fwd,
        m.y_fwd,
    )

    if m.auto_offboard:
        try:
            m.set_mode(0, "OFFBOARD")
        except rospy.ServiceException as e:
            rospy.logwarn("[MAVROS] set_mode: %s | EN: service", e)
    if m.auto_arm:
        try:
            m.arming(True)
        except rospy.ServiceException as e:
            rospy.logwarn("[MAVROS] arm: %s | EN: service", e)

    if m.offboard_stream_enable:
        start_offboard_setpoint_stream(
            m, m.offboard_stream_hz, use_wall_clock=m.offboard_stream_use_wall_clock
        )

    m.phase = m.PH_HOVER_1M
    m.t_phase0 = rospy.get_time()
    vision_xyz = None

    try:
        while not rospy.is_shutdown() and m.phase != m.PH_DONE:
            px = m.pose.pose.position.x
            py = m.pose.pose.position.y
            pz = m.pose.pose.position.z
            yaw = m.offboard_yaw_cmd()

            if m.phase == m.PH_HOVER_1M:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, yaw)
                if m.dist3(px, py, pz, m.x0, m.y0, m.z_high) < max(
                    m.wp_tol_xy, m.wp_tol_z
                ):
                    if m._hold_since is None:
                        m._hold_since = time.time()
                    elif time.time() - m._hold_since > m.hold_sec:
                        rospy.loginfo(
                            "[PHASE] 悬停完成→arm_ready | EN: hover done, arm ready"
                        )
                        m._hold_since = None
                        m.phase = m.PH_ARM_READY
                else:
                    m._hold_since = None
                rate.sleep()
                continue

            if m.phase == m.PH_ARM_READY:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, yaw)
                rospy.loginfo(
                    "[PHASE] MoveIt→arm_ready | EN: commanding ready pose"
                )
                ok = m.exec_arm_joints(
                    m.arm_ready_joints,
                    vel_scale=m.arm_ready_vel_scaling,
                    accel_scale=m.arm_ready_accel_scaling,
                )
                if not ok:
                    rospy.logwarn(
                        "[PHASE] arm_ready 失败→慢速重试 | EN: retry slow"
                    )
                    ok = m.exec_arm_joints(
                        m.arm_ready_joints_fallback,
                        vel_scale=0.05,
                        accel_scale=0.05,
                        joint_tol=0.2,
                    )
                if not ok:
                    rospy.logerr(
                        "[PHASE] arm_ready 仍失败 | EN: arm_ready failed"
                    )
                m.phase = m.PH_ARM_SETTLE
                m.t_phase0 = rospy.get_time()
                rate.sleep()
                continue

            if m.phase == m.PH_ARM_SETTLE:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, yaw)
                if rospy.get_time() - m.t_phase0 >= m.hold_sec:
                    rospy.loginfo(
                        "[PHASE] settle→前飞 %.2f m (extra=%.2f) ramp=%.1fs | EN: forward %.2f m extra %.2f ramp %.1f s",
                        m.forward_m + m.approach_forward_extra_m,
                        m.approach_forward_extra_m,
                        m.forward_ramp_sec,
                        m.forward_m + m.approach_forward_extra_m,
                        m.approach_forward_extra_m,
                        m.forward_ramp_sec,
                    )
                    m._fwd_ramp_x0 = m.x0
                    m._fwd_ramp_y0 = m.y0
                    m._fwd_ramp_x1 = m.x_fwd
                    m._fwd_ramp_y1 = m.y_fwd
                    m.phase = m.PH_FWD
                    m.t_phase0 = rospy.get_time()
                rate.sleep()
                continue

            if m.phase == m.PH_FWD:
                fx, fy = m._forward_offboard_xy()
                maybe_publish_offboard(m, fx, fy, m.z_high, yaw)
                if m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_high) < m.wp_tol_xy * 1.2:
                    rospy.loginfo(
                        "[PHASE] 到达前飞点 hold %.1fs | EN: at forward WP",
                        m.hold_sec,
                    )
                    m.phase = m.PH_FWD_SETTLE
                    m.t_phase0 = rospy.get_time()
                rate.sleep()
                continue

            if m.phase == m.PH_FWD_SETTLE:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_high, yaw)
                if rospy.get_time() - m.t_phase0 >= m.hold_sec:
                    m.z_sp = m.z_high
                    m.phase = m.PH_DESCEND
                    m.t_phase0 = rospy.get_time()
                rate.sleep()
                continue

            if m.phase == m.PH_DESCEND:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                t = rospy.get_time() - m.t_phase0
                alpha_lin = min(1.0, t / max(0.5, m.descend_duration))
                # smoothstep：末端减速接近 z_low，减轻过冲触底再被控/碰撞弹起
                alpha = alpha_lin * alpha_lin * (3.0 - 2.0 * alpha_lin)
                m.z_sp = m.z_high + (m.z_low - m.z_high) * alpha
                if alpha_lin >= 1.0 and m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_low) < max(
                    m.wp_tol_xy, m.wp_tol_z
                ):
                    rospy.loginfo(
                        "[PHASE] 到达低高度 hold %.1fs | EN: at low alt",
                        m.low_hold_sec,
                    )
                    m.z_sp = m.z_low
                    m.phase = m.PH_LOW_HOLD
                    m.t_phase0 = rospy.get_time()
                rate.sleep()
                continue

            if m.phase == m.PH_LOW_HOLD:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                if rospy.get_time() - m.t_phase0 >= m.low_hold_sec:
                    m._stable_cnt = 0
                    m._last_uv = None
                    m._grasp_attempt = 0
                    m._refine_xy_cumulative_m = 0.0
                    m._refine_frame_tick = 0
                    m.phase = m.PH_DETECT
                rate.sleep()
                continue

            if m.phase == m.PH_DETECT:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                uv = m.detect_blue_uv()
                if uv is not None:
                    if m._stable_cnt < m.detection_stable:
                        m._refine_frame_tick += 1
                        if (
                            m._refine_frame_tick
                            % max(1, int(m.detect_refine_xy_stride))
                            == 0
                        ):
                            m.refine_xy_toward_blue(uv[0], uv[1])
                    if m._last_uv and abs(uv[0] - m._last_uv[0]) < 10 and abs(
                        uv[1] - m._last_uv[1]
                    ) < 10:
                        m._stable_cnt += 1
                    else:
                        m._stable_cnt = 1
                    m._last_uv = uv
                else:
                    m._stable_cnt = 0
                    rospy.loginfo_throttle(
                        3.0,
                        "[VISION] 检测中，无稳定蓝块 | EN: detecting, no blob",
                    )
                if m._stable_cnt >= m.detection_stable and m._last_uv is not None:
                    u, v = m._last_uv
                    vision_xyz = m.print_blue_pose(u, v)
                    rospy.loginfo(
                        "[VISION] 检测稳定→抓取相位 | EN: stable, go grasp"
                    )
                    m.phase = m.PH_GRASP
                rate.sleep()
                continue

            if m.phase == m.PH_GRASP:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                if m._last_uv is None:
                    m._refine_xy_cumulative_m = 0.0
                    m._refine_frame_tick = 0
                    m.phase = m.PH_DETECT
                    rate.sleep()
                    continue
                u0, v0 = m._last_uv
                fresh = m.print_blue_pose(u0, v0)
                if fresh is not None:
                    vision_xyz = fresh
                    rospy.loginfo(
                        "[VISION] 抓取前刷新观测 | EN: refresh blob pose"
                    )
                vx = vision_xyz
                if vx is None:
                    u, v = m._last_uv
                    pc = m.point_cam(u, v)
                    if pc is not None:
                        pb = transform_point(
                            m.tf_buffer, pc, m.ref_frame, m.camera_optical_frame
                        )
                        if pb is not None:
                            vx = (pb.x, pb.y, pb.z)
                ok_mv = m.exec_arm_sequence(vx)
                m.report_grasp_success(ok_mv, vx)
                if ok_mv or m._grasp_attempt >= m.grasp_max_retries:
                    m._grasp_attempt = 0
                    m.phase = m.PH_ARM_HOME
                else:
                    m._grasp_attempt += 1
                    cy, sy = math.cos(m.yaw0), math.sin(m.yaw0)
                    m.x_fwd += m.grasp_retry_forward_m * cy
                    m.y_fwd += m.grasp_retry_forward_m * sy
                    rospy.logwarn(
                        "[GRASP] 失败重试 fwd=%.3f m (%d/%d) | EN: grasp retry",
                        m.grasp_retry_forward_m,
                        m._grasp_attempt,
                        m.grasp_max_retries,
                    )
                    vision_xyz = None
                    m._stable_cnt = 0
                    m._last_uv = None
                    m._refine_xy_cumulative_m = 0.0
                    m._refine_frame_tick = 0
                    m.phase = m.PH_DETECT
                rate.sleep()
                continue

            if m.phase == m.PH_ARM_HOME:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                rospy.loginfo(
                    "[PHASE] 臂→home/ready | EN: arm to home pose"
                )
                ok_h = m.exec_arm_joints(
                    m.arm_ready_joints,
                    vel_scale=m.arm_ready_vel_scaling,
                    accel_scale=m.arm_ready_accel_scaling,
                )
                if not ok_h:
                    m.exec_arm_joints(
                        m.arm_ready_joints_fallback,
                        vel_scale=0.05,
                        accel_scale=0.05,
                        joint_tol=0.2,
                    )
                m.phase = m.PH_CLIMB
                m.t_phase0 = rospy.get_time()
                rate.sleep()
                continue

            if m.phase == m.PH_CLIMB:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_high, yaw)
                if m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_high) < max(
                    m.wp_tol_xy, m.wp_tol_z
                ):
                    if m._hold_since is None:
                        m._hold_since = time.time()
                    elif time.time() - m._hold_since > m.hold_sec:
                        m._hold_since = None
                        m.phase = m.PH_CLIMB_SETTLE
                        m.t_phase0 = rospy.get_time()
                else:
                    m._hold_since = None
                rate.sleep()
                continue

            if m.phase == m.PH_CLIMB_SETTLE:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_high, yaw)
                if rospy.get_time() - m.t_phase0 >= m.hold_sec:
                    rospy.loginfo(
                        "[PHASE] 爬升后→返航 | EN: climb done, RTL"
                    )
                    m.phase = m.PH_RTL
                rate.sleep()
                continue

            if m.phase == m.PH_RTL:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, m.yaw0)
                if m.dist3(px, py, pz, m.x0, m.y0, m.z_high) < m.wp_tol_xy * 1.5:
                    rospy.loginfo(
                        "[PHASE] 到起点上方→降落 | EN: at home, land"
                    )
                    m.phase = m.PH_LAND
                rate.sleep()
                continue

            if m.phase == m.PH_LAND:
                try:
                    m.set_mode(0, "AUTO.LAND")
                except rospy.ServiceException as e:
                    rospy.logwarn("[MAVROS] AUTO.LAND: %s | EN: service", e)
                m.phase = m.PH_DONE
                rate.sleep()
                continue

            rate.sleep()

        rospy.loginfo(
            "[MISSION] offboard_grasp 正常结束 | EN: mission complete"
        )
    finally:
        if m._moveit_cpp_inited:
            moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
 