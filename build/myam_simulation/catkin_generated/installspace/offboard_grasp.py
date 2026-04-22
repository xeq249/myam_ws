#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offboard 抓取流程（myuam + myam_grasping 场景）：
  1m 悬停 2s → 机械臂关节 [0,-1.57,2.93,0,0] → 悬停 2s → 前飞 1.6m 悬停 2s → 缓慢降至相对高度 0.4m 悬停
  → 深度相机识别蓝块位姿（终端输出）→ MoveIt 规划抓取并闭合夹爪 → 输出是否夹住
  → 机械臂回准备姿态 → 爬升至 1m 悬停 2s → 返航 → AUTO.LAND

依赖：先启动 sitl_task.launch（或 sitl_moveit + myam_grasping）与 MAVROS、move_group。

说明：抓取精度主要取决于**深度/分割 + TF** 与**机体是否对准目标**；不必在脚本里做整机重心估计。
MoveIt 在机械臂基座系规划，PX4 负责飞行姿态。伸臂后机体轻微倾斜常见；若末端总够不着，
优先检查视觉三维点是否合理（本脚本对 planning_frame 下 z 做工作区过滤）并微调前飞/高度。

ABORTED: TIMED_OUT 常见原因：① ros_control 臂轨迹 constraints.goal_time 过短；② 名义关节速度过快、
effort 跟踪慢；③ MoveIt 轨迹执行时长裕度不足；④ 单次末端位移过大，整段关节空间路径过长。
见 gazebo_controllers.yaml、trajectory_execution.launch.xml、move_group.launch 节点内同名参数；
视觉段默认 **ompl**（~vision_path_mode）+ 单段/少段位置目标；~vision_reach_seed 在 launch 中默认关。
**务必**将 myam_sys_moveit/config/joint_limits.yaml 中 default_velocity_scaling_factor 提到 1.0（原 0.1 会使名义轨迹极慢）。

无法「数学保证」一定抓起：仿真有随机性与控制极限。工程上可提高成功率：手眼标定（launch 中
~vision_target_offset_x/y/z）、抓取失败前向微调（~grasp_retry_forward_m）、home 关节留裕度、
控制器 goal_time；默认关闭 ~vision_reach_seed_enable 以免弦长人为拉大。

排障要点（相机能看见 ≠ MoveIt 一定能执行）：
  - 约 20s 的 TIMED_OUT：多为轨迹执行时长监控/effort 跟踪慢，未必是「够不着」；若 ~plan() 成功而 go 超时更印证此点。
  - RViz MotionPlanning 会通过 dynamic_reconfigure 把 monitoring/scaling 改回默认，可把 ~vision_fix_trajectory_execution_via_dynreconf 保持 true。
  - 视觉默认用位置目标（~vision_cartesian_use_pose=false）减轻姿态约束与奇异位形导致的难规划。
  - roarm-m3 等短臂（约 0.3～0.5 m）：开启 ~vision_workspace_enable，将目标径向夹紧到 ~vision_workspace_radius_max_m，
    避免质心在可达球外导致 OMPL「Unable to sample goal」长时间超时。
  - 失败时开启 ~vision_log_plan_on_fail，根据 MoveItErrorCodes 区分不可达/碰撞/无效约束等。
分步排除（与桌面文档一致）：~vision_joint_staging_enable 先到关节预摆位；~vision_preflight_plan 先仅 plan()
  预接近/下落；预接近不可行时可 ~vision_preflight_skip_pose_if_pregrasp_fails 跳过耗时 Pose 链改走关节备用。
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
        rospy.loginfo("已启动 rospy.timer.AsyncSpinner(threads=%d)", num_threads)
        return spin
    if hasattr(rospy, "AsyncSpinner"):
        spin = rospy.AsyncSpinner(num_threads)
        spin.start()
        rospy.loginfo("已启动 rospy.AsyncSpinner(threads=%d)", num_threads)
        return spin
    thr = threading.Thread(name="rospy_spin_callbacks", target=rospy.spin)
    thr.daemon = True
    thr.start()
    rospy.loginfo("已启动后台 rospy.spin() 线程")
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
    rospy.loginfo("Offboard 设定点线程 %.1f Hz 已启动", hz)
    return th


def maybe_publish_offboard(m, x, y, z, yaw):
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
        return do_transform_point(ps, tfm).point
    except Exception as e:
        rospy.logwarn_throttle(3.0, "TF %s->%s: %s", source_frame, target_frame, e)
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
        rospy.logwarn_throttle(5.0, "TF vector %s->%s: %s", source_frame, target_frame, e)
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
        self.low_rel_m = float(rospy.get_param("~low_rel_m", 0.4))
        self.forward_m = float(rospy.get_param("~forward_m", 1.6))
        # 前飞落点相对机头做侧向平移（米），与 forward_m 同时作用在 local 水平面：在 (cos,sin) 上垂向
        # 加 (sin,-cos)。约定：正值 = 从机尾向机头看，落点向**右**平移，可修正「整体偏在机头左侧」
        self.approach_lateral_m = float(rospy.get_param("~approach_lateral_m", 0.0))
        self.wp_tol_xy = float(rospy.get_param("~wp_tol_xy", 0.15))
        self.wp_tol_z = float(rospy.get_param("~wp_tol_z", 0.12))
        self.hold_sec = float(rospy.get_param("~hold_sec", 2.0))
        self.descend_duration = float(rospy.get_param("~descend_duration", 12.0))
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
        self.gripper_open = float(rospy.get_param("~gripper_open", 0.0))
        # 预抓取前「尽量张大」的目标角；默认沿用 gripper_open（本机 URDF 夹爪为 0~1.5 rad，通常小=开）
        self.gripper_open_max = float(
            rospy.get_param("~gripper_open_max", self.gripper_open)
        )
        self.gripper_close = float(rospy.get_param("~gripper_close", 1.0))
        self.gripper_goal_tolerance = float(
            rospy.get_param("~gripper_goal_tolerance", 0.2)
        )
        self.gripper_joint_name = rospy.get_param(
            "~gripper_joint_name", "link5_to_gripper_link"
        )
        self.grasp_joint_closed_threshold = float(
            rospy.get_param("~grasp_joint_closed_threshold", 0.35)
        )
        # 规划系下桌面目标大致高度带；超出则丢弃视觉点（避免 z≈1m 等错误深度导致 TIMED_OUT）
        self.vision_reframe_z_min = float(rospy.get_param("~vision_reframe_z_min", -0.25))
        self.vision_reframe_z_max = float(rospy.get_param("~vision_reframe_z_max", 0.62))
        self.grasp_ee_to_target_max_m = float(
            rospy.get_param("~grasp_ee_to_target_max_m", 0.12)
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
            "~vision_pregrasp_delta", [0.0, 0.0, 0.08]
        )
        self.vision_grasp_delta = rospy.get_param(
            "~vision_grasp_delta", [0.0, 0.0, -0.06]
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
        self.detect_refine_xy_gain = float(rospy.get_param("~detect_refine_xy_gain", 0.2))
        self.detect_refine_xy_max_step = float(
            rospy.get_param("~detect_refine_xy_max_step", 0.025)
        )

        self.ref_frame = rospy.get_param("~planning_frame", "roarm_base_link")
        self.camera_optical_frame = rospy.get_param(
            "~camera_optical_frame", "camera_color_optical_frame"
        )
        self.tf_refine_target_frame = rospy.get_param("~tf_refine_target_frame", "").strip()

        self.grasp_max_retries = int(rospy.get_param("~grasp_max_retries", 2))
        self.grasp_retry_forward_m = float(rospy.get_param("~grasp_retry_forward_m", 0.0))
        if self.grasp_retry_forward_m > 1e-6:
            rospy.logwarn(
                "grasp_retry_forward_m=%.3f：失败后机体会前飞再识别，相对台面移动会使蓝块在 "
                "roarm_base_link 下坐标跳变，日志里「预接近欧氏距」可能不降反升；不需则 launch 设为 0",
                self.grasp_retry_forward_m,
            )
        self.gripper_joint_upper = float(rospy.get_param("~gripper_joint_upper", 1.52))

        self.state = State()
        self.pose = PoseStamped()
        self.phase = self.PH_HOVER_1M
        self.bridge = CvBridge()
        self._img = None
        self._depth = None
        self._K = None
        self._stable_cnt = 0
        self._last_uv = None
        self._gripper_joint_pos = None
        self.t_phase0 = 0.0
        self._hold_since = None
        self._grasp_attempt = 0

        self.x0 = self.y0 = self.z0 = 0.0
        self.yaw0 = 0.0
        self.z_high = 0.0
        self.z_low = 0.0
        self.z_sp = 0.0
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
        rospy.Subscriber("/joint_states", JointState, self._cb_joint_states, queue_size=1)

        self.sp_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=10
        )
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        rospy.loginfo(
            "offboard_grasp: color=%s depth=%s planning_frame=%s",
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

    def _cb_joint_states(self, msg):
        try:
            i = msg.name.index(self.gripper_joint_name)
            self._gripper_joint_pos = float(msg.position[i])
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

    def get_offboard_sp_tuple(self):
        yaw_live = self.yaw_from_pose(self.pose)
        p = self.phase
        if p == self.PH_DONE:
            return None
        if p in (
            self.PH_HOVER_1M,
            self.PH_ARM_READY,
            self.PH_ARM_SETTLE,
        ):
            return self.x0, self.y0, self.z_high, yaw_live
        if p in (self.PH_FWD, self.PH_FWD_SETTLE):
            return self.x_fwd, self.y_fwd, self.z_high, yaw_live
        if p in (
            self.PH_DESCEND,
            self.PH_LOW_HOLD,
            self.PH_DETECT,
            self.PH_GRASP,
            self.PH_ARM_HOME,
        ):
            return self.x_fwd, self.y_fwd, self.z_sp, yaw_live
        if p in (self.PH_CLIMB, self.PH_CLIMB_SETTLE):
            return self.x_fwd, self.y_fwd, self.z_high, yaw_live
        if p in (self.PH_RTL, self.PH_LAND):
            return self.x0, self.y0, self.z_high, self.yaw0
        return self.x0, self.y0, self.z_high, yaw_live

    def wait_move_group(self, timeout=120.0):
        svc = rospy.get_param("~move_group_ready_service", "/move_group/get_loggers")
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            try:
                rospy.wait_for_service(svc, timeout=1.0)
                rospy.loginfo("move_group 已就绪（%s）", svc)
                return True
            except rospy.ROSException:
                rospy.loginfo_throttle(5.0, "等待 move_group …")
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
                rospy.logwarn("未安装 dynamic_reconfigure，无法强制 trajectory_execution 参数")
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
                        "dynamic_reconfigure %s → monitoring=false execution_velocity_scaling=%.2f (%s)",
                        self.move_group_trajectory_execution_ns,
                        ev,
                        reason,
                    )
                else:
                    rospy.loginfo(
                        "dynamic_reconfigure %s → monitoring=false execution_velocity_scaling=%.2f (%s)",
                        self.move_group_trajectory_execution_ns,
                        ev,
                        reason,
                    )
            else:
                rospy.loginfo_once(
                    "dynamic_reconfigure %s → monitoring=false execution_velocity_scaling=%.2f",
                    self.move_group_trajectory_execution_ns,
                    ev,
                )
        except Exception as e:
            rospy.logwarn_throttle(
                30.0,
                "dynamic_reconfigure 设置 %s 失败: %s",
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
            rospy.logwarn("无 moveit_msgs，跳过规划探针")
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
            rospy.logwarn(
                "MoveIt 规划探针「%s」: success=%s planning_time=%.3fs err=%s traj_waypoints=%d",
                label,
                ok,
                float(ptime),
                self._moveit_error_name(err.val),
                npt,
            )
            if ok:
                rospy.logwarn(
                    "→ 规划成功但 arm.go 曾失败：更可能是 **执行超时/effort 跟踪** 或 dynreconf 把 monitoring 打开，而非目标不可达。"
                )
            elif err.val == _M.NO_IK_SOLUTION:
                rospy.logwarn("→ 无解 IK：检查工作空间、关节限位或改用 vision_cartesian_use_pose=false。")
            elif err.val in (_M.START_STATE_IN_COLLISION, _M.GOAL_IN_COLLISION):
                rospy.logwarn("→ 起/终点碰撞：检查 SRDF、桌面碰撞体与自碰。")
            elif err.val == _M.PLANNING_FAILED:
                rospy.logwarn("→ 规划失败：可能不可达、奇异附近路径过长或场景过紧。")
            elif err.val == _M.INVALID_GOAL_CONSTRAINTS:
                rospy.logwarn("→ 目标约束无效：尝试放宽 vision_goal_*_tolerance 或仅位置目标。")
            elif err.val == _M.TIMED_OUT:
                rospy.logwarn(
                    "→ 探针为 TIMED_OUT 且 planning_time≈0：多为 **move 动作端未复位** 的假读数，"
                    "不等价于「目标不可达」；根因仍看前面三次 arm.go 的执行超时。"
                )
        except Exception as e:
            rospy.logwarn("规划探针异常「%s」: %s", label, e)
        finally:
            try:
                arm.clear_pose_targets()
            except Exception:
                pass

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
                apply_goal()
                plan_ok, _traj, ptime, err = arm.plan()
                en = self._moveit_error_name(err.val)
                last_err_name = en
                last_ptime = float(ptime)
                last_mode = mode_tag
                if plan_ok:
                    ok = True
                    rospy.loginfo(
                        "视觉预检「%s」: plan_ok=True mode=%s planning_time=%.2fs err=%s",
                        label,
                        mode_tag,
                        last_ptime,
                        en,
                    )
                    break
                rospy.logwarn(
                    "视觉预检「%s」: plan_ok=False mode=%s planning_time=%.2fs err=%s",
                    label,
                    mode_tag,
                    last_ptime,
                    en,
                )
            if not ok:
                rospy.logwarn(
                    "预检「%s」最终失败 (err=%s, last_mode=%s)："
                    "调 offset/悬停/碰撞体，或增大 vision_workspace_radius_max_m；"
                    "亦可关 vision_preflight_plan 或设 vision_preflight_skip_pose_if_pregrasp_fails:=false",
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
            rospy.logwarn("视觉预检异常「%s」: %s", label, ex)
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
            rospy.logerr("arm MoveGroupCommander: %s", e)
            return None
        self._maybe_fix_trajectory_execution_dynreconf(reason="arm MoveGroup 就绪后")
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
            rospy.logerr("gripper MoveGroupCommander: %s", e)
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
        g = self.detect_refine_xy_gain
        vx = g * (float(u) - ppx) / fx * zd
        vy = g * (float(v) - ppy) / fy * zd
        tgt = self.tf_refine_target_frame or self.ref_frame
        out = transform_vector(
            self.tf_buffer, Vector3(vx, vy, 0.0), tgt, self.camera_optical_frame
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

    def vision_xyz_plausible(self, xyz):
        """桌面蓝块在 roarm_base_link 下 z 通常远小于 1m；超界多为深度/背景混入。"""
        if xyz is None or len(xyz) != 3:
            return False
        z = float(xyz[2])
        if z < self.vision_reframe_z_min or z > self.vision_reframe_z_max:
            rospy.logerr(
                "视觉质心 %s 下 z=%.3f 超出合理工作区 [%.2f, %.2f]，"
                "已丢弃该次视觉抓取（易触发 MoveIt TIMED_OUT）；请改善对准/深度或调整参数 vision_reframe_z_*",
                self.ref_frame,
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
                    "质心距臂座 %.3f m 已超过 vision_workspace_radius_max_m=%.2f；"
                    "抓取链上将按球形边界夹紧目标（roarm 短臂）",
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
            "roarm 工作空间夹紧: 径向缩放 %.4f (max(norm) %.3f → 有效 %.3f m)，"
            "预接近/下落共线保留",
            s,
            rlim,
            r_eff,
        )
        return True, (pp * s).tolist(), (pf * s).tolist(), ""

    def print_blue_pose(self, u, v):
        """质心在相机光学系与 planning_frame 下的估计（米），供终端查看。"""
        pc = self.point_cam(u, v)
        if pc is None:
            rospy.logwarn("蓝块位姿: 无 camera_info 或深度无效")
            return None
        pb = transform_point(
            self.tf_buffer, pc, self.ref_frame, self.camera_optical_frame
        )
        if pb is not None:
            raw = (pb.x, pb.y, pb.z)
            pl = self.vision_xyz_plausible(raw)
            rospy.loginfo(
                "\n======== 蓝块估计位姿（质心）========\n"
                "  相机光学系 %s (m): x=%.4f y=%.4f z=%.4f\n"
                "  规划系 %s (m): x=%.4f y=%.4f z=%.4f\n"
                "  工作区检查 z∈[%.2f,%.2f]: %s\n"
                "========================================",
                self.camera_optical_frame,
                pc.x,
                pc.y,
                pc.z,
                self.ref_frame,
                pb.x,
                pb.y,
                pb.z,
                self.vision_reframe_z_min,
                self.vision_reframe_z_max,
                "通过" if pl else "未通过（本次抓取将不用视觉位姿）",
            )
            if pl:
                return raw
            return None
        rospy.loginfo(
            "\n======== 蓝块估计位姿（质心）========\n"
            "  相机光学系 %s (m): x=%.4f y=%.4f z=%.4f\n"
            "  规划系 %s: TF 失败（检查 TF 树与 %s）\n"
            "========================================",
            self.camera_optical_frame,
            pc.x,
            pc.y,
            pc.z,
            self.ref_frame,
            self.camera_optical_frame,
        )
        return None

    def _apply_vision_execution_tolerances(self, arm):
        try:
            arm.set_goal_joint_tolerance(self.vision_relaxed_joint_tolerance)
            arm.set_goal_position_tolerance(self.vision_goal_position_tolerance)
            arm.set_goal_orientation_tolerance(self.vision_goal_orientation_tolerance)
        except Exception as e:
            rospy.logwarn("视觉容差: %s", e)

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
            rospy.logwarn("预检容差: %s", e)

    def _preflight_goal_pose_approach(self, arm, pt):
        """末端 +Z 对齐接近轴（与笛卡尔段一致），便于 5 自由度在约束姿态下求 IK。"""
        pose = Pose()
        pose.position.x = float(pt[0])
        pose.position.y = float(pt[1])
        pose.position.z = float(pt[2])
        cp = None
        try:
            cp = arm.get_current_pose().pose
            fp = [float(cp.position.x), float(cp.position.y), float(cp.position.z)]
        except Exception:
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
            rospy.logwarn("%s 关节前插跳过: %s", label, e)
            return True
        if len(q0) != 5:
            return True
        n = max(1, min(12, int(self.vision_joint_lead_in_steps)))
        frac = max(0.05, min(0.5, float(self.vision_joint_lead_in_total_fraction)))
        vmin = max(0.08, min(0.95, float(self.vision_min_velocity_scale)))
        vcap = max(0.2, min(1.0, float(self.vision_waypoint_vel_cap)))
        vs = max(vmin, min(vcap, float(vs0)))
        rospy.loginfo("%s 关节空间前插 %d 步 (向 seed 共 %.0f%%)", label, n, frac * 100.0)
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
                rospy.logwarn("%s 失败", lab)
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
                rospy.logwarn("%s 关节前插失败，仍用原起点折线", label)
            else:
                joint_lead_ok = True
                try:
                    cp = arm.get_current_pose().pose.position
                    p0_list = [float(cp.x), float(cp.y), float(cp.z)]
                except Exception as e:
                    rospy.logwarn("%s 关节前插后取末端失败: %s", label, e)
                    joint_lead_ok = False
        d_after = float(
            np.linalg.norm(
                np.array(p1, dtype=np.float64) - np.array(p0_list, dtype=np.float64)
            )
        )
        if joint_lead_ok:
            rospy.loginfo(
                "%s 关节前插后末端弦长 %.3f m (前插前 %.3f m)",
                label,
                d_after,
                dtot,
            )
        if joint_lead_ok and self.vision_polyline_skip_cartesian_micro_after_joint:
            rospy.loginfo("%s 跳过笛卡尔微步，直接主折线", label)
            return self._go_vision_polyline(arm, p0_list, p1, label, vs0)
        prefix = self._polyline_lead_in_waypoints(p0_list, p1)
        if not prefix:
            return self._go_vision_polyline(arm, p0_list, p1, label, vs0)
        if self.vision_fix_trajectory_execution_via_dynreconf:
            self._maybe_fix_trajectory_execution_dynreconf(reason="视觉折线微步 %s" % label)
        dtot = float(
            np.linalg.norm(
                np.array(p1, dtype=np.float64) - np.array(p0_list, dtype=np.float64)
            )
        )
        rospy.loginfo(
            "%s 折线前微步 %d 点 (弦长≈%.3f m, 步≈%.3f m)",
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
            rospy.logwarn("%s 笛卡尔失败，回退 OMPL …", label)
        elif want_cart and skip_cs:
            rospy.logwarn(
                "%s 跨度 %.3f m > %.3f m，跳过笛卡尔仅用 OMPL",
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
        try:
            ep = arm.get_current_pose().pose.position
            rospy.loginfo(
                "视觉前关节预伸展: ok=%s 用时=%.2fs 末端约在(%.3f,%.3f,%.3f)",
                ok,
                dt,
                ep.x,
                ep.y,
                ep.z,
            )
        except Exception:
            rospy.loginfo("视觉前关节预伸展: ok=%s 用时=%.2fs", ok, dt)
        if not ok:
            rospy.logwarn(
                "视觉前关节预伸展未完全到位（仍继续视觉分段）；可调 vision_reach_seed_joints 或略降速度"
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
            rospy.logwarn("%s get_current_pose 失败: %s", label, e)
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
            "%s cartesian best_fraction=%.3f eef_step=%.4f keep_ori=%s axis_sign=%.1f wp=%s",
            label,
            best_frac,
            best_step,
            self.vision_cartesian_keep_orientation,
            self.vision_cartesian_approach_axis_sign,
            best_ptag,
        )
        if best_traj is None or best_frac < min_frac:
            rospy.logwarn(
                "%s 笛卡尔路径过短 best_fraction=%.3f < min=%.3f",
                label,
                best_frac,
                min_frac,
            )
            return False
        try:
            return bool(arm.execute(best_traj, wait=True))
        except MoveItCommanderException as e:
            rospy.logwarn("%s execute: %s", label, e)
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
            self._maybe_fix_trajectory_execution_dynreconf(reason="视觉折线 %s" % label)
        try:
            arm.set_planning_time(float(self.vision_ompl_planning_time))
            arm.set_num_planning_attempts(
                max(1, int(self.vision_ompl_planning_attempts))
            )
        except Exception:
            pass
        n = self._vision_num_segments(p0, p1)
        wps = self._interp_line_xyz(p0, p1, n)
        step_len = float(np.linalg.norm(np.array(wps[0]) - np.array(p0))) if wps else 0.0
        rospy.loginfo(
            "视觉分段 %s: %d 段 (欧氏距≈%.3f m, 首段步长≈%.3f m)",
            label,
            n,
            float(np.linalg.norm(np.array(p1, dtype=np.float64) - np.array(p0, dtype=np.float64))),
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
                rospy.logwarn("%s att %d: %s", label, att + 1, e)
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

    def _joint_pregrasp_grasp(self, arm, gripper):
        if not self._open_gripper_wide(gripper):
            rospy.logerr("预抓取：夹爪全开失败（检查 gripper_open_max / 冲突轨迹）")
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
        self.configure_arm_move_group(arm)
        gripper.set_max_velocity_scaling_factor(0.4)
        gripper.set_goal_joint_tolerance(self.gripper_goal_tolerance)
        arm.set_pose_reference_frame(self.ref_frame)
        self._vision_wp_pose_hint = ""

        if not self._open_gripper_wide(gripper):
            rospy.logerr("夹爪打开 MoveIt 失败（gripper_open_max）")
            return False

        use_vis = vision_xyz is not None and len(vision_xyz) == 3
        ok_vis = False
        motion_ok = False

        if use_vis:
            base = np.array(vision_xyz, dtype=np.float64)
            off = np.array(self.vision_target_offset, dtype=np.float64)
            d0 = np.array(self.vision_pregrasp_delta, dtype=np.float64)
            d1 = np.array(self.vision_grasp_delta, dtype=np.float64)
            p_pre = (base + off + d0).tolist()
            p_fin = (base + off + d0 + d1).tolist()
            ws_ok, p_pre, p_fin, ws_note = self._workspace_adjust_grasp_targets(
                p_pre, p_fin
            )
            try:
                arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                # 每次进入视觉段前再写一次，减轻 RViz 在运行中途改 trajectory_execution
                if time.time() - self._last_traj_dynreconf_wall > 3.0:
                    self._maybe_fix_trajectory_execution_dynreconf(reason="视觉抓取前")
                if self.vision_joint_staging_enable:
                    rospy.loginfo("vision_joint_staging → pregrasp_joints …")
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
                            "staging 未到 pregrasp_joints，继续 seed/视觉"
                        )
                p_pre_arr = np.asarray(p_pre, dtype=np.float64)
                d_before_seed = None
                if self.vision_reach_seed_enable:
                    try:
                        cpb = arm.get_current_pose()
                        pb = np.array(
                            [
                                cpb.pose.position.x,
                                cpb.pose.position.y,
                                cpb.pose.position.z,
                            ],
                            dtype=np.float64,
                        )
                        d_before_seed = float(np.linalg.norm(p_pre_arr - pb))
                    except Exception as ex:
                        rospy.logwarn("视觉 seed 前取末端位姿失败: %s", ex)
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
                        "%s → 跳过视觉 OMPL/预检，仅尝试关节 pregrasp/grasp",
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
                            "预检：预接近点无可行规划 → 跳过 Pose 末端链，"
                            "直接尝试关节 pregrasp/grasp（排障：先区分无解与执行超时）"
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
                    try:
                        cp = arm.get_current_pose()
                        p_cur = [
                            float(cp.pose.position.x),
                            float(cp.pose.position.y),
                            float(cp.pose.position.z),
                        ]
                    except Exception as e:
                        rospy.logwarn(
                            "get_current_pose 失败 (%s)，起点用质心近似", e
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
                            "关节预伸展使末端离预接近点更远 (%.3f m → %.3f m)，"
                            "回退 arm_ready_joints（常见：seed 构型与视觉目标在 roarm_base_link 下 z 方向相反）",
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
                        try:
                            cp = arm.get_current_pose()
                            p_cur = [
                                float(cp.pose.position.x),
                                float(cp.pose.position.y),
                                float(cp.pose.position.z),
                            ]
                        except Exception:
                            pass
                        dist_pre = float(
                            np.linalg.norm(
                                p_pre_arr - np.array(p_cur, dtype=np.float64)
                            )
                        )
                    rospy.loginfo(
                        "视觉接近: mode=%s 预接近欧氏距≈%.3f m (seed/回退后末端)；"
                        "预检成功策略 pre=%s fin=%s",
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
                        ok_vis = leg_a and self._vision_motion_leg(
                            arm, p_pre, p_fin, "下落", vs0
                        )
                    elif self.vision_use_segmented_pose:
                        leg_a = self._go_vision_polyline_with_leadin(
                            arm, p_cur, p_pre, "预接近", vs0
                        )
                        self._vision_wp_pose_hint = mode_fin if ok_pf_fin else ""
                        ok_vis = leg_a and self._go_vision_polyline_with_leadin(
                            arm, p_pre, p_fin, "下落", vs0
                        )
                    else:
                        leg_a = self._go_vision_waypoint(
                            arm, p_pre, "预接近", vs0
                        )
                        self._vision_wp_pose_hint = mode_fin if ok_pf_fin else ""
                        ok_vis = leg_a and self._go_vision_waypoint(
                            arm, p_fin, "下落", vs0
                        )

                if not ok_vis:
                    rospy.logwarn(
                        "视觉两步 Pose 失败或已跳过（常见 ABORTED:TIMED_OUT / 预检无解）；"
                        "将尝试关节备用 pregrasp/grasp"
                    )
                    if not self._joint_pregrasp_grasp(arm, gripper):
                        self._reset_vision_execution_tolerances(arm)
                        return False
                    ok_vis = True
            except MoveItCommanderException:
                if not self._joint_pregrasp_grasp(arm, gripper):
                    self._reset_vision_execution_tolerances(arm)
                    return False
                ok_vis = True
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
                "手臂未到达视觉目标位姿，跳过夹爪闭合（避免空夹误判）。"
                "若为 TIMED_OUT：查 trajectory_execution 是否被 RViz 改回、或提高 vision_*_vel_scale；"
                "若规划探针已打印 err：按 MoveIt 错误码区分不可达/碰撞/约束。"
            )
            return False

        gripper.set_joint_value_target([self.gripper_close])
        if not gripper.go(wait=True):
            rospy.logerr("夹爪闭合 MoveIt 失败（可能 TIMED_OUT）")
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

    def report_grasp_success(self, moveit_ok, vision_xyz_used):
        """
        启发式：须 MoveIt 全程成功 + 夹爪关到位 +（若用了视觉）末端接近目标点。
        空中闭合夹爪也会关紧，故不能仅凭关节角判定「夹住物体」。
        """
        rospy.sleep(0.4)
        jp = self._gripper_joint_pos
        jp_sane = (
            jp is not None
            and 0.0 <= jp <= self.gripper_joint_upper
        )
        if jp is not None and not jp_sane:
            rospy.logwarn(
                "夹爪关节读数 %.4f 超出合理范围 [0,%.2f]，忽略闭合判定（检查 joint_states 与 joint 名）",
                jp,
                self.gripper_joint_upper,
            )
        closed = jp_sane and jp >= self.grasp_joint_closed_threshold

        # MoveIt 失败时不应把「末端接近」判为 True（此前仅在 moveit_ok 时才算距离，导致日志矛盾）
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
                    tgt = np.array(vision_xyz_used, dtype=np.float64) + off + d0 + d1
                    cp = arm.get_current_pose().pose.position
                    dist = math.sqrt(
                        (tgt[0] - cp.x) ** 2
                        + (tgt[1] - cp.y) ** 2
                        + (tgt[2] - cp.z) ** 2
                    )
                    ee_near = dist < self.grasp_ee_to_target_max_m
                    rospy.loginfo(
                        "末端到视觉下落点距离 %.3f m（判接触阈值 < %.3f m）；"
                        "末端位姿为 arm 组 tip_link=hand_tcp（与 MoveIt SRDF 一致）；"
                        "若预检失败后仅走关节 pregrasp/grasp，末端未跟踪视觉下落点则距离大属预期",
                        dist,
                        self.grasp_ee_to_target_max_m,
                    )
                except Exception as e:
                    rospy.logwarn("末端距离判定失败: %s", e)
                    ee_near = False

        ok = bool(moveit_ok and closed and ee_near)
        rospy.logwarn(
            "【抓取结果】蓝方块是否被夹住: %s | MoveIt完成=%s, 夹爪%s=%.4f(>=%.2f), 末端接近目标=%s",
            "是（高概率）" if ok else "否或不确定",
            moveit_ok,
            self.gripper_joint_name,
            jp if jp is not None else float("nan"),
            self.grasp_joint_closed_threshold,
            ee_near,
        )
        return ok


def main():
    m = GraspMission()
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
    m.z_high = m.z0 + m.hover_rel_m
    m.z_low = m.z0 + m.low_rel_m
    m.z_sp = m.z_high
    lat = float(m.approach_lateral_m)
    m.x_fwd = m.x0 + m.forward_m * c + lat * s
    m.y_fwd = m.y0 + m.forward_m * s - lat * c

    rospy.loginfo(
        "起点 (%.2f,%.2f,%.2f)  z_high=%.2f z_low=%.2f  "
        "前飞 %.2f m + 侧向(lat) %.2f m → 落点 (%.2f,%.2f)",
        m.x0,
        m.y0,
        m.z0,
        m.z_high,
        m.z_low,
        m.forward_m,
        lat,
        m.x_fwd,
        m.y_fwd,
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
        start_offboard_setpoint_stream(
            m, m.offboard_stream_hz, use_wall_clock=m.offboard_stream_use_wall_clock
        )

    m.phase = m.PH_HOVER_1M
    m.t_phase0 = time.time()
    vision_xyz = None

    try:
        while not rospy.is_shutdown() and m.phase != m.PH_DONE:
            px = m.pose.pose.position.x
            py = m.pose.pose.position.y
            pz = m.pose.pose.position.z
            yaw = m.yaw_from_pose(m.pose)

            if m.phase == m.PH_HOVER_1M:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, yaw)
                if m.dist3(px, py, pz, m.x0, m.y0, m.z_high) < max(
                    m.wp_tol_xy, m.wp_tol_z
                ):
                    if m._hold_since is None:
                        m._hold_since = time.time()
                    elif time.time() - m._hold_since > m.hold_sec:
                        rospy.loginfo("1m 悬停完成 → 机械臂准备姿态")
                        m._hold_since = None
                        m.phase = m.PH_ARM_READY
                else:
                    m._hold_since = None
                rate.sleep()
                continue

            if m.phase == m.PH_ARM_READY:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, yaw)
                rospy.loginfo("MoveIt: 机械臂 → arm_ready_joints …")
                ok = m.exec_arm_joints(
                    m.arm_ready_joints,
                    vel_scale=m.arm_ready_vel_scaling,
                    accel_scale=m.arm_ready_accel_scaling,
                )
                if not ok:
                    rospy.logwarn(
                        "准备姿态失败，使用保守关节 + 更慢速度 + 略松关节容差重试 …"
                    )
                    ok = m.exec_arm_joints(
                        m.arm_ready_joints_fallback,
                        vel_scale=0.05,
                        accel_scale=0.05,
                        joint_tol=0.2,
                    )
                if not ok:
                    rospy.logerr("机械臂准备姿态仍失败（易影响后续抓取）")
                m.phase = m.PH_ARM_SETTLE
                m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.PH_ARM_SETTLE:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, yaw)
                if time.time() - m.t_phase0 >= m.hold_sec:
                    rospy.loginfo("准备姿态后悬停结束 → 前飞 %.2f m", m.forward_m)
                    m.phase = m.PH_FWD
                    m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.PH_FWD:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_high, yaw)
                if m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_high) < m.wp_tol_xy * 1.2:
                    rospy.loginfo("到达前飞点 → 悬停 %.1f s", m.hold_sec)
                    m.phase = m.PH_FWD_SETTLE
                    m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.PH_FWD_SETTLE:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_high, yaw)
                if time.time() - m.t_phase0 >= m.hold_sec:
                    m.z_sp = m.z_high
                    m.phase = m.PH_DESCEND
                    m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.PH_DESCEND:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                t = time.time() - m.t_phase0
                alpha = min(1.0, t / max(0.5, m.descend_duration))
                m.z_sp = m.z_high + (m.z_low - m.z_high) * alpha
                if alpha >= 1.0 and m.dist3(px, py, pz, m.x_fwd, m.y_fwd, m.z_low) < max(
                    m.wp_tol_xy, m.wp_tol_z
                ):
                    rospy.loginfo("已缓慢降至低高度 → 维持悬停 %.1f s", m.low_hold_sec)
                    m.z_sp = m.z_low
                    m.phase = m.PH_LOW_HOLD
                    m.t_phase0 = time.time()
                rate.sleep()
                continue

            if m.phase == m.PH_LOW_HOLD:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                if time.time() - m.t_phase0 >= m.low_hold_sec:
                    m._stable_cnt = 0
                    m._last_uv = None
                    m._grasp_attempt = 0
                    m.phase = m.PH_DETECT
                rate.sleep()
                continue

            if m.phase == m.PH_DETECT:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                uv = m.detect_blue_uv()
                if uv is not None:
                    if m._stable_cnt < m.detection_stable:
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
                    rospy.loginfo_throttle(3.0, "检测中…未分割到足够大蓝块")
                if m._stable_cnt >= m.detection_stable and m._last_uv is not None:
                    u, v = m._last_uv
                    vision_xyz = m.print_blue_pose(u, v)
                    rospy.loginfo("检测稳定 → 开始抓取")
                    m.phase = m.PH_GRASP
                rate.sleep()
                continue

            if m.phase == m.PH_GRASP:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                if m._last_uv is None:
                    m.phase = m.PH_DETECT
                    rate.sleep()
                    continue
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
                        "抓取失败：沿机头前向微调 %.3f m 后重新检测（%d/%d）",
                        m.grasp_retry_forward_m,
                        m._grasp_attempt,
                        m.grasp_max_retries,
                    )
                    vision_xyz = None
                    m._stable_cnt = 0
                    m._last_uv = None
                    m.phase = m.PH_DETECT
                rate.sleep()
                continue

            if m.phase == m.PH_ARM_HOME:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_sp, yaw)
                rospy.loginfo("机械臂回到准备关节 …")
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
                m.t_phase0 = time.time()
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
                        m.t_phase0 = time.time()
                else:
                    m._hold_since = None
                rate.sleep()
                continue

            if m.phase == m.PH_CLIMB_SETTLE:
                maybe_publish_offboard(m, m.x_fwd, m.y_fwd, m.z_high, yaw)
                if time.time() - m.t_phase0 >= m.hold_sec:
                    rospy.loginfo("1m 悬停结束 → 返航")
                    m.phase = m.PH_RTL
                rate.sleep()
                continue

            if m.phase == m.PH_RTL:
                maybe_publish_offboard(m, m.x0, m.y0, m.z_high, m.yaw0)
                if m.dist3(px, py, pz, m.x0, m.y0, m.z_high) < m.wp_tol_xy * 1.5:
                    rospy.loginfo("到达起点上方 → 降落")
                    m.phase = m.PH_LAND
                rate.sleep()
                continue

            if m.phase == m.PH_LAND:
                try:
                    m.set_mode(0, "AUTO.LAND")
                except rospy.ServiceException as e:
                    rospy.logwarn("%s", e)
                m.phase = m.PH_DONE
                rate.sleep()
                continue

            rate.sleep()

        rospy.loginfo("offboard_grasp 流程结束")
    finally:
        if m._moveit_cpp_inited:
            moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
