#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D435 彩色 + 深度：HSV 连通域质心 -> 相机系 3D -> TF 到 roarm_base_link，
再调用 MoveIt（arm / gripper）做预接近与抓取。

不依赖 MAVROS、Gazebo、PX4；需已运行 realsense2_camera、move_group、
以及 camera_optical_frame -> planning_frame 的 TF（静态或 robot_state_publisher）。

服务（默认私有命名空间下，见 launch）::
  ~/run_grasp   (std_srvs/Trigger)  检测稳定后执行一次抓取
  ~/peek_pose  (std_srvs/Trigger)  仅打印当前帧质心在 planning_frame 下的 3D（不运动）
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
from geometry_msgs.msg import Point, PointStamped
from moveit_commander import MoveGroupCommander
from moveit_commander.exception import MoveItCommanderException
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse

try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    do_transform_point = None


def _start_async_spinner(threads=4):
    try:
        from rospy.timer import AsyncSpinner as _AsyncSpinner
    except ImportError:
        _AsyncSpinner = None
    if _AsyncSpinner is not None:
        spin = _AsyncSpinner(threads)
        spin.start()
        return spin
    if hasattr(rospy, "AsyncSpinner"):
        spin = rospy.AsyncSpinner(threads)
        spin.start()
        return spin
    thr = threading.Thread(target=rospy.spin)
    thr.daemon = True
    thr.start()
    return thr


def _wait_move_group(timeout_sec=120.0):
    t0 = time.time()
    svcs = rospy.get_param(
        "~move_group_ready_services",
        ["/get_planning_scene", "/move_group/get_planning_scene"],
    )
    if isinstance(svcs, str):
        svcs = [svcs]
    while time.time() - t0 < timeout_sec and not rospy.is_shutdown():
        for svc in svcs:
            try:
                rospy.wait_for_service(svc, timeout=0.35)
                rospy.loginfo("[real_grasp] move_group 就绪: %s", svc)
                return True
            except rospy.ROSException:
                continue
        rospy.loginfo_throttle(
            5.0, "[real_grasp] 等待 move_group … (%s)", ", ".join(svcs)
        )
    return False


def _transform_point(buf, pt, target_frame, source_frame, timeout=2.0):
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
        rospy.logwarn(
            "[TF] %s <- %s 失败: %s", target_frame, source_frame, e
        )
        return None


class D435BlobGraspNode(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._img = None
        self._depth = None
        self._K = None

        self.color_topic = rospy.get_param(
            "~color_topic", "/camera/color/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.camera_optical_frame = rospy.get_param(
            "~camera_optical_frame", "camera_color_optical_frame"
        )
        self.planning_frame = rospy.get_param(
            "~planning_frame", "roarm_base_link"
        )

        self.hsv_lower = np.array(
            rospy.get_param("~hsv_lower", [72, 18, 25]), dtype=np.int32
        )
        self.hsv_upper = np.array(
            rospy.get_param("~hsv_upper", [138, 255, 255]), dtype=np.int32
        )
        self.min_blob_area = int(rospy.get_param("~min_blob_area", 80))
        self.mask_erode_iters = int(rospy.get_param("~mask_erode_iters", 0))
        self.mask_dilate_iters = int(rospy.get_param("~mask_dilate_iters", 2))

        self.camera_depth_max_m = float(
            rospy.get_param("~camera_depth_max_m", 2.0)
        )
        self.depth_median_radius = int(
            rospy.get_param("~depth_median_radius", 4)
        )
        self.use_depth = bool(rospy.get_param("~use_depth", True))
        self.assume_depth_m = float(rospy.get_param("~assume_depth_m", 0.45))

        self.vision_target_offset = np.array(
            rospy.get_param("~vision_target_offset", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        self.vision_pregrasp_delta = np.array(
            rospy.get_param("~vision_pregrasp_delta", [0.0, 0.0, 0.08]),
            dtype=np.float64,
        )
        self.vision_grasp_delta = np.array(
            rospy.get_param("~vision_grasp_delta", [0.0, 0.0, -0.04]),
            dtype=np.float64,
        )
        self.vision_tcp_extra_z_m = float(
            rospy.get_param("~vision_tcp_extra_z_m", 0.0)
        )

        self.blob_z_min = float(rospy.get_param("~blob_z_min", -0.35))
        self.blob_z_max = float(rospy.get_param("~blob_z_max", 0.75))

        self.detection_stable_frames = int(
            rospy.get_param("~detection_stable_frames", 4)
        )
        self.detection_pixel_tol = float(
            rospy.get_param("~detection_pixel_tol", 12.0)
        )
        self.detection_collect_timeout = float(
            rospy.get_param("~detection_collect_timeout", 2.5)
        )

        self.arm_group = rospy.get_param("~arm_group", "arm")
        self.gripper_group = rospy.get_param("~gripper_group", "gripper")
        self.ee_tcp_frame = rospy.get_param("~ee_tcp_frame", "hand_tcp")

        self.gripper_open = float(rospy.get_param("~gripper_open", 1.5))
        self.gripper_close = float(rospy.get_param("~gripper_close", 0.0))

        self.use_named_pregrasp = bool(
            rospy.get_param("~use_named_pregrasp", True)
        )
        self.pregrasp_named_target = rospy.get_param(
            "~pregrasp_named_target", "pregrasp"
        )
        self.use_joint_pregrasp = bool(
            rospy.get_param("~use_joint_pregrasp", False)
        )
        self.joint_pregrasp = rospy.get_param(
            "~joint_pregrasp",
            [0.0, -1.02, 2.28, -0.52, 0.0],
        )

        self.arm_vel = float(rospy.get_param("~arm_vel_scaling", 0.22))
        self.arm_acc = float(rospy.get_param("~arm_accel_scaling", 0.22))
        self.planning_time = float(rospy.get_param("~planning_time_sec", 25.0))
        self.exec_retries = int(rospy.get_param("~execute_max_retries", 5))
        self.exec_scale_decay = float(
            rospy.get_param("~execute_retry_vel_scale_decay", 0.58)
        )
        self.post_exec_settle = float(
            rospy.get_param("~post_exec_settle_sec", 0.25)
        )

        self.z_backoff_steps = int(rospy.get_param("~position_goal_z_backoff_steps", 5))
        self.z_backoff_dz = float(rospy.get_param("~position_goal_z_backoff_dz", 0.028))

        self.cartesian_final = bool(
            rospy.get_param("~cartesian_final_approach", True)
        )
        self.cartesian_eef_step = float(
            rospy.get_param("~cartesian_eef_step", 0.01)
        )
        self.cartesian_min_fraction = float(
            rospy.get_param("~cartesian_min_fraction", 0.75)
        )
        self.cartesian_avoid_collision = bool(
            rospy.get_param("~cartesian_avoid_collision", True)
        )
        self.grasp_xy_retry_offsets = rospy.get_param(
            "~grasp_xy_retry_offsets",
            [[0.0, 0.0], [0.006, 0.0], [-0.006, 0.0], [0.0, 0.006], [0.0, -0.006]],
        )

        self.return_home = bool(rospy.get_param("~return_home", True))
        self.home_named_target = rospy.get_param("~home_named_target", "home")
        self.retreat_delta_z = float(rospy.get_param("~retreat_delta_z", 0.10))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
        self._tf_ls = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.color_topic, Image, self._cb_color, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._cb_depth, queue_size=1)
        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._cb_info, queue_size=1
        )

        self._arm = None
        self._gripper = None
        self._moveit_inited = False
        self._moveit_lock = threading.Lock()

        rospy.Service("~run_grasp", Trigger, self._srv_run_grasp)
        rospy.Service("~peek_pose", Trigger, self._srv_peek_pose)

        rospy.loginfo(
            "[real_grasp] color=%s depth=%s | optical=%s -> %s",
            self.color_topic,
            self.depth_topic,
            self.camera_optical_frame,
            self.planning_frame,
        )

    def _cb_color(self, msg):
        try:
            im = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn("[real_grasp] color cv_bridge: %s", e)
            return
        with self._lock:
            self._img = im

    def _cb_depth(self, msg):
        try:
            enc = msg.encoding
            if enc == "32FC1":
                d = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            elif enc == "16UC1":
                d = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(
                    np.float32
                ) * 0.001
            else:
                d = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if d.dtype == np.uint16:
                    d = d.astype(np.float32) * 0.001
        except CvBridgeError as e:
            rospy.logwarn("[real_grasp] depth cv_bridge: %s", e)
            return
        with self._lock:
            self._depth = np.asarray(d, dtype=np.float32)

    def _cb_info(self, msg):
        k = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        with self._lock:
            self._K = k

    def _blue_mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lo = np.clip(self.hsv_lower, 0, 255).astype(np.uint8)
        hi = np.clip(self.hsv_upper, 0, 255).astype(np.uint8)
        mask = cv2.inRange(hsv, lo, hi)
        if self.mask_erode_iters > 0:
            mask = cv2.erode(mask, None, iterations=self.mask_erode_iters)
        if self.mask_dilate_iters > 0:
            mask = cv2.dilate(mask, None, iterations=self.mask_dilate_iters)
        return mask

    def detect_blob_uv(self, bgr):
        mask = self._blue_mask(bgr)
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

    def _uv_in_depth(self, u, v, hi, wi, hd, wd):
        if hi == hd and wi == wd:
            return u, v
        u2 = int(round(u * wd / float(wi)))
        v2 = int(round(v * hd / float(hi)))
        return max(0, min(wd - 1, u2)), max(0, min(hd - 1, v2))

    def depth_median(self, u, v, depth_img, color_shape):
        if depth_img is None:
            return None
        hi, wi = color_shape[:2]
        h, w = depth_img.shape[:2]
        u, v = self._uv_in_depth(u, v, hi, wi, h, w)
        r = max(0, int(self.depth_median_radius))
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))
        patch = depth_img[v - r : v + r + 1, u - r : u + r + 1]
        zmax = min(15.0, self.camera_depth_max_m)
        vals = patch[np.isfinite(patch) & (patch > 0.05) & (patch < zmax)]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def point_cam(self, u, v, depth_img, color_shape):
        z = min(self.assume_depth_m, self.camera_depth_max_m)
        if self.use_depth:
            zd = self.depth_median(u, v, depth_img, color_shape)
            if zd is not None:
                z = min(zd, self.camera_depth_max_m)
        with self._lock:
            k = None if self._K is None else self._K.copy()
        if k is None:
            return None
        fx, fy = k[0, 0], k[1, 1]
        cx, cy = k[0, 2], k[1, 2]
        p = Point()
        p.x = (u - cx) * z / fx
        p.y = (v - cy) * z / fy
        p.z = z
        return p

    def blob_xyz_planning_frame(self):
        with self._lock:
            img = None if self._img is None else self._img.copy()
            depth = None if self._depth is None else self._depth.copy()
        if img is None:
            return None, "无彩色图"
        uv = self.detect_blob_uv(img)
        if uv is None:
            return None, "未检测到色块"
        pc = self.point_cam(uv[0], uv[1], depth, img.shape)
        if pc is None:
            return None, "无 camera_info 或深度无效"
        pb = _transform_point(
            self.tf_buffer,
            pc,
            self.planning_frame,
            self.camera_optical_frame,
            timeout=3.0,
        )
        if pb is None:
            return None, "TF 到 %s 失败" % self.planning_frame
        z = float(pb.z)
        if z < self.blob_z_min or z > self.blob_z_max:
            return (
                None,
                "质心 z=%.3f 超出合理范围 [%.2f, %.2f]" % (z, self.blob_z_min, self.blob_z_max),
            )
        return (float(pb.x), float(pb.y), float(pb.z)), "OK"

    def _collect_stable_blob(self):
        t0 = rospy.get_time()
        stable = 0
        last_uv = None
        while rospy.get_time() - t0 < self.detection_collect_timeout:
            with self._lock:
                img = None if self._img is None else self._img.copy()
            if img is None:
                rospy.sleep(0.03)
                continue
            uv = self.detect_blob_uv(img)
            if uv is None:
                stable = 0
                last_uv = None
                rospy.sleep(0.02)
                continue
            if last_uv is not None:
                du = abs(uv[0] - last_uv[0])
                dv = abs(uv[1] - last_uv[1])
                if du < self.detection_pixel_tol and dv < self.detection_pixel_tol:
                    stable += 1
                else:
                    stable = 1
            else:
                stable = 1
            last_uv = uv
            if stable >= self.detection_stable_frames:
                return uv
            rospy.sleep(0.02)
        return None

    def _ensure_moveit(self):
        with self._moveit_lock:
            if self._moveit_inited:
                return self._arm is not None and self._gripper is not None
            if not _wait_move_group(
                float(rospy.get_param("~move_group_wait_sec", 120.0))
            ):
                return False
            rospy.sleep(0.5)
            moveit_commander.roscpp_initialize(sys.argv)
            try:
                self._arm = MoveGroupCommander(self.arm_group)
                self._gripper = MoveGroupCommander(self.gripper_group)
            except Exception as e:
                rospy.logerr("[real_grasp] MoveGroup 创建失败: %s", e)
                moveit_commander.roscpp_shutdown()
                return False
            self._arm.set_planning_time(self.planning_time)
            self._arm.set_num_planning_attempts(10)
            self._arm.set_pose_reference_frame(self.planning_frame)
            try:
                self._arm.set_end_effector_link(self.ee_tcp_frame)
            except Exception:
                pass
            try:
                self._arm.set_goal_position_tolerance(0.04)
                self._arm.set_goal_orientation_tolerance(0.6)
            except Exception:
                pass
            self._arm.set_max_velocity_scaling_factor(self.arm_vel)
            self._arm.set_max_acceleration_scaling_factor(self.arm_acc)
            self._gripper.set_max_velocity_scaling_factor(0.5)
            self._gripper.set_max_acceleration_scaling_factor(0.5)
            self._moveit_inited = True
            rospy.loginfo("[real_grasp] MoveIt 已初始化")
            return True

    def _plan_execute_at_z(self, label, px, py, pz):
        arm = self._arm
        v0, a0 = self.arm_vel, self.arm_acc
        try:
            arm.clear_pose_targets()
        except MoveItCommanderException:
            pass
        arm.set_position_target([float(px), float(py), float(pz)])
        for attempt in range(max(1, self.exec_retries)):
            scale = max(0.06, self.exec_scale_decay ** attempt)
            arm.set_max_velocity_scaling_factor(min(1.0, v0 * scale))
            arm.set_max_acceleration_scaling_factor(min(1.0, a0 * scale))
            try:
                arm.set_start_state_to_current_state()
            except Exception:
                pass
            ok_plan, traj, _, err = False, None, None, None
            try:
                ok_plan, traj, _, err = arm.plan()
            except MoveItCommanderException as e:
                rospy.logwarn("%s: plan 异常 %s", label, e)
            npts = 0
            if traj is not None and hasattr(traj, "joint_trajectory"):
                npts = len(traj.joint_trajectory.points)
            if not ok_plan or traj is None or npts < 1:
                rospy.logwarn(
                    "%s: 规划失败 attempt %d err=%s",
                    label,
                    attempt + 1,
                    getattr(err, "val", err),
                )
                continue
            try:
                ex_ok = arm.execute(traj, wait=True)
            except MoveItCommanderException as e:
                rospy.logwarn("%s: execute 异常 %s", label, e)
                ex_ok = False
            if ex_ok:
                arm.set_max_velocity_scaling_factor(v0)
                arm.set_max_acceleration_scaling_factor(a0)
                return True
            rospy.sleep(self.post_exec_settle)
        arm.set_max_velocity_scaling_factor(v0)
        arm.set_max_acceleration_scaling_factor(a0)
        return False

    def _plan_execute_position(self, label, px, py, pz_nominal):
        pz_nominal = float(pz_nominal)
        for zb in range(max(1, self.z_backoff_steps + 1)):
            pz = pz_nominal - zb * self.z_backoff_dz
            lab = label if zb == 0 else "%s Z=%.3f" % (label, pz)
            if self._plan_execute_at_z(lab, px, py, pz):
                return True
        return False

    def _go_xyz(self, label, px, py, pz):
        return self._plan_execute_position(label, px, py, pz)

    def _cartesian_line_to(self, label, xg, yg, zg):
        arm = self._arm
        try:
            arm.clear_pose_targets()
        except MoveItCommanderException:
            pass
        cur = arm.get_current_pose().pose
        dist = math.sqrt(
            (cur.position.x - xg) ** 2
            + (cur.position.y - yg) ** 2
            + (cur.position.z - zg) ** 2
        )
        if dist < 0.0015:
            return True
        n_seg = max(2, min(48, int(dist / max(0.004, self.cartesian_eef_step)) + 1))
        wps = []
        for k in range(1, n_seg + 1):
            alpha = k / float(n_seg)
            p = copy.deepcopy(cur)
            p.position.x = cur.position.x * (1.0 - alpha) + xg * alpha
            p.position.y = cur.position.y * (1.0 - alpha) + yg * alpha
            p.position.z = cur.position.z * (1.0 - alpha) + zg * alpha
            wps.append(p)
        v0, a0 = self.arm_vel, self.arm_acc
        arm.set_max_velocity_scaling_factor(min(1.0, max(0.06, v0 * 0.82)))
        arm.set_max_acceleration_scaling_factor(min(1.0, max(0.06, a0 * 0.82)))
        try:
            arm.set_start_state_to_current_state()
        except Exception:
            pass
        try:
            traj, fraction = arm.compute_cartesian_path(
                wps,
                self.cartesian_eef_step,
                avoid_collisions=self.cartesian_avoid_collision,
            )
        except MoveItCommanderException as e:
            rospy.logwarn("%s: 笛卡尔规划异常 %s", label, e)
            arm.set_max_velocity_scaling_factor(v0)
            arm.set_max_acceleration_scaling_factor(a0)
            return False
        if fraction + 1e-4 < self.cartesian_min_fraction:
            rospy.logwarn(
                "%s: 笛卡尔 fraction=%.2f < %.2f",
                label,
                fraction,
                self.cartesian_min_fraction,
            )
            arm.set_max_velocity_scaling_factor(v0)
            arm.set_max_acceleration_scaling_factor(a0)
            return False
        try:
            ex_ok = arm.execute(traj, wait=True)
        except MoveItCommanderException as e:
            rospy.logwarn("%s: execute %s", label, e)
            ex_ok = False
        arm.set_max_velocity_scaling_factor(v0)
        arm.set_max_acceleration_scaling_factor(a0)
        return bool(ex_ok)

    def _reach_grasp_xy(self, gx, gy, gz):
        for off in self.grasp_xy_retry_offsets:
            if len(off) < 2:
                continue
            odx, ody = float(off[0]), float(off[1])
            x, y = gx + odx, gy + ody
            if self.cartesian_final:
                if self._cartesian_line_to("末段笛卡尔下落", x, y, gz):
                    return True
            else:
                if self._go_xyz("抓取位 OMPL", x, y, gz):
                    return True
        return False

    def _targets_from_base(self, base_xyz):
        b = np.asarray(base_xyz, dtype=np.float64).reshape(3)
        off = self.vision_target_offset
        d0 = self.vision_pregrasp_delta
        d1 = self.vision_grasp_delta
        p_pre = (b + off + d0).tolist()
        p_fin = (b + off + d0 + d1).tolist()
        ez = float(self.vision_tcp_extra_z_m)
        if abs(ez) > 1e-9:
            p_pre[2] += ez
            p_fin[2] += ez
        return p_pre, p_fin

    def _srv_peek_pose(self, _req):
        xyz, msg = self.blob_xyz_planning_frame()
        if xyz is None:
            return TriggerResponse(success=False, message=msg)
        return TriggerResponse(
            success=True,
            message="%s: (%.4f, %.4f, %.4f) m" % (self.planning_frame, xyz[0], xyz[1], xyz[2]),
        )

    def _srv_run_grasp(self, _req):
        if do_transform_point is None:
            return TriggerResponse(
                success=False, message="缺少 tf2_geometry_msgs"
            )
        if not self._ensure_moveit():
            return TriggerResponse(success=False, message="MoveIt 未就绪")
        uv = self._collect_stable_blob()
        if uv is None:
            return TriggerResponse(
                success=False,
                message="检测不稳定或超时（调 HSV/光照/aligned_depth）",
            )
        xyz, msg = self.blob_xyz_planning_frame()
        if xyz is None:
            return TriggerResponse(success=False, message=msg)
        p_pre, p_fin = self._targets_from_base(xyz)
        rospy.loginfo(
            "[real_grasp] base=(%.3f,%.3f,%.3f) pre=(%.3f,%.3f,%.3f) fin=(%.3f,%.3f,%.3f)",
            xyz[0],
            xyz[1],
            xyz[2],
            p_pre[0],
            p_pre[1],
            p_pre[2],
            p_fin[0],
            p_fin[1],
            p_fin[2],
        )

        g = self._gripper
        g.set_joint_value_target([self.gripper_open])
        if not g.go(wait=True):
            return TriggerResponse(success=False, message="夹爪张开失败")

        if self.use_named_pregrasp:
            try:
                self._arm.set_named_target(self.pregrasp_named_target)
                if not self._arm.go(wait=True):
                    return TriggerResponse(
                        success=False,
                        message="命名位姿 %s 失败" % self.pregrasp_named_target,
                    )
            except Exception as e:
                return TriggerResponse(
                    success=False,
                    message="命名位姿不可用: %s" % e,
                )
        elif self.use_joint_pregrasp:
            try:
                self._arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            self._arm.set_joint_value_target(
                [float(x) for x in self.joint_pregrasp]
            )
            if not self._arm.go(wait=True):
                return TriggerResponse(success=False, message="joint_pregrasp 失败")

        if not self._go_xyz("预接近", p_pre[0], p_pre[1], p_pre[2]):
            return TriggerResponse(success=False, message="预接近规划/执行失败")

        if not self._reach_grasp_xy(p_fin[0], p_fin[1], p_fin[2]):
            return TriggerResponse(success=False, message="抓取位失败")

        g.set_joint_value_target([self.gripper_close])
        if not g.go(wait=True):
            rospy.logwarn("[real_grasp] 闭爪 go 返回 false，仍尝试抬升")

        rz = p_fin[2] + self.retreat_delta_z
        if not self._go_xyz("抬升", p_fin[0], p_fin[1], rz):
            rospy.logwarn("[real_grasp] 抬升失败（可手动收回）")

        if self.return_home:
            try:
                self._arm.clear_pose_targets()
            except MoveItCommanderException:
                pass
            try:
                self._arm.set_named_target(self.home_named_target)
                self._arm.go(wait=True)
            except Exception:
                rospy.logwarn("[real_grasp] 回 home 跳过")

        return TriggerResponse(success=True, message="抓取流程已执行（请目视确认）")


def main():
    rospy.init_node("d435_blob_grasp_node")
    _start_async_spinner(threads=6)
    D435BlobGraspNode()
    rospy.loginfo("[real_grasp] 就绪：rosservice call .../run_grasp 或 peek_pose")
    rospy.spin()


if __name__ == "__main__":
    main()
