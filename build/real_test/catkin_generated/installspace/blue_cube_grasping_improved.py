#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
白色立方体：D435 视觉 + RoArm-M3 串口 JSON 运动（不经过 MoveIt）
参考: https://www.waveshare.net/wiki/RoArm-M3 （JSON 指令、串口 115200、Python 例程 serial_simple_ctrl.py）

白色在 HSV 中主要为低饱和度 S、高亮度 V，Hue 覆盖 0–179 全范围；
若桌面/背景也是浅白色，易与目标混淆，请用深色垫布、或调大 ~min_blob_area、或缩小相机视野/ROI。

本节点完成:
  1) 与 d435_object_position_only 相同的 HSV+质心+深度+K，得到色块在 camera_optical_frame 下的 (x,y,z)（米）;
  2) 若 TF 可用，变换到 ~base_link_frame（默认机械臂基座 base_link）;
  3) 在稳定检测若干帧后，按序通过串口发送 JSON 行（每行一条指令，以 \\n 结尾）:
     针对 3cm 白色立方体 + 单舵机双指夹爪：张爪 -> 预抓取点 -> 下降抓取点 -> 闭爪 -> 抬升。

【务必自行核对】
  **x,y,z,t,r,g 的单位与坐标轴** 以官方《RoArm-M3_S_JSON》/下位机为准。
  末端笛卡尔位移默认采用 Wiki 中 **CMD_XYZT_GOAL_CTRL**（T=104，x/y/z 毫米，spd 浮点）；
  参见: https://www.waveshare.com/wiki/RoArm-M3-S_Robotic_Arm_Control

【依赖】
  pip 安装: pyserial  （sudo apt install python3-serial 在 Ubuntu 上亦可）

【示例】
  rosrun real_test blue_cube_grasping_improved.py _serial_port:=/dev/ttyUSB0
  rosservice call /white_cube_grasping/run_grasp_sequence "{}"

源码 `WhiteCubeGraspingNode.__init__` 内凡标有「【必须配置】」「【需配置】」「【按需配置】」的注释处，
均为上线前应核对或通过 launch/rosparam 修改的参数。
"""

from __future__ import print_function

import json
import sys
import threading
import time

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse

try:
    import serial
except ImportError:
    serial = None

try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    do_transform_point = None


def map_uv_to_depth_res(u, v, color_shape, depth_shape):
    hi, wi = color_shape[0], color_shape[1]
    hd, wd = depth_shape[0], depth_shape[1]
    if hi == hd and wi == wd:
        return u, v
    u2 = int(round(u * wd / float(wi)))
    v2 = int(round(v * hd / float(hi)))
    return max(0, min(wd - 1, u2)), max(0, min(hd - 1, v2))


def depth_patch_median(u, v, depth_m, r, z_max_m):
    h, w = depth_m.shape[:2]
    u = max(0, min(w - 1, u))
    v = max(0, min(h - 1, v))
    r = max(0, int(r))
    p = depth_m[v - r : v + r + 1, u - r : u + r + 1]
    zhi = min(15.0, float(z_max_m))
    vals = p[np.isfinite(p) & (p > 0.05) & (p < zhi)]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def pixel_to_optical(u, v, z_m, k):
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[0, 2], k[1, 2]
    return (float(u) - cx) * z_m / fx, (float(v) - cy) * z_m / fy, float(z_m)


# ---------------------------------------------------------------------------


def _read_serial_loop(ser, log_lines):
    """后台读串口，打印固件回传（便于调参）。"""

    def _run():
        while not rospy.is_shutdown() and ser.is_open:
            try:
                if ser.in_waiting:
                    line = ser.readline()
                    if line and log_lines:
                        rospy.loginfo_throttle(1.0, "[UART RX] %s", line[:200])
            except Exception:  # noqa: BLE001
                break
            time.sleep(0.02)

    t = threading.Thread(target=_run)
    t.daemon = True
    t.start()
    return t


class WhiteCubeGraspingNode(object):
    def __init__(self):
        rospy.init_node("white_cube_grasping", anonymous=False)

        if serial is None:
            rospy.logfatal("未安装 pyserial: sudo apt install python3-serial")
            sys.exit(1)
        if do_transform_point is None:
            rospy.logfatal("需安装 tf2_geometry_msgs: sudo apt install ros-$ROS_DISTRO-tf2-geometry-msgs")
            sys.exit(1)

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._bgr = None
        self._depth = None
        self._k = None

        # 【需配置】相机话题：默认 realsense；若使用 launch 里的 camera_ns，三项须一同改掉。
        self._color = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self._depth_top = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self._info = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        # 【需配置】光学系帧名：须与 CameraInfo.header.frame_id、TF 树一致。
        self._optical = rospy.get_param("~camera_optical_frame", "camera_color_optical_frame")
        # 【需配置】机械臂基座 TF；须有 base_link <- optical 的可查询变换（见 static_transform / URDF）。
        self._base = rospy.get_param("~base_link_frame", "base_link").strip()

        # 【需配置】白色块 HSV（OpenCV：H 0–179）：低 S + 高 V；光照/背景浅色时需缩小 ~hsv_upper 的 S、调 V 或 min_blob_area。
        self._hsv_lo = np.array(rospy.get_param("~hsv_lower", [0, 0, 165]), dtype=np.uint8)
        self._hsv_hi = np.array(rospy.get_param("~hsv_upper", [179, 65, 255]), dtype=np.uint8)
        self._min_area = int(rospy.get_param("~min_blob_area", 100))
        self._er = int(rospy.get_param("~mask_erode_iters", 0))
        self._di = int(rospy.get_param("~mask_dilate_iters", 2))

        # 【需配置】深度：无对齐深度时可关 ~use_depth 并调 assume_depth_m（精度差）；object_z_* 为抓取安全边界。
        self._use_depth = bool(rospy.get_param("~use_depth", True))
        self._assume_z = float(rospy.get_param("~assume_depth_m", 0.5))
        self._z_max = float(rospy.get_param("~camera_depth_max_m", 2.0))
        self._r_med = int(rospy.get_param("~depth_median_radius", 4))
        self._zmin_ok = float(rospy.get_param("~object_z_min_m", -0.5))
        self._zmax_ok = float(rospy.get_param("~object_z_max_m", 0.85))

        # 【按需配置】稳定判定帧数、超时与位移阈值：场地振动或检测抖动时可微调。
        self._stable_n = int(rospy.get_param("~detection_stable_frames", 5))
        self._pixel_tol = float(rospy.get_param("~detection_pixel_tol", 12.0))
        self._collect_to = float(rospy.get_param("~detection_collect_timeout", 3.0))
        # 在基座系下连续帧位姿变化小于该值则计为稳定
        self._st_tol = float(rospy.get_param("~stability_trans_tol_m", 0.01))

        # 【需配置】目标点在基座系下的补偿：手眼残差、轴向定义差异时用 offset/scale；单位是否与固件一致看 coord_units_in_mm。
        self._off = np.array(rospy.get_param("~arm_target_offset_m", [0.0, 0.0, 0.0]), dtype=np.float64)
        self._scale = np.array(rospy.get_param("~arm_axis_scale", [1.0, 1.0, 1.0]), dtype=np.float64)
        # RoArm-M3-S CMD_XYZT_GOAL_CTRL(T=104) 的 x,y,z 为 **毫米**（见 Wiki CMD_XYZT_GOAL_CTRL）。
        self._coord_in_mm = bool(rospy.get_param("~coord_units_in_mm", True))

        # 【需配置】串口设备节点与波特率（官方常为 115200）；设备号随 USB 切换会变。
        self._port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self._baud = int(rospy.get_param("~serial_baud", 115200))
        self._log_rx = bool(rospy.get_param("~log_serial_rx", True))
        self._ser = None
        self._open_serial()

        # 【必须配置】末端坐标：RoArm-M3-S 为 CMD_XYZT_GOAL_CTRL -> T=104（毫米 + spd 浮点，Wiki 无 acc）。
        # 设为 0 则拒绝运动（用于强制你在 launch 里核对后再开）。
        self._coord_t = int(rospy.get_param("~roarm_cmd_coord_t", 104))
        self._t_ang = float(rospy.get_param("~coord_angle_t", 0.0))
        self._r_ang = float(rospy.get_param("~coord_angle_r", 0.0))
        self._g_ang = float(rospy.get_param("~coord_angle_g", 3.14))
        # T=104 示例里 spd 为曲线速度系数，浮点，如 0.25；0 常表示最速/由固件决定。
        self._coord_spd = float(rospy.get_param("~coord_spd", 0.25))
        self._coord_acc = int(rospy.get_param("~coord_acc", 0))
        self._include_spd = bool(rospy.get_param("~coord_json_include_spd", True))
        # T=104 官方 JSON 不含 acc；T=103 等若需要可设 True（与固件一致为准）。
        self._include_acc = bool(rospy.get_param("~coord_json_include_acc", False))
        # 兼容旧参数名：曾为 true 时同时带 spd+acc
        if rospy.has_param("~coord_json_include_spd_acc"):
            legacy = bool(rospy.get_param("~coord_json_include_spd_acc"))
            self._include_spd = legacy
            self._include_acc = legacy

        # 【需配置】单舵机双指夹爪。RoArm-M3 官方 EOAT 夹爪/腕关节常用 T=106。
        # 官方说明：T=106 控制 EOAT_JOINT，cmd 为弧度；角度减小通常为张开，增大为闭合。
        # 你换成“双指向中间收缩”的夹爪后，本质仍是一个舵机控制，因此只需要重新标定 open/close 的 cmd。
        self._gripper_t = int(rospy.get_param("~gripper_cmd_t", 106))
        self._gripper_open_cmd = float(rospy.get_param("~gripper_open_cmd_rad", 1.20))
        # 重要：3cm 方块不要完全闭死，close_cmd 应标定到“刚好夹住 3cm 方块”。
        self._gripper_close_cmd = float(rospy.get_param("~gripper_close_cmd_rad", 2.35))
        self._gripper_spd = int(rospy.get_param("~gripper_spd", 0))
        self._gripper_acc = int(rospy.get_param("~gripper_acc", 0))
        # 仍保留手写 JSON 覆盖方式；若为空，则自动用上面的 gripper_* 参数生成。
        self._grip_open = rospy.get_param("~json_gripper_open", "")
        self._grip_close = rospy.get_param("~json_gripper_close", "")
        self._send_open = bool(rospy.get_param("~send_gripper_open_first", True))
        self._send_close = bool(rospy.get_param("~send_gripper_close_last", True))

        # 【3cm 白色立方体抓取参数】
        self._cube_size = float(rospy.get_param("~cube_size_m", 0.03))
        # 如果检测点来自顶部表面，夹爪中心应比检测点低半个边长；若相机从侧面看并检测的是侧面中心，可设为 0。
        self._grasp_center_z_from_detected = float(
            rospy.get_param("~grasp_center_z_from_detected_m", -0.5 * self._cube_size)
        )
        # 预抓取点和抬升点相对“抓取中心点”的高度。默认 z 轴向上；若你的 RoArm 坐标 z 方向相反，把这两个值设为负数。
        self._pre_grasp_up = float(rospy.get_param("~pre_grasp_up_m", 0.07))
        self._lift_up = float(rospy.get_param("~lift_up_m", 0.10))
        # 夹爪中心相对末端坐标系的补偿。若末端坐标原点不在两指中心，可用这个参数修正。
        self._gripper_center_offset = np.array(
            rospy.get_param("~gripper_center_offset_m", [0.0, 0.0, 0.0]), dtype=np.float64
        )
        # 运动阶段延时：真实舵机/机械臂动作未完成时增大。
        self._delay_after_open = float(rospy.get_param("~sleep_after_gripper_open_s", 0.6))
        self._delay_after_pre = float(rospy.get_param("~sleep_after_pre_grasp_s", 1.2))
        self._delay_after_down = float(rospy.get_param("~sleep_after_down_s", 1.0))
        self._delay_after_close = float(rospy.get_param("~sleep_after_gripper_close_s", 0.6))
        self._delay_after_lift = float(rospy.get_param("~sleep_after_lift_s", 1.0))

        self._tf = tf2_ros.Buffer(rospy.Duration(30.0))
        self._tf_l = tf2_ros.TransformListener(self._tf)

        rospy.Subscriber(self._color, Image, self._on_color, queue_size=1)
        rospy.Subscriber(self._depth_top, Image, self._on_depth, queue_size=1)
        rospy.Subscriber(self._info, CameraInfo, self._on_info, queue_size=1)

        rospy.Service("~run_grasp_sequence", Trigger, self._srv_run)

        if self._coord_t == 0:
            rospy.logwarn(
                "参数 ~roarm_cmd_coord_t=0 未设置。"
                "请在官方 JSON 表中找到 **末端坐标/COORD** 对应的整数 T 后设参，否则服务将拒绝运动。"
            )
        rospy.loginfo(
            "[white_cube_grasping] 串口=%s %d | base=%s | coord_T=%d",
            self._port, self._baud, self._base, self._coord_t,
        )

    def _open_serial(self):
        try:
            self._ser = serial.Serial(
                self._port, self._baud, timeout=0.2, write_timeout=1.0
            )
            self._ser.setRTS(False)
            self._ser.setDTR(False)
            _read_serial_loop(self._ser, self._log_rx)
            rospy.loginfo("[white_cube_grasping] 串口已打开")
        except Exception as e:  # noqa: BLE001
            rospy.logfatal("无法打开串口 %s: %s", self._port, e)
            sys.exit(1)

    def _write_json_line(self, s):
        """每行一条 JSON，与 Wiki serial_simple_ctrl 一致加换行符。"""
        if not s.endswith("\n"):
            s = s + "\n"
        b = s.encode("utf-8")
        self._ser.write(b)
        self._ser.flush()
        rospy.loginfo("[UART TX] %s", s.strip()[:200])

    def _on_color(self, msg):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn("color: %s", e)
            return
        with self._lock:
            self._bgr = bgr

    def _on_depth(self, msg):
        try:
            if msg.encoding == "32FC1":
                d = self._bridge.imgmsg_to_cv2(msg, "passthrough")
            elif msg.encoding == "16UC1":
                d = (
                    self._bridge.imgmsg_to_cv2(msg, "passthrough").astype(
                        np.float32
                    )
                    * 0.001
                )
            else:
                d = self._bridge.imgmsg_to_cv2(msg, "passthrough")
                if d.dtype == np.uint16:
                    d = d.astype(np.float32) * 0.001
        except CvBridgeError as e:
            rospy.logwarn("depth: %s", e)
            return
        with self._lock:
            self._depth = np.asarray(d, dtype=np.float32)

    def _on_info(self, msg):
        with self._lock:
            self._k = np.array(msg.K, dtype=np.float64).reshape(3, 3)

    def _blob_uv(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._hsv_lo, self._hsv_hi)
        if self._er > 0:
            mask = cv2.erode(mask, None, iterations=self._er)
        if self._di > 0:
            mask = cv2.dilate(mask, None, iterations=self._di)
        cts, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        best, best_a = None, 0
        for c in cts:
            a = cv2.contourArea(c)
            if a > self._min_area and a > best_a:
                best_a, best = a, c
        if best is None:
            return None
        m = cv2.moments(best)
        if m["m00"] < 1e-6:
            return None
        u = int(m["m10"] / m["m00"])
        v = int(m["m01"] / m["m00"])
        return (u, v)

    def _compute(self):
        with self._lock:
            bgr = None if self._bgr is None else self._bgr.copy()
            dep = None if self._depth is None else self._depth.copy()
            k = None if self._k is None else self._k.copy()
        if bgr is None or k is None:
            return False, None
        uv = self._blob_uv(bgr)
        if uv is None:
            return False, None
        u, v = uv[0], uv[1]
        z = min(self._assume_z, self._z_max)
        if self._use_depth and dep is not None:
            uu, vv = map_uv_to_depth_res(u, v, bgr.shape, dep.shape)
            zd = depth_patch_median(uu, vv, dep, self._r_med, self._z_max)
            if zd is not None:
                z = min(float(zd), self._z_max)
        x, y, zz = pixel_to_optical(u, v, z, k)
        return True, (x, y, zz)

    def _to_base(self, x, y, z):
        # ~base_link_frame 为空则整条路径禁用 TF（一般不推荐）；否则依赖外部发布的 TF。
        if not self._base:
            return None
        ps = PointStamped()
        ps.header.stamp = rospy.Time(0)
        ps.header.frame_id = self._optical
        ps.point = Point(x=float(x), y=float(y), z=float(z))
        try:
            # 【按需配置】lookup 超时：TF 发布慢时可略增大 rospy.Duration
            tfm = self._tf.lookup_transform(
                self._base, self._optical, rospy.Time(0), rospy.Duration(0.5)
            )
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("TF %s<-%s: %s", self._base, self._optical, e)
            return None
        try:
            out = do_transform_point(ps, tfm)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("do_transform_point: %s", e)
            return None
        return (out.point.x, out.point.y, out.point.z)

    def _wait_stable(self):
        t0 = rospy.get_time()
        ok_cnt = 0
        last = None
        while rospy.get_time() - t0 < self._collect_to and not rospy.is_shutdown():
            cok, pt = self._compute()
            if not cok or pt is None:
                ok_cnt = 0
                last = None
                rospy.sleep(0.02)
                continue
            px, py, pz = pt[0], pt[1], pt[2]
            pb = self._to_base(px, py, pz) if self._base else (px, py, pz)
            if pb is None:
                rospy.sleep(0.02)
                continue
            if last is not None:
                # 在基座系下比较稳定性（与像素稳定二选一，这里用基座 3D 小阈值更物理）
                d = max(
                    abs(pb[0] - last[0]),
                    abs(pb[1] - last[1]),
                    abs(pb[2] - last[2]),
                )
                if d < self._st_tol:
                    ok_cnt += 1
                else:
                    ok_cnt = 1
            else:
                ok_cnt = 1
            last = pb
            if ok_cnt >= self._stable_n:
                return True, last
            rospy.sleep(0.03)
        return False, None

    def _build_coord_dict(self, x, y, z):
        d = {
            "T": int(self._coord_t),
            "x": float(x * self._scale[0] + self._off[0]),
            "y": float(y * self._scale[1] + self._off[1]),
            "z": float(z * self._scale[2] + self._off[2]),
            "t": float(self._t_ang),
            "r": float(self._r_ang),
            "g": float(self._g_ang),
        }
        if self._coord_in_mm:
            d["x"] = round(d["x"] * 1000.0, 2)
            d["y"] = round(d["y"] * 1000.0, 2)
            d["z"] = round(d["z"] * 1000.0, 2)
        # Wiki T=104 仅含 "spd"（浮点）；无 acc。T=106 单独含 spd/acc。
        if self._include_spd:
            d["spd"] = float(self._coord_spd)
        if self._include_acc:
            d["acc"] = int(self._coord_acc)
        return d

    def _make_gripper_json(self, cmd_rad):
        return json.dumps({
            "T": int(self._gripper_t),
            "cmd": float(cmd_rad),
            "spd": int(self._gripper_spd),
            "acc": int(self._gripper_acc),
        }, separators=(",", ":"))

    def _open_gripper(self):
        # 单舵机双指夹爪：张开角度需要实测标定，默认用 gripper_open_cmd_rad。
        s = self._grip_open.strip() if self._grip_open else self._make_gripper_json(self._gripper_open_cmd)
        self._write_json_line(s)

    def _close_gripper(self):
        # 3cm 方块：闭合角度应夹到略小于 3cm 的等效宽度，避免完全闭死把方块挤飞。
        s = self._grip_close.strip() if self._grip_close else self._make_gripper_json(self._gripper_close_cmd)
        self._write_json_line(s)

    def _send_coord_xyz(self, x, y, z):
        coord = self._build_coord_dict(x, y, z)
        self._write_json_line(json.dumps(coord, separators=(",", ":")))
        return coord

    def _target_points_for_cube(self, detected_xyz):
        # detected_xyz 是白色区域深度点转换到 base_link 下的位置。
        # 对 3cm 正方体，若看到的是顶部表面，抓取中心点 = 顶部点 - 1.5cm。
        base = np.array(detected_xyz, dtype=np.float64)
        grasp = base + np.array([0.0, 0.0, self._grasp_center_z_from_detected], dtype=np.float64)
        # gripper_center_offset 用来把“物块中心”换算为“机械臂末端坐标目标”。
        grasp = grasp + self._gripper_center_offset
        pre = grasp + np.array([0.0, 0.0, self._pre_grasp_up], dtype=np.float64)
        lift = grasp + np.array([0.0, 0.0, self._lift_up], dtype=np.float64)
        return tuple(pre.tolist()), tuple(grasp.tolist()), tuple(lift.tolist())

    def _srv_run(self, _req):
        if self._coord_t == 0:
            rospy.logwarn(
                "[white_cube_grasping] 拒绝执行：~roarm_cmd_coord_t=0，不会发送任何 UART；"
                "请设为官方末端坐标指令 T（如 RoArm-M3-S 使用 104）。"
            )
            return TriggerResponse(
                success=False,
                message="请设 ~roarm_cmd_coord_t 为官方 COORD/末端坐标 指令的 T（当前为0）",
            )
        st, base_xyz = self._wait_stable()
        if not st or base_xyz is None:
            rospy.logwarn(
                "[white_cube_grasping] 未发送串口：检测未稳定或缺相机/TF/深度/HSV 目标。"
            )
            return TriggerResponse(
                success=False, message="检测未稳定/无 TF/无内参/无深度。检查话题、HSV、深度图与 base 系 TF。",
            )

        detected_x, detected_y, detected_z = base_xyz[0], base_xyz[1], base_xyz[2]
        if detected_z < self._zmin_ok or detected_z > self._zmax_ok:
            return TriggerResponse(
                success=False,
                message="检测点 z 超出安全范围 [%.2f,%.2f] m" % (self._zmin_ok, self._zmax_ok),
            )

        pre_xyz, grasp_xyz, lift_xyz = self._target_points_for_cube(base_xyz)

        # 对抓取点也做一次安全检查，防止因 -1.5cm 修正后打到桌面或超出机械臂安全范围。
        if grasp_xyz[2] < self._zmin_ok or grasp_xyz[2] > self._zmax_ok:
            return TriggerResponse(
                success=False,
                message=(
                    "抓取点 z=%.4f 超出安全范围 [%.2f,%.2f] m；"
                    "请调整 ~grasp_center_z_from_detected_m 或 object_z_*"
                ) % (grasp_xyz[2], self._zmin_ok, self._zmax_ok),
            )

        sent = {}
        try:
            # 1) 张开夹爪。单舵机双指夹爪仍然只发一个 EOAT 舵机角度。
            if self._send_open:
                self._open_gripper()
                time.sleep(self._delay_after_open)

            # 2) 到白块上方预抓取点。
            sent["pre_grasp"] = self._send_coord_xyz(*pre_xyz)
            time.sleep(self._delay_after_pre)

            # 3) 下降到抓取中心高度。
            sent["grasp"] = self._send_coord_xyz(*grasp_xyz)
            time.sleep(self._delay_after_down)

            # 4) 闭合夹爪到“刚好夹住 3cm 方块”的角度，不建议完全闭死。
            if self._send_close:
                self._close_gripper()
                time.sleep(self._delay_after_close)

            # 5) 抬升，便于验证是否真正夹住。
            sent["lift"] = self._send_coord_xyz(*lift_xyz)
            time.sleep(self._delay_after_lift)

        except Exception as e:  # noqa: BLE001
            return TriggerResponse(success=False, message="串口发送失败: %s" % e)

        return TriggerResponse(
            success=True,
            message=(
                "已执行 3cm 白色立方体抓取序列 | "
                "detected=(%.4f,%.4f,%.4f)m, pre=(%.4f,%.4f,%.4f)m, "
                "grasp=(%.4f,%.4f,%.4f)m, lift=(%.4f,%.4f,%.4f)m | sent=%s"
            ) % (
                detected_x, detected_y, detected_z,
                pre_xyz[0], pre_xyz[1], pre_xyz[2],
                grasp_xyz[0], grasp_xyz[1], grasp_xyz[2],
                lift_xyz[0], lift_xyz[1], lift_xyz[2],
                sent,
            ),
        )


def main():
    WhiteCubeGraspingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
