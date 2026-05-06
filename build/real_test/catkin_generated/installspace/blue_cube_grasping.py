#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝色立方体：D435 视觉 + RoArm-M3 串口 JSON 运动（不经过 MoveIt）
参考: https://www.waveshare.net/wiki/RoArm-M3 （JSON 指令、串口 115200、Python 例程 serial_simple_ctrl.py）

本节点完成:
  1) 与 d435_object_position_only 相同的 HSV+质心+深度+K，得到色块在 camera_optical_frame 下的 (x,y,z)（米）;
  2) 若 TF 可用，变换到 ~base_link_frame（默认机械臂基座 base_link）;
  3) 在稳定检测若干帧后，按序通过串口发送 JSON 行（每行一条指令，以 \\n 结尾）:
     可选: 先张爪 -> 再末端坐标（COORD 类）-> 再闭爪。

【务必自行核对】
  微雪 COORD/末端坐标 对应 JSON 的 "T" 值与 **x,y,z,t,r,g 含义、单位、坐标轴方向** 以
  官方《RoArm-M3_S_JSON 指令含义》/ 下位机 json_cmd.h 为准。本程序默认
  `~roarm_cmd_coord_t`：RoArm-M3-S 常用 104；设为 **0** 则拒绝运动且不发送串口。

【依赖】
  pip 安装: pyserial  （sudo apt install python3-serial 在 Ubuntu 上亦可）

【示例】
  rosrun real_test blue_cube_grasping.py _serial_port:=/dev/ttyUSB0 \\
    _roarm_cmd_coord_t:=<官方COORD对应的整数T>
  rosservice call /blue_cube_grasping/run_grasp_sequence "{}"

源码 `BlueCubeGraspingNode.__init__` 内凡标有「【必须配置】」「【需配置】」「【按需配置】」的注释处，
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


class BlueCubeGraspingNode(object):
    def __init__(self):
        rospy.init_node("blue_cube_grasping", anonymous=False)

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

        # 【需配置】蓝色块 HSV：光照/物体颜色变化时需用 rqt_reconfigure 或 rosparam 重标定上下界。
        self._hsv_lo = np.array(rospy.get_param("~hsv_lower", [70, 40, 40]), dtype=np.uint8)
        self._hsv_hi = np.array(rospy.get_param("~hsv_upper", [130, 255, 255]), dtype=np.uint8)
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
        self._coord_in_mm = bool(rospy.get_param("~coord_units_in_mm", False))

        # 【需配置】串口设备节点与波特率（官方常为 115200）；设备号随 USB 切换会变。
        self._port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self._baud = int(rospy.get_param("~serial_baud", 115200))
        self._log_rx = bool(rospy.get_param("~log_serial_rx", True))
        self._ser = None
        self._open_serial()

        # 【必须配置】末端坐标指令类型 T：查官方 JSON 表填写；为 0 时节点拒绝运动。t,r,g/spd/acc 按官方含义与单位。
        self._coord_t = int(rospy.get_param("~roarm_cmd_coord_t", 104))
        self._t_ang = float(rospy.get_param("~coord_angle_t", 0.0))
        self._r_ang = float(rospy.get_param("~coord_angle_r", 0.0))
        self._g_ang = float(rospy.get_param("~coord_angle_g", 0.0))
        self._spd = int(rospy.get_param("~coord_spd", 0))
        self._acc = int(rospy.get_param("~coord_acc", 0))
        self._include_spd = bool(rospy.get_param("~coord_json_include_spd_acc", True))

        # 【需配置】夹爪 JSON：T/cmd 须与你的固件一致（以下为 Wiki 示例）；字段是否与官方 JSON 格式一致请自查。
        self._grip_open = rospy.get_param(
            "~json_gripper_open",
            '{"T":106,"cmd":1.57,"spd":0,"acc":0}',
        )
        self._grip_close = rospy.get_param(
            "~json_gripper_close",
            '{"T":106,"cmd":3.14,"spd":0,"acc":0}',
        )
        self._send_open = bool(rospy.get_param("~send_gripper_open_first", True))
        self._send_close = bool(rospy.get_param("~send_gripper_close_last", True))
        # 【按需配置】每条指令间的延时：动作未完成可增大。
        self._delay_after_open = float(rospy.get_param("~sleep_after_gripper_open_s", 0.5))
        self._delay_after_coord = float(rospy.get_param("~sleep_after_coord_s", 1.0))
        self._delay_after_close = float(rospy.get_param("~sleep_after_gripper_close_s", 0.2))

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
            "[blue_cube_grasping] 串口=%s %d | base=%s | coord_T=%d",
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
            rospy.loginfo("[blue_cube_grasping] 串口已打开")
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

    def _diagnose_why_grasp_blocked(self):
        """服务失败时拼装可读原因（success=False 时的 message）。"""
        parts = []
        with self._lock:
            has_bgr = self._bgr is not None
            has_k = self._k is not None
            bgr_c = None if self._bgr is None else self._bgr.copy()
            k_c = None if self._k is None else self._k.copy()
        if not has_bgr:
            parts.append("未收到彩色图(检查话题 %s / realsense 是否已启动)" % self._color)
            return " ".join(parts)
        if not has_k:
            parts.append(
                "未收到相机内参(检查话题 %s 与 realsense_camera node)"
                % self._info
            )
            return " ".join(parts)
        if self._blob_uv(bgr_c) is None:
            parts.append(
                "HSV 未检出物体(场景中需有蓝色块；调 ~hsv_lower ~hsv_upper ~min_blob_area"
                " 或改抓白色时请用 blue_cube_grasping_improved)"
            )
            return " ".join(parts)
        cok, pt = self._compute()
        if not cok or pt is None:
            parts.append("视觉计算失败(少见，检查深度/内参是否与彩色同步)")
            return " ".join(parts)
        pb = (
            self._to_base(pt[0], pt[1], pt[2])
            if self._base
            else (pt[0], pt[1], pt[2])
        )
        if pb is None:
            parts.append(
                "TF 查询失败(需可查 %s <- %s；核对静态外参帧名是否与 CameraInfo.frame_id 一致)"
                % (self._base, self._optical)
            )
            return " ".join(parts)
        parts.append(
            "已检测到点且 TF 可用，但未在 %.1fs 内连续满足稳定 %d 帧 "
            "(调大 ~detection_collect_timeout / 放宽 ~stability_trans_tol_m 或 ~/detection_stable_frames)"
            % (self._collect_to, self._stable_n)
        )
        return " ".join(parts)

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
        if self._include_spd:
            d["spd"] = int(self._spd)
            d["acc"] = int(self._acc)
        return d

    def _srv_run(self, _req):
        if self._coord_t == 0:
            rospy.logwarn(
                "[blue_cube_grasping] 拒绝执行：~roarm_cmd_coord_t=0，不会发送任何 UART；"
                "请设为官方末端坐标指令 T（如 RoArm-M3-S 使用 104）。"
            )
            return TriggerResponse(
                success=False,
                message="请设 ~roarm_cmd_coord_t 为官方 COORD/末端坐标 指令的 T（当前为0）",
            )
        st, base_xyz = self._wait_stable()
        if not st or base_xyz is None:
            detail = self._diagnose_why_grasp_blocked()
            rospy.logwarn("[blue_cube_grasping] success=False：%s", detail)
            return TriggerResponse(
                success=False,
                message=detail,
            )
        x, y, z = base_xyz[0], base_xyz[1], base_xyz[2]
        # 【需配置】与上方 ~object_z_min_m / ~object_z_max_m 一致：防止打到台面或过高的误触发。
        if z < self._zmin_ok or z > self._zmax_ok:
            return TriggerResponse(
                success=False, message="目标 z 超出安全范围 [%.2f,%.2f] m" % (self._zmin_ok, self._zmax_ok),
            )
        try:
            if self._send_open and self._grip_open:
                self._write_json_line(self._grip_open)
                time.sleep(self._delay_after_open)
            coord = self._build_coord_dict(x, y, z)
            self._write_json_line(json.dumps(coord, separators=(",", ":")))
            time.sleep(self._delay_after_coord)
            if self._send_close and self._grip_close:
                self._write_json_line(self._grip_close)
                time.sleep(self._delay_after_close)
        except Exception as e:  # noqa: BLE001
            return TriggerResponse(success=False, message="串口发送失败: %s" % e)
        return TriggerResponse(
            success=True,
            message="已发送: base=(%.4f,%.4f,%.4f) m | coord=%s" % (x, y, z, coord),
        )


def main():
    BlueCubeGraspingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
