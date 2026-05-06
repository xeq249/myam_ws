#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手外：用多对 3D 对应点（同一点在基座系 p_b 与相机光学系 p_o）做刚体 Kabsch，
解算 p_b = R * p_o + t，并输出可喂给 static_transform_publisher 的
parent=基座, child=相机光学系 的平移+四元数。

标定方法（与 MoveIt/官方接口无关，仅需 TF 与 D435 话题）：
  1) 在标定物上选若干可重复识别的点（如色块质心+深度、或标定球心）。
  2) 对每个点：在基座下记录位置（如末端针尖/指尖 TF 在 roarm_base_link 下的平移），
     同时在相机光学系下记录同一点的 (x,y,z)（与 d435_object_position_only 同模型）。
  3) 采集 >=3 对（建议 >=6 且分布到工作空间不同区域），调用 ~compute 得到外参。

采集方式二选一（或混用）：
  ~add_sample_from_vision: std_srvs/Trigger
      用当前 TF: base <- tip_frame 的平移作为 p_b，用 D435+HSV+深度 得到 p_o
      —— 须让针尖与视觉目标「物理上」为同一点。
  real_test/AddEyeToHandSample: 手填 6 个数。

  ~compute: 求解、打印、写 ~result_path（YAML 片段，含 static_transform 命令行）
  ~reset: 清空样本
  ~publish_tf: 参数为 true 时，compute 后在本节点内发布 static TF（调试用）
"""

from __future__ import print_function

import os
import threading

import cv2
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse

from real_test.srv import AddEyeToHandSample, AddEyeToHandSampleResponse


# ---------------------------------------------------------------------------
# 与 real_roarm_d435_grasp 中相同的彩色/深度 -> 光学系

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
def kabsch_rigid(P_o, P_b):
    """
    求 R,t 使 p_b ≈ R @ p_o + t（N×3, 行向量）。
    返回 R(3,3), t(3,)。
    """
    assert P_o.shape == P_b.shape and P_o.shape[1] == 3
    n = P_o.shape[0]
    if n < 3:
        raise ValueError("至少需要 3 个对应点")
    co = P_o.mean(axis=0)
    cb = P_b.mean(axis=0)
    X = P_o - co
    Y = P_b - cb
    h = X.T @ Y
    u, s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = cb - r @ co
    return r, t


# ---------------------------------------------------------------------------


class CameraBaseCalibEyeToHand(object):
    def __init__(self):
        rospy.init_node("camera_base_calib_eye_to_hand", anonymous=False)

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._bgr = None
        self._depth = None
        self._k = None

        self._base = rospy.get_param("~base_link_frame", "roarm_base_link")
        self._optical = rospy.get_param(
            "~camera_optical_frame", "camera_color_optical_frame"
        )
        self._tip = rospy.get_param("~gripper_tip_frame", "roarm_link7").strip()

        self._color = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self._depth_top = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self._info = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )

        self._hsv_lo = np.array(
            rospy.get_param("~hsv_lower", [70, 40, 40]), dtype=np.uint8
        )
        self._hsv_hi = np.array(
            rospy.get_param("~hsv_upper", [130, 255, 255]), dtype=np.uint8
        )
        self._min_area = int(rospy.get_param("~min_blob_area", 100))
        self._er = int(rospy.get_param("~mask_erode_iters", 0))
        self._di = int(rospy.get_param("~mask_dilate_iters", 2))
        self._z_max = float(rospy.get_param("~camera_depth_max_m", 2.0))
        self._r_med = int(rospy.get_param("~depth_median_radius", 4))
        self._assume_z = float(rospy.get_param("~assume_depth_m", 0.5))
        self._use_depth = bool(rospy.get_param("~use_depth", True))

        self._min_pairs = int(rospy.get_param("~min_point_pairs", 3))
        self._result_path = os.path.expanduser(
            rospy.get_param("~result_path", "~/camera_base_eye_to_hand_result.yaml")
        )
        self._publish_tf = bool(rospy.get_param("~publish_tf", False))
        self._static_pub = None
        if self._publish_tf:
            from tf2_ros import StaticTransformBroadcaster

            self._static_pub = StaticTransformBroadcaster()

        self._samples_base = []  # list of [x,y,z]
        self._samples_opt = []

        self._tf = tf2_ros.Buffer(rospy.Duration(30.0))
        self._tf_l = tf2_ros.TransformListener(self._tf)

        rospy.Subscriber(self._color, Image, self._on_color, queue_size=1)
        rospy.Subscriber(self._depth_top, Image, self._on_depth, queue_size=1)
        rospy.Subscriber(self._info, CameraInfo, self._on_info, queue_size=1)

        rospy.Service("~add_sample_from_vision", Trigger, self._srv_add_vision)
        rospy.Service("~add_sample_manual", AddEyeToHandSample, self._srv_add_manual)
        rospy.Service("~compute", Trigger, self._srv_compute)
        rospy.Service("~reset", Trigger, self._srv_reset)
        rospy.Service("~print_status", Trigger, self._srv_status)

        rospy.loginfo(
            "[calib] 眼在手外: base=%s optical=%s tip=%s | 话题 color=%s",
            self._base,
            self._optical,
            self._tip or "(仅手动)",
            self._color,
        )

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

    def _one_optical_point(self):
        with self._lock:
            bgr = None if self._bgr is None else self._bgr.copy()
            dep = None if self._depth is None else self._depth.copy()
            k = None if self._k is None else self._k.copy()
        if bgr is None or k is None:
            return None, "无彩色图或 camera_info K"
        uv = self._blob_uv(bgr)
        if uv is None:
            return None, "未找到色块/HSV 无目标"
        u, v = uv[0], uv[1]
        z = min(self._assume_z, self._z_max)
        if self._use_depth and dep is not None:
            uu, vv = map_uv_to_depth_res(u, v, bgr.shape, dep.shape)
            zd = depth_patch_median(uu, vv, dep, self._r_med, self._z_max)
            if zd is not None:
                z = min(float(zd), self._z_max)
        x, y, zz = pixel_to_optical(u, v, z, k)
        return (x, y, zz), None

    def _tip_in_base(self):
        if not self._tip:
            return None, "未设置 ~gripper_tip_frame"
        try:
            tr = self._tf.lookup_transform(
                self._base, self._tip, rospy.Time(0), rospy.Duration(0.5)
            )
        except Exception as e:  # noqa: BLE001
            return None, "TF %s<-%s: %s" % (self._base, self._tip, e)
        t = tr.transform.translation
        return (float(t.x), float(t.y), float(t.z)), None

    def _srv_add_vision(self, _req):
        pb, e = self._tip_in_base()
        if pb is None:
            return TriggerResponse(success=False, message=e)
        po, e2 = self._one_optical_point()
        if po is None:
            return TriggerResponse(
                success=False, message="光学系: " + (e2 or "失败"),
            )
        self._samples_base.append(list(pb))
        self._samples_opt.append(list(po))
        n = len(self._samples_base)
        s = "已添加第 %d 对: base=%s optical=%s" % (n, pb, po)
        rospy.loginfo(s)
        return TriggerResponse(success=True, message=s)

    def _srv_add_manual(self, req):
        pb = (req.base_x, req.base_y, req.base_z)
        po = (req.optical_x, req.optical_y, req.optical_z)
        self._samples_base.append(list(pb))
        self._samples_opt.append(list(po))
        n = len(self._samples_base)
        s = "手动添加第 %d 对: base=%s optical=%s" % (n, pb, po)
        rospy.loginfo(s)
        return AddEyeToHandSampleResponse(success=True, message=s, count=n)

    def _srv_reset(self, _req):
        self._samples_base = []
        self._samples_opt = []
        return TriggerResponse(success=True, message="已清空样本")

    def _srv_status(self, _req):
        n = len(self._samples_base)
        m = "样本数 %d, 至少需要 %d" % (n, self._min_pairs)
        return TriggerResponse(success=True, message=m)

    def _build_static_command(self, tx, ty, tz, qx, qy, qz, qw):
        return (
            "rosrun tf static_transform_publisher "
            "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %s %s 100"
            % (tx, ty, tz, qx, qy, qz, qw, self._base, self._optical)
        )

    def _srv_compute(self, _req):
        n = len(self._samples_base)
        if n < self._min_pairs:
            return TriggerResponse(
                success=False,
                message="样本不足: %d < %d" % (n, self._min_pairs),
            )
        P_o = np.array(self._samples_opt, dtype=np.float64)
        P_b = np.array(self._samples_base, dtype=np.float64)
        r, t = kabsch_rigid(P_o, P_b)
        pred = (r @ P_o.T).T + t
        err = P_b - pred
        rms = float(np.sqrt(np.mean(err**2)))
        max_e = float(np.max(np.linalg.norm(err, axis=1)))

        T = np.eye(4)
        T[:3, :3] = r
        T[:3, 3] = t
        qx, qy, qz, qw = tft.quaternion_from_matrix(T)

        cmd = self._build_static_command(
            t[0], t[1], t[2], qx, qy, qz, qw
        )
        text = self._result_yaml(rms, max_e, t, (qx, qy, qz, qw), cmd)
        try:
            out = open(os.path.expanduser(self._result_path), "w")
            try:
                out.write(text)
            finally:
                out.close()
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("写入 %s 失败: %s", self._result_path, e)

        if self._publish_tf and self._static_pub is not None:
            st = TransformStamped()
            st.header.stamp = rospy.Time.now()
            st.header.frame_id = self._base
            st.child_frame_id = self._optical
            st.transform.translation.x = t[0]
            st.transform.translation.y = t[1]
            st.transform.translation.z = t[2]
            st.transform.rotation.x = qx
            st.transform.rotation.y = qy
            st.transform.rotation.z = qz
            st.transform.rotation.w = qw
            self._static_pub.sendTransform(st)

        rospy.loginfo("RMS=%.4f m max_point_err=%.4f m\n%s", rms, max_e, cmd)
        return TriggerResponse(
            success=True,
            message="rms=%.4f m, 结果已写 %s | %s" % (rms, self._result_path, cmd),
        )

    def _result_yaml(self, rms, max_e, t, quat, cmd):
        qx, qy, qz, qw = quat
        lines = [
            "# 眼在手外: parent=%s child=%s" % (self._base, self._optical),
            "# p_base = R * p_optical + t",
            "calibration:",
            "  base_frame: \"%s\"" % self._base,
            "  optical_frame: \"%s\"" % self._optical,
            "  translation: [%.8f, %.8f, %.8f]" % (t[0], t[1], t[2]),
            "  rotation_xyzw: [%.8f, %.8f, %.8f, %.8f]" % (qx, qy, qz, qw),
            "  rms_m: %.8f" % rms,
            "  max_point_error_m: %.8f" % max_e,
            "static_transform_command: |",
            "  " + cmd,
        ]
        return "\n".join(lines) + "\n"


def main():
    CameraBaseCalibEyeToHand()
    rospy.spin()


if __name__ == "__main__":
    main()
