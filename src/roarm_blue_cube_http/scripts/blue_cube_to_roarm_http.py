#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time
import urllib.parse

import cv2
import numpy as np
import requests
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return Rz.dot(Ry).dot(Rx)


class BlueCubeToRoArmHTTP(object):
    def __init__(self):
        rospy.init_node("blue_cube_to_roarm_http")
        self.bridge = CvBridge()

        # -------- topics --------
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.debug_topic = rospy.get_param("~debug_topic", "/blue_cube_to_roarm_http/debug_image")

        # -------- run mode --------
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.send_pose_only = bool(rospy.get_param("~send_pose_only", True))
        self.auto_grasp = bool(rospy.get_param("~auto_grasp", False))
        self.send_once = bool(rospy.get_param("~send_once", True))
        self.print_interval = float(rospy.get_param("~print_interval", 0.5))
        self.command_interval = float(rospy.get_param("~command_interval", 1.0))

        # -------- HTTP --------
        self.roarm_ip = rospy.get_param("~roarm_ip", "192.168.4.1")
        self.http_timeout = float(rospy.get_param("~http_timeout", 2.0))
        self.use_T104 = bool(rospy.get_param("~use_T104", True))  # True: interpolation, False: direct 1041
        self.move_spd = float(rospy.get_param("~move_spd", 0.20))

        # -------- blue detection defaults for light blue cube --------
        self.blue_h_min = int(rospy.get_param("~blue_h_min", 85))
        self.blue_h_max = int(rospy.get_param("~blue_h_max", 115))
        self.blue_s_min = int(rospy.get_param("~blue_s_min", 30))
        self.blue_v_min = int(rospy.get_param("~blue_v_min", 120))
        self.min_area = float(rospy.get_param("~min_area", 300))
        self.max_area = float(rospy.get_param("~max_area", 30000))
        self.min_aspect = float(rospy.get_param("~min_aspect", 0.45))
        self.max_aspect = float(rospy.get_param("~max_aspect", 2.2))
        self.depth_roi = int(rospy.get_param("~depth_roi", 7))
        self.depth_min_m = float(rospy.get_param("~depth_min_m", 0.12))
        self.depth_max_m = float(rospy.get_param("~depth_max_m", 2.0))

        # -------- camera extrinsic: camera optical frame -> base_link/user frame --------
        # Current installation: camera looks forward-down. User states optical +Z is 55 deg from horizontal.
        # With base_link: +X forward, +Z downward. Approx mapping default:
        # camera optical +Z = cos55 * base +X + sin55 * base +Z.
        self.cam_x = float(rospy.get_param("~cam_x", 0.10))
        self.cam_y = float(rospy.get_param("~cam_y", 0.0))
        self.cam_z = float(rospy.get_param("~cam_z", -0.05))
        self.cam_roll = float(rospy.get_param("~cam_roll", 0.0))
        self.cam_pitch = float(rospy.get_param("~cam_pitch", math.radians(35.0)))
        self.cam_yaw = float(rospy.get_param("~cam_yaw", 0.0))
        self.R_base_cam = rpy_to_R(self.cam_roll, self.cam_pitch, self.cam_yaw)
        self.t_base_cam = np.array([self.cam_x, self.cam_y, self.cam_z], dtype=np.float64)

        # -------- RoArm coordinate mapping --------
        # User: inverted arm, base_link +X forward, +Z downward, cube is front-down so x,z are positive.
        self.roarm_x_sign = float(rospy.get_param("~roarm_x_sign", 1.0))
        self.roarm_y_sign = float(rospy.get_param("~roarm_y_sign", 1.0))
        self.roarm_z_sign = float(rospy.get_param("~roarm_z_sign", 1.0))
        self.target_offset_x_mm = float(rospy.get_param("~target_offset_x_mm", 0.0))
        self.target_offset_y_mm = float(rospy.get_param("~target_offset_y_mm", 0.0))
        self.target_offset_z_mm = float(rospy.get_param("~target_offset_z_mm", 0.0))

        # -------- software limit switch: disabled by default per request --------
        self.reject_out_of_range = bool(rospy.get_param("~reject_out_of_range", False))
        self.clamp_out_of_range = bool(rospy.get_param("~clamp_out_of_range", False))
        self.x_min_mm = float(rospy.get_param("~x_min_mm", -10000.0))
        self.x_max_mm = float(rospy.get_param("~x_max_mm", 10000.0))
        self.y_min_mm = float(rospy.get_param("~y_min_mm", -10000.0))
        self.y_max_mm = float(rospy.get_param("~y_max_mm", 10000.0))
        self.z_min_mm = float(rospy.get_param("~z_min_mm", -10000.0))
        self.z_max_mm = float(rospy.get_param("~z_max_mm", 10000.0))

        # -------- grasp: no pregrasp; open -> go to centroid -> close --------
        self.g_open = float(rospy.get_param("~g_open", 1.10))
        self.g_close = float(rospy.get_param("~g_close", 3.14))
        self.wrist_t = float(rospy.get_param("~wrist_t", 0.0))
        self.wrist_r = float(rospy.get_param("~wrist_r", 0.0))
        self.open_before_move = bool(rospy.get_param("~open_before_move", True))
        self.close_after_move = bool(rospy.get_param("~close_after_move", False))
        self.settle_before_close = float(rospy.get_param("~settle_before_close", 1.0))

        # -------- stability / filter --------
        self.stable_frames = int(rospy.get_param("~stable_frames", 5))
        self.stable_pixel_tol = float(rospy.get_param("~stable_pixel_tol", 12.0))
        self.filter_alpha = float(rospy.get_param("~filter_alpha", 0.15))
        self.max_jump_mm = float(rospy.get_param("~max_jump_mm", 150.0))
        self._filtered_xyz = None
        self._last_uv = None
        self._stable_cnt = 0
        self._sent = False
        self._last_cmd_time = 0.0
        self._last_print_time = 0.0

        self._img = None
        self._depth = None
        self._K = None

        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.cmd_pub = rospy.Publisher("/blue_cube_to_roarm_http/last_json_cmd", String, queue_size=1)
        self.point_pub = rospy.Publisher("/blue_cube_to_roarm_http/cube_centroid_base", PointStamped, queue_size=1)

        rospy.Subscriber(self.color_topic, Image, self.cb_img, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        rospy.loginfo("[INIT] blue cube HTTP node started")
        rospy.loginfo("[INIT] dry_run=%s send_pose_only=%s auto_grasp=%s roarm_ip=%s", self.dry_run, self.send_pose_only, self.auto_grasp, self.roarm_ip)
        rospy.loginfo("[INIT] software range reject=%s clamp=%s limits x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]", self.reject_out_of_range, self.clamp_out_of_range, self.x_min_mm, self.x_max_mm, self.y_min_mm, self.y_max_mm, self.z_min_mm, self.z_max_mm)
        rospy.loginfo("[INIT] camera extrinsic xyz=(%.3f,%.3f,%.3f)m rpy=(%.3f,%.3f,%.3f)rad", self.cam_x, self.cam_y, self.cam_z, self.cam_roll, self.cam_pitch, self.cam_yaw)

    def cb_info(self, msg):
        self._K = np.array(msg.K, dtype=np.float64).reshape(3, 3)

    def cb_img(self, msg):
        try:
            self._img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, "cv_bridge color error: %s", e)

    def cb_depth(self, msg):
        try:
            if msg.encoding == "16UC1":
                self._depth = self.bridge.imgmsg_to_cv2(msg, "16UC1").astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                self._depth = self.bridge.imgmsg_to_cv2(msg, "32FC1").astype(np.float32)
            else:
                self._depth = self.bridge.imgmsg_to_cv2(msg).astype(np.float32)
                # Many RealSense depth topics are mm when not 32FC1.
                if np.nanmedian(self._depth) > 10.0:
                    self._depth = self._depth / 1000.0
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, "cv_bridge depth error: %s", e)

    def check_range(self, x, y, z):
        ok = (self.x_min_mm <= x <= self.x_max_mm and
              self.y_min_mm <= y <= self.y_max_mm and
              self.z_min_mm <= z <= self.z_max_mm)
        msg = "x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f]" % (
            self.x_min_mm, self.x_max_mm, self.y_min_mm, self.y_max_mm, self.z_min_mm, self.z_max_mm)
        return ok, msg

    def clamp_range(self, x, y, z):
        return (min(max(x, self.x_min_mm), self.x_max_mm),
                min(max(y, self.y_min_mm), self.y_max_mm),
                min(max(z, self.z_min_mm), self.z_max_mm))

    def make_mask(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([self.blue_h_min, self.blue_s_min, self.blue_v_min], dtype=np.uint8)
        upper = np.array([self.blue_h_max, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def select_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area or area > self.max_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < 10 or h < 10:
                continue
            aspect = float(w) / max(1.0, float(h))
            if aspect < self.min_aspect or aspect > self.max_aspect:
                continue
            fill = area / max(1.0, float(w * h))
            if fill < 0.25:
                continue
            valid.append((area, c))
        if not valid:
            return None
        return max(valid, key=lambda p: p[0])[1]

    def depth_at(self, u, v, mask=None, cnt=None):
        if self._depth is None:
            return None
        d = self._depth.astype(np.float32)
        h, w = d.shape[:2]
        vals = None
        if mask is not None and cnt is not None:
            obj = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(obj, [cnt], -1, 255, thickness=-1)
            kernel = np.ones((5, 5), np.uint8)
            obj = cv2.erode(obj, kernel, iterations=1)
            obj = cv2.bitwise_and(obj, mask)
            valid = (obj > 0) & np.isfinite(d) & (d >= self.depth_min_m) & (d <= self.depth_max_m)
            vals = d[valid]
        if vals is None or vals.size < 10:
            r = max(1, int(self.depth_roi))
            u0, u1 = max(0, u - r), min(w, u + r + 1)
            v0, v1 = max(0, v - r), min(h, v + r + 1)
            roi = d[v0:v1, u0:u1].reshape(-1)
            vals = roi[np.isfinite(roi)]
            vals = vals[(vals >= self.depth_min_m) & (vals <= self.depth_max_m)]
        if vals.size < 5:
            return None
        return float(np.median(vals))

    def detect(self):
        if self._img is None or self._K is None or self._depth is None:
            return None, None
        img = self._img.copy()
        mask = self.make_mask(img)
        cnt = self.select_contour(mask)
        debug = img.copy()
        if cnt is None:
            cv2.putText(debug, "no blue object", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            return None, debug
        area = cv2.contourArea(cnt)
        M = cv2.moments(cnt)
        if abs(M["m00"]) < 1e-6:
            return None, debug
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        depth = self.depth_at(u, v, mask=mask, cnt=cnt)
        cv2.drawContours(debug, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(debug, (u, v), 5, (0, 0, 255), -1)
        cv2.putText(debug, "u=%d v=%d" % (u, v), (u + 8, v), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        if depth is None:
            cv2.putText(debug, "invalid depth", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            return None, debug

        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth
        p_cam = np.array([x_cam, y_cam, z_cam], dtype=np.float64)
        p_base = self.R_base_cam.dot(p_cam) + self.t_base_cam

        x_raw = self.roarm_x_sign * p_base[0] * 1000.0
        y_raw = self.roarm_y_sign * p_base[1] * 1000.0
        z_raw = self.roarm_z_sign * p_base[2] * 1000.0
        x_mm = x_raw + self.target_offset_x_mm
        y_mm = y_raw + self.target_offset_y_mm
        z_mm = z_raw + self.target_offset_z_mm

        raw_xyz = np.array([x_mm, y_mm, z_mm], dtype=np.float64)
        if self._filtered_xyz is None:
            self._filtered_xyz = raw_xyz
        else:
            jump = float(np.linalg.norm(raw_xyz - self._filtered_xyz))
            if jump <= self.max_jump_mm:
                a = max(0.0, min(1.0, self.filter_alpha))
                self._filtered_xyz = a * raw_xyz + (1.0 - a) * self._filtered_xyz
            else:
                rospy.logwarn_throttle(1.0, "[FILTER] target jump %.1fmm ignored raw=%s filtered=%s", jump, raw_xyz, self._filtered_xyz)
        x_mm, y_mm, z_mm = [float(vv) for vv in self._filtered_xyz]

        uv = np.array([u, v], dtype=np.float64)
        if self._last_uv is not None and np.linalg.norm(uv - self._last_uv) <= self.stable_pixel_tol:
            self._stable_cnt += 1
        else:
            self._stable_cnt = 1
        self._last_uv = uv

        in_range, range_msg = self.check_range(x_mm, y_mm, z_mm)
        if not in_range and self.clamp_out_of_range and not self.reject_out_of_range:
            x_mm, y_mm, z_mm = self.clamp_range(x_mm, y_mm, z_mm)
            in_range = True
            range_msg += " clamped"

        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "roarm_base_link"
        ps.point.x, ps.point.y, ps.point.z = [float(vv) for vv in p_base]
        self.point_pub.publish(ps)

        result = {
            "u": u, "v": v, "area": area, "depth": depth,
            "p_cam": p_cam, "p_base": p_base,
            "raw_mm": np.array([x_raw, y_raw, z_raw]),
            "x_mm": x_mm, "y_mm": y_mm, "z_mm": z_mm,
            "stable_cnt": self._stable_cnt,
            "stable": self._stable_cnt >= self.stable_frames,
            "range_ok": in_range,
            "range_msg": range_msg,
        }
        status = "stable %d/%d" % (self._stable_cnt, self.stable_frames) if in_range else "out of range"
        cv2.putText(debug, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        return result, debug

    def http_send(self, cmd):
        cmd_str = json.dumps(cmd, separators=(",", ":"))
        self.cmd_pub.publish(String(cmd_str))
        url = "http://%s/js?json=%s" % (self.roarm_ip, urllib.parse.quote(cmd_str))
        rospy.loginfo("[HTTP] %s", cmd_str)
        if self.dry_run:
            rospy.logwarn("[HTTP] dry_run=True, not sending")
            return False
        try:
            r = requests.get(url, timeout=self.http_timeout)
            rospy.loginfo("[HTTP] status=%s body=%s", r.status_code, r.text[:120])
            return 200 <= r.status_code < 300
        except Exception as e:
            rospy.logerr("[HTTP] send failed: %s", e)
            return False

    def move_cmd(self, x, y, z, g=None):
        T = 104 if self.use_T104 else 1041
        cmd = {"T": T, "x": round(x, 1), "y": round(y, 1), "z": round(z, 1), "t": self.wrist_t, "r": self.wrist_r, "g": self.g_open if g is None else g}
        if T == 104:
            cmd["spd"] = self.move_spd
        return cmd

    def gripper_cmd(self, g):
        return {"T": 106, "cmd": float(g), "spd": self.move_spd}

    def maybe_print(self, res):
        now = time.time()
        if now - self._last_print_time < self.print_interval:
            return
        self._last_print_time = now
        rospy.loginfo("========== 蓝色立方块识别结果 / Blue cube ==========")
        rospy.loginfo("像素质心: u=%d v=%d | 面积=%.1f | 稳定帧=%d/%d", res["u"], res["v"], res["area"], res["stable_cnt"], self.stable_frames)
        rospy.loginfo("深度: %.3f m", res["depth"])
        pc, pb, raw = res["p_cam"], res["p_base"], res["raw_mm"]
        rospy.loginfo("相机光学坐标 camera optical: x=%.3f y=%.3f z=%.3f m", pc[0], pc[1], pc[2])
        rospy.loginfo("机械臂基座坐标 base_link: x=%.3f y=%.3f z=%.3f m", pb[0], pb[1], pb[2])
        rospy.loginfo("RoArm raw/mm: x=%.1f y=%.1f z=%.1f", raw[0], raw[1], raw[2])
        rospy.loginfo("RoArm final/mm: x=%.1f y=%.1f z=%.1f | range_ok=%s | %s", res["x_mm"], res["y_mm"], res["z_mm"], res["range_ok"], res["range_msg"])
        rospy.loginfo("dry_run=%s | send_pose_only=%s | auto_grasp=%s", self.dry_run, self.send_pose_only, self.auto_grasp)

    def maybe_send(self, res):
        if not res["stable"]:
            return
        if (not res["range_ok"]) and self.reject_out_of_range:
            rospy.logwarn_throttle(1.0, "[LIMIT] target rejected: x=%.1f y=%.1f z=%.1f mm | %s", res["x_mm"], res["y_mm"], res["z_mm"], res["range_msg"])
            return
        now = time.time()
        if self.send_once and self._sent:
            return
        if now - self._last_cmd_time < self.command_interval:
            return
        self._last_cmd_time = now

        x, y, z = res["x_mm"], res["y_mm"], res["z_mm"]
        if self.auto_grasp and not self.send_pose_only:
            if self.open_before_move:
                self.http_send(self.gripper_cmd(self.g_open))
                rospy.sleep(0.3)
            self.http_send(self.move_cmd(x, y, z, g=self.g_open))
            rospy.sleep(self.settle_before_close)
            self.http_send(self.gripper_cmd(self.g_close))
        else:
            # default: only open gripper and move clamp center to cube centroid. No pregrasp, no close.
            if self.open_before_move:
                self.http_send(self.gripper_cmd(self.g_open))
                rospy.sleep(0.15)
            self.http_send(self.move_cmd(x, y, z, g=self.g_open))
            if self.close_after_move:
                rospy.sleep(self.settle_before_close)
                self.http_send(self.gripper_cmd(self.g_close))
        self._sent = True

    def spin(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            res, dbg = self.detect()
            if dbg is not None:
                try:
                    self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
                except CvBridgeError:
                    pass
            if res is not None:
                self.maybe_print(res)
                self.maybe_send(res)
            rate.sleep()


if __name__ == "__main__":
    BlueCubeToRoArmHTTP().spin()
