#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offboard：定高巡航正方形，检测 AprilTag 则悬停 5s 后降落；未检测则回原点悬停后退出。

高度：默认 altitude_mode=absolute，z_target=~target_alt_m（local 系绝对高度）；
若 altitude_mode=relative，则 z_target=z0+~altitude_rel_m。
"""

from __future__ import print_function

import math
import time

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode


class OffboardDetectApriltag(object):
    RISE = "rise"
    CRUISE = "cruise"
    TAG_HOLD = "tag_hold"
    LAND = "land"
    HOVER_ORIGIN = "hover_origin"
    DONE = "done"

    def __init__(self):
        rospy.init_node("offboard_detect_apriltag")

        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.warmup_iters = int(rospy.get_param("~warmup_iters", 40))
        self.altitude_mode = rospy.get_param("~altitude_mode", "absolute").lower().strip()
        if self.altitude_mode not in ("absolute", "relative"):
            rospy.logwarn("~altitude_mode 应为 absolute 或 relative，已回退为 absolute")
            self.altitude_mode = "absolute"
        self.target_alt_m = float(rospy.get_param("~target_alt_m", 3.0))
        self.altitude_rel_m = float(rospy.get_param("~altitude_rel_m", 2.0))
        self.square_side_m = float(rospy.get_param("~square_side_m", 2.0))
        self.wp_tol_xy = float(rospy.get_param("~wp_tol_xy", 0.15))
        self.wp_tol_z = float(rospy.get_param("~wp_tol_z", 0.2))
        self.tag_hold_sec = float(rospy.get_param("~tag_hold_sec", 5.0))
        self.hover_origin_sec = float(rospy.get_param("~hover_origin_sec", 3.0))
        self.tag_id = int(rospy.get_param("~tag_id", -1))
        self.auto_arm = bool(rospy.get_param("~auto_arm", True))
        self.auto_offboard = bool(rospy.get_param("~auto_offboard", True))

        self.state = State()
        self.pose = PoseStamped()
        self.tags = AprilTagDetectionArray()
        self.phase = self.RISE
        self.tag_hold_t0 = None
        self.hover_origin_t0 = None
        self.pose_printed = False
        self.waypoints = []
        self.wp_idx = 0
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.z_target = 0.0
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0
        self._last_det = None
        self._land_sent = False

        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self._tag_cb)
        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _tag_cb(self, msg):
        self.tags = msg

    def _setpoint_position(self, x, y, z, yaw):
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
    def _yaw_from_pose(pose_stamped):
        q = pose_stamped.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _pick_detection(self):
        for det in self.tags.detections:
            if self.tag_id < 0:
                return det
            if self.tag_id in list(det.id):
                return det
        return None

    def _dist_xy(self, x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    def _dist_xyz(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

    def _build_square_ccw(self, x0, y0, zt, side):
        """ENU（俯视 +z 逆时针）：前(+x) → 左(+y) → 后(-x) → 右(-y) 回到起点。"""
        return [
            (x0 + side, y0, zt),
            (x0 + side, y0 + side, zt),
            (x0, y0 + side, zt),
            (x0, y0, zt),
        ]

    def _print_tag_pose(self, det):
        ps = det.pose.pose.pose
        p = ps.position
        o = ps.orientation
        fid = det.pose.header.frame_id
        ids = list(det.id)
        line = (
            "[AprilTag] id={} frame={}\n"
            "  position: x={:.4f} y={:.4f} z={:.4f}\n"
            "  orientation: x={:.4f} y={:.4f} z={:.4f} w={:.4f}"
        ).format(ids, fid, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        rospy.loginfo("\n%s", line)
        print(line, flush=True)

    def run(self):
        r = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown() and not self.state.connected:
            rospy.loginfo_throttle(2.0, "等待 MAVROS 连接 FCU…")
            r.sleep()

        yaw_ref = self._yaw_from_pose(self.pose)

        for _ in range(self.warmup_iters):
            px = self.pose.pose.position.x
            py = self.pose.pose.position.y
            pz = self.pose.pose.position.z
            self.sp_pub.publish(self._setpoint_position(px, py, pz, yaw_ref))
            r.sleep()

        self.x0 = self.pose.pose.position.x
        self.y0 = self.pose.pose.position.y
        self.z0 = self.pose.pose.position.z
        if self.altitude_mode == "absolute":
            self.z_target = self.target_alt_m
        else:
            self.z_target = self.z0 + self.altitude_rel_m
        yaw_ref = self._yaw_from_pose(self.pose)

        self.waypoints = self._build_square_ccw(self.x0, self.y0, self.z_target, self.square_side_m)
        if self.altitude_mode == "absolute":
            rospy.loginfo(
                "起点 (local): (%.3f, %.3f, %.3f)，当前 z=%.3f；高度模式=绝对 → z_target=%.3f m；正方形边长 %.2f m (CCW)",
                self.x0,
                self.y0,
                self.z0,
                self.z0,
                self.z_target,
                self.square_side_m,
            )
        else:
            rospy.loginfo(
                "起点 (local): (%.3f, %.3f, %.3f)，高度模式=相对 → z_target=z0+%.2f=%.3f m；正方形边长 %.2f m (CCW)",
                self.x0,
                self.y0,
                self.z0,
                self.altitude_rel_m,
                self.z_target,
                self.square_side_m,
            )

        if self.auto_offboard:
            try:
                self.set_mode(0, "OFFBOARD")
            except rospy.ServiceException as exc:
                rospy.logwarn("切 OFFBOARD 失败: %s", exc)
        if self.auto_arm:
            try:
                self.arming(True)
            except rospy.ServiceException as exc:
                rospy.logwarn("解锁失败: %s", exc)

        self.phase = self.RISE

        while not rospy.is_shutdown() and self.phase != self.DONE:
            px = self.pose.pose.position.x
            py = self.pose.pose.position.y
            pz = self.pose.pose.position.z
            yaw_ref = self._yaw_from_pose(self.pose)

            det = self._pick_detection()
            if det is not None and self.phase in (self.RISE, self.CRUISE):
                self.hold_x = px
                self.hold_y = py
                self.hold_z = pz
                self._last_det = det
                self.phase = self.TAG_HOLD
                self.tag_hold_t0 = time.time()
                self.pose_printed = False
                rospy.loginfo("检测到 AprilTag，原地悬停 %.1f s 后降落…", self.tag_hold_sec)

            if self.phase == self.RISE:
                self.sp_pub.publish(self._setpoint_position(self.x0, self.y0, self.z_target, yaw_ref))
                if self._dist_xyz(px, py, pz, self.x0, self.y0, self.z_target) < max(self.wp_tol_xy, self.wp_tol_z):
                    rospy.loginfo("已到达目标高度，开始正方形巡航")
                    self.phase = self.CRUISE
                    self.wp_idx = 0

            elif self.phase == self.CRUISE:
                if self.wp_idx >= len(self.waypoints):
                    rospy.loginfo("正方形航线完成，回到原点悬停…")
                    self.phase = self.HOVER_ORIGIN
                    self.hover_origin_t0 = time.time()
                else:
                    tx, ty, tz = self.waypoints[self.wp_idx]
                    self.sp_pub.publish(self._setpoint_position(tx, ty, tz, yaw_ref))

                    if self._dist_xy(px, py, tx, ty) < self.wp_tol_xy and abs(pz - tz) < self.wp_tol_z:
                        rospy.loginfo(
                            "到达航点 %d / %d: (%.3f, %.3f, %.3f)",
                            self.wp_idx + 1,
                            len(self.waypoints),
                            tx,
                            ty,
                            tz,
                        )
                        self.wp_idx += 1

            elif self.phase == self.TAG_HOLD:
                self.sp_pub.publish(self._setpoint_position(self.hold_x, self.hold_y, self.hold_z, yaw_ref))
                if not self.pose_printed and self._last_det is not None:
                    self._print_tag_pose(self._last_det)
                    self.pose_printed = True

                if self.tag_hold_t0 is not None and (time.time() - self.tag_hold_t0) >= self.tag_hold_sec:
                    rospy.loginfo("悬停结束，执行降落…")
                    self.phase = self.LAND

            elif self.phase == self.LAND:
                self.sp_pub.publish(self._setpoint_position(self.hold_x, self.hold_y, self.hold_z, yaw_ref))
                if not self._land_sent:
                    try:
                        self.set_mode(0, "AUTO.LAND")
                    except rospy.ServiceException as exc:
                        rospy.logwarn("AUTO.LAND 失败: %s", exc)
                    self._land_sent = True
                    rospy.loginfo("已发送降落模式，程序结束。")
                    print("[AprilTag] 任务结束：已降落。", flush=True)
                self.phase = self.DONE

            elif self.phase == self.HOVER_ORIGIN:
                self.sp_pub.publish(self._setpoint_position(self.x0, self.y0, self.z_target, yaw_ref))
                if self.hover_origin_t0 is not None and (time.time() - self.hover_origin_t0) >= self.hover_origin_sec:
                    rospy.loginfo("原点悬停完成，未检测到 AprilTag，程序结束。")
                    print("[巡航] 未检测到标签，已回原点悬停并退出。", flush=True)
                    self.phase = self.DONE

            r.sleep()


def main():
    try:
        OffboardDetectApriltag().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
