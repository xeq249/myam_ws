#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MAVROS offboard hover over AprilTag, then execute a MoveIt grasp sequence."""

import sys

import moveit_commander
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander


class OffboardArmApriltagGrasp(object):
    CRUISE = "cruise"
    TAG_ALIGN = "tag_align"
    ARM_EXEC = "arm_exec"
    RETREAT = "retreat"
    LAND = "land"
    DONE = "done"

    def __init__(self):
        rospy.init_node("offboard_arm_apriltag_grasp")

        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.warmup_iters = int(rospy.get_param("~warmup_iters", 40))
        self.hover_alt = float(rospy.get_param("~hover_alt", 1.6))
        self.horiz_tol = float(rospy.get_param("~horiz_tol", 0.12))
        self.vert_tol = float(rospy.get_param("~vert_tol", 0.12))
        self.tag_id = int(rospy.get_param("~tag_id", -1))
        self.auto_arm = bool(rospy.get_param("~auto_arm", False))
        self.auto_offboard = bool(rospy.get_param("~auto_offboard", False))
        self.land_after_grasp = bool(rospy.get_param("~land_after_grasp", False))

        self.pregrasp_joints = rospy.get_param("~pregrasp_joints", [0.0, -1.1, 1.7, -0.25, 0.0])
        self.grasp_joints = rospy.get_param("~grasp_joints", [0.0, -1.45, 2.1, -0.35, 0.0])
        self.retreat_joints = rospy.get_param("~retreat_joints", [0.0, -0.9, 1.2, -0.2, 0.0])
        self.gripper_open = float(rospy.get_param("~gripper_open", 1.5))
        self.gripper_close = float(rospy.get_param("~gripper_close", 0.0))

        self.state = State()
        self.pose = PoseStamped()
        self.tags = AprilTagDetectionArray()
        self.phase = self.CRUISE
        self.task_done = False
        self.tag_tx = 0.0
        self.tag_ty = 0.0
        self.tag_tz = 0.0

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

    def _setpoint_position(self, x, y, z, yaw=0.0):
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

    def _pick_detection(self):
        for det in self.tags.detections:
            if self.tag_id < 0 or self.tag_id in list(det.id):
                return det
        return None

    def _wait_for_move_group(self, timeout_sec=180.0):
        import time
        import rosnode
        import rosservice

        t0 = time.time()
        node_seen_at = None
        while time.time() - t0 < timeout_sec and not rospy.is_shutdown():
            try:
                sl = set(rosservice.get_service_list())
            except Exception:
                sl = set()
            if "/move_group/apply_planning_scene" in sl or "/move_group/get_planning_scene" in sl:
                return True
            try:
                nodes = rosnode.get_node_names()
            except Exception:
                nodes = []
            if "/move_group" in nodes:
                if node_seen_at is None:
                    node_seen_at = time.time()
                elif time.time() - node_seen_at > 2.0:
                    return True
            time.sleep(0.5)
        return False

    def _execute_grasp(self):
        if self.task_done:
            return True

        moveit_commander.roscpp_initialize(sys.argv)
        if not self._wait_for_move_group():
            rospy.logerr("move_group 未就绪，无法执行抓取")
            return False

        arm = MoveGroupCommander("arm")
        gripper = MoveGroupCommander("gripper")
        arm.set_planning_time(15.0)
        arm.set_num_planning_attempts(10)
        arm.set_max_velocity_scaling_factor(0.25)
        arm.set_max_acceleration_scaling_factor(0.25)
        gripper.set_max_velocity_scaling_factor(0.4)
        gripper.set_max_acceleration_scaling_factor(0.4)

        gripper.set_joint_value_target([float(self.gripper_open)])
        gripper.go(wait=True)

        for joints in (self.pregrasp_joints, self.grasp_joints):
            arm.set_joint_value_target([float(x) for x in joints])
            if not arm.go(wait=True):
                rospy.logerr("机械臂轨迹执行失败: %s", joints)
                moveit_commander.roscpp_shutdown()
                return False

        gripper.set_joint_value_target([float(self.gripper_close)])
        gripper.go(wait=True)

        arm.set_joint_value_target([float(x) for x in self.retreat_joints])
        arm.go(wait=True)
        arm.set_named_target("home")
        arm.go(wait=True)

        moveit_commander.roscpp_shutdown()
        self.task_done = True
        return True

    def run(self):
        r = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown() and not self.state.connected:
            rospy.loginfo_throttle(2.0, "等待 MAVROS 连接 FCU…")
            r.sleep()

        for _ in range(self.warmup_iters):
            px = self.pose.pose.position.x
            py = self.pose.pose.position.y
            pz = self.pose.pose.position.z
            self.sp_pub.publish(self._setpoint_position(px, py, pz))
            r.sleep()

        px0 = self.pose.pose.position.x
        py0 = self.pose.pose.position.y
        pz0 = self.pose.pose.position.z

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

        while not rospy.is_shutdown():
            px = self.pose.pose.position.x
            py = self.pose.pose.position.y
            pz = self.pose.pose.position.z
            tx, ty, tz = px0, py0, pz0 + self.hover_alt

            if self.phase in (self.CRUISE, self.TAG_ALIGN):
                det = self._pick_detection()
                if det is not None:
                    ax = det.pose.pose.pose.position.x
                    ay = det.pose.pose.pose.position.y
                    self.tag_tx = px - ay
                    self.tag_ty = py - ax
                    self.tag_tz = pz0 + self.hover_alt
                    self.phase = self.TAG_ALIGN
                    tx, ty, tz = self.tag_tx, self.tag_ty, self.tag_tz
                    if abs(px - tx) < self.horiz_tol and abs(py - ty) < self.horiz_tol and abs(pz - tz) < self.vert_tol:
                        rospy.loginfo("已悬停到标签上方，开始机械臂抓取")
                        self.phase = self.ARM_EXEC
                else:
                    tx, ty, tz = px0, py0, pz0 + self.hover_alt

            if self.phase == self.ARM_EXEC:
                self.sp_pub.publish(self._setpoint_position(self.tag_tx, self.tag_ty, self.tag_tz))
                if self._execute_grasp():
                    self.phase = self.LAND if self.land_after_grasp else self.DONE
                    rospy.loginfo("抓取动作结束")
                else:
                    self.phase = self.DONE

            if self.phase == self.LAND:
                try:
                    self.set_mode(0, "AUTO.LAND")
                except rospy.ServiceException as exc:
                    rospy.logwarn("AUTO.LAND 失败: %s", exc)
                self.phase = self.DONE

            if self.phase != self.ARM_EXEC and self.phase != self.DONE:
                self.sp_pub.publish(self._setpoint_position(tx, ty, tz))

            if self.phase == self.DONE:
                self.sp_pub.publish(self._setpoint_position(self.tag_tx or px0, self.tag_ty or py0, self.tag_tz or (pz0 + self.hover_alt)))

            r.sleep()


def main():
    try:
        OffboardArmApriltagGrasp().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
