#!/usr/bin/env python3
"""在 arm_controller / gripper_controller 进入 running 后各发一条 home 轨迹。

注意：JointTrajectory 的 action 在控制器 load 后就会 advertise，但 goal 必须在
switch_controller 启动为 running 之后才能被接受；仅 wait_for_server 不够。"""
import time

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
]
GRIP_JOINTS = ["link5_to_gripper_link"]

REQUIRED_CONTROLLERS = (
    "joint_state_controller",
    "arm_controller",
    "gripper_controller",
)


def _wait_controllers_running(timeout_sec):
    rospy.loginfo(
        "等待 arm_controller/gripper_controller 进入 running（若仿真一直暂停，"
        "部分环境下需先解除暂停一次，spawner 才能完成启动）"
    )
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=60.0)
    list_srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    # 仿真暂停时 rospy.Time 可能不前进，超时必须用墙钟/单调钟
    t_deadline = time.monotonic() + float(timeout_sec)
    states = {}
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            resp = list_srv()
            states = {c.name: c.state for c in resp.controller}
            if all(states.get(n) == "running" for n in REQUIRED_CONTROLLERS):
                rospy.loginfo("Controllers running: %s", states)
                return True
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "list_controllers: %s", e)
        if time.monotonic() > t_deadline:
            break
        rate.sleep()
    rospy.logerr("Timeout waiting for controllers running. Last states: %s", states)
    return False


def _send_goal(client, joint_names, positions, duration_sec):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.velocities = [0.0] * len(joint_names)
    pt.time_from_start = rospy.Duration(duration_sec)
    goal.trajectory.points = [pt]
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(30.0))
    return client.get_state()


def main():
    rospy.init_node("arm_gripper_hold_home", anonymous=False)
    arm_pos = rospy.get_param(
        "~arm_positions",
        [0.0, 1.05, 1.57, 0.0, 0.0],
    )
    g_pos = rospy.get_param("~gripper_position", 0.0)
    dur = rospy.get_param("~time_to_home", 1.0)

    if len(arm_pos) != 5:
        rospy.logfatal("~arm_positions must have 5 values")
        return

    arm_ns = rospy.get_param("~arm_action", "/arm_controller/follow_joint_trajectory")
    grip_ns = rospy.get_param("~gripper_action", "/gripper_controller/follow_joint_trajectory")

    arm_client = actionlib.SimpleActionClient(arm_ns, FollowJointTrajectoryAction)
    grip_client = actionlib.SimpleActionClient(grip_ns, FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for arm trajectory action: %s", arm_ns)
    if not arm_client.wait_for_server(rospy.Duration(120.0)):
        rospy.logerr("Timeout waiting for arm controller action server")
        return
    rospy.loginfo("Waiting for gripper trajectory action: %s", grip_ns)
    if not grip_client.wait_for_server(rospy.Duration(60.0)):
        rospy.logerr("Timeout waiting for gripper controller action server")
        return

    if not _wait_controllers_running(120.0):
        rospy.logerr("Controllers not running; skip home trajectory")
        return

    st = _send_goal(arm_client, ARM_JOINTS, arm_pos, dur)
    rospy.loginfo("Arm home trajectory finished with state %s", st)
    st2 = _send_goal(grip_client, GRIP_JOINTS, [float(g_pos)], dur)
    rospy.loginfo("Gripper home trajectory finished with state %s", st2)


if __name__ == "__main__":
    main()
