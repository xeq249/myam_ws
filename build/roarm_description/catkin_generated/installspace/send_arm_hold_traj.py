#!/usr/bin/env python3
"""启动后向 arm_controller / gripper_controller 发单点轨迹，力矩 PID 抗重力。"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
]
GRIPPER_JOINTS = ["link5_to_gripper_link"]


def _wait_pub(pub, topic, timeout=30.0):
    t0 = rospy.Time.now()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout:
            rospy.logerr("Timeout waiting for subscribers on %s", topic)
            return False
        rospy.sleep(0.1)
    return True


def main():
    rospy.init_node("send_arm_hold_traj", anonymous=True)
    wait = rospy.get_param("~wait_after_start", 3.0)
    duration = rospy.get_param("~move_duration", 4.0)

    arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
    gripper_topic = rospy.get_param("~gripper_command_topic", "/gripper_controller/command")
    send_gripper = rospy.get_param("~send_gripper_hold", True)

    arm_positions = rospy.get_param(
        "~hold_positions",
        [0.0, 0.0, 0.0, 0.0, 0.0],
    )
    if len(arm_positions) != len(ARM_JOINTS):
        rospy.logfatal("~hold_positions length must be %d (arm only)", len(ARM_JOINTS))
        return

    gripper_positions = rospy.get_param("~hold_gripper_positions", [0.0])
    if len(gripper_positions) != len(GRIPPER_JOINTS):
        rospy.logfatal("~hold_gripper_positions length must be %d", len(GRIPPER_JOINTS))
        return

    rospy.sleep(wait)

    pub_arm = rospy.Publisher(arm_topic, JointTrajectory, queue_size=1)
    if not _wait_pub(pub_arm, arm_topic):
        return

    msg_arm = JointTrajectory()
    msg_arm.joint_names = list(ARM_JOINTS)
    pt = JointTrajectoryPoint()
    pt.positions = [float(x) for x in arm_positions]
    pt.velocities = []
    pt.accelerations = []
    pt.time_from_start = rospy.Duration(duration)
    msg_arm.points.append(pt)
    pub_arm.publish(msg_arm)
    rospy.loginfo("Published arm hold on %s", arm_topic)

    if send_gripper:
        pub_g = rospy.Publisher(gripper_topic, JointTrajectory, queue_size=1)
        if not _wait_pub(pub_g, gripper_topic):
            return
        msg_g = JointTrajectory()
        msg_g.joint_names = list(GRIPPER_JOINTS)
        ptg = JointTrajectoryPoint()
        ptg.positions = [float(x) for x in gripper_positions]
        ptg.velocities = []
        ptg.accelerations = []
        ptg.time_from_start = rospy.Duration(min(duration, 2.0))
        msg_g.points.append(ptg)
        pub_g.publish(msg_g)
        rospy.loginfo("Published gripper hold on %s", gripper_topic)


if __name__ == "__main__":
    main()
