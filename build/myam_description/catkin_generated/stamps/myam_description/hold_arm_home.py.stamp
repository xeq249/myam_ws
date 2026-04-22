#!/usr/bin/env python3
"""控制器就绪后：可选在暂停态用 set_model_configuration 注入 home，并持续向 arm/gripper 发轨迹维持 home（解暂停后仍有效）。"""
import time

import rospy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import SetModelConfiguration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
]
GRIPPER_JOINTS = ["link5_to_gripper_link"]


def _wait_controllers(controller_manager_ns, names, timeout_s):
    svc = "{}/list_controllers".format(controller_manager_ns.rstrip("/"))
    rospy.wait_for_service(svc, timeout=timeout_s)
    proxy = rospy.ServiceProxy(svc, ListControllers)
    deadline = time.time() + timeout_s
    need = set(names)
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            resp = proxy()
            st = {c.name: c.state for c in resp.controller}
            if need.issubset(st.keys()) and all(st[n] == "running" for n in need):
                return True
        except rospy.ServiceException:
            pass
        time.sleep(0.2)
    return False


def _set_joint_configuration(model_name, urdf_param_name, arm_pos, grip_pos, timeout_s):
    rospy.wait_for_service("/gazebo/set_model_configuration", timeout=timeout_s)
    proxy = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
    joint_names = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
    joint_positions = [float(x) for x in arm_pos] + [float(x) for x in grip_pos]
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            r = proxy(
                model_name=model_name,
                urdf_param_name=urdf_param_name,
                joint_names=joint_names,
                joint_positions=joint_positions,
            )
            if getattr(r, "success", True):
                rospy.loginfo("set_model_configuration: home pose applied to model %s", model_name)
                return True
        except rospy.ServiceException as e:
            rospy.logwarn("set_model_configuration retry: %s", e)
        time.sleep(0.2)
    return False


def _publish(pub, joints, positions, duration_sec):
    msg = JointTrajectory()
    msg.joint_names = list(joints)
    pt = JointTrajectoryPoint()
    pt.positions = [float(x) for x in positions]
    pt.time_from_start = rospy.Duration(duration_sec)
    msg.points.append(pt)
    pub.publish(msg)


def main():
    rospy.init_node("hold_arm_home", anonymous=False)

    arm_pos = rospy.get_param("~hold_positions")
    grip_pos = rospy.get_param("~hold_gripper_positions")
    if len(arm_pos) != len(ARM_JOINTS):
        rospy.logfatal("~hold_positions must have length %d", len(ARM_JOINTS))
        return
    if len(grip_pos) != len(GRIPPER_JOINTS):
        rospy.logfatal("~hold_gripper_positions must have length %d", len(GRIPPER_JOINTS))
        return

    rate_hz = float(rospy.get_param("~publish_rate", 30.0))
    point_duration = float(rospy.get_param("~point_duration", 0.15))
    send_gripper = bool(rospy.get_param("~send_gripper", True))
    cm_ns = rospy.get_param("~controller_manager_ns", "/controller_manager")
    wait_cm = float(rospy.get_param("~wait_controllers_timeout", 120.0))
    model_name = rospy.get_param("~gazebo_model_name", "myam_sys")
    urdf_param = rospy.get_param("~urdf_param_name", "robot_description")
    use_set_config = bool(rospy.get_param("~set_model_configuration", True))
    arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
    grip_topic = rospy.get_param("~gripper_command_topic", "/gripper_controller/command")

    need = ["joint_state_controller", "arm_controller"]
    if send_gripper:
        need.append("gripper_controller")

    rospy.loginfo("Waiting for controllers: %s", ", ".join(need))
    if not _wait_controllers(cm_ns, need, wait_cm):
        rospy.logerr("Timeout waiting for controllers; exiting.")
        return

    if use_set_config:
        _set_joint_configuration(model_name, urdf_param, arm_pos, grip_pos, wait_cm)

    pub_arm = rospy.Publisher(arm_topic, JointTrajectory, queue_size=3)
    pub_grip = None
    if send_gripper:
        pub_grip = rospy.Publisher(grip_topic, JointTrajectory, queue_size=3)

    t0 = time.time()
    while pub_arm.get_num_connections() == 0 and time.time() - t0 < wait_cm and not rospy.is_shutdown():
        time.sleep(0.05)
    if send_gripper and pub_grip is not None:
        t0 = time.time()
        while pub_grip.get_num_connections() == 0 and time.time() - t0 < wait_cm and not rospy.is_shutdown():
            time.sleep(0.05)

    rospy.loginfo(
        "Publishing home hold at %.1f Hz (point_duration=%.3f s). Unpause Gazebo when ready.",
        rate_hz,
        point_duration,
    )
    sleep_t = 1.0 / max(rate_hz, 1.0)
    while not rospy.is_shutdown():
        _publish(pub_arm, ARM_JOINTS, arm_pos, point_duration)
        if send_gripper and pub_grip is not None:
            _publish(pub_grip, GRIPPER_JOINTS, grip_pos, min(point_duration, 0.5))
        time.sleep(sleep_t)


if __name__ == "__main__":
    main()
