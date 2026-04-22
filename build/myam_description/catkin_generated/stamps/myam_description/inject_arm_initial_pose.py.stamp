#!/usr/bin/env python3
"""spawn 之后再次注入机械臂关节角（带 urdf_param_name=robot_description）。

spawn_model 内置的 -J 调用 set_model_configuration 时 urdf_param_name 为空字符串；
在暂停 t=0 时偶发无法同步到可视化/或与 ros_control 竞态。本节点在墙钟延时后
用完整 URDF 参数名重设一次关节，使「一出现」即为收起姿态（在暂停下亦应更新姿态）。
"""
import sys
import time

import rospy
from gazebo_msgs.srv import SetModelConfiguration

ARM_JOINTS = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
]
GRIPPER_JOINTS = ["link5_to_gripper_link"]


def main():
    rospy.init_node("inject_arm_initial_pose", anonymous=True)
    delay = float(rospy.get_param("~delay_before_inject", 2.0))
    model_name = rospy.get_param("~model_name", "myam_sys")
    urdf_param = rospy.get_param("~urdf_param_name", "robot_description")
    service = rospy.get_param("~set_config_service", "/gazebo/set_model_configuration")
    timeout = float(rospy.get_param("~service_timeout", 60.0))

    arm_pos = rospy.get_param("~arm_positions", [0.0, -1.5708, 2.95, 0.0, 0.0])
    grip_pos = rospy.get_param("~gripper_positions", [0.0])
    if len(arm_pos) != len(ARM_JOINTS):
        rospy.logfatal("~arm_positions 长度须为 %d", len(ARM_JOINTS))
        sys.exit(1)
    if len(grip_pos) != len(GRIPPER_JOINTS):
        rospy.logfatal("~gripper_positions 长度须为 %d", len(GRIPPER_JOINTS))
        sys.exit(1)

    joint_names = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
    joint_positions = [float(x) for x in arm_pos] + [float(x) for x in grip_pos]

    rospy.loginfo("将在 %.1f s 后调用 %s 注入关节（model=%s, urdf_param=%s）",
                  delay, service, model_name, urdf_param)
    time.sleep(delay)

    try:
        rospy.wait_for_service(service, timeout=timeout)
    except rospy.ROSException as exc:
        rospy.logerr("等待 %s 失败: %s", service, exc)
        sys.exit(1)

    proxy = rospy.ServiceProxy(service, SetModelConfiguration)
    try:
        resp = proxy(
            model_name=model_name,
            urdf_param_name=urdf_param,
            joint_names=joint_names,
            joint_positions=joint_positions,
        )
        if resp.success:
            rospy.loginfo("SetModelConfiguration: %s", resp.status_message)
        else:
            rospy.logerr("SetModelConfiguration 失败: %s", resp.status_message)
            sys.exit(1)
    except rospy.ServiceException as exc:
        rospy.logerr("SetModelConfiguration 异常: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
