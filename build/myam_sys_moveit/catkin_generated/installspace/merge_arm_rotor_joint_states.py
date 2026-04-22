#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Subscribe to ros_control /joint_states (臂+夹爪)，补全 PX4 驱动的旋翼关节角后发布，供 MoveIt 整机状态完整。"""
from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState

ROTOR_JOINTS = ["rotor_%d_joint" % i for i in range(1, 7)]


def merge_and_publish(pub, msg):
    names = list(msg.name)
    pos = list(msg.position)
    vel = list(msg.velocity) if msg.velocity else []
    eff = list(msg.effort) if msg.effort else []
    have_vel = len(vel) == len(names)
    have_eff = len(eff) == len(names)

    existing = set(names)
    for jn in ROTOR_JOINTS:
        if jn in existing:
            continue
        names.append(jn)
        pos.append(0.0)
        if have_vel:
            vel.append(0.0)
        if have_eff:
            eff.append(0.0)

    out = JointState()
    out.header = msg.header
    out.name = names
    out.position = pos
    if have_vel:
        out.velocity = vel
    if have_eff:
        out.effort = eff
    pub.publish(out)


def main():
    rospy.init_node("merge_arm_rotor_joint_states", anonymous=False)
    src = rospy.get_param("~source_topic", "/joint_states")
    dst = rospy.get_param("~output_topic", "/joint_states_merged")
    rospy.loginfo(
        "merge_arm_rotor_joint_states: %s -> %s (+ %d rotor joints if missing)",
        src,
        dst,
        len(ROTOR_JOINTS),
    )
    pub = rospy.Publisher(dst, JointState, queue_size=10)

    def cb(msg):
        merge_and_publish(pub, msg)

    rospy.Subscriber(src, JointState, cb, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
