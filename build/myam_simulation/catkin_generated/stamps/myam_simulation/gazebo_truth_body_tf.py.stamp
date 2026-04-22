#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 Gazebo Classic /gazebo/link_states 读取机体 base_link 在世界系下的位姿，
广播 TF parent_frame -> child_frame（默认 map -> base_link）。

仿真里 MAVROS 的 map→base_link 常与物理高度不一致（EKF 原点 vs spawn z），
导致 RViz 里机体「陷进地面」而 Gazebo 正常。本节点与 Gazebo 几何一致。

前提：parent_frame（map）与 Gazebo world 原点对齐（SITL 常用）。
"""

from __future__ import print_function

import sys

import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped


class GazeboTruthBodyTf(object):
    def __init__(self):
        rospy.init_node("gazebo_truth_body_tf", anonymous=False)
        self.parent_frame = rospy.get_param("~parent_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        self.model_name = rospy.get_param("~model_name", "myuam")
        self.rate_hz = float(rospy.get_param("~publish_rate", 40.0))

        self._link_index = None
        self._pose = None
        self._warned = False
        # use_sim_time 下 clock 可能短暂停步；同一 stamp 重复发 map→base_link 会触发 TF_REPEATED_DATA
        self._last_tf_stamp = None

        self._br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/gazebo/link_states", LinkStates, self._on_link_states, queue_size=2)

        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._on_timer)
        rospy.loginfo(
            "gazebo_truth_body_tf: %s -> %s from model '%s'::base_link",
            self.parent_frame,
            self.child_frame,
            self.model_name,
        )

    def _resolve_index(self, names):
        exact = "%s::base_link" % self.model_name
        if exact in names:
            return names.index(exact)
        for i, n in enumerate(names):
            if n.endswith("::base_link") and self.model_name in n:
                return i
        return None

    def _on_link_states(self, msg):
        if self._link_index is None:
            self._link_index = self._resolve_index(msg.name)
            if self._link_index is None:
                if not self._warned:
                    rospy.logwarn_throttle(
                        10.0,
                        "未找到链路名包含 '%s::base_link'；当前 link 示例: %s",
                        self.model_name,
                        msg.name[: min(6, len(msg.name))],
                    )
                    self._warned = True
                return
            rospy.loginfo("使用 link_states[%d] = %s", self._link_index, msg.name[self._link_index])

        try:
            self._pose = msg.pose[self._link_index]
        except Exception:
            pass

    def _monotonic_stamp(self):
        """保证每次发布的 stamp 严格递增，避免 tf2 报 TF_REPEATED_DATA。"""
        t = rospy.Time.now()
        if self._last_tf_stamp is not None and t <= self._last_tf_stamp:
            ns = self._last_tf_stamp.nsecs + 1
            sec = self._last_tf_stamp.secs
            if ns >= 1000000000:
                sec += 1
                ns = 0
            t = rospy.Time(sec, ns)
        self._last_tf_stamp = t
        return t

    def _on_timer(self, _evt):
        if self._pose is None:
            return
        ts = TransformStamped()
        ts.header.stamp = self._monotonic_stamp()
        ts.header.frame_id = self.parent_frame
        ts.child_frame_id = self.child_frame
        p = self._pose.position
        o = self._pose.orientation
        ts.transform.translation.x = p.x
        ts.transform.translation.y = p.y
        ts.transform.translation.z = p.z
        ts.transform.rotation.x = o.x
        ts.transform.rotation.y = o.y
        ts.transform.rotation.z = o.z
        ts.transform.rotation.w = o.w
        self._br.sendTransform(ts)


def main():
    try:
        GazeboTruthBodyTf()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
