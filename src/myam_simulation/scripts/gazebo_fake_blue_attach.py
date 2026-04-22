#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo Classic 伪抓取：默认在「夹爪闭合 + TCP/方块距离」满足时吸附；
吸附后按 rate_hz 把 grasp_target_cube 锁到 hand_tcp（等）下，爬升/返航/降落期间
只要夹爪保持闭合（jp 未高于 open_above），方块会一直跟着机体。
可选 attach_while_open_enable：开爪极近也可吸（演示用）。
夹爪重新张到 open_above 以上则解除吸附，方块按物理下落。

仅用于仿真演示 / 任务联调，不是真实接触力抓取。

参数：
  ~enable                 总开关
  ~arm_model_name         Gazebo 机体模型名，默认 myuam
  ~attach_pose_source      tf=用 TF 中 attach_tf_frame（默认 hand_tcp，指尖/规划 TCP）；
                          link_states=用 attach_link_name 的 Gazebo link 位姿（腕部关节常在 link5 侧）
  ~attach_tf_frame         TF 子坐标系名，默认 hand_tcp（须与 robot_state_publisher 一致）
  ~tf_parent_frame        TF 父系，默认 map（须与 Gazebo world 对齐，否则位姿会偏）
  ~attach_link_name       link_states 模式下的 link 短名，默认 gripper_link
  ~cube_model_name        蓝块模型名，默认 grasp_target_cube
  ~cube_link_name         蓝块 link 短名，默认 cube_link
  ~gripper_joint_name     闭合判据关节，默认 link5_to_gripper_link
  ~gripper_closed_below   关节角小于该值视为已闭合（默认略放宽，便于轻夹后吸附）
  ~gripper_open_above     关节角大于该值视为已张开（解除吸附；须 > closed_below）
  ~max_attach_dist_m      参考 link 与方块质心距离小于此才允许吸附
  ~attach_while_open_enable  true：夹爪仍张开但 TCP 已极近方块时也可吸附（再靠 offboard_grasp 微闭爪）
  ~gripper_open_min_for_proximity_attach  张开吸附路径：关节角须 ≥ 此值（表示明显张开）
  ~max_attach_dist_open_m  张开吸附：TCP-方块距离阈值（米），宜略小于 max_attach_dist_m，防远距离误吸
  ~skip_distance_check   true：仅依据夹爪闭合+去抖即吸附（易穿帮，仅演示「必夹住」）
  ~offset_x/y/z_m         方块中心在参考 link 坐标系下的偏移（米）
  ~debounce_sec            闭合状态持续多久才触发吸附
  ~rate_hz                 吸附后同步频率
"""

from __future__ import print_function

import math

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState


def _quat_msg_to_R(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    return m[:3, :3]


def _pose_apply_local_offset(pose, ox, oy, oz):
    R = _quat_msg_to_R(pose.orientation)
    off = np.array([ox, oy, oz], dtype=np.float64)
    w = (
        np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
        + R.dot(off)
    )
    out = Pose()
    out.position.x, out.position.y, out.position.z = float(w[0]), float(w[1]), float(w[2])
    out.orientation = pose.orientation
    return out


class GazeboFakeBlueAttach(object):
    def __init__(self):
        rospy.init_node("gazebo_fake_blue_attach")
        self.enable = bool(rospy.get_param("~enable", False))
        self.arm_model = rospy.get_param("~arm_model_name", "myuam").strip()
        self.attach_pose_source = rospy.get_param(
            "~attach_pose_source", "tf"
        ).strip().lower()
        self.attach_tf_frame = rospy.get_param("~attach_tf_frame", "hand_tcp").strip()
        self.tf_parent_frame = rospy.get_param("~tf_parent_frame", "map").strip()
        self.attach_link = rospy.get_param("~attach_link_name", "gripper_link").strip()
        self.cube_model = rospy.get_param("~cube_model_name", "grasp_target_cube").strip()
        self.cube_link = rospy.get_param("~cube_link_name", "cube_link").strip()
        self.gripper_joint = rospy.get_param(
            "~gripper_joint_name", "link5_to_gripper_link"
        ).strip()
        # 略放宽：略张的「轻夹」也能进入闭爪吸附判据，便于闭爪返航后伪抓取跟上
        self.closed_below = float(rospy.get_param("~gripper_closed_below", 0.52))
        self.open_above = float(rospy.get_param("~gripper_open_above", 0.58))
        self.max_attach_dist = float(rospy.get_param("~max_attach_dist_m", 0.22))
        self.attach_while_open_enable = bool(
            rospy.get_param("~attach_while_open_enable", False)
        )
        self.open_min_jp = float(
            rospy.get_param("~gripper_open_min_for_proximity_attach", 0.58)
        )
        self.max_attach_dist_open = float(
            rospy.get_param(
                "~max_attach_dist_open_m",
                min(0.14, self.max_attach_dist),
            )
        )
        self.skip_distance_check = bool(
            rospy.get_param("~skip_distance_check", False)
        )
        # TF 模式默认 0：方块中心落在 hand_tcp；link_states 模式可保留小偏移补偿网格
        self.offset_x = float(rospy.get_param("~offset_x_m", 0.0))
        self.offset_y = float(rospy.get_param("~offset_y_m", 0.0))
        self.offset_z = float(rospy.get_param("~offset_z_m", 0.0))
        self.debounce = float(rospy.get_param("~debounce_sec", 0.28))
        self.rate_hz = float(rospy.get_param("~rate_hz", 40.0))

        self._full_grip = "%s::%s" % (self.arm_model, self.attach_link)
        self._full_cube = "%s::%s" % (self.cube_model, self.cube_link)

        self._ls = None
        self._ig = self._ic = None
        self._jp = None
        self._attached = False
        self._close_since = None
        self._open_prox_since = None

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.Subscriber("/gazebo/link_states", LinkStates, self._cb_link_states, queue_size=2)
        jst = rospy.get_param("~joint_states_topic", "/joint_states_merged")
        rospy.Subscriber(jst, JointState, self._cb_joint, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Timer(rospy.Duration(1.0 / max(8.0, self.rate_hz)), self._on_tick)

        rospy.loginfo(
            "[FAKE_ATTACH] enable=%s skip_dist=%s open_prox=%s cube=%s pose_src=%s (%s) | EN: magnet grasp",
            self.enable,
            self.skip_distance_check,
            self.attach_while_open_enable,
            self.cube_model,
            self.attach_pose_source,
            (
                "%s→%s" % (self.tf_parent_frame, self.attach_tf_frame)
                if self.attach_pose_source == "tf"
                else self._full_grip
            ),
        )

    def _resolve_indices(self, names):
        if self._ig is None:
            if self._full_grip in names:
                self._ig = names.index(self._full_grip)
            else:
                for i, n in enumerate(names):
                    if n.endswith("::" + self.attach_link) and self.arm_model in n:
                        self._ig = i
                        break
        if self._ic is None:
            if self._full_cube in names:
                self._ic = names.index(self._full_cube)
            else:
                for i, n in enumerate(names):
                    if n.endswith("::" + self.cube_link) and self.cube_model in n:
                        self._ic = i
                        break

    def _cb_link_states(self, msg):
        self._ls = msg
        self._resolve_indices(msg.name)

    def _cb_joint(self, msg):
        try:
            i = msg.name.index(self.gripper_joint)
            self._jp = float(msg.position[i])
        except ValueError:
            pass

    @staticmethod
    def _dist3(a, b):
        return math.sqrt(
            (a.position.x - b.position.x) ** 2
            + (a.position.y - b.position.y) ** 2
            + (a.position.z - b.position.z) ** 2
        )

    def _get_tcp_pose_world(self):
        """hand_tcp（等）在 tf_parent_frame 下的位姿，当作 Gazebo world 用（与 map 对齐时）。"""
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                self.attach_tf_frame,
                rospy.Time(0),
                rospy.Duration(0.35),
            )
        except Exception as e:
            rospy.logwarn_throttle(
                4.0,
                "[FAKE_ATTACH] TF %s→%s 失败: %s | EN: check robot_state_publisher / frame name",
                self.tf_parent_frame,
                self.attach_tf_frame,
                e,
            )
            return None
        tr = tfm.transform
        p = Pose()
        p.position.x = tr.translation.x
        p.position.y = tr.translation.y
        p.position.z = tr.translation.z
        p.orientation = tr.rotation
        return p

    def _distance_for_attach(self, gpose, cpose):
        """吸附判据距离：tf 模式用 TCP-方块，否则用 gripper_link-方块。"""
        if self.attach_pose_source == "tf" and cpose is not None:
            tp = self._get_tcp_pose_world()
            if tp is not None:
                return self._dist3(tp, cpose)
        return self._dist3(gpose, cpose)

    def _ref_pose_for_magnet(self):
        if self.attach_pose_source == "tf":
            return self._get_tcp_pose_world()
        if self._ls is not None and self._ig is not None:
            return self._ls.pose[self._ig]
        return None

    def _on_tick(self, _evt):
        if not self.enable:
            return
        if self._ls is None:
            rospy.logwarn_throttle(
                15.0,
                "[FAKE_ATTACH] 未收到 /gazebo/link_states | EN: start Gazebo (e.g. sitl_task.launch)",
            )
            return
        if self._jp is None:
            rospy.logwarn_throttle(
                10.0,
                "[FAKE_ATTACH] 尚无关节 '%s'（检查 joint_states_topic）| EN: no gripper joint yet",
                self.gripper_joint,
            )
            return
        if self._ig is None:
            rospy.logwarn_throttle(
                10.0,
                "[FAKE_ATTACH] 未找到 link_states 中的 %s",
                self._full_grip,
            )
            return
        if not self.skip_distance_check and self._ic is None:
            rospy.logwarn_throttle(
                10.0,
                "[FAKE_ATTACH] 未找到 link_states 中的 %s",
                self._full_cube,
            )
            return

        gpose = self._ls.pose[self._ig]
        if self._ic is not None:
            cpose = self._ls.pose[self._ic]
            d = self._distance_for_attach(gpose, cpose)
        else:
            d = float("nan")

        jp = float(self._jp)
        now = rospy.get_time()

        if self._attached:
            if jp > self.open_above:
                self._attached = False
                self._close_since = None
                self._open_prox_since = None
                rospy.loginfo("[FAKE_ATTACH] 松开→停止吸附 | EN: detach")
            else:
                ref = self._ref_pose_for_magnet()
                if ref is not None:
                    self._publish_cube(ref)
            return

        if jp < self.closed_below:
            self._open_prox_since = None
            if self._close_since is None:
                self._close_since = now
            can_dist = self.skip_distance_check or (
                not math.isnan(d) and d <= self.max_attach_dist
            )
            if now - self._close_since >= self.debounce and not can_dist:
                rospy.logwarn_throttle(
                    5.0,
                    "[FAKE_ATTACH] 爪已闭合 jp=%.3f 但 d=%.3f > max=%.3f m，不吸附 | EN: raise max_attach_dist_m or skip_distance_check:=true",
                    jp,
                    d,
                    self.max_attach_dist,
                )
            if now - self._close_since >= self.debounce and can_dist:
                ref = self._ref_pose_for_magnet()
                if ref is None:
                    rospy.logwarn_throttle(
                        3.0,
                        "[FAKE_ATTACH] 条件满足但无参考位姿（TF 未就绪？）| EN: no ref pose, skip attach",
                    )
                else:
                    self._attached = True
                    rospy.loginfo(
                        "[FAKE_ATTACH] 吸附(闭爪路径) d=%s jp=%.3f | EN: attach",
                        ("%.3f m" % d) if not math.isnan(d) else "n/a",
                        jp,
                    )
                    self._publish_cube(ref)
        elif self.attach_while_open_enable and jp >= self.open_min_jp:
            self._close_since = None
            can_open = self.skip_distance_check or (
                not math.isnan(d) and d <= self.max_attach_dist_open
            )
            if can_open:
                if self._open_prox_since is None:
                    self._open_prox_since = now
                if now - self._open_prox_since >= self.debounce:
                    ref = self._ref_pose_for_magnet()
                    if ref is None:
                        rospy.logwarn_throttle(
                            3.0,
                            "[FAKE_ATTACH] 张开近距但无参考位姿（TF？）| EN: no ref pose",
                        )
                    else:
                        self._attached = True
                        self._open_prox_since = None
                        rospy.loginfo(
                            "[FAKE_ATTACH] 吸附(开爪近距) d=%.3f m jp=%.3f (dmax_open=%.3f) | EN: attach open+prox",
                            d if not math.isnan(d) else float("nan"),
                            jp,
                            self.max_attach_dist_open,
                        )
                        self._publish_cube(ref)
            else:
                self._open_prox_since = None
        else:
            if self.attach_while_open_enable:
                rospy.loginfo_throttle(
                    25.0,
                    "[FAKE_ATTACH] 等待闭合(jp<%.3f)或开爪近距(jp≥%.3f+d<%.3f) | EN: wait close or open+prox",
                    self.closed_below,
                    self.open_min_jp,
                    self.max_attach_dist_open,
                )
            else:
                rospy.loginfo_throttle(
                    25.0,
                    "[FAKE_ATTACH] 等待夹爪闭合 jp=%.3f（需 < %.3f）后吸附；伪节点不闭爪，闭爪由 offboard_grasp | EN: wait close",
                    jp,
                    self.closed_below,
                )
            self._close_since = None
            self._open_prox_since = None

    def _publish_cube(self, ref_pose):
        p = _pose_apply_local_offset(
            ref_pose, self.offset_x, self.offset_y, self.offset_z
        )
        m = ModelState()
        m.model_name = self.cube_model
        m.pose = p
        m.twist = Twist()
        m.reference_frame = ""
        try:
            r = self._set_state(model_state=m)
            if not r.success:
                rospy.logwarn_throttle(
                    2.0,
                    "[FAKE_ATTACH] set_model_state: %s",
                    r.status_message,
                )
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "[FAKE_ATTACH] set_model_state: %s", e)


def main():
    GazeboFakeBlueAttach()
    rospy.spin()


if __name__ == "__main__":
    main()
