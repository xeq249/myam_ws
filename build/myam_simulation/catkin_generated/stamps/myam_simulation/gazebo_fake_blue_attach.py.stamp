#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo Classic 伪抓取：默认在「夹爪闭合 + TCP/方块距离」满足时吸附；
吸附后按 rate_hz 把 grasp_target_cube 锁到 hand_tcp（等）下，爬升/返航/降落期间
只要夹爪保持闭合（jp 未高于 open_above），方块会一直跟着机体。
可选 attach_while_open_enable：开爪极近也可吸（演示用）。
夹爪重新张到 open_above 以上持续 detach_open_debounce_sec 后才解除吸附（默认），方块按物理下落。

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
  ~gripper_closed_below   关节角小于该值视为已闭合（默认 0.45，与开爪演示区分）
  ~gripper_open_above     关节角大于该值视为已张开（默认 0.55，解除吸附）
  ~max_attach_dist_m      参考 link 与方块质心距离小于此才允许吸附
  ~attach_while_open_enable  true：夹爪仍张开但 TCP 已极近方块时也可吸附（再靠 offboard_grasp 微闭爪）
  ~gripper_open_min_for_proximity_attach  张开吸附路径：关节角须 ≥ 此值（表示明显张开）
  ~max_attach_dist_open_m  张开吸附：TCP-方块距离阈值（米），宜略小于 max_attach_dist_m，防远距离误吸
  ~skip_distance_check   true：仅依据夹爪闭合+去抖即吸附（易穿帮，仅演示「必夹住」）
  ~offset_x/y/z_m         方块中心在参考 link 坐标系下的偏移（米）
  ~debounce_sec            闭合状态持续多久才触发吸附
  ~detach_open_debounce_sec  jp>open_above 持续多久才解除吸附（防关节抖动误脱附）；0=立即脱附
  ~rate_hz                 吸附后同步频率（relative_frame 模式下可适当降至 40~60 减轻竞争）
  ~attach_use_reference_frame   true（默认）：相对手爪连杆 set_model_state，减轻 TF/world teleport 抖动
  ~attach_reference_link   覆盖参考连杆名（空则用 arm_model::attach_tf_frame，例如 myuam::hand_tcp）
  ~attach_soft_physics_on_attach 吸附时弱化方块动力学（质量×scale、关重力）；脱附自动恢复
  ~attach_soft_mass_scale  弱化系数，约 0.03~0.15
"""

from __future__ import print_function

import copy
import math

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import GetLinkProperties, SetLinkProperties, SetModelState
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
        # 经典判据：jp 低于 closed_below 视为已闭（可吸附）；高于 open_above 视为重新张开（解除吸附）
        self.closed_below = float(rospy.get_param("~gripper_closed_below", 0.45))
        self.open_above = float(rospy.get_param("~gripper_open_above", 0.62))
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
        self.detach_open_debounce_sec = float(
            rospy.get_param("~detach_open_debounce_sec", 0.35)
        )
        self.rate_hz = float(rospy.get_param("~rate_hz", 60.0))
        self.attach_use_reference_frame = bool(
            rospy.get_param("~attach_use_reference_frame", True)
        )
        _arl = rospy.get_param("~attach_reference_link", "").strip()
        if _arl:
            self._attach_ref_frame_body = _arl
        else:
            # 与 offset 约定一致：默认与 attach_tf_frame 同名连杆（URDF 中 hand_tcp 为固定连杆）
            self._attach_ref_frame_body = "%s::%s" % (
                self.arm_model,
                self.attach_tf_frame,
            )
        self.attach_soft_physics_on_attach = bool(
            rospy.get_param("~attach_soft_physics_on_attach", True)
        )
        self.attach_soft_mass_scale = float(
            rospy.get_param("~attach_soft_mass_scale", 0.06)
        )

        self._full_grip = "%s::%s" % (self.arm_model, self.attach_link)
        self._full_cube = "%s::%s" % (self.cube_model, self.cube_link)

        self._ls = None
        self._ig = self._ic = None
        self._jp = None
        self._attached = False
        self._close_since = None
        self._open_prox_since = None
        self._detach_open_since = None
        self._cube_link_props_backup = None
        self._cube_phys_softened = False

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.wait_for_service("/gazebo/get_link_properties", timeout=30.0)
        rospy.wait_for_service("/gazebo/set_link_properties", timeout=30.0)
        self._get_link_props = rospy.ServiceProxy(
            "/gazebo/get_link_properties", GetLinkProperties
        )
        self._set_link_props = rospy.ServiceProxy(
            "/gazebo/set_link_properties", SetLinkProperties
        )

        rospy.Subscriber("/gazebo/link_states", LinkStates, self._cb_link_states, queue_size=2)
        jst = rospy.get_param("~joint_states_topic", "/joint_states_merged")
        rospy.Subscriber(jst, JointState, self._cb_joint, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Timer(rospy.Duration(1.0 / max(8.0, self.rate_hz)), self._on_tick)

        rospy.loginfo(
            "[FAKE_ATTACH] enable=%s skip_dist=%s open_prox=%s cube=%s pose_src=%s (%s) ref_body=%s soft_phys=%s | EN: magnet grasp",
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
            self._attach_ref_frame_body,
            self.attach_soft_physics_on_attach,
        )

    def _maybe_soften_cube_physics(self):
        """吸附后减轻方块与爪碰撞「弹飞」：降质量、关重力；脱附恢复。"""
        if not self.attach_soft_physics_on_attach or self._cube_phys_softened:
            return
        link_name = "%s::%s" % (self.cube_model, self.cube_link)
        try:
            r = self._get_link_props(link_name)
            if not r.success:
                rospy.logwarn_throttle(
                    5.0,
                    "[FAKE_ATTACH] get_link_properties failed: %s",
                    r.status_message,
                )
                return
            scale = float(self.attach_soft_mass_scale)
            scale = max(0.02, min(1.0, scale))
            nm = max(1e-6, float(r.mass) * scale)
            sr = self._set_link_props(
                link_name=link_name,
                com=r.com,
                gravity_mode=False,
                mass=nm,
                ixx=float(r.ixx) * scale,
                ixy=float(r.ixy) * scale,
                ixz=float(r.ixz) * scale,
                iyy=float(r.iyy) * scale,
                iyz=float(r.iyz) * scale,
                izz=float(r.izz) * scale,
            )
            if sr.success:
                self._cube_link_props_backup = (
                    float(r.mass),
                    bool(r.gravity_mode),
                    float(r.ixx),
                    float(r.ixy),
                    float(r.ixz),
                    float(r.iyy),
                    float(r.iyz),
                    float(r.izz),
                    copy.deepcopy(r.com),
                )
                self._cube_phys_softened = True
                rospy.loginfo(
                    "[FAKE_ATTACH] 方块动力学软化 mass×%.2f 关重力 | EN: soft cube physics",
                    scale,
                )
            else:
                rospy.logwarn_throttle(
                    3.0,
                    "[FAKE_ATTACH] set_link_properties: %s",
                    sr.status_message,
                )
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "[FAKE_ATTACH] soften physics: %s", e)

    def _restore_cube_physics(self):
        if self._cube_link_props_backup is None:
            self._cube_phys_softened = False
            return
        link_name = "%s::%s" % (self.cube_model, self.cube_link)
        (
            mass,
            grav,
            ixx,
            ixy,
            ixz,
            iyy,
            iyz,
            izz,
            com,
        ) = self._cube_link_props_backup
        try:
            sr = self._set_link_props(
                link_name=link_name,
                com=com,
                gravity_mode=grav,
                mass=mass,
                ixx=ixx,
                ixy=ixy,
                ixz=ixz,
                iyy=iyy,
                iyz=iyz,
                izz=izz,
            )
            if sr.success:
                rospy.loginfo("[FAKE_ATTACH] 已恢复方块原始质量/重力 | EN: restore cube physics")
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(3.0, "[FAKE_ATTACH] restore physics: %s", e)
        self._cube_phys_softened = False
        self._cube_link_props_backup = None

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
        was_attached = self._attached
        try:
            if not self.enable:
                if was_attached:
                    self._attached = False
                    self._close_since = None
                    self._open_prox_since = None
                    self._detach_open_since = None
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
                    ddb = float(self.detach_open_debounce_sec)
                    if ddb <= 1e-9:
                        self._attached = False
                        self._close_since = None
                        self._open_prox_since = None
                        self._detach_open_since = None
                        rospy.loginfo("[FAKE_ATTACH] 松开→停止吸附 | EN: detach")
                    else:
                        if self._detach_open_since is None:
                            self._detach_open_since = now
                        elif now - self._detach_open_since >= ddb:
                            self._attached = False
                            self._close_since = None
                            self._open_prox_since = None
                            self._detach_open_since = None
                            rospy.loginfo(
                                "[FAKE_ATTACH] 松开(持续 %.2fs>%.3f)→停止吸附 | EN: detach debounced",
                                ddb,
                                self.open_above,
                            )
                        # 去抖窗口内仍保持吸附同步，否则 physics 会与 set_model_state 竞赛导致方块掉落
                else:
                    self._detach_open_since = None

                if self._attached:
                    ref = self._ref_pose_for_magnet()
                    # reference_frame 相对连杆吸附：不依赖 TF 世界位姿；TF 抖动/丢失时仍能同步方块
                    if self.attach_use_reference_frame:
                        self._publish_cube(ref)
                    elif ref is not None:
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

        finally:
            # 任意 return 路径上脱附后统一恢复方块原始 link 动力学（含 enable:=false）
            if was_attached and not self._attached:
                self._restore_cube_physics()

    def _publish_cube(self, ref_pose):
        self._maybe_soften_cube_physics()
        if self.attach_use_reference_frame:
            # 相对夹爪连杆位姿：避免每帧用 map/world 绝对坐标 teleport 与接触约束对冲（抖动→ODE 发散）
            p = Pose()
            p.position.x = float(self.offset_x)
            p.position.y = float(self.offset_y)
            p.position.z = float(self.offset_z)
            p.orientation.w = 1.0
            m = ModelState()
            m.model_name = self.cube_model
            m.pose = p
            m.twist = Twist()
            m.reference_frame = self._attach_ref_frame_body
        else:
            if ref_pose is None:
                rospy.logwarn_throttle(
                    3.0,
                    "[FAKE_ATTACH] world 模式无参考位姿，跳过 set_model_state | EN: no ref pose",
                )
                return
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
