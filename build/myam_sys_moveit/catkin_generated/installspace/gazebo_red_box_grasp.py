#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo 红色方块自动规划抓取（配合 gazebo_grasp_red_box.launch）。

启动示例::

  roslaunch myam_sys_moveit gazebo_grasp_red_box.launch
  # 另开终端（或见 gazebo_grasp_red_box_with_grasp.launch）::
  rosrun myam_sys_moveit gazebo_red_box_grasp.py

流程：从 /gazebo/get_model_state 读取 red_grasp_cube 位姿 → TF 到 roarm_base_link
→ [可选]在规划场景加碰撞盒（默认关，与 red_grasp_cube_rviz_sync 只保留一只盒）
→ 夹爪张开 → [可选]预抓取(超高) → 预抓取(高) → [可选]慢接近点 → 抓取位 → 闭合 → 抬升 → named target home。

参数要点（~auto_surface_offsets 默认 true）：
  路点 = 方块质心 (cx,cy,cz) + 偏移，Z 沿 planning_frame（默认 roarm_base_link）的 +Z；
  机体倾斜时请调 clearance 或改 auto_surface_offsets:=false 用手写偏移。

依赖：move_group、Gazebo、ros_control 机械臂与夹爪控制器。

5DOF + 自动抓取要点：
- 路点只用 TCP 位置（set_position_target），与 kinematics.yaml 的 position_only_ik 一致。
- 用 plan()+execute() 与降速重试区分「规划失败」与「执行 PATH_TOLERANCE_VIOLATED」；勿在仅位置失败后
  盲目 set_pose_target（易触发 Unable to sample goal）。可选 ~waypoints_pose_fallback:=true 才回退姿态。

借鉴 ROS2 官方 roarm_moveit_mtc_demo（MTC）思路在 ROS1 中的近似：
- 闭合前 ~mtc_style_allow_hand_object:=true 时对规划场景 ACM 临时允许「夹爪连杆 ↔ 抓取碰撞体」
  （对应 MTC 的 allow collision (hand, object)）。
- ~cartesian_final_approach 末段用 compute_cartesian_path 直线插补到抓取高度（对应 Cartesian 接近）。
- ~grasp_xy_retry_offsets 在抓取高度上尝试若干 XY 微偏（对应多抓取角/多 IK 采样思想）。

重要：MTC 官方 demo 有桌面等碰撞体；若 MoveIt 里**没有地面**，规划仍可能把指尖送进 Gazebo 地板，
仅靠手-物 ACM 不能防地碰。请用 ~grasp_extra_z_for_finger_length_m / clearance 抬高 TCP，或在规划场景加地面。
"""

from __future__ import print_function

import copy
import math
import sys
import threading
import time

import moveit_commander
import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.exception import MoveItCommanderException
from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene, PlanningSceneComponents

try:
    from tf2_geometry_msgs import do_transform_pose
except ImportError:
    do_transform_pose = None


def _planning_scene_components_acm():
    comp = PlanningSceneComponents()
    comp.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    return comp


def _acm_expand_entry(acm, name):
    """在 AllowedCollisionMatrix 中追加一行/列（与 MoveIt 语义一致的对称阵）。"""
    if name in acm.entry_names:
        return
    n = len(acm.entry_names)
    for i in range(n):
        acm.entry_values[i].enabled.append(False)
    row = AllowedCollisionEntry()
    row.enabled = [False] * (n + 1)
    acm.entry_names.append(name)
    acm.entry_values.append(row)


def _allow_hand_object_collision(scene_psi, object_id, touch_links):
    """
    临时允许 object_id 与 touch_links 中各连杆的碰撞（MTC 中 allow collision hand-object 的 ROS1 近似）。
    返回 (是否提交成功, restore_fn)；restore_fn 将对应条目恢复为修改前的布尔值。
    """
    if not object_id or not touch_links:
        return False, lambda: None
    psc = _planning_scene_components_acm()
    try:
        pm = scene_psi.get_planning_scene(psc)
        acm = copy.deepcopy(pm.allowed_collision_matrix)
    except Exception as e:
        rospy.logwarn("读取 ACM 失败，跳过手-物碰撞许可: %s", e)
        return False, lambda: None

    uniq_links = []
    for L in touch_links:
        if L and L not in uniq_links:
            uniq_links.append(L)
    backup = []
    for L in uniq_links:
        _acm_expand_entry(acm, object_id)
        _acm_expand_entry(acm, L)
        io = acm.entry_names.index(object_id)
        il = acm.entry_names.index(L)
        backup.append((object_id, L, bool(acm.entry_values[io].enabled[il])))
        acm.entry_values[io].enabled[il] = True
        acm.entry_values[il].enabled[io] = True

    out = PlanningScene()
    out.is_diff = True
    out.allowed_collision_matrix = acm
    try:
        scene_psi.apply_planning_scene(out)
    except Exception as e:
        rospy.logwarn("apply_planning_scene(ACM) 异常: %s", e)
        return False, lambda: None

    def restore():
        try:
            pm2 = scene_psi.get_planning_scene(psc)
            acm_r = pm2.allowed_collision_matrix
            for oa, ob, old in backup:
                if oa not in acm_r.entry_names or ob not in acm_r.entry_names:
                    continue
                ia = acm_r.entry_names.index(oa)
                ib = acm_r.entry_names.index(ob)
                acm_r.entry_values[ia].enabled[ib] = old
                acm_r.entry_values[ib].enabled[ia] = old
            out2 = PlanningScene()
            out2.is_diff = True
            out2.allowed_collision_matrix = acm_r
            scene_psi.apply_planning_scene(out2)
        except Exception as e:
            rospy.logwarn("恢复 ACM 失败（可重启 move_group）: %s", e)

    rospy.loginfo(
        "已临时允许 ACM: 物体 %s 与夹爪连杆 %s 碰撞（闭合后恢复）",
        object_id,
        ", ".join(uniq_links),
    )
    return True, restore


def _start_async_spinner(threads=4):
    try:
        from rospy.timer import AsyncSpinner as _AsyncSpinner
    except ImportError:
        _AsyncSpinner = None
    if _AsyncSpinner is not None:
        spin = _AsyncSpinner(threads)
        spin.start()
        return spin
    if hasattr(rospy, "AsyncSpinner"):
        spin = rospy.AsyncSpinner(threads)
        spin.start()
        return spin
    thr = threading.Thread(target=rospy.spin)
    thr.daemon = True
    thr.start()
    return thr


def _wait_move_group(timeout_sec=120.0):
    """move_group 在不同 launch 下可能挂 /get_planning_scene 或 /move_group/get_planning_scene。"""
    t0 = time.time()
    svcs = rospy.get_param(
        "~move_group_ready_services",
        ["/get_planning_scene", "/move_group/get_planning_scene"],
    )
    if isinstance(svcs, str):
        svcs = [svcs]
    while time.time() - t0 < timeout_sec and not rospy.is_shutdown():
        for svc in svcs:
            try:
                rospy.wait_for_service(svc, timeout=0.35)
                rospy.loginfo("move_group 就绪: %s", svc)
                return True
            except rospy.ROSException:
                continue
        rospy.loginfo_throttle(5.0, "等待 move_group（尝试服务: %s）…", ", ".join(svcs))
    return False


def main():
    rospy.init_node("gazebo_red_box_grasp", anonymous=False)
    _start_async_spinner()

    model_name = rospy.get_param("~gazebo_model_name", "red_grasp_cube")
    ref_frame = rospy.get_param("~planning_frame", "roarm_base_link")
    tf_timeout = rospy.get_param("~tf_timeout_sec", 5.0)
    box_size = rospy.get_param("~box_size", [0.05, 0.05, 0.05])
    scene_obj_id = rospy.get_param("~scene_object_id", "red_grasp_cube_collision")
    # launch 里已有 red_grasp_cube_rviz_sync 时勿再 add 第二只盒子，否则易「起始碰盒」+ 适配器改起点 → 轨迹首点与真机偏差
    add_scene_box = bool(rospy.get_param("~add_extra_collision_box", False))
    post_exec_settle = float(rospy.get_param("~post_exec_settle_sec", 0.35))

    auto_surface = rospy.get_param("~auto_surface_offsets", True)
    half_z = 0.5 * float(box_size[2])
    if auto_surface:
        # 相对质心：顶面在 cz+half_z。仅位置 IK 常选「下探」构型，指尖在 hand_tcp 下方可达约 10～18 cm；
        # 若只把 TCP 放在「盒顶 + 几厘米」，指尖仍会杵地/挤盒角 → Gazebo 接触爆炸（terminal18 Ogre 断言）+ 关节角越界。
        # ~grasp_extra_z_for_finger_length_m：把整段抓取路点在 +Z 上抬高，给指尖留余量（比官方 MTC 桌面 demo 更关键，
        # 因 MoveIt 里往往没有「地面」碰撞体，规划器不知道地板存在）。
        pre_above_top = float(rospy.get_param("~pregrasp_clearance_above_top_m", 0.12))
        grasp_above_top = float(rospy.get_param("~grasp_clearance_above_top_m", 0.042))
        finger_pad = float(rospy.get_param("~grasp_extra_z_for_finger_length_m", 0.07))
        pre_dx = float(rospy.get_param("~pregrasp_offset_x", 0.0))
        pre_dy = float(rospy.get_param("~pregrasp_offset_y", 0.0))
        grasp_dx = float(rospy.get_param("~grasp_offset_x", 0.0))
        grasp_dy = float(rospy.get_param("~grasp_offset_y", 0.0))
        pre_offset = [pre_dx, pre_dy, half_z + pre_above_top + finger_pad]
        grasp_offset = [grasp_dx, grasp_dy, half_z + grasp_above_top + finger_pad]
    else:
        finger_pad = 0.0
        pre_offset = list(rospy.get_param("~pregrasp_offset_base", [0.0, 0.0, 0.12]))
        grasp_offset = list(rospy.get_param("~grasp_offset_base", [0.0, 0.0, 0.0]))
    retreat_offset = rospy.get_param("~retreat_offset_base", [0.0, 0.0, 0.10])
    retreat_from_grasp = bool(rospy.get_param("~retreat_relative_to_grasp", True))
    slow_approach_dz = float(rospy.get_param("~slow_approach_delta_z", 0.06))
    # 第一次下降前可选「再抬高」一段；默认 0：过高时 TCP 易超出可达空间 → Unable to sample goal（见 terminal14）
    pre_staged_lift = float(rospy.get_param("~pregrasp_staged_extra_lift_m", 0.0))

    gripper_open = float(rospy.get_param("~gripper_open", 1.5))
    gripper_close = float(rospy.get_param("~gripper_close", 0.0))
    use_named_pregrasp = bool(rospy.get_param("~use_named_pregrasp", False))
    pregrasp_named_target = rospy.get_param("~pregrasp_named_target", "pregrasp")
    return_home = bool(rospy.get_param("~return_home", True))
    joint_template_fallback = bool(rospy.get_param("~joint_template_fallback", False))
    fallback_pregrasp_joints = rospy.get_param(
        "~fallback_pregrasp_joints", [0.0, -1.10, 1.70, -0.25, 0.0]
    )
    fallback_grasp_joints = rospy.get_param(
        "~fallback_grasp_joints", [0.0, -1.45, 2.10, -0.35, 0.0]
    )
    fallback_retreat_joints = rospy.get_param(
        "~fallback_retreat_joints", [0.0, -0.90, 1.20, -0.20, 0.0]
    )

    # 略降默认比例，减轻 Gazebo effort 关节轨迹跟踪超差（见 terminal PATH_TOLERANCE_VIOLATED）
    arm_vel = float(rospy.get_param("~arm_vel_scaling", 0.14))
    arm_acc = float(rospy.get_param("~arm_accel_scaling", 0.14))
    planning_time = float(rospy.get_param("~planning_time_sec", 20.0))
    exec_retries = int(rospy.get_param("~execute_max_retries", 4))
    exec_scale_decay = float(rospy.get_param("~execute_retry_vel_scale_decay", 0.55))
    pose_fallback = bool(rospy.get_param("~waypoints_pose_fallback", False))
    z_backoff_steps = int(rospy.get_param("~position_goal_z_backoff_steps", 6))
    z_backoff_dz = float(rospy.get_param("~position_goal_z_backoff_dz", 0.032))

    if do_transform_pose is None:
        rospy.logfatal("请安装 ros-$ROS_DISTRO-tf2-geometry-msgs")
        return 1

    rospy.loginfo("等待 Gazebo 与 move_group …")
    try:
        rospy.wait_for_service("/gazebo/get_model_state", timeout=180.0)
    except rospy.ROSException:
        rospy.logerr("未连接 /gazebo/get_model_state，请先启动 gazebo_grasp_red_box.launch")
        return 1

    if not _wait_move_group(180.0):
        rospy.logerr("move_group 未就绪")
        return 1

    rospy.sleep(1.0)

    moveit_commander.roscpp_initialize(sys.argv)
    scene = PlanningSceneInterface()
    arm = MoveGroupCommander("arm")
    gripper = MoveGroupCommander("gripper")
    robot = moveit_commander.RobotCommander()

    arm.set_planning_time(planning_time)
    arm.set_num_planning_attempts(12)
    arm.set_max_velocity_scaling_factor(arm_vel)
    arm.set_max_acceleration_scaling_factor(arm_acc)
    arm.set_pose_reference_frame(ref_frame)
    try:
        arm.set_goal_position_tolerance(0.035)
        arm.set_goal_orientation_tolerance(0.55)
    except Exception:
        pass

    gripper.set_max_velocity_scaling_factor(0.45)
    gripper.set_max_acceleration_scaling_factor(0.45)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    try:
        state = get_state(model_name, "world")
    except rospy.ServiceException as e:
        rospy.logerr("get_model_state 失败: %s", e)
        moveit_commander.roscpp_shutdown()
        return 1

    if not state.success:
        rospy.logerr("Gazebo 中未找到模型 %s（success=false）", model_name)
        moveit_commander.roscpp_shutdown()
        return 1

    ps_world = PoseStamped()
    ps_world.header.frame_id = "world"
    ps_world.header.stamp = rospy.Time(0)
    ps_world.pose = state.pose

    rospy.loginfo(
        "方块 Gazebo world 位姿: pos=(%.3f, %.3f, %.3f)",
        state.pose.position.x,
        state.pose.position.y,
        state.pose.position.z,
    )

    try:
        tfm = tf_buffer.lookup_transform(
            ref_frame, "world", rospy.Time(0), rospy.Duration(tf_timeout)
        )
        ps_base = do_transform_pose(ps_world, tfm)
    except Exception as e:
        rospy.logerr("TF world -> %s 失败: %s", ref_frame, e)
        moveit_commander.roscpp_shutdown()
        return 1

    cx = ps_base.pose.position.x
    cy = ps_base.pose.position.y
    cz = ps_base.pose.position.z
    rospy.loginfo("方块质心 %s: (%.3f, %.3f, %.3f)", ref_frame, cx, cy, cz)

    added_scene_box = False
    if add_scene_box:
        box_ps = PoseStamped()
        box_ps.header.frame_id = ref_frame
        box_ps.header.stamp = rospy.Time.now()
        box_ps.pose = ps_base.pose
        scene.add_box(scene_obj_id, box_ps, tuple(float(x) for x in box_size))
        rospy.sleep(0.8)
        added_scene_box = True
        rospy.loginfo("已在规划场景添加碰撞盒 %s", scene_obj_id)
    else:
        rospy.loginfo(
            "跳过本节点添加碰撞盒（默认与 red_grasp_cube_rviz_sync 共用场景）；"
            "无同步节点时请设 ~add_extra_collision_box:=true"
        )

    def _remove_scene_box_if_any():
        if added_scene_box:
            try:
                scene.remove_world_object(scene_obj_id)
            except Exception:
                pass

    def _pose_at_xyz(px, py, pz):
        cp = arm.get_current_pose()
        pose = cp.pose
        pose.position.x = float(px)
        pose.position.y = float(py)
        pose.position.z = float(pz)
        return pose

    use_pos_only = bool(rospy.get_param("~waypoints_position_only_first", True))

    def _plan_execute_at_z(label, px, py, pz):
        """固定 Z 的仅位置规划+执行（内部速度重试）。"""
        try:
            arm.clear_pose_targets()
        except MoveItCommanderException:
            pass
        arm.set_position_target([float(px), float(py), float(pz)])
        v0, a0 = arm_vel, arm_acc
        for attempt in range(max(1, exec_retries)):
            scale = max(0.06, (exec_scale_decay ** attempt))
            arm.set_max_velocity_scaling_factor(min(1.0, v0 * scale))
            arm.set_max_acceleration_scaling_factor(min(1.0, a0 * scale))
            try:
                arm.set_start_state_to_current_state()
            except Exception:
                pass
            err = None
            try:
                ok_plan, traj, _pt, err = arm.plan()
            except MoveItCommanderException as e:
                rospy.logwarn("%s: plan 异常 %s", label, e)
                ok_plan = False
                traj = None
            npts = 0
            if traj is not None and hasattr(traj, "joint_trajectory"):
                npts = len(traj.joint_trajectory.points)
            if not ok_plan or traj is None or npts < 1:
                rospy.logwarn(
                    "%s: 规划失败 (attempt %d/%d) err=%s points=%d",
                    label,
                    attempt + 1,
                    exec_retries,
                    getattr(err, "val", err) if err is not None else None,
                    npts,
                )
                continue
            try:
                ex_ok = arm.execute(traj, wait=True)
            except MoveItCommanderException as e:
                rospy.logwarn("%s: execute 异常 %s", label, e)
                ex_ok = False
            if ex_ok:
                arm.set_max_velocity_scaling_factor(v0)
                arm.set_max_acceleration_scaling_factor(a0)
                return True
            rospy.sleep(post_exec_settle)
            rospy.logwarn(
                "%s: 轨迹执行失败 (attempt %d/%d)，降速重试（effort 跟踪 / 起点偏差）",
                label,
                attempt + 1,
                exec_retries,
            )
        arm.set_max_velocity_scaling_factor(v0)
        arm.set_max_acceleration_scaling_factor(a0)
        return False

    def _plan_execute_position(label, px, py, pz_nominal):
        """在名义 Z 失败时沿 −Z 回退若干次，缓解「Unable to sample goal」与略超工作空间。"""
        pz_nominal = float(pz_nominal)
        for zb in range(max(1, z_backoff_steps + 1)):
            pz = pz_nominal - zb * z_backoff_dz
            lab = label if zb == 0 else "%s Z=%.3f" % (label, pz)
            if _plan_execute_at_z(lab, px, py, pz):
                if zb > 0:
                    rospy.loginfo(
                        "TCP 高度回退成功: 名义 z=%.3f → 使用 z=%.3f（共降 %.3f）",
                        pz_nominal,
                        pz,
                        zb * z_backoff_dz,
                    )
                return True
            if zb < z_backoff_steps:
                rospy.loginfo(
                    "%s: 规划/执行未成功，下一尝试将 TCP Z 降低 %.3f m",
                    label,
                    z_backoff_dz,
                )
        return False

    def _go_xyz(label, px, py, pz):
        try:
            if use_pos_only:
                if _plan_execute_position(label, px, py, pz):
                    return True
                if pose_fallback:
                    rospy.logwarn("%s: 仅位置多次失败，尝试完整位姿（5DOF 慎用）", label)
                    try:
                        arm.clear_pose_targets()
                    except MoveItCommanderException:
                        pass
                    arm.set_pose_target(_pose_at_xyz(px, py, pz))
                    return bool(arm.go(wait=True))
                return False
            arm.clear_pose_targets()
            arm.set_pose_target(_pose_at_xyz(px, py, pz))
            ok = arm.go(wait=True)
            if not ok:
                rospy.logwarn("%s: go() 失败，尝试仅位置", label)
                return _plan_execute_position(label, px, py, pz)
            return True
        except MoveItCommanderException as e:
            rospy.logerr("%s: %s", label, e)
            return False

    def _go_joints(label, joints):
        try:
            arm.clear_pose_targets()
        except MoveItCommanderException:
            pass
        try:
            arm.set_start_state_to_current_state()
        except Exception:
            pass
        try:
            arm.set_joint_value_target([float(x) for x in joints])
            ok = arm.go(wait=True)
        except MoveItCommanderException as e:
            rospy.logerr("%s: %s", label, e)
            return False
        if not ok:
            rospy.logwarn("%s: 关节目标执行失败 %s", label, joints)
        return bool(ok)

    # 预接近 / 抓取 / 抬升 路点（均在 planning_frame 下；须分别加 cx,cy,cz，勿误用 cx 三遍）
    cxyz = [cx, cy, cz]
    px = [float(pre_offset[i]) + cxyz[i] for i in range(3)]
    gx = [float(grasp_offset[i]) + cxyz[i] for i in range(3)]
    if retreat_from_grasp:
        rx = [float(retreat_offset[i]) + gx[i] for i in range(3)]
    else:
        rx = [float(retreat_offset[i]) + cxyz[i] for i in range(3)]

    rospy.loginfo(
        "预抓取(高): (%.3f, %.3f, %.3f) | 抓取 TCP: (%.3f, %.3f, %.3f) | 抬升: (%.3f, %.3f, %.3f)%s",
        px[0],
        px[1],
        px[2],
        gx[0],
        gx[1],
        gx[2],
        rx[0],
        rx[1],
        rx[2],
        (" | finger_pad=%.3fm（指尖余量）" % finger_pad) if auto_surface else "",
    )

    gripper.set_joint_value_target([gripper_open])
    if not gripper.go(wait=True):
        rospy.logerr("夹爪张开失败")
        _remove_scene_box_if_any()
        moveit_commander.roscpp_shutdown()
        return 1

    if use_named_pregrasp:
        try:
            rospy.loginfo("先移动到 arm 命名位姿 '%s'", pregrasp_named_target)
            arm.set_named_target(pregrasp_named_target)
            if not arm.go(wait=True):
                rospy.logerr("命名位姿 %s 执行失败", pregrasp_named_target)
                _remove_scene_box_if_any()
                moveit_commander.roscpp_shutdown()
                return 1
        except Exception as e:
            rospy.logerr("命名位姿 %s 不可用: %s", pregrasp_named_target, e)
            _remove_scene_box_if_any()
            moveit_commander.roscpp_shutdown()
            return 1

    ultra_z = px[2] + max(0.0, pre_staged_lift)
    if rospy.has_param("~pregrasp_ultra_tcp_max_z"):
        cap = float(rospy.get_param("~pregrasp_ultra_tcp_max_z"))
        if ultra_z > cap:
            rospy.logwarn(
                "预抓取超高 Z=%.3f 超过 ~pregrasp_ultra_tcp_max_z=%.3f，已钳位（避免不可达）",
                ultra_z,
                cap,
            )
            ultra_z = cap
    ok = True
    if pre_staged_lift > 1e-4 and (ultra_z - px[2]) > 0.02:
        ok = ok and _go_xyz("预抓取(超高)", px[0], px[1], ultra_z)
    ok = ok and _go_xyz("预抓取(高)", px[0], px[1], px[2])
    if slow_approach_dz > 1e-4:
        ok = ok and _go_xyz(
            "预抓取(抓取位上方)",
            gx[0],
            gx[1],
            gx[2] + slow_approach_dz,
        )

    # ---- 借鉴 MTC：手-物 ACM、末段笛卡尔直线、抓取 XY 微偏多试 ----
    raw_acm_id = rospy.get_param("~allow_collision_object_id", None)
    if raw_acm_id in (None, ""):
        acm_object_id = scene_obj_id if add_scene_box else "red_grasp_cube"
    else:
        acm_object_id = str(raw_acm_id)
    mtc_acm = bool(rospy.get_param("~mtc_style_allow_hand_object", True))
    cart_final = bool(rospy.get_param("~cartesian_final_approach", True))
    cart_step = float(rospy.get_param("~cartesian_eef_step", 0.012))
    cart_frac = float(rospy.get_param("~cartesian_min_fraction", 0.82))
    grasp_xy_offsets = rospy.get_param(
        "~grasp_xy_retry_offsets",
        [[0.0, 0.0], [0.008, 0.0], [-0.008, 0.0], [0.0, 0.008], [0.0, -0.008]],
    )

    touch_links = rospy.get_param("~gripper_touch_links", None)
    if touch_links in (None, []):
        try:
            touch_links = list(robot.get_link_names("gripper"))
        except Exception:
            touch_links = []
        if not touch_links:
            touch_links = ["gripper_link", "link5"]
    else:
        touch_links = list(touch_links)

    acm_restore = None
    acm_armed = False
    if mtc_acm and ok:
        _acm_ok, acm_restore = _allow_hand_object_collision(scene, acm_object_id, touch_links)
        acm_armed = bool(_acm_ok)
        rospy.sleep(0.15)

    def _cartesian_line_to(label, xg, yg, zg, eef_step, min_frac):
        try:
            arm.clear_pose_targets()
        except MoveItCommanderException:
            pass
        cur = arm.get_current_pose().pose
        dist = math.sqrt(
            (cur.position.x - xg) ** 2 + (cur.position.y - yg) ** 2 + (cur.position.z - zg) ** 2
        )
        if dist < 0.0015:
            return True
        n_seg = max(2, min(48, int(dist / max(0.004, eef_step)) + 1))
        wps = []
        for k in range(1, n_seg + 1):
            alpha = k / float(n_seg)
            p = copy.deepcopy(cur)
            p.position.x = cur.position.x * (1.0 - alpha) + xg * alpha
            p.position.y = cur.position.y * (1.0 - alpha) + yg * alpha
            p.position.z = cur.position.z * (1.0 - alpha) + zg * alpha
            wps.append(p)
        v0, a0 = arm_vel, arm_acc
        arm.set_max_velocity_scaling_factor(min(1.0, max(0.05, v0 * 0.85)))
        arm.set_max_acceleration_scaling_factor(min(1.0, max(0.05, a0 * 0.85)))
        try:
            arm.set_start_state_to_current_state()
        except Exception:
            pass
        try:
            traj, fraction = arm.compute_cartesian_path(wps, eef_step, avoid_collisions=True)
        except MoveItCommanderException as e:
            rospy.logwarn("%s: 笛卡尔规划异常 %s", label, e)
            arm.set_max_velocity_scaling_factor(v0)
            arm.set_max_acceleration_scaling_factor(a0)
            return False
        if fraction + 1e-4 < min_frac:
            rospy.logwarn("%s: 笛卡尔路径过短 fraction=%.2f (需>=%.2f)", label, fraction, min_frac)
            arm.set_max_velocity_scaling_factor(v0)
            arm.set_max_acceleration_scaling_factor(a0)
            return False
        try:
            ex_ok = arm.execute(traj, wait=True)
        except MoveItCommanderException as e:
            rospy.logwarn("%s: execute 异常 %s", label, e)
            ex_ok = False
        arm.set_max_velocity_scaling_factor(v0)
        arm.set_max_acceleration_scaling_factor(a0)
        return bool(ex_ok)

    def _reach_grasp_pose():
        for off in grasp_xy_offsets:
            if len(off) < 2:
                continue
            odx, ody = float(off[0]), float(off[1])
            x = gx[0] + odx
            y = gx[1] + ody
            z = gx[2]
            sub = "" if (abs(odx) < 1e-9 and abs(ody) < 1e-9) else " Δxy(%+.4f,%+.4f)" % (odx, ody)
            label = "下落至抓取位" + sub
            use_cart = bool(cart_final and slow_approach_dz > 1e-4)
            if use_cart and _cartesian_line_to(
                label + "(笛卡尔)", x, y, z, cart_step, cart_frac
            ):
                return True
            if _go_xyz(label, x, y, z):
                return True
        return False

    def _joint_template_sequence():
        rospy.logwarn("切换到关节模板兜底抓取模式")
        if not _go_joints("模板预抓取", fallback_pregrasp_joints):
            return False
        if not _go_joints("模板抓取", fallback_grasp_joints):
            return False
        return True

    grasp_ok = False
    if ok:
        grasp_ok = _reach_grasp_pose()
    if (not ok or not grasp_ok) and joint_template_fallback:
        grasp_ok = _joint_template_sequence()
        ok = grasp_ok
    if not ok or not grasp_ok:
        if acm_armed:
            try:
                acm_restore()
            except Exception:
                pass
        rospy.logerr(
            "手臂规划/执行失败：可调 ~pregrasp_clearance_above_top_m、~pregrasp_staged_extra_lift_m、"
            "~grasp_clearance_above_top_m、~slow_approach_delta_z、~pregrasp_offset_x/y、~arm_vel_scaling、"
            "~mtc_style_allow_hand_object、~cartesian_min_fraction、~grasp_xy_retry_offsets、~joint_template_fallback"
        )
        _remove_scene_box_if_any()
        moveit_commander.roscpp_shutdown()
        return 1

    try:
        gripper.set_joint_value_target([gripper_close])
        if not gripper.go(wait=True):
            rospy.logwarn("夹爪闭合未完全成功（可继续观察仿真）")
        if joint_template_fallback and _go_joints("模板抬升", fallback_retreat_joints):
            pass
        else:
            _go_xyz("抬升(相对抓取位)" if retreat_from_grasp else "抬升", rx[0], rx[1], rx[2])
    finally:
        if acm_armed:
            try:
                acm_restore()
            except Exception:
                pass

    if return_home:
        try:
            arm.set_named_target("home")
            arm.go(wait=True)
        except Exception as e:
            rospy.logwarn("回 home 失败: %s", e)

    _remove_scene_box_if_any()
    rospy.loginfo("抓取流程结束（若曾添加本节点碰撞盒则已移除）")
    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main() or 0)
    except rospy.ROSInterruptException:
        sys.exit(1)
