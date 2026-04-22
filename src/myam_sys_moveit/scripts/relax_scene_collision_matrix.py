#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
临时把 MoveIt planning scene 的 Allowed Collision Matrix (ACM) 全部设为允许碰撞，
用于区分「IK/工作空间不可达」与「因碰撞约束导致规划失败」。

用法（move_group 已运行时）：
  rosrun myam_sys_moveit relax_scene_collision_matrix.py
  rosrun myam_sys_moveit relax_scene_collision_matrix.py --restore-defaults

注意：
  - 这是仿真调试用；实机请勿使用。
  - 「全部允许」不等价于禁用碰撞检测实现的每一层（笛卡尔仍有 avoid_collisions 参数）。
  - 重启 move_group 或重新加载场景后 ACM 会恢复。

对应关系简述：
  - 笛卡尔 best_fraction≈0：常见于路径中途碰模型、IK 断裂、步长过大；关掉碰撞后若 fraction 变好，多半原先卡在碰撞。
  - OMPL TIME_OUT / 欧氏距很大：多为路径长或采样慢；若放松碰撞后立刻能解，也可能是碰撞钳制了可行域。
"""

from __future__ import print_function

import argparse
import sys

import rospy
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest, GetPlanningScene


def _flatten_acm(acm):
    """把所有成对碰撞检查设为允许（对称矩阵）。"""
    changed = False
    for entry in acm.entry_values:
        if not entry.enabled:
            continue
        new_e = [True] * len(entry.enabled)
        if list(entry.enabled) != new_e:
            changed = True
        entry.enabled = new_e
    for i in range(len(acm.default_entry_values)):
        if not acm.default_entry_values[i]:
            changed = True
        acm.default_entry_values[i] = True
    return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--restore-defaults",
        action="store_true",
        help="仅请求场景组件；不修改 ACM（用于触发 move_group 刷新；真正恢复请重启 move_group）",
    )
    args = parser.parse_args()

    rospy.init_node("relax_scene_collision_matrix", anonymous=False)

    get_nm = rospy.resolve_name("/get_planning_scene")
    apply_nm = rospy.resolve_name("/apply_planning_scene")
    rospy.loginfo("等待服务: %s , %s", get_nm, apply_nm)
    rospy.wait_for_service(get_nm, timeout=60.0)
    rospy.wait_for_service(apply_nm, timeout=60.0)

    get_scene = rospy.ServiceProxy(get_nm, GetPlanningScene)
    apply_scene = rospy.ServiceProxy(apply_nm, ApplyPlanningScene)

    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX

    try:
        resp = get_scene(req)
    except rospy.ServiceException as ex:
        rospy.logerr("get_planning_scene 失败: %s", ex)
        return 1

    if args.restore_defaults:
        rospy.logwarn(
            "未实现从 SRDF 重建默认 ACM；请重启 move_group 以恢复碰撞矩阵。"
        )
        return 0

    scene = resp.scene
    acm = scene.allowed_collision_matrix
    if not acm.entry_names:
        rospy.logwarn("当前 ACM 为空；仍尝试提交全允许 diff（部分版本需先有 entry）")

    _flatten_acm(acm)

    out = PlanningScene()
    out.is_diff = True
    out.allowed_collision_matrix = acm

    try:
        areq = ApplyPlanningSceneRequest()
        areq.scene = out
        ok = apply_scene(areq)
        if not ok.success:
            rospy.logerr("apply_planning_scene 返回 success=False")
            return 1
    except rospy.ServiceException as ex:
        rospy.logerr("apply_planning_scene 失败: %s", ex)
        return 1

    rospy.logwarn(
        "已提交「全允许」ACM diff。请在 RViz 里重新 Plan；验证完请重启 move_group 恢复。"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
