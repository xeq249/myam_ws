#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 Gazebo 里生成的红色抓取方块同步到 MoveIt Planning Scene，便于 RViz MotionPlanning 中看到障碍物并测试 goal。

默认与 gazebo_grasp_red_box.launch 中 spawn 位姿、red_grasp_cube/model.sdf 尺寸一致：
  - 规划系使用 Gazebo 的 world（与 RViz Fixed Frame=world 一致）
  - 中心 (cube_x, cube_y, cube_z)，边长 0.05 m

可选在规划场景中加入一块**薄地面**（box），使 MoveIt 知道 z≈0 附近有障碍，避免指尖穿地。
默认用**方块附近局部地面**（~ground_anchor_xy_to_cube），勿用过大整块板：否则从机体下探到
方块的连杆易与整块地面相交，OMPL 会出现「Unable to sample any valid states for goal tree」。

只发规划场景，不跑完整抓取；需已启动 move_group。

  rosrun myam_sys_moveit red_grasp_cube_rviz_sync.py
  # 或在 gazebo_grasp_red_box.launch 中已自动带起
"""

from __future__ import print_function

import sys

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from moveit_commander.planning_scene_interface import PlanningSceneInterface


def main():
    rospy.init_node("red_grasp_cube_rviz_sync", anonymous=False)

    scene_frame = rospy.get_param("~scene_frame", "world")
    pos = rospy.get_param("~position", [0.3, 0.2, 0.025])
    size = rospy.get_param("~size", [0.05, 0.05, 0.05])
    object_id = rospy.get_param("~collision_object_id", "red_grasp_cube")
    tf_frame = rospy.get_param("~tf_frame", "red_grasp_cube_center")
    startup_sleep = float(rospy.get_param("~startup_sleep", 2.0))

    add_ground = bool(rospy.get_param("~add_ground_plane", True))
    ground_id = rospy.get_param("~ground_object_id", "gazebo_ground_plane")
    ground_center = rospy.get_param("~ground_center", [0.0, 0.0, -0.01])
    # 默认略大于方块工作区；过大整块板易挡「机体→方块」下探链路（terminal19）
    ground_size = rospy.get_param("~ground_size", [0.55, 0.55, 0.02])
    ground_anchor_xy = bool(rospy.get_param("~ground_anchor_xy_to_cube", True))
    ground_z_center = float(rospy.get_param("~ground_plane_z_center", -0.01))

    if len(pos) != 3 or len(size) != 3:
        rospy.logerr("~position / ~size 须为长度 3 的列表")
        return 1
    if add_ground and (len(ground_center) != 3 or len(ground_size) != 3):
        rospy.logerr("~ground_center / ~ground_size 须为长度 3 的列表")
        return 1

    rospy.sleep(startup_sleep)

    try:
        import tf2_ros
    except ImportError as ex:
        rospy.logerr("tf2_ros: %s", ex)
        return 1

    br = tf2_ros.StaticTransformBroadcaster()
    psi = PlanningSceneInterface(synchronous=True)
    if add_ground:
        psi.remove_world_object(ground_id)
    psi.remove_world_object(object_id)

    now = rospy.Time.now()

    if add_ground:
        # 薄 box：顶面约在 z=0（H=0.02 时 center z=-0.01）
        if ground_anchor_xy:
            gcx = float(pos[0])
            gcy = float(pos[1])
            gcz = ground_z_center
        else:
            gcx = float(ground_center[0])
            gcy = float(ground_center[1])
            gcz = float(ground_center[2])
        gp = PoseStamped()
        gp.header.frame_id = scene_frame
        gp.header.stamp = now
        gp.pose.position.x = gcx
        gp.pose.position.y = gcy
        gp.pose.position.z = gcz
        gp.pose.orientation.w = 1.0
        psi.add_box(
            ground_id,
            gp,
            size=(float(ground_size[0]), float(ground_size[1]), float(ground_size[2])),
        )
        rospy.loginfo(
            "Planning scene ground '%s' frame=%s center=(%.3f,%.3f,%.3f) size=(%.3f,%.3f,%.3f) anchor_xy=%s",
            ground_id,
            scene_frame,
            gcx,
            gcy,
            gcz,
            ground_size[0],
            ground_size[1],
            ground_size[2],
            ground_anchor_xy,
        )
        rospy.sleep(0.15)

    pose = PoseStamped()
    pose.header.frame_id = scene_frame
    pose.header.stamp = now
    pose.pose.position.x = float(pos[0])
    pose.pose.position.y = float(pos[1])
    pose.pose.position.z = float(pos[2])
    pose.pose.orientation.w = 1.0

    psi.add_box(
        object_id,
        pose,
        size=(float(size[0]), float(size[1]), float(size[2])),
    )

    ts = geometry_msgs.msg.TransformStamped()
    ts.header.stamp = now
    ts.header.frame_id = scene_frame
    ts.child_frame_id = tf_frame
    ts.transform.translation.x = float(pos[0])
    ts.transform.translation.y = float(pos[1])
    ts.transform.translation.z = float(pos[2])
    ts.transform.rotation.w = 1.0
    br.sendTransform(ts)

    rospy.loginfo(
        "Planning scene box '%s' frame=%s center=(%.3f,%.3f,%.3f) size=(%.3f,%.3f,%.3f)",
        object_id,
        scene_frame,
        pos[0],
        pos[1],
        pos[2],
        size[0],
        size[1],
        size[2],
    )

    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
