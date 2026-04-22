#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将仿真场景中的台面/蓝块映射到 RViz + MoveIt：
  - 发布 static TF: scene_frame -> 各物体子坐标系（RViz TF 树）
  - 向 Planning Scene 添加 boxes（MotionPlanning → Scene Geometry 可见）

用法一（兼容旧 launch）：单物体，参数 ~position / ~size / ~collision_object_id

用法二：参数 ~objects 为非空 YAML 列表，每项含 id、position、size，可选 tf_frame：
  roslaunch myam_simulation myam_grasping_scene_sync.launch

默认 scene_frame=map：需 MAVROS local_position/tf（sitl 常见）。RViz Fixed Frame 请选 map（或
与 scene_frame 一致），否则盒子与机体对不齐。
"""

from __future__ import print_function

import sys

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from moveit_commander.planning_scene_interface import PlanningSceneInterface


def main():
    rospy.init_node("grasp_blue_cube_rviz_sync", anonymous=False)

    scene_frame = rospy.get_param("~scene_frame", "map")
    cube_tf_frame = rospy.get_param("~cube_tf_frame", "grasp_blue_cube")
    collision_id = rospy.get_param("~collision_object_id", "grasp_blue_cube")
    pos = rospy.get_param("~position", [1.46, 0.98, 0.02])
    size = rospy.get_param("~size", [0.04, 0.04, 0.04])
    startup_sleep = rospy.get_param("~startup_sleep", 3.0)

    objects_param = rospy.get_param("~objects", None)
    if objects_param is not None and len(objects_param) > 0:
        objects = objects_param
    else:
        if len(pos) != 3 or len(size) != 3:
            rospy.logerr("position / size must be length-3 lists")
            return 1
        objects = [
            {
                "id": collision_id,
                "tf_frame": cube_tf_frame,
                "position": pos,
                "size": size,
            }
        ]

    for i, obj in enumerate(objects):
        if "id" not in obj or "position" not in obj or "size" not in obj:
            rospy.logerr("objects[%d] 需要 id, position, size", i)
            return 1
        p = obj["position"]
        s = obj["size"]
        if len(p) != 3 or len(s) != 3:
            rospy.logerr("objects[%d] position/size 长度须为 3", i)
            return 1

    rospy.sleep(startup_sleep)

    try:
        import tf2_ros
    except ImportError as ex:
        rospy.logerr("tf2_ros: %s", ex)
        return 1

    br = tf2_ros.StaticTransformBroadcaster()
    psi = PlanningSceneInterface(synchronous=True)

    now = rospy.Time.now()
    tf_list = []
    for obj in objects:
        oid = str(obj["id"])
        p = [float(x) for x in obj["position"]]
        s = [float(x) for x in obj["size"]]
        child = str(obj.get("tf_frame", oid))

        psi.remove_world_object(oid)

        pose = PoseStamped()
        pose.header.frame_id = scene_frame
        pose.header.stamp = now
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.w = 1.0

        psi.add_box(
            oid,
            pose,
            size=(s[0], s[1], s[2]),
        )
        rospy.loginfo(
            "Planning scene box '%s' in %s center=(%.3f,%.3f,%.3f) size=(%.3f,%.3f,%.3f)",
            oid,
            scene_frame,
            p[0],
            p[1],
            p[2],
            s[0],
            s[1],
            s[2],
        )

        ts = geometry_msgs.msg.TransformStamped()
        ts.header.stamp = now
        ts.header.frame_id = scene_frame
        ts.child_frame_id = child
        ts.transform.translation.x = p[0]
        ts.transform.translation.y = p[1]
        ts.transform.translation.z = p[2]
        ts.transform.rotation.x = 0.0
        ts.transform.rotation.y = 0.0
        ts.transform.rotation.z = 0.0
        ts.transform.rotation.w = 1.0
        tf_list.append(ts)
        rospy.loginfo(
            "static TF %s -> %s (%.3f, %.3f, %.3f)",
            scene_frame,
            child,
            p[0],
            p[1],
            p[2],
        )

    br.sendTransform(tf_list)

    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
