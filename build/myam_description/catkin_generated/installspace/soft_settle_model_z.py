#!/usr/bin/env python3
"""
在 Gazebo 仍暂停时，将模型从当前高度沿 world Z 缓降到 target_z，再交给用户手动解暂停。
用于避免「从高处静止解暂停 ≈ 自由落体第一帧」与地面的剧烈碰撞/卡地。
"""
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState


def main():
    rospy.init_node("soft_settle_model_z", anonymous=False)
    model = rospy.get_param("~model_name", "myam_sys")
    target_z = float(rospy.get_param("~target_z", 0.43))
    steps = max(2, int(rospy.get_param("~steps", 25)))
    wait_spawn = float(rospy.get_param("~wait_after_spawn_sec", 3.0))
    timeout = float(rospy.get_param("~service_timeout_sec", 60.0))

    rospy.sleep(wait_spawn)

    get_srv = "/gazebo/get_model_state"
    set_srv = "/gazebo/set_model_state"
    rospy.wait_for_service(get_srv, timeout=timeout)
    rospy.wait_for_service(set_srv, timeout=timeout)
    get_p = rospy.ServiceProxy(get_srv, GetModelState)
    set_p = rospy.ServiceProxy(set_srv, SetModelState)

    try:
        cur = get_p(model_name=model, relative_entity_name="world")
    except rospy.ServiceException as e:
        rospy.logerr("soft_settle: get_model_state failed: %s", e)
        return
    if hasattr(cur, "success") and not cur.success:
        rospy.logerr("soft_settle: model %s not found", model)
        return

    z0 = float(cur.pose.position.z)
    z1 = target_z
    if abs(z0 - z1) < 1e-4:
        rospy.loginfo("soft_settle: z already %.4f, skip.", z0)
        return

    rospy.loginfo("soft_settle: ramp z from %.3f to %.3f in %d steps (paused).", z0, z1, steps)
    dt = 0.04
    for i in range(1, steps + 1):
        if rospy.is_shutdown():
            return
        alpha = float(i) / float(steps)
        zi = z0 + alpha * (z1 - z0)
        st = ModelState()
        st.model_name = model
        st.pose = cur.pose
        st.pose.position.z = zi
        st.twist = cur.twist
        st.reference_frame = "world"
        try:
            set_p(model_state=st)
        except rospy.ServiceException as e:
            rospy.logwarn("soft_settle: set_model_state: %s", e)
        time.sleep(dt)

    rospy.loginfo(
        "soft_settle: done. 请在 Gazebo 中解暂停；target_z 需略高于真实接地点，避免穿地。"
    )


if __name__ == "__main__":
    main()
