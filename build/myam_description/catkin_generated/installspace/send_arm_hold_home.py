#!/usr/bin/env python3
"""myam_description：地面 spawn + 暂停启动后，等控制器就绪，解暂停前/后持续发 home 轨迹。

说明：
- Gazebo 暂停时仿真时间不前进，关节轨迹在「物理步进」意义上不会积分；真正抗重力主要靠解暂停后的高频维持。
- 暂停阶段持续发指令仍有用：确保与 arm_controller 的订阅已连通，并在解暂停瞬间即有目标。
- 模型初始姿态务必与 spawn 的 -J 一致，且与 ~hold_positions 一致（由 launch 从同一组 spawn_j* 传入）。
- 默认 Gazebo 模型名为 myam_sys，URDF 参数为 robot_description（与 gazebo.launch 一致）。
"""
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, SetModelState
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
]
GRIPPER_JOINTS = ["link5_to_gripper_link"]


def _sleep_wall(sec):
    time.sleep(sec)


def _wait_pub(pub, topic, timeout=30.0):
    t0 = time.time()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if (time.time() - t0) > timeout:
            rospy.logerr("Timeout waiting for subscribers on %s", topic)
            return False
        _sleep_wall(0.1)
    return True


def _publish_hold(pub, joints, positions, duration):
    msg = JointTrajectory()
    msg.joint_names = list(joints)
    pt = JointTrajectoryPoint()
    pt.positions = [float(x) for x in positions]
    pt.velocities = []
    pt.accelerations = []
    pt.time_from_start = rospy.Duration(duration)
    msg.points.append(pt)
    pub.publish(msg)


def _wait_controllers_running(controller_manager_ns, controller_names, timeout_s=30.0):
    service_name = "{}/list_controllers".format(controller_manager_ns.rstrip("/"))
    rospy.loginfo("Waiting for running controllers via %s ...", service_name)
    try:
        rospy.wait_for_service(service_name, timeout=timeout_s)
    except rospy.ROSException as exc:
        rospy.logwarn("Service %s not available: %s", service_name, exc)
        return False

    proxy = rospy.ServiceProxy(service_name, ListControllers)
    deadline = time.time() + timeout_s
    required = set(controller_names)

    while time.time() < deadline and not rospy.is_shutdown():
        try:
            resp = proxy()
            state_map = {c.name: c.state for c in resp.controller}
            if required.issubset(set(state_map.keys())) and all(state_map[name] == "running" for name in required):
                rospy.loginfo("Controllers are running: %s", ", ".join(sorted(required)))
                return True
        except rospy.ServiceException as exc:
            rospy.logwarn("Retrying %s: %s", service_name, exc)
        _sleep_wall(0.2)

    rospy.logwarn("Timed out waiting controllers to run: %s", ", ".join(sorted(required)))
    return False


def _set_model_pose_z(model_name, target_z, timeout_s=60.0):
    get_service = "/gazebo/get_model_state"
    set_service = "/gazebo/set_model_state"
    rospy.loginfo("Waiting for %s and %s to set model height...", get_service, set_service)
    try:
        rospy.wait_for_service(get_service, timeout=timeout_s)
        rospy.wait_for_service(set_service, timeout=timeout_s)
    except rospy.ROSException as exc:
        rospy.logwarn("Model state services not available: %s", exc)
        return False

    get_proxy = rospy.ServiceProxy(get_service, GetModelState)
    set_proxy = rospy.ServiceProxy(set_service, SetModelState)
    deadline = time.time() + timeout_s

    while time.time() < deadline and not rospy.is_shutdown():
        try:
            current = get_proxy(model_name=model_name, relative_entity_name="world")
            if hasattr(current, "success") and not current.success:
                rospy.logwarn("get_model_state not ready yet: %s", getattr(current, "status_message", "unknown"))
                _sleep_wall(0.2)
                continue

            state = ModelState()
            state.model_name = model_name
            state.pose = current.pose
            state.pose.position.z = float(target_z)
            state.twist = current.twist
            state.reference_frame = "world"

            resp = set_proxy(model_state=state)
            if hasattr(resp, "success") and not resp.success:
                rospy.logwarn("set_model_state not ready yet: %s", getattr(resp, "status_message", "unknown"))
                _sleep_wall(0.2)
                continue

            rospy.loginfo("Set model %s base height to z=%.3f", model_name, float(target_z))
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn("Retrying set_model_state: %s", exc)
            _sleep_wall(0.2)

    rospy.logwarn("Timed out setting model %s base height to z=%.3f", model_name, float(target_z))
    return False


def _ramp_model_pose_z(model_name, target_z, steps=10, duration_s=1.5, timeout_s=60.0):
    get_service = "/gazebo/get_model_state"
    set_service = "/gazebo/set_model_state"
    try:
        rospy.wait_for_service(get_service, timeout=timeout_s)
        rospy.wait_for_service(set_service, timeout=timeout_s)
    except rospy.ROSException as exc:
        rospy.logwarn("Model state services not available for ramp: %s", exc)
        return False

    get_proxy = rospy.ServiceProxy(get_service, GetModelState)
    set_proxy = rospy.ServiceProxy(set_service, SetModelState)
    try:
        current = get_proxy(model_name=model_name, relative_entity_name="world")
    except rospy.ServiceException as exc:
        rospy.logwarn("Failed to read model %s state before ramp: %s", model_name, exc)
        return False
    if hasattr(current, "success") and not current.success:
        rospy.logwarn("get_model_state before ramp failed: %s", getattr(current, "status_message", "unknown"))
        return False

    z0 = float(current.pose.position.z)
    z1 = float(target_z)
    n = max(1, int(steps))
    dt = 0.0 if n <= 0 else max(0.0, float(duration_s)) / float(n)

    for i in range(1, n + 1):
        alpha = float(i) / float(n)
        zi = z0 + alpha * (z1 - z0)
        state = ModelState()
        state.model_name = model_name
        state.pose = current.pose
        state.pose.position.z = zi
        state.twist = current.twist
        state.reference_frame = "world"
        try:
            resp = set_proxy(model_state=state)
            if hasattr(resp, "success") and not resp.success:
                rospy.logwarn("set_model_state ramp step failed: %s", getattr(resp, "status_message", "unknown"))
                return False
        except rospy.ServiceException as exc:
            rospy.logwarn("set_model_state ramp step exception: %s", exc)
            return False
        if dt > 0.0:
            _sleep_wall(dt)

    rospy.loginfo("Ramped model %s base height from z=%.3f to z=%.3f", model_name, z0, z1)
    return True


def _set_model_joint_home(model_name, urdf_param_name, arm_positions, gripper_positions, timeout_s=60.0):
    service_name = "/gazebo/set_model_configuration"
    rospy.loginfo("Waiting for %s to set startup home pose...", service_name)
    try:
        rospy.wait_for_service(service_name, timeout=timeout_s)
    except rospy.ROSException as exc:
        rospy.logwarn("Service %s not available: %s", service_name, exc)
        return False

    joint_names = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
    joint_positions = [float(x) for x in arm_positions] + [float(x) for x in gripper_positions]
    proxy = rospy.ServiceProxy(service_name, SetModelConfiguration)
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            resp = proxy(model_name=model_name, urdf_param_name=urdf_param_name,
                         joint_names=joint_names, joint_positions=joint_positions)
            if hasattr(resp, "success") and not resp.success:
                rospy.logwarn("set_model_configuration not ready yet: %s", getattr(resp, "status_message", "unknown"))
                _sleep_wall(0.2)
                continue
            rospy.loginfo("Set model %s joint configuration to startup home.", model_name)
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn("Retrying set_model_configuration: %s", exc)
            _sleep_wall(0.2)
    rospy.logwarn("Timed out setting model %s startup home pose.", model_name)
    return False


def main():
    rospy.init_node("send_arm_hold_home", anonymous=True)

    wait = rospy.get_param("~wait_after_start", 2.0)
    duration = rospy.get_param("~move_duration", 0.3)
    subscriber_timeout = rospy.get_param("~wait_for_subscribers_timeout", 180.0)
    arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
    gripper_topic = rospy.get_param("~gripper_command_topic", "/gripper_controller/command")
    send_gripper = rospy.get_param("~send_gripper_hold", True)
    unpause_gazebo = rospy.get_param("~unpause_gazebo", True)
    unpause_service = rospy.get_param("~unpause_service", "/gazebo/unpause_physics")
    set_home_with_gazebo = rospy.get_param("~set_home_with_gazebo", False)
    gazebo_model_name = rospy.get_param("~gazebo_model_name", "myam_sys")
    urdf_param_name = rospy.get_param("~urdf_param_name", "robot_description")
    pre_home_air_z = rospy.get_param("~pre_home_air_z", -1.0)
    ground_z_before_unpause = rospy.get_param("~ground_z_before_unpause", -1.0)
    lower_to_ground_after_unpause = rospy.get_param("~lower_to_ground_after_unpause", False)
    ground_settle_after_unpause_sec = rospy.get_param("~ground_settle_after_unpause_sec", 1.5)
    hold_republish_seconds = rospy.get_param("~hold_republish_seconds", 25.0)
    hold_publish_rate_hz = rospy.get_param("~hold_publish_rate_hz", 30.0)
    controller_manager_ns = rospy.get_param("~controller_manager_ns", "/controller_manager")
    wait_controllers_timeout = rospy.get_param("~wait_controllers_timeout", 120.0)
    ground_lower_steps = rospy.get_param("~ground_lower_steps", 12)
    ground_lower_duration_s = rospy.get_param("~ground_lower_duration_s", 2.0)
    pre_unpause_hold_seconds = rospy.get_param("~pre_unpause_hold_seconds", 12.0)
    hold_rate_hz_while_paused = rospy.get_param("~hold_rate_hz_while_paused", 30.0)
    hold_point_duration = rospy.get_param("~hold_point_duration", 0.05)

    arm_positions = rospy.get_param("~hold_positions", [0.0, -1.5708, 2.95, 0.0, 0.0])
    if len(arm_positions) != len(ARM_JOINTS):
        rospy.logfatal("~hold_positions length must be %d", len(ARM_JOINTS))
        return

    gripper_positions = rospy.get_param("~hold_gripper_positions", [0.0])
    if len(gripper_positions) != len(GRIPPER_JOINTS):
        rospy.logfatal("~hold_gripper_positions length must be %d", len(GRIPPER_JOINTS))
        return

    _sleep_wall(wait)

    if float(pre_home_air_z) >= 0.0:
        _set_model_pose_z(gazebo_model_name, pre_home_air_z, subscriber_timeout)

    if set_home_with_gazebo:
        _set_model_joint_home(gazebo_model_name, urdf_param_name, arm_positions, gripper_positions, subscriber_timeout)

    if float(ground_z_before_unpause) >= 0.0 and not lower_to_ground_after_unpause:
        _set_model_pose_z(gazebo_model_name, ground_z_before_unpause, subscriber_timeout)

    pub_arm = rospy.Publisher(arm_topic, JointTrajectory, queue_size=1)
    if not _wait_pub(pub_arm, arm_topic, subscriber_timeout):
        return

    if send_gripper:
        pub_gripper = rospy.Publisher(gripper_topic, JointTrajectory, queue_size=1)
        if not _wait_pub(pub_gripper, gripper_topic, subscriber_timeout):
            return
    else:
        pub_gripper = None

    controllers = ["joint_state_controller", "arm_controller"]
    if send_gripper:
        controllers.append("gripper_controller")
    if not _wait_controllers_running(controller_manager_ns, controllers, wait_controllers_timeout):
        rospy.logwarn("Controllers not all running before hold; continuing anyway.")

    pt_dur = float(hold_point_duration)
    if pre_unpause_hold_seconds > 0.0 and hold_rate_hz_while_paused > 0.0:
        period = 1.0 / float(hold_rate_hz_while_paused)
        deadline = time.time() + float(pre_unpause_hold_seconds)
        rospy.loginfo(
            "Holding home for %.1f s at %.1f Hz while Gazebo paused (before unpause).",
            float(pre_unpause_hold_seconds),
            float(hold_rate_hz_while_paused),
        )
        while time.time() < deadline and not rospy.is_shutdown():
            _publish_hold(pub_arm, ARM_JOINTS, arm_positions, pt_dur)
            if send_gripper and pub_gripper is not None:
                _publish_hold(pub_gripper, GRIPPER_JOINTS, gripper_positions, min(pt_dur, 2.0))
            _sleep_wall(period)

    unpause_ok = True
    if unpause_gazebo:
        try:
            rospy.wait_for_service(unpause_service, timeout=10.0)
            unpause = rospy.ServiceProxy(unpause_service, Empty)
            unpause()
            rospy.loginfo("Unpaused Gazebo via %s", unpause_service)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            unpause_ok = False
            rospy.logerr("Failed to call %s: %s", unpause_service, exc)
            rospy.logerr(
                "若出现 Connection reset：通常是 gzserver 已崩溃。请检查物理/接触与控制器。"
            )

    if not unpause_ok:
        rospy.logwarn("Skipping post-unpause hold: Gazebo is not running.")
        return

    if hold_republish_seconds > 0.0 and hold_publish_rate_hz > 0.0:
        period = 1.0 / float(hold_publish_rate_hz)
        deadline = time.time() + float(hold_republish_seconds)
        while time.time() < deadline and not rospy.is_shutdown():
            _publish_hold(pub_arm, ARM_JOINTS, arm_positions, duration)
            if send_gripper and pub_gripper is not None:
                _publish_hold(pub_gripper, GRIPPER_JOINTS, gripper_positions, min(duration, 2.0))
            _sleep_wall(period)
        rospy.loginfo(
            "Re-published hold for %.1f s at %.1f Hz after unpause.",
            float(hold_republish_seconds),
            float(hold_publish_rate_hz),
        )

    if lower_to_ground_after_unpause and float(ground_z_before_unpause) >= 0.0:
        if ground_settle_after_unpause_sec > 0.0:
            _sleep_wall(float(ground_settle_after_unpause_sec))
        _ramp_model_pose_z(
            gazebo_model_name,
            ground_z_before_unpause,
            steps=ground_lower_steps,
            duration_s=ground_lower_duration_s,
            timeout_s=subscriber_timeout,
        )


if __name__ == "__main__":
    main()
