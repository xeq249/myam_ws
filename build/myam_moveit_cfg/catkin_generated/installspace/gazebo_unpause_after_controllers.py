#!/usr/bin/env python3
"""
Gazebo 暂停时：controller_manager 往往无法把控制器切到 running，FollowJointTrajectory 也无法执行。

正确顺序：等 gazebo_ros_control 的 load_controller 可用 → 先 unpause → 再等控制器 running → 发保持轨迹。
"""
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTolerance,
)
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

MANIP_JOINTS = [
    "arm_base_link_to_arm_link1",
    "arm_link1_to_arm_link2",
    "arm_link2_to_arm_link3",
    "arm_link3_to_arm_link4",
    "arm_link4_to_arm_link5",
]
GRIPPER_JOINTS = ["arm_link5_to_gripper_link"]
DEFAULT_MANIP = [0.0, -1.57, -1.0, 0.5699, 0.0]
DEFAULT_GRIP = [0.0]


def _positions(js, names, defaults):
    out = []
    for i, n in enumerate(names):
        try:
            out.append(js.position[js.name.index(n)])
        except (ValueError, AttributeError):
            out.append(defaults[i])
    return out


def _send_hold(client_ns, joint_names, positions, sim_duration_sec):
    """Action 失败时退回 topic command（部分环境下容差/时间戳更挑剔）。"""
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    traj.header.stamp = rospy.Time(0)
    p = JointTrajectoryPoint()
    p.positions = positions
    p.time_from_start = rospy.Duration(sim_duration_sec)
    traj.points = [p]

    client = actionlib.SimpleActionClient(
        client_ns + "/follow_joint_trajectory", FollowJointTrajectoryAction
    )
    if client.wait_for_server(rospy.Duration(10.0)):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        # position=-1 表示不按默认紧容差判失败（spawn 后关节角可能已有偏差）
        goal.path_tolerance = [
            JointTolerance(name=n, position=-1.0) for n in joint_names
        ]
        goal.goal_tolerance = [
            JointTolerance(name=n, position=-1.0) for n in joint_names
        ]
        goal.goal_time_tolerance = rospy.Duration(2.0)
        st = client.send_goal_and_wait(
            goal, rospy.Duration(max(20.0, sim_duration_sec + 8.0))
        )
        if st == actionlib.GoalStatus.SUCCEEDED:
            return True
        rospy.logwarn(
            "%s follow_joint_trajectory ended with state %s, trying ~/command topic",
            client_ns,
            st,
        )

    pub = rospy.Publisher(client_ns + "/command", JointTrajectory, queue_size=1)
    t0 = rospy.Time.now()
    while pub.get_num_connections() < 1 and (rospy.Time.now() - t0).to_sec() < 5.0:
        rospy.sleep(0.05)
    pub.publish(traj)
    rospy.sleep(sim_duration_sec + 0.5)
    rospy.loginfo("published hold trajectory to %s/command", client_ns)
    return True


def main():
    rospy.init_node("gazebo_unpause_after_controllers")
    rospy.wait_for_service("/controller_manager/load_controller", timeout=180.0)
    rospy.wait_for_service("/gazebo/unpause_physics", timeout=180.0)
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=180.0)
    list_ctrl = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

    rospy.loginfo("gazebo_ros_control ready: unpausing so controller_spawner can switch controllers …")
    unpause()
    rospy.sleep(0.3)

    need = {"joint_state_controller", "manipulator_controller", "gripper_controller"}
    rospy.loginfo("waiting for controllers running: %s", need)
    rate = rospy.Rate(30)
    deadline = rospy.Time.now() + rospy.Duration(90.0)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        running = {c.name for c in list_ctrl().controller if c.state == "running"}
        if need.issubset(running):
            break
        rate.sleep()
    else:
        rospy.logerr("timeout waiting for controllers to reach running; arm may droop")

    try:
        js = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
        manip_q = _positions(js, MANIP_JOINTS, DEFAULT_MANIP)
        grip_q = _positions(js, GRIPPER_JOINTS, DEFAULT_GRIP)
    except rospy.ROSException:
        rospy.logwarn("no /joint_states, using default spawn angles")
        manip_q, grip_q = list(DEFAULT_MANIP), list(DEFAULT_GRIP)

    rospy.loginfo("sending hold trajectories at spawn pose …")
    # 用仿真时间尺度：略长一点便于 PID 跟上
    if not _send_hold("/manipulator_controller", MANIP_JOINTS, manip_q, 1.5):
        rospy.logwarn("manipulator hold trajectory failed")
    if not _send_hold("/gripper_controller", GRIPPER_JOINTS, grip_q, 1.0):
        rospy.logwarn("gripper hold trajectory failed")
    rospy.loginfo("gazebo arm hold done (physics already running)")


if __name__ == "__main__":
    main()
