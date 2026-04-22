cmake_minimum_required(VERSION 3.1.3)
project(myam_sys_moveit)

find_package(catkin REQUIRED)

catkin_package()

# 将关键 config 同步到 devel/share，避免 tools / 旧 launch 仍读到向导生成的 KDL、default_velocity_scaling_factor=0.1
if(DEFINED CATKIN_DEVEL_PREFIX AND CATKIN_DEVEL_PREFIX)
  set(_mcfg "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/config")
  file(MAKE_DIRECTORY "${_mcfg}")
  foreach(_myaml kinematics.yaml joint_limits.yaml cartesian_limits.yaml myam_sys.srdf)
    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/config/${_myaml}"
      "${_mcfg}/${_myaml}"
      COPYONLY)
  endforeach()
endif()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/send_arm_hold_traj.py
  scripts/offboard_arm_apriltag_grasp.py
  scripts/gazebo_red_box_grasp.py
  scripts/red_grasp_cube_rviz_sync.py
  scripts/merge_arm_rotor_joint_states.py
  scripts/relax_scene_collision_matrix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS
  scripts/start_px4_sitl_gazebo.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})