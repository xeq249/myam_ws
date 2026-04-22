cmake_minimum_required(VERSION 3.0.2)
project(myam_simulation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/start_px4_myam_sys_d435_sitl.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/offboard_detect_apriltag.py
  scripts/offboard_grasp_blue_cube.py
  scripts/offboard_grasp.py
  scripts/grasp_blue_cube_rviz_sync.py
  scripts/gazebo_truth_body_tf.py
  scripts/gazebo_fake_blue_attach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
