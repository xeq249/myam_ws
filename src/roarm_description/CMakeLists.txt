cmake_minimum_required(VERSION 3.0.2)
project(roarm_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)
catkin_package()

roslaunch_add_file_check(launch/display.launch)
roslaunch_add_file_check(launch/gazebo.launch)

foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

catkin_install_python(PROGRAMS scripts/send_arm_hold_traj.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
