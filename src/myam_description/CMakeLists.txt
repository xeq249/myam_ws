cmake_minimum_required(VERSION 2.8.3)

project(myam_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf rviz xacro worlds)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS scripts/arm_gripper_hold_home.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
