cmake_minimum_required(VERSION 3.0.2)
project(go2_framework_sim_v3)
find_package(catkin REQUIRED)
catkin_package()
# rosrun resolves the executable in the source tree. Install modules adjacent to it.
install(PROGRAMS ros_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES core.py simulation.py geometry.py acoustic_task.py heading_planner.py runtime.py config.json DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
