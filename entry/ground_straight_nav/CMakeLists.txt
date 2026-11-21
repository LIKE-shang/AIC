cmake_minimum_required(VERSION 3.0.2)
project(ground_straight_nav)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs std_srvs geometry_msgs nav_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy std_msgs std_srvs geometry_msgs nav_msgs)

catkin_install_python(PROGRAMS
  scripts/straight_nav_node.py
  tools/check_waypoints.py
  tools/axis_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_straight_nav_core.py)
  catkin_add_nosetests(test/test_node_boundary.py)
  catkin_add_nosetests(test/test_waypoint_check.py)
  catkin_add_nosetests(test/test_axis_check.py)
endif()
