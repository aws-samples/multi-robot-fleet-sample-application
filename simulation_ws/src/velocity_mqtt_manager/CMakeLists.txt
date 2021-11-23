cmake_minimum_required(VERSION 3.0.2)
project(velocity_mqtt_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  rospy
)

catkin_package(
)

catkin_install_python(PROGRAMS
  scripts/mqtt_connector.py
  scripts/velocity_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch rviz config certs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


