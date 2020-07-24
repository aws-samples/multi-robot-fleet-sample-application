cmake_minimum_required(VERSION 2.8.3)
project(robot_fleet)

# c++11 if using kinetic
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(move_object_plugin src/move_object.cc)
target_link_libraries(move_object_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

catkin_install_python(PROGRAMS
  scripts/client_rosbridge.py
  scripts/gazebo_model_mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch static_models_w_plugin rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS move_object_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
