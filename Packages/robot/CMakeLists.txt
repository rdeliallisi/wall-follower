cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(HighLevelControlNode src/high_level_control_node.cpp src/high_level_control.cpp)
target_link_libraries(HighLevelControlNode ${catkin_LIBRARIES})
add_dependencies(HighLevelControlNode robot_generate_messages_cpp)