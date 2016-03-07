cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

find_package(GTest REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${GTest_INCLUDE_DIRS})

#High Level Control Node
add_executable(HighLevelControlNode src/high_level_control_node.cpp src/high_level_control.cpp)
target_link_libraries(HighLevelControlNode ${catkin_LIBRARIES})
add_dependencies(HighLevelControlNode robot_generate_messages_cpp)

#Unit Tests
catkin_add_gtest(HLCN_test src/test/high_level_control_node_test.cpp)
target_link_libraries(HLCN_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
