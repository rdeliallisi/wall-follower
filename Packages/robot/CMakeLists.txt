cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)



find_package( OpenCV REQUIRED)
find_package(GTest REQUIRED)
find_package(rostest REQUIRED)

catkin_package()

# Build Circle Detector
include_directories(include ${catkin_INCLUDE_DIRS} ${GTest_INCLUDE_DIRS})
add_library(my_library src/circle_detector_node.cpp)


add_executable(circle_detector_node src/circle_detector_node.cpp)
target_link_libraries ( circle_detector_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(circle_detector_node robot_generate_messages_cpp)

# Unit Tests

target_link_libraries(circle_detector_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

