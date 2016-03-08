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
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS} ${GTest_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_library(my_library src/high_level_control.cpp src/circle_detector.cpp)
target_link_libraries(my_library ${OpenCV_LIBRARIES})

catkin_package()

#High Level Control Node

add_executable(HighLevelControlNode src/high_level_control_node.cpp src/high_level_control.cpp)
target_link_libraries(HighLevelControlNode ${catkin_LIBRARIES})
add_dependencies(HighLevelControlNode robot_generate_messages_cpp)

add_executable(CircleDetector src/circle_detector.cpp src/circle_detector_node.cpp)
target_link_libraries ( CircleDetector ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(CircleDetector robot_generate_messages_cpp)

#Unit Tests

add_rostest_gtest(HLCN_start_position_test launch/start_position.launch src/test/HLCH_start_position_test.cpp)
target_link_libraries(HLCN_start_position_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(HLCN_wall_proximity_test launch/wall_proximity.launch src/test/HLCH_wall_proximity_test.cpp)
target_link_libraries(HLCN_wall_proximity_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(CD_no_circle_test launch/start_position.launch src/test/CD_no_circle_test.cpp)
target_link_libraries(CD_no_circle_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(CD_circle_present_test launch/circle_detect.launch src/test/CD_circle_present_test.cpp)
target_link_libraries(CD_circle_present_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(CD_circle_present_test2 launch/circle_detect2.launch src/test/CD_circle_present_test.cpp)
target_link_libraries(CD_circle_present_test2 my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})