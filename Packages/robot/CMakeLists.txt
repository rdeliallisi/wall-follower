cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	rospy
  	sensor_msgs
  	std_msgs
  	message_generation
)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
find_package(rostest REQUIRED)

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS} ${GTest_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(my_library src/high_level_control.cpp src/circle_detector.cpp src/util_functions.cpp)
target_link_libraries(my_library ${OpenCV_LIBRARIES})

add_message_files(
    FILES
    circle_detect_msg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

#High Level Control Node

add_executable(HighLevelControl src/high_level_control_node.cpp src/high_level_control.cpp src/util_functions.cpp)
target_link_libraries(HighLevelControl ${catkin_LIBRARIES})
add_dependencies(HighLevelControl robot_generate_messages_cpp)

add_executable(CircleDetector src/circle_detector.cpp src/circle_detector_node.cpp)
target_link_libraries ( CircleDetector ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(CircleDetector robot_generate_messages_cpp)

#Unit Tests

#add_rostest_gtest(HLCN_start_position_test launch/start_position.launch test/HLCH_start_position_test.cpp)
#target_link_libraries(HLCN_start_position_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#
#add_rostest_gtest(HLCN_wall_proximity_test launch/wall_proximity.launch test/HLCH_wall_proximity_test.cpp)
#target_link_libraries(HLCN_wall_proximity_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#
#add_rostest_gtest(CD_no_circle_test launch/no_circle.launch test/CD_no_circle_test.cpp)
#target_link_libraries(CD_no_circle_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#
#add_rostest_gtest(CD_circle_present_test_1 launch/circle_present_1.launch test/CD_circle_present_test.cpp)
#target_link_libraries(CD_circle_present_test_1 my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#
#add_rostest_gtest(CD_circle_present_test_2 launch/circle_present_2.launch test/CD_circle_present_test.cpp)
#target_link_libraries(CD_circle_present_test_2 my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


# System tests

add_rostest_gtest(ST_robot_easy_test test/ST_robot_easy.test test/ST_robot_easy_test.cpp)
target_link_libraries(ST_robot_easy_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(ST_robot_medium_test test/ST_robot_medium.test test/ST_robot_medium_test.cpp)
target_link_libraries(ST_robot_medium_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(ST_robot_hard_test test/ST_robot_hard.test test/ST_robot_hard_test.cpp)
target_link_libraries(ST_robot_hard_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


































