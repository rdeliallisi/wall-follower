cmake_minimum_required(VERSION 2.8.3)
project(robot)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

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

add_library(my_library src/high_level_control.cpp src/circle_detector.cpp src/util_functions.cpp src/logger.cpp)
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

# Nodes

add_executable(HighLevelControl src/high_level_control_node.cpp src/high_level_control.cpp src/util_functions.cpp src/logger.cpp)
target_link_libraries(HighLevelControl ${catkin_LIBRARIES})
add_dependencies(HighLevelControl robot_generate_messages_cpp)

add_executable(CircleDetector src/circle_detector.cpp src/circle_detector_node.cpp src/logger.cpp)
target_link_libraries ( CircleDetector ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(CircleDetector robot_generate_messages_cpp)

#Unit tests

add_rostest_gtest(HLC_unit_test test/HLC_unit_test.test test/HLC_unit_test.cpp)
target_link_libraries(HLC_unit_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(HLC_unit_test_real test/HLC_unit_test_real.test test/HLC_unit_test_real.cpp)
target_link_libraries(HLC_unit_test_real my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(HLC_ros_test test/HLC_ros_test.test test/HLC_ros_test.cpp)
target_link_libraries(HLC_ros_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

catkin_add_gtest(ROBOT_util_functions test/ROBOT_util_functions_test.cpp)
target_link_libraries(ROBOT_util_functions my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(CD_utils_test test/CD_utils_test.test test/CD_utils_test.cpp)
target_link_libraries(CD_utils_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(CD_utils_test_real test/CD_utils_test_real.test test/CD_utils_test_real.cpp)
target_link_libraries(CD_utils_test_real my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


#Integration Tests
add_rostest_gtest(IT_circle_hit_test test/IT_circle_hit_test.test test/IT_circle_hit_test.cpp)
target_link_libraries(IT_circle_hit_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(IT_wall_follow_test test/IT_wall_follow_test.test test/IT_wall_follow_test.cpp)
target_link_libraries(IT_wall_follow_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(IT_loop_exit_test test/IT_loop_exit_test.test test/IT_loop_exit_test.cpp)
target_link_libraries(IT_loop_exit_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

# System Tests
add_rostest_gtest(ST_robot_easy_test test/ST_robot_easy.test test/ST_robot_easy_test.cpp)
target_link_libraries(ST_robot_easy_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(ST_robot_medium_test test/ST_robot_medium.test test/ST_robot_medium_test.cpp)
target_link_libraries(ST_robot_medium_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(ST_robot_hard_test test/ST_robot_hard.test test/ST_robot_hard_test.cpp)
target_link_libraries(ST_robot_hard_test my_library ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
