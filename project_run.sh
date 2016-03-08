catkin_make
source devel/setup.bash
roscore &
wait(5)
roslaunch lab_simulator simulator.launch &
rosrun robot HighLevelControlNode &
rosrun robot CircleDetector
