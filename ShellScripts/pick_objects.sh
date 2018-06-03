#!/bin/sh

xterm -T "roscore" -e " source /home/workspace/catkin_ws/devel/setup.bash; roscore" &
sleep 5
xterm -T "world" -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/my_world.world" &
sleep 1
xterm -T "amcl" -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/World/my_world_map.yaml" &
sleep 1
xterm -T "rviz" -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 1
xterm -T "pickup" -e " source /home/workspace/catkin_ws/devel/setup.bash; rosrun pick_objects pick_objects"

