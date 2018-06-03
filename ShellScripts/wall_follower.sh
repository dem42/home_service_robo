#!/bin/sh

xterm -T "roscore" -e " source /home/workspace/catkin_ws/devel/setup.bash; roscore" &
sleep 5
xterm -T "world" -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/my_world.world" &
sleep 1
xterm -T "gmap" -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=/home/workspace/catkin_ws/src/ShellScripts/gmapping_custom.xml" &
sleep 1
xterm -T "rviz" -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 1
xterm -T "teleop" -e " source /home/workspace/catkin_ws/devel/setup.bash; rosrun wall_follower wall_follower"

