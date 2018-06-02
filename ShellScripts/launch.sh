#!/bin/bash

xterm -e " gazebo" &
sleep 2
xterm -e " rosrun rviz rviz"
