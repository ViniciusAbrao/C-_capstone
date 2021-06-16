#!/bin/sh
xterm  -e  " roslaunch my_robot world.launch" &
sleep 5
xterm  -e  " roslaunch my_robot teleop.launch" & 
sleep 5
xterm  -e  " roslaunch my_robot my_gmapping_launch.launch"  
