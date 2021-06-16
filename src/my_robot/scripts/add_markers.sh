#!/bin/sh
xterm  -e  " roslaunch my_robot world_markers.launch" &
sleep 5
xterm  -e  " roslaunch my_robot amcl.launch" & 
sleep 5
xterm  -e  " rosrun add_markers add_markers" 

