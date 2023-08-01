#! /bin/bash

x-terminal-emulator -e roslaunch turtlebot3_manipulation_gazebo rosto_manu.launch 2>/dev/null &&
sleep 5 &&

x-terminal-emulator -e roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=$HOME/map.yaml 2>/dev/null &&

sleep 5 &&

cd src/rosto_pkgs/scripts 

x-terminal-emulator -e python3 red_line_node.py 2>/dev/null 2>/dev/null &&
sleep 5 &&

cd 

x-terminal-emulator -e roslaunch turtlebot3_navigation pcl_to_lscan_final.launch 2>/dev/null &&
sleep 5 &&

x-terminal-emulator -e roslaunch ira_laser_tools laserscan_multi_merger.launch 2>/dev/null &&
sleep 5 &&

x-terminal-emulator -e roslaunch turtlebot3_manipulation_moveit_config move_group.launch 2>/dev/null &&
sleep 5 &&

x-terminal-emulator -e rosrun rosto_pkgs points_pub.py 2>/dev/null &&
sleep 10 &