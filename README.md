# Background: 

Mobile robots have become indispensable in industry, especially in tasks that pose a threat to humans’ lives, this is due to the advancements in the building blocks of the mobile robots’ navigation namely: perception, mapping, localization, cognition, path planning, and motion control [1]. The project addresses the programming of a mobile robot to pick and place hazardous containers from and to specific points (navigating) through specific lanes. In order to achieve this goal, some technical requirements have to be fulfilled by the robot:
1. Onboard LIDAR
2. Vision system (rgb camer)
3. Differential drive
4. Manipulator

# Objective: 

1.	Provide continuous localization of the Turtlebot3
2.	Navigate from a starting to a designated point in the shortest time
3.	Avoid crossing the red lines
4.	Update the map with obstacles
5.	Handle dynamic obstacle
  
# Proposed Method(s): 

The following steps will be taken in order to develop a preplanned path approach 
•	Navigation: The robot will be guided to the pickup and drop-off locations using intermediate goals to guide it without crossing the red lines. 
o	Global planner: Dijkstra planner will be used from the NavfnROS package, and it will be given the intermediate goals to plan the path for the TurtleBot and replan whenever needed.
o	Local planner: A custom approach will be implemented where the TurtleBot will avoid the obstacles in addition to taking into consideration avoiding crossing the red lines. 
•	Localization: The TurtleBot will be able to localize itself in the pre-given map using Adaptive Monte Carlo Localization (AMCL).
•	Control: The TurtleBot will receive its speed messages from the move base package interpreting the path given by the local planner.
•	Manipulation: After reaching the pickup position the TurtleBot will adjust its orientation using its vision by centering the container pixels in the camera frame, then executing a pickup script using the open manipulator package.

# Please follow the following steps carefully to run the project

1) Move the "map.pgm" and "map.yaml" from the "groupA" folder to the home directory
2) In "map.yaml" file change the image path to : /home/{your device name}/map.pgm
3) open terminal and run the following lines:
  $ cd groupA
  $ rosdep install --from-paths src --ignore-src -r -y
  $ sudo apt-get install ros-noetic-pointcloud-to-laserscan
  $ sudo apt-get install ros-noetic-dwa-local-planner
  $ catkin_make
  $ cd
  $ gedit .bashrc
4) Copy the following lines at the end of the bashrc:
   source /opt/ros/noetic/setup.bash
   export TURTLEBOT3_MODEL=waffle_pi
   source ~/groupA/devel/setup.bash
5) open terminal and run the following lines:
   $ cd groupA
   $ ./rosto_sim.sh
#   Ps: Make sure to run the simulation in Gazebo when it launches 

