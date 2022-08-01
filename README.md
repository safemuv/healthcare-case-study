# Instructions

This case study should work on ROS Melodic

# Dependencies

apt-get install ros-melodic-move-base ros-melodic-turtlebot3 ros-melodic-turtlebot3-description ros-melodic-map-server ros-melodic-amcl ros-melodic-dwa-local-planner

# Installation

* Copy the directories "multiple_robots", "myWorlds", "navigation_with_objective" under the ROS hierarchy under ~/catkin_ws/src
* Copy the directory "configurationFiles" to your home directory.
* Next run "catkin_make" in "~/catkin_ws"

# Username setting

Some launch/configuration files may contain the
usernames "ubuntu" or "osboxes" which should be replaced with your username.
In particular, the ".py" files under ~/catkin_ws/src/navigation_with_objective

And the launch file multiple_robots/launch/mutli_burgersCorridor.launch
contains the path for whiteCorridorV2.world in which the username must be set

catkin_ws/src/maps/mapWhiteCorridorV2.yaml has the "image" path that has
to be set

In map_navigation_corridor_white.py here is a path in readRoomsFile that needs
the username set:

And two in map_navigation_corridor_white_autostart.py - lines 413 and 521

# Manul startup

First run "export TURTLEBOT3_MODEL=burger"

1) roslaunch multiple_robots multi_burgersCorridor.launch

2) rviz -d /home/$USER/catkin_ws/src/multiple_robots/rviz/turtlebot3_navigationDos.rviz

For each robot tb3_xxx (replace xxx with robot IDs 0, 1 and 2)

3) ROS_NAMESPACE=tb3_xxx roslaunch navigation_with_objective coordinates.launch
4) ROS_NAMESPACE=tb3_xxx roslaunch navigation_with_objective StartGoalMonitor.launch

# Automatic startup

To launch the case study automatically:
```
./scripts/auto_launch_healthcare.sh
```

To terminate it:
```
./scrits/terminate.sh
```
