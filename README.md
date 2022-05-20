# Instructions

This case study should work on ROS Melodic

# Installation

* Copy the directories "multiple_robots", "navigation_with_objective" under the ROS hierarchy under ~/catkin_ws/src
* Copy the directory "configurationFiles" to your home directory.
* Next run "catkin_make" in "~/catkin_ws"

Some launch/configuration files may contain the
usernames "ubuntu" or "osboxes" which should be replaced with your username.

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