# MIMRee Mission Planner Executor

This package is part of the MIMRee project (https://github.com/EEEManchester/MIMRee) that represents assets to work under ROS framework.

## Requirements :
1. ROS.
2. Gazebo version 9.x or greater.
4. laser_geometry (http://wiki.ros.org/laser_geometry)

## HELP

How to Launch (in conjunction with https://github.com/EEEManchester/MIMRee MIMRee simulator):

There is a launch file  
```
roslaunch mimree_description mangalia.launch repair_mission:=[true | false] model_path:=[PATH_TO_MIMREE_PROJECT_FOLDER]
```
that can be run to replace
```
gazebo --verbose worlds/mimree_mangalia.world
```
from MIMRee project repository (https://github.com/EEEManchester/MIMRee) with `repair_mission:=true` to set UAV only, `repair_mission:=false` to set UAV and ASV in the simulation, and `model_path:=[PATH_TO_MIMREE_PROJECT_FOLDER]` needing to point to the MIMRee project folder.
