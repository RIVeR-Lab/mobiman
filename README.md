# mobiman

## Installation

The system is tested under
Ubuntu 20.04 + ROS Noetic

### Required packages:

* [move_base](https://github.com/ros-planning/navigation) package
* [stretch_ros](https://github.com/hello-robot/stretch_ros.git) package
* [gmapping]() package

### Install Moveit 1 for Noetic:
```bash
sudo apt install ros-noetic-moveit
```

## How to run

### move_base demo

To run the move_base without SLAM demo:

```bash
roslaunch mobiman navigation_no_map.launch
```

To run the move_base with SLAM demo:

```bash
roslaunch mobiman navigation_with_map.launch 
```

Moveit demo shows motion planning using Moveit planner for the stretch arm.

To run the Moveit Demo:
```bash
roslaunch mobiman moveit_demo.launch
```