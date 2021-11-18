# mobiman

## Installation

The system is tested under
Ubuntu 20.04 + ROS Noetic

Required packages:

* [move_base](https://github.com/ros-planning/navigation) package
* [stretch_ros](https://github.com/hello-robot/stretch_ros.git) package
* [gmapping]() package

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