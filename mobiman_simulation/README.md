# mobiman

## Installation

The system is tested under
Ubuntu 20.04 + ROS Noetic

### Required packages:

* [move_base](https://github.com/ros-planning/navigation) package
* [stretch_ros](https://github.com/hello-robot/stretch_ros.git) package
* [gmapping](http://wiki.ros.org/gmapping) package

### Install Moveit 1 for Noetic:
```bash
sudo apt install ros-noetic-moveit
```

### Install teb local planner:
```
sudo apt install ros-noetic-teb-local-planner
```

### Install Gazebo Models (Optional)
```
cd ~/.gazebo/models
git clone https://github.com/osrf/gazebo_models.git
```

## How to run

### move_base demo

To run the move_base without SLAM demo:

```bash
roslaunch mobiman_simulation navigation_no_map.launch
```

To run the move_base with SLAM demo:

```bash
roslaunch mobiman_simulation navigation_with_map.launch 
```

### Moveit demo 

This demo shows motion planning using Moveit planner for the stretch arm and the moving base.

Run the Moveit Demo launch file:
```bash
roslaunch mobiman_simulation moveit_demo.launch
```

To set the target position, publish to the topic ``` /moveit_demo/gripper_goal_pose ``` using tool like ```rqt```

note that ``` z ``` should not exceeds 1.9 which is the height limit of the robot