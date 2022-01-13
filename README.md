# mobiman

## Installation:

## Depending ROS packages:

### pinocchio
### ocs2
### hpp-fcl
### anymal_c_simple_description
### stretch_ros

```
git clone git@github.com:RIVeR-Lab/mobiman.git
git clone --recurse-submodules git@github.com:RIVeR-Lab/pinocchio.git #'noetic-akmandor' branch*
git clone git@github.com:leggedrobotics/ocs2.git
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
git clone https://github.com/ANYbotics/anymal_c_simple_description.git
git clone https://github.com/hello-robot/stretch_ros.git
```

## Other dependencies:
```
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
```

## Examples:

1. Mobile Manipulation of Hello Stretch Using OCS2 Toolbox:
```
roslaunch ocs2_mobile_manipulator_ros mobile_manipulator_stretch.launch
```

## Notes:
1. sometimes first "catkin build" gives error! Building again compiles.