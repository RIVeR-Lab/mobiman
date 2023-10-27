#!/bin/bash

# Author: Sarvesh Prajapati (SP)
#	  Neset Unver Akmandor (NUA)
# E-Mail: prajapati.s@northeastern.edu
#	  akmandor.n@northeastern.edu
# TODO:
#
# NOTES:
#

cd ..

# 3.1 Install outsourced ROS packages.

# 3.1.1 hpp-fcl
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git

# 3.1.2 ocs2_robotic_assets
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git

# 3.1.3 coveyor_demo
git clone --recurse-submodules https://github.com/rokokoo/conveyor_demo.git

# 3.1.4 gazebo_ros_link_attacher
git clone git@github.com:pal-robotics/gazebo_ros_link_attacher.git

# 3.1.5 robot_collision_checking
git clone git@github.com:mazrk7/robot_collision_checking.git

# 3.2 Install edited ROS packages (checkout to the specified branch is required).

# 3.2.1 ocs2
git clone git@github.com:RIVeR-Lab/ocs2.git
cd ocs2/
git checkout noetic-akmandor-v12
cd ..

# 3.2.2 pinocchio
git clone --recurse-submodules https://github.com/RIVeR-Lab/pinocchio.git
cd pinocchio/
git checkout noetic-akmandor
cd ..

# 3.2.3 jackal
git clone git@github.com:RIVeR-Lab/jackal.git
cd jackal/
git checkout noetic-akmandor
cd ..

# 3.2.4 universal_robot
git clone git@github.com:RIVeR-Lab/universal_robot.git
cd universal_robot/
git checkout noetic-akmandor
cd ..

# 3.2.5 geometry2
git clone git@github.com:RIVeR-Lab/geometry2.git
cd geometry2/
git checkout noetic-akmandor
cd ..

# 3.2.6 pedsim_ros
git clone git@github.com:RIVeR-Lab/pedsim_ros.git
cd pedsim_ros
git submodule update --init --recursive
git checkout noetic-akmandor
cd ..

# 3.2.7 kinova-ROS
git clone git@github.com:RIVeR-Lab/kinova-ros.git
cd kinova-ros
git checkout noetic-akmandor
cd ..

# 3.2.8 openai_ros
git clone git@github.com:RIVeR-Lab/openai_ros.git
cd openai_ros
git checkout devel-v2
cd ..

# 3.2.9 mocap_optitrack
git clone git@github.com:RIVeR-Lab/mocap_optitrack.git
cd mocap_optitrack
git checkout noetic-mobiman-v1
cd ..

# 3.3 Install other ROS dependencies:
sudo apt install ros-noetic-octomap*
sudo apt install ros-noetic-pointcloud-to-laserscan
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
sudo apt-get install nlohmann-json3-dev
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev libglpk-dev
sudo apt install ros-noetic-pybind11-catkin
sudo apt install python3-catkin-tools
sudo apt install doxygen doxygen-latex
sudo apt-get install ros-noetic-rqt-multiplot
sudo apt install ros-noetic-moveit

# 3.4 Install remaining ROS dependencies using rosdep tool:
wait
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3.4 Install Python dependencies (required for tentabot drl).
pip install stable-baselines3[extra]

# 3.5 Set catkin configuration:
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release

# 3.6 Build the catkin workspace:
#catkin build
