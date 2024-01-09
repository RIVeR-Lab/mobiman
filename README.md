# mobiman
<hr>

## Installation: 
1. Clone the mobiman repository into the src folder of catkin workspace:
```
git clone git@github.com:RIVeR-Lab/mobiman.git
cd mobiman
./install_mobiman.sh
```

2. Install ROS dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```
## Run (Manual mode):
1. Set configurations in [mobiman_framework.launch](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/launch/mobiman_framework.launch)
     1. Set parameter "config_mobiman_framework" to
        - "config_mobiman_framework_gazebo.yaml" for Gazebo
        - "config_mobiman_framework_igibson.yaml" for iGibson

2. Launch mobiman framework:
```
roslaunch mobiman_simulation mobiman_framework.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
```

Note: grep command added to avoid stream of warnings as depicted in [here](https://github.com/ms-iot/ROSOnWindows/issues/279).

## Run (DRL mode):
1. Set configurations in [config_mobiman_framework](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/config/config_mobiman_framework.yaml)
  - Set parameter "sim: gazebo" for simulation in Gazebo
  - Set parameter "sim: igibson" for simulation in iGibson

2. Launch mobiman framework from the ROS workspace:
```
# Do not forget to source: source devel/setup.bash
roslaunch mobiman_simulation mobiman_framework.launch
```

3. Run the training script (python) from the igibson ros scripts folder (such as /home/akmandor/projects/iGibson/igibson/examples/ros/igibson-ros/scripts):
```
# Do not forget to source: source ~/ros_workspaces/igibson_ws/devel/setup.bash
python stable_baselines3_ros_jackalJaco.py
```

## !!! BELOW IS DEPRECATED: 

## Run (Manual mode in iGibson):

0. Make sure that 'drlFlag' is set false in [task config file](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/config/task/task_jackal_jaco_igibson.info).

In seperate terminals:

1. Start simulation in iGibson:
```
roslaunch igibson-ros mobiman_jackal_jaco.launch
```

2. Start the map server:
```
roslaunch mobiman_simulation map_server.launch
```

3. Start the target manager:
```
roslaunch mobiman_simulation ocs2_target.launch
```

4. Start the motion planning:
```
roslaunch mobiman_simulation ocs2_m4.launch
```


## Run (Manual mode):

0. Make sure that 'drlFlag' is set false in [task config file](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/config/task/task_jackal_jaco_gazebo.info).

In seperate terminals:

1. Start simulation in Gazebo:
```
roslaunch mobiman_simulation gazebo.launch
```

2. Start the target manager:
```
roslaunch mobiman_simulation ocs2_target.launch
```

3. Start the distance server:
```
roslaunch mobiman_simulation distance_server.launch
```

4. Start the motion planning:
```
roslaunch mobiman_simulation ocs2_m4.launch
```

## Run (DRL mode):

0. Make sure that 'drlFlag' is set true in [task config file](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/config/task/task_jackal_jaco.info).

In seperate terminals:

1. Start simulation in Gazebo:
```
roslaunch mobiman_simulation drl.launch
```

2. Start the target manager:
```
roslaunch mobiman_simulation ocs2_target.launch
```

3. Start the distance server:
```
roslaunch mobiman_simulation distance_server.launch
```

4. 1. Start the motion planning:
```
roslaunch mobiman_simulation ocs2_m4.launch
```

4. 2. Wait until you see the following message on the terminal screen:
```
[MobileManipulatorInterface::runMPC] START DRL TRAINING!!!
```

5. Start the DRL training:
```
roslaunch mobiman_simulation mobiman_drl_training.launch
```

<hr/>

# Motion Capture Setup


```
# The required installations will be added in installation script.
# Navigate to your mobiman_ws directory (~/mobiman_ws in my case)
sudo apt-get install ros-noetic-vrpn-client-ros
cd ~/mobiman_ws/src
git clone git@github.com:RIVeR-Lab/mocap_optitrack.git
```
<hr>

# Credentials of Clearpath Jackal
username: administrator

password: clearpath
<!-- >>>>>>> main -->
