# mobiman
<hr>

## Gazebo:

## Installation: 
```
git clone git@github.com:RIVeR-Lab/mobiman.git
cd mobiman
./mobiman_install.sh
```

## Run (Manual mode):

0. Make sure that 'drlFlag' is set false in [task config file](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/config/task/task_jackal_jaco.info).

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
