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

<hr>

## Isaac Sim:

1. Download and execute (install) Omniverse from here -- [NVIDIA's Omniverse Installation Page](https://www.nvidia.com/en-us/omniverse/download/)

2. Once Installed open Omniverse and install Isaac Sim and Cache from the `Exchange` tab.
![Installation Image](https://i.ibb.co/DY38vVJ/image.png)
3. Click on `NUCLEUS` tab in Isaac Sim and set up the NEUCLEUS server by clicking on `Add Local Nucleus Service`.
![Nucleus_Init](https://i.ibb.co/1s9H3P6/P1.jpg)
4. Select the appropriate data path (`/home` is just an example).
![Data Path](https://i.ibb.co/qxdbvfj/P2.jpg)
5. Fill out the final bit and `Complete the Setup`.
![Config](https://i.ibb.co/SnqqtnS/P3.jpg)
<!-- <<<<<<< HEAD -->
6. Once Isaac Sim is up and running, open the `jaco_jackal_warehouse.usd` (this will take some time) located in `~/mobiman_ws/src/mobiman_simulation/models/usd/scenes` in Isaac.
![Open Scene](https://i.ibb.co/yFpV49F/image.png)
7. In a new terminal run `roscore` and check connectivity with kinova_arm and jackal with ros by running the following commands:
<!-- ======= -->

8. Once Isaac Sim is up and running, open the `jaco_jackal_warehouse.usd` located in `~/mobiman_ws/src/mobiman_simulation/models/usd/scenes` in Isaac.
![Open Scene](https://i.ibb.co/yFpV49F/image.png)

9. In a new terminal run `roscore` and check connectivity with kinova_arm and jackal with ros by running the following commands:

<!-- >>>>>>> f7f68994aad70905a9e4d69a640e054643323497 -->
```
rosrun mobiman_simulation kinova_check_bridge.py
rosrun mobiman_simulation jackal_check_bridge.py

```
<!-- ![Kinnova](https://i.ibb.co/BfK5QfD/kinnova.gif) -->
<img src="https://i.ibb.co/BfK5QfD/kinnova.gif" alt="Kinova" width="600" height="400">

<!-- ![Jackal](https://i.ibb.co/994BChj/jackal.gif) -->
<img src="https://i.ibb.co/994BChj/jackal.gif" alt="Jackal" width="600" height="400">

<hr>

# Network Setup

- In `~/.bashrc`, set the ROS_IP and ROS_MASTER_URI
```
# END OF .bashrc
# ROS_IP
export ROS_IP=0.0.0.0
# Before setting ROS_MASTER_URI check the jackal's IP using network scan and change it here.
export ROS_HOSTNAME= #IP OF COMPUTER
export ROS_MASTER_URI=http://192.168.0.102:11311
```

- In the `/etc/hosts` (edit with sudo) file before IPv6 add the IP for jackal
```
# SOME IP Definitions above
# Add jackal's IP
192.168.0.102	jackal

# The following lines are desirable for IPv6 capable hosts
# Some IPv6 defs
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

# LOCALIZATION WITH APRIL TAGS
#Installations
INSTALL THE ROS DRIVER FOR REALSENSE FROM HERE	
https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

INSTALL apriltag_ros package from here :
https://github.com/AprilRobotics/apriltag_ros

# INSTRUCTIONS AND PROCEDURES

First you need to get the serial numbers of the two cameras. Connect the two cameras individually and run the following for both separately :
```
roslaunch mobiman_simulation rs_camera.launch
```
Note the serial numbers and use them as arguments.
For first camera, add the serial number in rs_camera_cam1.launch like so :
```
<arg name="serial_no"           default="044422250566"/>
```
Similarly for the second camera, add the serial number in rs_camera_cam2.launch

```
<arg name="serial_no"           default="133522250294"/>
```

In a new terminal do : 
```
roslaunch mobiman_simulation tag_localization_dual_camera.launch
```


Notes : Add the tags you are expecting to detect along with their size in the `apriltags.yaml` file. 

## Procedure regarding the world_frame_broadcaster.py
For, the world_frame_broadcaster.py, here is the procedure before you launch the system :
1. Designate a tag as the world frame. Tag 0 if possible.
2. Have camera 2 look at it and broadcast it. 
For this just connect one of the cameras to the pc and do the following :
```
roslaunch mobiman_simulation rs_camera.launch
```
```
roslaunch mobiman_simulation tag_continuous_detection.launch
```

3. In a new terminal, echo the transform.
```
rostopic echo /tf
```
Note the transform of the tag w.r.t. camera_color_optical_frame. It should be fairly constant if the camera is not undergoing any vibrations.

4. Put those transform values (x,y,z) and (x,y,z,w) in the `world_frame_broadcaster.py`. Here the `cam2_color_optical_frame` would be the parent and `world` would be the child.
<!-- ======= -->
# MOBIMAN BOT CREDENTIALS
username: administrator

password: clearpath
<!-- >>>>>>> main -->
