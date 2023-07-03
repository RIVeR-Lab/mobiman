# mobiman
<hr>

## Gazebo:

## Installation: 

Please follow the instructions in [MANUAL_INSTALLATION](https://github.com/RIVeR-Lab/mobiman/blob/main/MANUAL_INSTALLATION).

OR

```
cd /tmp
wget https://www.dropbox.com/s/vj4gy18t8ax0bb3/mobiman_install.sh
chmod +x mobiman_install.sh
./mobiman_install.sh -d=mobiman_ws # mobiman_ws is empty in this case, don't run on existing workspace so things don't break.
```


## Run:

In seperate terminals:

1. Start simulation in Gazebo (Default arm is jaco, edit 'robot_arm' for ur5):
```
roslaunch mobiman_simulation gazebo.launch
```

2. Start the motion planning:
```
roslaunch mobiman_simulation ocs2_planner_jackal_jaco.launch
#roslaunch mobiman_simulation ocs2_planner_jackal_ur5.launch #if 'robot_arm = ur5'
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
=======
# MOBIMAN BOT CREDENTIALS
username: administrator

password: clearpath
>>>>>>> main
