# mobiman
<hr>

## Gazebo:

## Installation: 

Please follow the instructions in [MANUAL_INSTALLATION](https://github.com/RIVeR-Lab/mobiman/blob/main/MANUAL_INSTALLATION).

## Run:

In seperate terminals:

1. Start simulation in Gazebo:
```
roslaunch mobiman_simulation gazebo.launch
```

2. Start the mapping server:
```
roslaunch mobiman_simulation map_server.launch
```

3. Start the motion planning:
```
roslaunch mobiman_simulation ocs2_planner_jackal_ur5.launch
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
6. Once Isaac Sim is up and running, open the `jackal_kinova.usd` located in `~/mobiman_ws/src/mobiman_simulation/models/usd/scenes` in Isaac.
![Open Scene](https://i.ibb.co/yFpV49F/image.png)
7. In a new terminal run `roscore` and check connectivity with kinova_arm and jackal with ros by running the following commands:
<!-- ======= -->

8. Once Isaac Sim is up and running, open the `jackal_kinova.usd` located in `~/mobiman_ws/src/mobiman_simulation/models/usd/scenes` in Isaac.
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
