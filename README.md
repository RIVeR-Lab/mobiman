# mobiman
<hr>

## Installation: 
1. Clone [libccd](https://github.com/danfis/libccd) into any directory (e.g. Home) and install libccd library from its source:
```
git clone https://github.com/danfis/libccd.git
cd libccd
mkdir build && cd build
cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
sudo make -j4
sudo make install
```
2. Clone [fcl](https://github.com/flexible-collision-library/fcl) into any directory (e.g. Home)  and install fcl library from its source:
```
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
mkdir build && cd build
cmake ..
sudo make -j4
sudo make install
```
3. Clone the mobiman repository into the src folder of catkin workspace and install all required ROS packages:
```
git clone git@github.com:RIVeR-Lab/mobiman.git
cd mobiman
./install_mobiman.sh
```
4. Install ROS dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```
5. Install iGibson simulator and its library by following the instructions in [igibson README](https://github.com/RIVeR-Lab/iGibson?tab=readme-ov-file#igibson-installation).

## Run (Manual mode):
1. Set configurations in [mobiman_framework.launch](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/launch/mobiman_framework.launch)
     1. Set parameter "config_mobiman_framework":
        - iGibson example: "config_mobiman_framework_igibson_manual"

2. In seperate terminals in ROS workspace:

     1. Launch mobiman framework:
     ```
     # Do not forget to source: source devel/setup.bash
     roslaunch mobiman_simulation mobiman_framework.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
     ```

     2. Run the (python) simulation script:
     ```
     # Do not forget to source: source devel/setup.bash
     python src/igibson-ros/scripts/mobiman_jackalJaco.py
     ```

Note: grep command added to avoid stream of warnings as depicted in [here](https://github.com/ms-iot/ROSOnWindows/issues/279).

## Run (DRL mode):
1. Set configurations in [mobiman_framework.launch](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/launch/mobiman_framework.launch)
     1. Set parameter "config_mobiman_framework":
        - iGibson example: "config_mobiman_framework_igibson_drl"

2. Set drl configurations in [config_mobiman_drl_pick](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/config/drl/config_mobiman_drl_pick.yaml)

3. In seperate terminals in ROS workspace:

     1. Launch mobiman framework:
     ```
     # Do not forget to source: source devel/setup.bash
     roslaunch mobiman_simulation mobiman_framework.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
     ```

     2. Run the (python) training/testing script:
          1. Training:
          ```
          # Do not forget to source: source devel/setup.bash
          python src/igibson-ros/scripts/drl_training_sb3_mobiman_jackalJaco.py
          ```
          2. Testing:
          ```
          # Do not forget to source: source devel/setup.bash
          python src/igibson-ros/scripts/drl_testing_sb3_mobiman_jackalJaco.py
          ```
