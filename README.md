# Re4MPC: Reactive Nonlinear MPC for Multi-model Motion Planning via Deep Reinforcement Learning

## 1. Citation
```
@inproceedings{akmandor2025re4mpc,
  title={Re4MPC: Reactive Nonlinear MPC for Multi-model Motion Planning via Deep Reinforcement Learning},
  author={Akmandor, Ne{\c{s}}et {\"U}nver and Prajapati, Sarvesh and Zolotas, Mark and Padir, Ta{\c{s}}kin},
  booktitle={2025 IEEE International Conference on Automation Science and Engineering (CASE)},
  year={2025},
  organization={IEEE}
}
```

## 2. Related Papers
* Akmandor, N.{\"U}., Prajapati, S., Zolotas, M. and Padır, T., 2025. **Re4MPC: Reactive Nonlinear MPC for Multi-model Motion Planning via Deep Reinforcement Learning**. arXiv preprint [arXiv:2506.08344](https://arxiv.org/abs/2506.08344).

## 3. Videos
* [CASE 2025 - supplementary video](https://youtu.be/4fj2EcKU6_Y)
* [CASE 2025 - presentation](https://youtu.be/3rbrT8R34bY)

## 4. Installation
### 4.1 Clone [libccd](https://github.com/danfis/libccd) into any directory (e.g. Home) and install libccd library from its source:
```
git clone https://github.com/danfis/libccd.git
cd libccd
mkdir build && cd build
cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
sudo make -j4
sudo make install
```
### 4.2 Clone [fcl](https://github.com/flexible-collision-library/fcl) into any directory (e.g. Home)  and install fcl library from its source:
```
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
mkdir build && cd build
cmake ..
sudo make -j4
sudo make install
```
### 4.3 Clone the mobiman repository into the src folder of catkin workspace and install all required ROS packages:
```
git clone git@github.com:RIVeR-Lab/mobiman.git
cd mobiman
./install_mobiman.sh
```
### 4.4 Install ROS dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```
### 4.5 Install iGibson simulator and its library by following the instructions in [igibson README](https://github.com/RIVeR-Lab/iGibson?tab=readme-ov-file#igibson-installation).

## 5. Run
### 5.1 Manual mode
#### 5.1 Set configurations in [mobiman_framework.launch](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/launch/mobiman_framework.launch)
Set parameter "config_mobiman_framework":
- iGibson example: "config_mobiman_framework_igibson_manual"

#### 5.2. In seperate terminals in ROS workspace:

##### 5.2.1 Launch mobiman framework:
```
# Do not forget to source: source devel/setup.bash
roslaunch mobiman_simulation mobiman_framework.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
```

##### 5.2.2 Run the (python) simulation script:
```
# Do not forget to source: source devel/setup.bash
python src/igibson-ros/scripts/mobiman_jackalJaco.py
```

###### Note: grep command added to avoid stream of warnings as depicted in [here](https://github.com/ms-iot/ROSOnWindows/issues/279).

### 5.2 DRL mode
#### 5.2.1 Set configurations in [mobiman_framework.launch](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/launch/mobiman_framework.launch)
##### 5.2.1.1 Set parameter "config_mobiman_framework":
        - iGibson example: "config_mobiman_framework_igibson_drl"

##### 5.2.1.2 Set drl configurations in [config_mobiman_drl_pick](https://github.com/RIVeR-Lab/mobiman/blob/main/mobiman_simulation/config/drl/config_mobiman_drl_pick.yaml)

#### 5.2.2 In seperate terminals in ROS workspace:

##### 5.2.2.1 Launch mobiman framework:
```
# Do not forget to source: source devel/setup.bash
roslaunch mobiman_simulation mobiman_framework.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
```

##### 5.2.2.3 Run the (python) training/testing script:
Training:
```
# Do not forget to source: source devel/setup.bash
python src/igibson-ros/scripts/drl_training_sb3_mobiman_jackalJaco.py
```
Testing:
```
# Do not forget to source: source devel/setup.bash
python src/igibson-ros/scripts/drl_testing_sb3_mobiman_jackalJaco.py
```

## 6. Dataset
[Trained models to reach static object on a conveyor belt](https://drive.google.com/drive/folders/19tXL5Gwg31mPUzghPzFzy-vLkjceiJAt?usp=sharing)

## 7. Code Contributors
Neset Unver Akmandor | akmandor.n@northeastern.edu \
Sarvesh Prajapati | prajapati.s@northeastern.edu

## 8. Credentials
mobiman was developed at the [RIVeR Lab, Northeastern University](http://robot.neu.edu/).
