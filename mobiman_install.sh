#!/bin/bash
# Parse Arguments
OPTIND=1
if [[ $# -eq 0 ]] ; then
    printf "Execute \'./mobiman_install.sh -h\' for more information\n"
    exit 0
fi
for i in "$@"; do
  case $i in
    -d=*|--directory=*)
      directory="${i#*=}"
      shift # past argument=value
      ;;
    -h|--help)
      printf "In order to execute this script\n./mobiman_install.sh -d=installation_dir\n-d is required argument.\n"
      shift # past argument with no value
      ;;
    -*|--*)
      printf "Unknown option $i\nExecute \'./mobiman_install.sh -h\' for more information\n"
      exit 1
      ;;
    *)
      ;;
  esac
done
echo $directory
# Create the WorkSpace
eval directory=$directory
echo $directory
mkdir -p $directory/src
cd $directory/src
### PROTOBUF VERSION CHECK AND INSTALL -- START
proto=$(protoc --version)
if [ "$proto" = "libprotoc 3.6.1" ]; then
  echo "[+] Protoc Version Satisfied"
else
  echo "[+] Installing Protoc 3.6.1"
  wget https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-all-3.6.1.tar.gz
  tar -xf protobuf-all-3.6.1.tar.gz
  cd protobuf-3.6.1
  ./configure
  make -j4
  sudo make install
  sudo ldconfig
  cd ..
  rm -rf protobuf-3.6.1
  rm -rf protobuf-all-3.6.1.tar.gz
fi
proto=$(protoc --version)
if [ "$proto" != "libprotoc 3.6.1" ]; then
  echo "[-] Protoc Installation Failed, please install protoc version 3.6.1 manually!"
fi
### PROTOBUF VERSION CHECK AND INSTALL -- END
# ### Clone mobiman
git clone git@github.com:RIVeR-Lab/mobiman.git

### Clone ocs2
git clone git@github.com:RIVeR-Lab/ocs2.git
cd ocs2/
git checkout noetic-akmandor-v10
cd ..

### Clone pinocchio
git clone --recurse-submodules https://github.com/RIVeR-Lab/pinocchio.git
cd pinocchio/
git checkout noetic-akmandor
cd ..

### Clone hpp-fcl
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git

### Clone ocs2_robotic_assets
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git

### Clone jackal
git clone git@github.com:RIVeR-Lab/jackal.git
cd jackal/
git checkout noetic-akmandor
cd ..

### Clone universal_robot
git clone git@github.com:RIVeR-Lab/universal_robot.git
cd universal_robot/
git checkout noetic-akmandor
cd ..

### Clone geometry2
git clone git@github.com:RIVeR-Lab/geometry2.git
cd geometry2/
git checkout noetic-akmandor
cd ..

### Clone pedsim_ros
git clone git@github.com:RIVeR-Lab/pedsim_ros.git
cd pedsim_ros
git submodule update --init --recursive
git checkout noetic-akmandor
cd ..

### Clone openai-ros
git clone git@github.com:RIVeR-Lab/openai_ros.git
cd openai_ros
git checkout devel_v0
cd ..

### Clone Kinova-ROS
git clone git@github.com:RIVeR-Lab/kinova-ros.git
cd kinova-ros
git checkout noetic-akmandor
cd ..

### Clone coveyor_demo
git clone --recurse-submodules https://github.com/rokokoo/conveyor_demo.git
### Clone Optitrack
git clone git@github.com:RIVeR-Lab/mocap_optitrack.git
cd mocap_optitrack
git checkout noetic-mobiman-v1
cd ..
### Clone gazebo_ros_link_attacher
git clone git@github.com:pal-robotics/gazebo_ros_link_attacher.git
cd ..

sudo apt install ros-noetic-octomap*
sudo apt install ros-noetic-pointcloud-to-laserscan
sudo apt-get install nlohmann-json3-dev
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev libglpk-dev
sudo apt install ros-noetic-pybind11-catkin
sudo apt install python3-catkin-tools
sudo apt install doxygen doxygen-latex
sudo apt-get install ros-noetic-rqt-multiplot
sudo apt install ros-noetic-moveit
rosdep install --from-paths src --ignore-src -r -y
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build

