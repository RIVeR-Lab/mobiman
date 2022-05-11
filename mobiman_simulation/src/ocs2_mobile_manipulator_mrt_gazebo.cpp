/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

// External libraries:
#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

// Custom libraries:
#include <ocs2_mobile_manipulator_visualization.h>
#include <ocs2_mrt_loop.h>

using namespace ocs2;
using namespace mobile_manipulator;
using namespace std;

int main(int argc, char** argv) 
{
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Get node parameters
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;

  // Robot Interface
  mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  // ManipulatorModelInfo
  ManipulatorModelInfo manipulatorModelInfo = interface.getManipulatorModelInfo();

  std::cout << "ocs2_mobile_manipulator_mrt_gazebo::main -> manipulatorModelType: " << modelTypeEnumToString(manipulatorModelInfo.manipulatorModelType) << std::endl;
  std::cout << "ocs2_mobile_manipulator_mrt_gazebo::main -> stateDim: " << manipulatorModelInfo.stateDim << std::endl;
  std::cout << "ocs2_mobile_manipulator_mrt_gazebo::main -> inputDim: " << manipulatorModelInfo.inputDim << std::endl;
  std::cout << "ocs2_mobile_manipulator_mrt_gazebo::main -> armDim: " << manipulatorModelInfo.armDim << std::endl;
  std::cout << "ocs2_mobile_manipulator_mrt_gazebo::main -> dofNames: " << std::endl;
  for (int i = 0; i < manipulatorModelInfo.dofNames.size(); ++i)
  {
    std::cout << i << ": " << manipulatorModelInfo.dofNames[i] << std::endl;
  }

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<mobile_manipulator::OCS2_Mobile_Manipulator_Visualization> ocs2_mm_visu(new mobile_manipulator::OCS2_Mobile_Manipulator_Visualization(nodeHandle, interface));

  // MRT loop
  OCS2_MRT_Loop mrt_loop(nodeHandle, mrt, interface.getManipulatorModelInfo(), interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  mrt_loop.subscribeObservers({ocs2_mm_visu});

  // initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
  initObservation.time = 0.0;

  // initial command
  vector_t initTarget(7);
  initTarget.head(3) << 1, 0, 1;
  initTarget.tail(4) << Eigen::Quaternion<scalar_t>(0, 0, 1, 0).coeffs();
  const vector_t zeroInput = vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});

  // Run mrt_loop (loops while ros is ok)
  mrt_loop.run(initTargetTrajectories);
  //mrt_loop.runSinglePolicy(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
