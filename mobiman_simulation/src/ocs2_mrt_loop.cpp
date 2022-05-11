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

#include "ocs2_mrt_loop.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OCS2_MRT_Loop::OCS2_MRT_Loop(ros::NodeHandle& nh,
                             MRT_ROS_Interface& mrt,
                             mobile_manipulator::ManipulatorModelInfo manipulatorModelInfo,
                             scalar_t mrtDesiredFrequency,
                             scalar_t mpcDesiredFrequency)
  : mrt_(mrt), manipulatorModelInfo_(manipulatorModelInfo), mrtDesiredFrequency_(mrtDesiredFrequency), mpcDesiredFrequency_(mpcDesiredFrequency) 
{
  std::cout << "OCS2_MRT_Loop::OCS2_MRT_Loop -> mrtDesiredFrequency_: " << mrtDesiredFrequency_ << std::endl;
  std::cout << "OCS2_MRT_Loop::OCS2_MRT_Loop -> mpcDesiredFrequency_: " << mpcDesiredFrequency_ << std::endl;
  
  dt_ = 1.0 / mrtDesiredFrequency_;

  setStateIndexMap();

  // SUBSCRIBE TO STATE INFO (sensor_msgs/JointState)
  
  // NUA TODO: Consider localization error
  //odometrySub_ = nh.subscribe("/jackal_velocity_controller/odom", 10, &OCS2_MRT_Loop::odometryCallback, this);
  linkStateSub_ = nh.subscribe("/gazebo/link_states", 10, &OCS2_MRT_Loop::linkStateCallback, this);
  //jointStateSub_ = nh.subscribe("/joint_states", 10, &OCS2_MRT_Loop::jointStateCallback, this);
  jointTrajectoryPControllerStateSub_ = nh.subscribe("/arm_controller/state", 10, &OCS2_MRT_Loop::jointTrajectoryControllerStateCallback, this);
  
  baseTwistPub_ = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 100);
  armJointTrajectoryPub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 100);

  if (mrtDesiredFrequency_ < 0) 
  {
    throw std::runtime_error("OCS2_MRT_Loop::OCS2_MRT_Loop -> Error: MRT loop frequency should be a positive number!");
  }

  if (mpcDesiredFrequency_ > 0) 
  {
    ROS_WARN_STREAM("OCS2_MRT_Loop::OCS2_MRT_Loop -> Warning: MPC loop is not realtime! For realtime setting, set mpcDesiredFrequency to any negative number.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2_MRT_Loop::run(const TargetTrajectories& initTargetTrajectories) 
{
  ROS_INFO_STREAM("OCS2_MRT_Loop::run -> Waiting for the initial policy ...");
  SystemObservation initObservation;

  // Reset MPC node
  mrt_.resetMpcNode(initTargetTrajectories);

  // Wait for the initial policy
  while (!mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
  {
    mrt_.spinMRT();

    // Get initial observation
    tfListener_.waitForTransform("/world", "/base_link", ros::Time::now(), ros::Duration(1.0));
    initObservation = getCurrentObservation(true);

    //std::cout << "OCS2_MRT_Loop::run -> initObservation:" << std::endl;
    //std::cout << initObservation << std::endl;

    mrt_.setCurrentObservation(initObservation);
    ros::Rate(mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("OCS2_MRT_Loop::run -> Initial policy has been received.");

  // Pick simulation loop mode
  if (mpcDesiredFrequency_ > 0.0) 
  {
    synchronizedDummyLoop(initObservation, initTargetTrajectories);
  } 
  else 
  {
    //std::cout << "OCS2_MRT_Loop::run -> mrtLoop" << std::endl;
    //realtimeDummyLoop(initObservation, initTargetTrajectories);
    mrtLoop();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//void OCS2_MRT_Loop::synchronizedDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) 
void OCS2_MRT_Loop::synchronizedDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) 
{
  // Determine the ratio between MPC updates and simulation steps.
  const auto mpcUpdateRatio = std::max(static_cast<size_t>(mrtDesiredFrequency_ / mpcDesiredFrequency_), size_t(1));

  // Loop variables
  size_t loopCounter = 0;
  SystemObservation currentObservation = initObservation;

  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an old policy instead of the latest one.
  const auto policyUpdatedForTime = [this](scalar_t time) 
  {
    constexpr scalar_t tol = 0.1;  // policy must start within this fraction of dt
    return mrt_.updatePolicy() && std::abs(mrt_.getPolicy().timeTrajectory_.front() - time) < (tol / mpcDesiredFrequency_);
  };

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) 
  {
    //std::cout << "OCS2_MRT_Loop::synchronizedDummyLoop -> START!" << std::endl;

    // Trigger MRT callbacks
    mrt_.spinMRT();

    //std::cout << "OCS2_MRT_Loop::synchronizedDummyLoop -> currentObservation.time: " << currentObservation.time << std::endl;

    // Update the MPC policy if it is time to do so
    if (loopCounter % mpcUpdateRatio == 0) 
    {
      // Wait for the policy to be updated
      //while (!policyUpdatedForTime(currentObservation.time) && ros::ok() && ros::master::check())
      while (!policyUpdatedForTime(currentObservation.time) && ros::ok() && ros::master::check())
      {
        mrt_.spinMRT();
        //std::cout << "currentObservation.time: " << currentObservation.time << std::endl;
      }
      std::cout << "OCS2_MRT_Loop::synchronizedDummyLoop -> policy start time: " << mrt_.getPolicy().timeTrajectory_.front() << std::endl;
    }

    PrimalSolution currentPolicy = mrt_.getPolicy();
    CommandData currentCommand = mrt_.getCommand();

    // Get Observation
    currentObservation = getCurrentObservation();

    // Forward simulation
    //currentObservation = forwardSimulation(currentObservation);
    SystemObservation targetObservation = forwardSimulation(currentObservation);

    // User-defined modifications before publishing
    modifyObservation(targetObservation);

    // Publish observation if at the next step we want a new policy
    if ((loopCounter + 1) % mpcUpdateRatio == 0) 
    {
      //mrt_.setCurrentObservation(currentObservation);
      mrt_.setCurrentObservation(currentObservation);
      //std::cout << "OCS2_MRT_Loop::synchronizedDummyLoop -> AFTER currentObservation time: " << currentObservation.time << std::endl;
    }

    // Update observers
    for (auto& observer : observers_) 
    {
      //observer -> update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
      observer -> update(currentObservation, currentPolicy, currentCommand);
    }

    //publishCommand(currentPolicy);

    ++loopCounter;
    time_ += dt_;

    //std::cout << "OCS2_MRT_Loop::synchronizedDummyLoop -> END!" << std::endl;

    ros::spinOnce();
    simRate.sleep();
  }
  //std::cout << "OCS2_MRT_Loop::synchronizedDummyLoop -> LOOP END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2_MRT_Loop::realtimeDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) 
{
  // Loop variables
  SystemObservation currentObservation = initObservation;

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) 
  {
    std::cout << "OCS2_MRT_Loop::realtimeDummyLoop -> BEFORE currentObservation time: " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the policy if a new on was received
    if (mrt_.updatePolicy()) 
    {
      std::cout << "OCS2_MRT_Loop::realtimeDummyLoop -> policy start time: " << mrt_.getPolicy().timeTrajectory_.front() << std::endl;
    }

    // Forward simulation
    currentObservation = forwardSimulation(currentObservation);

    // User-defined modifications before publishing
    modifyObservation(currentObservation);

    // Publish observation
    mrt_.setCurrentObservation(currentObservation);

    // Update observers
    for (auto& observer : observers_) 
    {
      observer -> update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ros::spinOnce();
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2_MRT_Loop::mrtLoop() 
{
  // Loop variables
  SystemObservation currentObservation;
  SystemObservation targetObservation;
  PrimalSolution currentPolicy;

  // Update the policy
  mrt_.updatePolicy();

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) 
  {
    //std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 0" << std::endl;
    mrt_.reset();

    while (!mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
    {
      mrt_.spinMRT();

      // Get initial observation
      currentObservation = getCurrentObservation(true);

      //std::cout << "OCS2_MRT_Loop::run -> initObservation:" << std::endl;
      //std::cout << initObservation << std::endl;

      mrt_.setCurrentObservation(currentObservation);
    }

    /*
    std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 1" << std::endl;
    currentObservation = getCurrentObservation(true);

    std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 2" << std::endl;
    // Publish observation
    mrt_.setCurrentObservation(currentObservation);

    std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 3" << std::endl;
    // Trigger MRT callbacks
    mrt_.spinMRT();
    */

    //std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 4" << std::endl;
    // Update the policy if a new on was received
    mrt_.updatePolicy();
    

    //std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 5" << std::endl;
    // Update observers
    for (auto& observer : observers_) 
    {
      observer -> update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    //currentObservation = getCurrentObservation();

    // Forward simulation
    //currentObservation = forwardSimulation(currentObservation);

    // Publish observation
    //mrt_.setCurrentObservation(currentObservation);

    //std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 6" << std::endl;
    //std::cout << "OCS2_MRT_Loop::mrtLoop -> publishCommand" << std::endl;
    publishCommand(currentObservation);

    time_ += dt_;

    ros::spinOnce();
    simRate.sleep();
    //std::cout << "OCS2_MRT_Loop::mrtLoop -> HUGO 7" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void OCS2_MRT_Loop::mrtLoop() 
{
  // Loop variables
  SystemObservation currentObservation;
  PrimalSolution currentPolicy;
  CommandData currentCommand;

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) 
  {
    std::cout << "OCS2_MRT_Loop::mrtLoop -> spinMRT" << std::endl;
    // Trigger MRT callbacks
    mrt_.spinMRT();

    std::cout << "OCS2_MRT_Loop::mrtLoop -> updatePolicy" << std::endl;
    mrt_.updatePolicy();

    std::cout << "OCS2_MRT_Loop::mrtLoop -> getPolicy" << std::endl;
    currentPolicy = mrt_.getPolicy();

    std::cout << "OCS2_MRT_Loop::mrtLoop -> getCommand" << std::endl;
    currentCommand = mrt_.getCommand();

    std::cout << "OCS2_MRT_Loop::mrtLoop -> publishCommand" << std::endl;
    publishCommand(currentPolicy);

    std::cout << "OCS2_MRT_Loop::mrtLoop -> nextTimeStep" << std::endl;
    time_ += dt_;

    std::cout << "OCS2_MRT_Loop::mrtLoop -> getCurrentObservation" << std::endl;
    currentObservation = getCurrentObservation(currentPolicy);

    std::cout << "OCS2_MRT_Loop::mrtLoop -> currentObservation:" << std::endl;
    std::cout << currentObservation << std::endl;

    std::cout << "OCS2_MRT_Loop::mrtLoop -> setCurrentObservation" << std::endl;
    // Publish observation
    mrt_.setCurrentObservation(currentObservation);

    std::cout << "OCS2_MRT_Loop::mrtLoop -> update" << std::endl;
    // Update observers
    for (auto& observer : observers_)
    {
      observer -> update(currentObservation, currentPolicy, currentCommand);
    }

    ros::spinOnce();
    simRate.sleep();
    std::cout << "OCS2_MRT_Loop::mrtLoop -> END" << std::endl;
    std::cout << "" << std::endl;
  }
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation OCS2_MRT_Loop::forwardSimulation(const SystemObservation& currentObservation) 
{
  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt_;
  
  if (mrt_.isRolloutSet()) 
  {  // If available, use the provided rollout as to integrate the dynamics.
    mrt_.rolloutPolicy(currentObservation.time, 
                       currentObservation.state, 
                       dt_, 
                       nextObservation.state, 
                       nextObservation.input,
                       nextObservation.mode);
  } 
  else 
  {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
    mrt_.evaluatePolicy(currentObservation.time + dt_, 
                        currentObservation.state, 
                        nextObservation.state, 
                        nextObservation.input,
                        nextObservation.mode);
  }

  return nextObservation;
}

void OCS2_MRT_Loop::setStateIndexMap()
{
  boost::shared_ptr<control_msgs::JointTrajectoryControllerState const> current_jointTrajectoryControllerStatePtrMsg = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/arm_controller/state");
  int n_joints = manipulatorModelInfo_.dofNames.size();
  stateIndexMap.clear();
  int c;

  if (manipulatorModelInfo_.dofNames.size() != current_jointTrajectoryControllerStatePtrMsg -> joint_names.size())
  {
    throw std::runtime_error("OCS2_MRT_Loop::setStateIndexMap -> Error: State dimension mismatch!");
  }
  
  for (int i = 0; i < n_joints; ++i)
  {
    //std::cout << "OCS2_MRT_Loop::setStateIndexMap -> jointTrajectoryControllerStatePtrMsg " << i << ": " << current_jointTrajectoryControllerStatePtrMsg -> joint_names[i] << std::endl;
    //std::cout << "OCS2_MRT_Loop::setStateIndexMap -> manipulatorModelInfo_ " << i << ": " << manipulatorModelInfo_.dofNames[i] << std::endl;
    //std::cout << "" << std::endl;

    c = 0;
    while (current_jointTrajectoryControllerStatePtrMsg -> joint_names[c] != manipulatorModelInfo_.dofNames[i] && c < n_joints)
    {
      c++;
    }

    if (current_jointTrajectoryControllerStatePtrMsg -> joint_names[c] == manipulatorModelInfo_.dofNames[i])
    {
      stateIndexMap.push_back(c);
    }
  }

  //std::cout << "manipulatorModelInfo_ -> jointTrajectoryControllerStatePtrMsg" << std::endl;
  //for (int i = 0; i < stateIndexMap.size(); ++i)
  //{
  //  std::cout << i << " -> " << stateIndexMap[i] << std::endl;
  //}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2_MRT_Loop::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometryMsg_ = *msg;
  /*
  for (int i = 0; i < msg -> name.size(); ++i)
  {
    std::cout << i << ": " << msg -> name[i] << std::endl;
  }
  std::cout << "" << std::endl;
  */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2_MRT_Loop::linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  //std::cout << "OCS2_MRT_Loop::linkStateCallback" << std::endl;
  for (int i = 0; i < msg -> name.size(); ++i)
  {
    //std::cout << i << ": " << msg -> name[i] << std::endl;

    if (msg -> name[i] == "mobiman::base_link")
    {
      robotBasePoseMsg_ = msg -> pose[i];
    }
  }
  //std::cout << "" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2_MRT_Loop::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointStateMsg_ = *msg;
  //std::cout << "OCS2_MRT_Loop::jointStateCallback -> frame_id: " << jointStateMsg_.header.frame_id << std::endl;
  /*
  for (int i = 0; i < msg -> name.size(); ++i)
  {
    std::cout << i << ": " << msg -> name[i] << std::endl;
  }
  std::cout << "" << std::endl;
  */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2_MRT_Loop::jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  jointTrajectoryControllerStateMsg_ = *msg;
  //std::cout << "OCS2_MRT_Loop::jointTrajectoryControllerStateCallback -> frame_id: " << jointTrajectoryControllerStateMsg_.header.frame_id << std::endl;
  /*
  for (int i = 0; i < msg -> joint_names.size(); ++i)
  {
    std::cout << i << ": " << msg -> joint_names[i] << std::endl;
  }
  std::cout << "" << std::endl;
  */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation OCS2_MRT_Loop::getCurrentObservation(bool initFlag)
{
  geometry_msgs::Pose current_robotBasePoseMsg = robotBasePoseMsg_;
  control_msgs::JointTrajectoryControllerState current_jointTrajectoryControllerStateMsg = jointTrajectoryControllerStateMsg_;

  SystemObservation currentObservation;
  currentObservation.mode = 0;
  currentObservation.time = time_;

  if (initFlag)
  {
    currentObservation.input.setZero(manipulatorModelInfo_.inputDim);
  }
  else
  {
    PrimalSolution primalSolution = mrt_.getPolicy();
    currentObservation.input = primalSolution.getDesiredInput(time_);
  }

  currentObservation.state.setZero(manipulatorModelInfo_.stateDim);
  //currentObservation.input.setZero(manipulatorModelInfo_.inputDim);
  
  //std::cout << "OCS2_MRT_Loop::getCurrentObservation -> time_: " << time_ << std::endl;

  // Lookup transform, NUA TODO: GENERALIZE FOR DIFFERENT NAMES!
  tf::StampedTransform tf_robot_wrt_world;
  try
  {
    tfListener_.lookupTransform("/world", "/base_link", ros::Time(0), tf_robot_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion quat_robot_wrt_world = tf_robot_wrt_world.getRotation();
  tf::Matrix3x3 matrix_robot_wrt_world = tf::Matrix3x3(quat_robot_wrt_world);
  double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
  matrix_robot_wrt_world.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);

  // Set mobile base states
  currentObservation.state[0] = current_robotBasePoseMsg.position.x;
  currentObservation.state[1] = current_robotBasePoseMsg.position.y;
  currentObservation.state[2] = yaw_robot_wrt_world;

  //std::cout << "OCS2_MRT_Loop::getCurrentObservation -> current_jointTrajectoryControllerStateMsg frame_id: " << current_jointTrajectoryControllerStateMsg.header.frame_id << std::endl;

  // Set arm states and inputs
  for (int i = 0; i < current_jointTrajectoryControllerStateMsg.joint_names.size(); ++i)
  {
    //std::cout << "armJointTrajectoryMsg " << i << " -> " << current_jointTrajectoryControllerStateMsg.joint_names[stateIndexMap[i]] << std::endl; //stateIndexMap[i]
    //std::cout << "manipulatorModelInfo_ " << i << " -> " << manipulatorModelInfo_.dofNames[i] << std::endl;
    //std::cout << "----------------" << std::endl;

    currentObservation.state[i+3] = current_jointTrajectoryControllerStateMsg.actual.positions[stateIndexMap[i]];
  }

  return currentObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//void OCS2_MRT_Loop::publishCommand(const PrimalSolution& primalSolution)
void OCS2_MRT_Loop::publishCommand(SystemObservation& currentObservation)
{
  geometry_msgs::Twist baseTwistMsg;
  trajectory_msgs::JointTrajectory armJointTrajectoryMsg;

  /*
  PrimalSolution primalSolution = mrt_.getPolicy();

  std::cout << "OCS2_MRT_Loop::publishCommand -> primalSolution.timeTrajectory_ size: " << primalSolution.timeTrajectory_.size() << std::endl;
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); ++i)
  {
    std::cout << i << ": " << primalSolution.timeTrajectory_[i] << std::endl;
  }

  auto nextState = primalSolution.getDesiredState(time_ + dt_);
  //auto nextState = primalSolution.stateTrajectory_[1];
  auto currentInput = primalSolution.getDesiredInput(time_);
  //auto currentInput = primalSolution.inputTrajectory_[0];

  std::cout << "OCS2_MRT_Loop::publishCommand -> dt_: " << dt_ << std::endl;
  std::cout << "OCS2_MRT_Loop::publishCommand -> currentInput[0]: " << currentInput[0] << std::endl;
  std::cout << "OCS2_MRT_Loop::publishCommand -> currentInput[1]: " << currentInput[1] << std::endl;
  */

  // Forward simulation
  //SystemObservation targetObservation = forwardSimulation(currentObservation);
  //auto nextState = targetObservation.state;
  //auto currentInput = currentObservation.input;

  PrimalSolution primalSolution = mrt_.getPolicy();
  auto nextState = primalSolution.getDesiredState(time_ + dt_);
  auto currentInput = primalSolution.getDesiredInput(time_);

  //std::cout << "OCS2_MRT_Loop::publishCommand -> currentInput size: " << currentInput.size() << std::endl;
  //std::cout << "OCS2_MRT_Loop::publishCommand -> nextState size: " << nextState.size() << std::endl;

  // Set base command
  baseTwistMsg.linear.x = currentInput[0];
  baseTwistMsg.angular.z = currentInput[1];

  // Set arm command
  int n_joints = manipulatorModelInfo_.dofNames.size();
  //armJointTrajectoryMsg.header.frame_id = "world";

  armJointTrajectoryMsg.joint_names.resize(n_joints);
  trajectory_msgs::JointTrajectoryPoint jtp;
  jtp.positions.resize(n_joints);
  jtp.time_from_start = ros::Duration(dt_);

  //std::cout << "OCS2_MRT_Loop::publishCommand -> n_joints:" << n_joints << std::endl;

  for (int i = 0; i < n_joints; ++i)
  {
    armJointTrajectoryMsg.joint_names[i] = manipulatorModelInfo_.dofNames[i];
    //std::cout << "OCS2_MRT_Loop::publishCommand -> dofNames " << i << ":" << manipulatorModelInfo_.dofNames[i] << std::endl;
    jtp.positions[i] = nextState[i+3];
  }
  
  /*
  jtp.positions[0] = 0.0;
  jtp.positions[1] = -2.0;
  jtp.positions[2] = 0.0;
  jtp.positions[3] = 2.0;
  jtp.positions[4] = -2.0;
  jtp.positions[5] = 1.0;
  */
  
  armJointTrajectoryMsg.points.push_back(jtp);

  // Publish command
  baseTwistPub_.publish(baseTwistMsg);
  armJointTrajectoryPub_.publish(armJointTrajectoryMsg);

  //std::cout << "OCS2_MRT_Loop::publishCommand -> END" << std::endl;
}

}  // namespace ocs2