/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <math.h>

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "ocs2_ros_interfaces/mrt/DummyObserver.h"
#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"
#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2 {

/**
 * This class implements a loop to test MPC-MRT communication interface using ROS.
 */
class OCS2_MRT_Loop 
{
  public:
    /**
     * Constructor.
     *
     * @param [in] mrt: The underlying MRT class to be used. If MRT contains a rollout object, the dummy will roll out
     * the received controller using the MRT::rolloutPolicy() method instead of just sending back a planned state.
     * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
     * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
     * will be simulated to run by this frequency. Note that this might not be the MPC's real-time frequency.
     */
    OCS2_MRT_Loop(ros::NodeHandle& nh,
                  MRT_ROS_Interface& mrt,
                  mobile_manipulator::ManipulatorModelInfo manipulatorModelInfo,
                  scalar_t mrtDesiredFrequency,
                  scalar_t mpcDesiredFrequency = -1);

    /**
     * Destructor.
     */
    virtual ~OCS2_MRT_Loop() = default;

    /**
     * Runs the dummy MRT loop.
     *
     * @param [in] initTargetTrajectories: The initial TargetTrajectories.
     */
    //void run(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories);
    void run(const TargetTrajectories& initTargetTrajectories);

    /**
     * Subscribe a set of observers to the dummy loop. Observers are updated in the provided order at the end of each timestep.
     * The previous list of observers is overwritten.
     *
     * @param observers : vector of observers.
     */
    void subscribeObservers(const std::vector<std::shared_ptr<DummyObserver>>& observers) 
    { 
      observers_ = observers; 
    }

  protected:
    /**
     * A user-defined function which modifies the observation before publishing.
     *
     * @param [in] observation: The current observation.
     */
    virtual void modifyObservation(SystemObservation& observation) {}

  private:
    /**
     * Runs a loop where mpc optimizations are synchronized with the forward simulation of the system
     */
    //void synchronizedDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories);
    void synchronizedDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories);

    /**
     * Runs a loop where mpc optimizations and simulation of the system are asynchronous.
     * The simulation runs as the specified mrtFrequency, and the MPC runs as fast as possible.
     */
    void realtimeDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories);

    /**
     * NUA TODO: Add decription.
     */
    void mrtLoop();

    /** Forward simulates the system from current observation*/
    SystemObservation forwardSimulation(const SystemObservation& currentObservation);

    /** NUA TODO: Add description */
    void setStateIndexMap();

    /** NUA TODO: Add description */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /** NUA TODO: Add description */
    void linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);

    /** NUA TODO: Add description */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    SystemObservation getCurrentObservation(bool initFlag=false);

    /** NUA TODO: Add description */
    //void publishCommand(const PrimalSolution& primalSolution);
    void publishCommand(SystemObservation& currentObservation);

    tf::TransformListener tfListener_;
    MRT_ROS_Interface& mrt_;
    scalar_t dt_;
    scalar_t time_;
  
    std::vector<std::shared_ptr<DummyObserver>> observers_;

    scalar_t mrtDesiredFrequency_;
    scalar_t mpcDesiredFrequency_;

    mobile_manipulator::ManipulatorModelInfo manipulatorModelInfo_;

    std::vector<int> stateIndexMap;

    nav_msgs::Odometry odometryMsg_;
    geometry_msgs::Pose robotBasePoseMsg_;
    sensor_msgs::JointState jointStateMsg_;
    control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg_;

    ros::Subscriber odometrySub_;
    ros::Subscriber linkStateSub_;
    ros::Subscriber jointStateSub_;
    ros::Subscriber jointTrajectoryPControllerStateSub_;

    ros::Publisher baseTwistPub_;
    ros::Publisher armJointTrajectoryPub_;
};

}  // namespace ocs2