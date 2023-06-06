// LAST UPDATE: 2023.06.05
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

// --EXTERNAL LIBRARIES--
#include <iostream>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// --CUSTOM LIBRARIES--

using namespace std;

int main(int argc, char **argv)
{
  std::cout << "[test_trajectory_publisher::main] START" << std::endl;

  // Initialize Node
  ros::init(argc, argv, "test_trajectory_publisher");

  // Initialize Node Handle for moveit config
  ros::NodeHandle nh("/move_group/planning_pipelines/ompl");

  // Initialize Node Handle for parameters
  ros::NodeHandle pnh("~");

  // Initialize and set parameters
  string group_name;
  vector<double> ee_goal;
  pnh.param<string>("/group_name", group_name, "");
  pnh.getParam("/ee_goal", ee_goal);

  cout << "[test_trajectory_publisher::main] group_name: " << group_name << std::endl;

  ros::Publisher jt_arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300/effort_joint_trajectory_controller/command", 1000);
  ros::Publisher jt_finger_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300/effort_finger_trajectory_controller/command", 1000);
  ros::Publisher jt_finger_tip1_pub = nh.advertise<std_msgs::Float64>("/j2n6s300/finger_tip_1_position_controller/command", 1000);
  ros::Publisher jt_finger_tip2_pub = nh.advertise<std_msgs::Float64>("/j2n6s300/finger_tip_2_position_controller/command", 1000);
  ros::Publisher jt_finger_tip3_pub = nh.advertise<std_msgs::Float64>("/j2n6s300/finger_tip_3_position_controller/command", 1000);

  // Set the Joint Trajectory for arm
  trajectory_msgs::JointTrajectory jt_arm_msg;
  jt_arm_msg.header.frame_id = "";
  jt_arm_msg.joint_names = {"j2n6s300_joint_1", "j2n6s300_joint_2","j2n6s300_joint_3","j2n6s300_joint_4","j2n6s300_joint_5","j2n6s300_joint_6"};
  
  trajectory_msgs::JointTrajectoryPoint jtp_arm;
  jtp_arm.positions.push_back(0.5*M_1_PI);  // j2n6s300_joint1
  jtp_arm.positions.push_back(-0.5*M_1_PI);     // j2n6s300_joint2
  jtp_arm.positions.push_back(0.0);     // j2n6s300_joint3
  jtp_arm.positions.push_back(0.0);     // j2n6s300_joint4
  jtp_arm.positions.push_back(0.0);     // j2n6s300_joint5
  jtp_arm.positions.push_back(0.0);     // j2n6s300_joint6

  jt_arm_msg.points.push_back(jtp_arm);
  jt_arm_pub.publish(jt_arm_msg);

  // Set the Joint Trajectory for finger
  trajectory_msgs::JointTrajectory jt_finger_msg;
  jt_finger_msg.header.frame_id = "";
  jt_finger_msg.joint_names = {"j2n6s300_joint_finger_1", "j2n6s300_joint_finger_2", "j2n6s300_joint_finger_3"};
  
  trajectory_msgs::JointTrajectoryPoint jtp_finger;
  jtp_finger.positions.push_back(1.0); // j2n6s300_joint_finger_1
  jtp_finger.positions.push_back(1.0); // j2n6s300_joint_finger_2
  jtp_finger.positions.push_back(1.0); // j2n6s300_joint_finger_3

  jt_finger_msg.points.push_back(jtp_finger);

  // Set the Joint Trajectory
  std_msgs::Float64 jt_figer_tip1_msg;
  jt_figer_tip1_msg.data = 0.0;

  std_msgs::Float64 jt_figer_tip2_msg;
  jt_figer_tip2_msg.data = 0.0;

  std_msgs::Float64 jt_figer_tip3_msg;
  jt_figer_tip3_msg.data = 0.0;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    //jt_arm_msg.header.stamp = ros::Time::now();
    //jt_arm_msg.header.stamp = ros::Time(0);
    //jt_arm_msg.header.seq++;
    
    //ros::spinOnce();

    //jt_finger_msg.header.stamp = ros::Time::now();
    //jt_finger_msg.header.seq++;
    //jt_finger_pub.publish(jt_finger_msg);
    
    ros::spinOnce();

    //jt_finger_tip1_pub.publish(jt_figer_tip1_msg);
    //ros::spinOnce();

    //jt_finger_tip2_pub.publish(jt_figer_tip2_msg);
    //ros::spinOnce();

    //jt_finger_tip3_pub.publish(jt_figer_tip3_msg);
    //ros::spinOnce();
    loop_rate.sleep();
  }

  ros::waitForShutdown();
  
  std::cout << "[test_trajectory_publisher::main] END" << std::endl;

  return 0;
}