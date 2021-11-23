#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void move_base(float x, float y, float theta)
{
  theta = 0;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, theta);
  myQuaternion.normalize();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = myQuaternion.getW();

  ROS_INFO("Sending goal: \nX: %f\nY: %f\nOmega: %f\n", x, y, theta);
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved");
  else
    ROS_INFO("The base failed to move");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(0.1);

  static const std::string ARM_PLANNING_GROUP = "stretch_arm";
  static const std::string whole_body_PLANNING_GROUP = "stretch_whole_body";

  moveit::planning_interface::MoveGroupInterface arm_move_group_interface(ARM_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface whole_body_move_group_interface(whole_body_PLANNING_GROUP);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  ROS_INFO("Arm Planning frame: %s", arm_move_group_interface.getPlanningFrame().c_str());
  ROS_INFO("whole_body Planning frame: %s", whole_body_move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Arm End effector link: %s", arm_move_group_interface.getEndEffectorLink().c_str());
  ROS_INFO("whole_body End effector link: %s", whole_body_move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO("Available Planning Groups:");
  std::copy(arm_move_group_interface.getJointModelGroupNames().begin(),
            arm_move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  ROS_INFO_STREAM("link_grasp_center pose: " << arm_move_group_interface.getCurrentPose("link_grasp_center") << '\n');

  const moveit::core::JointModelGroup *arm_joint_model_group = kinematic_model->getJointModelGroup("stretch_arm");
  const moveit::core::JointModelGroup *whole_body_joint_model_group = kinematic_model->getJointModelGroup("stretch_whole_body");

  const std::vector<std::string> &arm_joint_names = arm_joint_model_group->getVariableNames();
  const std::vector<std::string> &whole_body_joint_names = whole_body_joint_model_group->getVariableNames();

  std::vector<double> arm_joint_values;
  std::vector<double> whole_body_joint_values;
  kinematic_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_values);
  kinematic_state->copyJointGroupPositions(whole_body_joint_model_group, whole_body_joint_values);
  for (std::size_t i = 0; i < arm_joint_names.size(); ++i)
  {
    ROS_INFO("#1.1 Joint %s: %f", arm_joint_names[i].c_str(), arm_joint_values[i]);
  }
  for (std::size_t i = 0; i < whole_body_joint_names.size(); ++i)
  {
    ROS_INFO("#1.2 Joint %s: %f", whole_body_joint_names[i].c_str(), whole_body_joint_values[i]);
  }

  geometry_msgs::Pose target_pose;
  node_handle.getParam("target_pose_x", target_pose.position.x);
  node_handle.getParam("target_pose_y", target_pose.position.y);
  node_handle.getParam("target_pose_z", target_pose.position.z);
  // target_pose.position.x = 3.0;
  // target_pose.position.y = 2.0;
  // target_pose.position.z = 1.0;
  bool success;
  success = whole_body_move_group_interface.setJointValueTarget(target_pose, "link_grasp_center");
  ROS_INFO("IK Planner %s\n", success ? "SUCCEED" : "FAILED");

  whole_body_move_group_interface.getJointValueTarget(whole_body_joint_values);

  for (std::size_t i = 0; i < arm_joint_names.size(); ++i)
  {
    ROS_INFO("#2.1 Joint %s: %f", arm_joint_names[i].c_str(), arm_joint_values[i]);
  }
  for (std::size_t i = 0; i < whole_body_joint_names.size(); ++i)
  {
    ROS_INFO("#2.2 Joint %s: %f", whole_body_joint_names[i].c_str(), whole_body_joint_values[i]);
  }

  double x_axis = whole_body_joint_values[0];
  double y_axis = whole_body_joint_values[1];
  double theta = whole_body_joint_values[2];
  arm_joint_values[0] = whole_body_joint_values[3];
  arm_joint_values[1] = whole_body_joint_values[4];
  arm_joint_values[2] = whole_body_joint_values[5];
  arm_joint_values[3] = whole_body_joint_values[6];
  arm_joint_values[4] = whole_body_joint_values[7];
  arm_joint_values[5] = whole_body_joint_values[8];

  success = arm_move_group_interface.setJointValueTarget(arm_joint_values);
  ROS_INFO("Joint Value Target Set %s\n", success ? "SUCCEED" : "FAILED");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (arm_move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Arm Planning %s\n", success ? "SUCCEED" : "FAILED");

  arm_move_group_interface.setMaxVelocityScalingFactor(0.20);
  arm_move_group_interface.setMaxAccelerationScalingFactor(0.20);

  arm_move_group_interface.execute(my_plan);

  ROS_INFO("arm moving");

  move_base(x_axis, y_axis, theta);

  ros::shutdown();
  return 0;
}