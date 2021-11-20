#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(0.1);

  static const std::string ARM_PLANNING_GROUP = "stretch_arm";
  static const std::string GRIPPER_PLANNING_GROUP = "stretch_gripper";

  moveit::planning_interface::MoveGroupInterface arm_move_group_interface(ARM_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_move_group_interface(GRIPPER_PLANNING_GROUP);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  while (ros::ok())
  {

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    ROS_INFO("Arm Planning frame: %s", arm_move_group_interface.getPlanningFrame().c_str());
    ROS_INFO("Gripper Planning frame: %s", gripper_move_group_interface.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Arm End effector link: %s", arm_move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO("Gripper End effector link: %s", gripper_move_group_interface.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO("Available Planning Groups:");
    std::copy(arm_move_group_interface.getJointModelGroupNames().begin(),
              arm_move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO_STREAM("link_grasp_center pose: " << arm_move_group_interface.getCurrentPose("link_grasp_center") << '\n');

    const moveit::core::JointModelGroup *arm_joint_model_group = kinematic_model->getJointModelGroup("stretch_arm");
    const moveit::core::JointModelGroup *gripper_joint_model_group = kinematic_model->getJointModelGroup("stretch_gripper");
    
    const std::vector<std::string> &arm_joint_names = arm_joint_model_group->getVariableNames();
    const std::vector<std::string> &gripper_joint_names = gripper_joint_model_group->getVariableNames();

    std::vector<double> arm_joint_values;
    std::vector<double> gripper_joint_values;
    kinematic_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_values);
    kinematic_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_values);
    for (std::size_t i = 0; i < arm_joint_names.size(); ++i)
    {
      ROS_INFO("#1.1 Joint %s: %f", arm_joint_names[i].c_str(), arm_joint_values[i]);
    }
    for (std::size_t i = 0; i < gripper_joint_names.size(); ++i)
    {
      ROS_INFO("#1.2 Joint %s: %f", gripper_joint_names[i].c_str(), gripper_joint_values[i]);
    }

    geometry_msgs::Pose target_pose;
    target_pose.orientation.z = 1.0;
    target_pose.orientation.w = 0.34;
    target_pose.position.x = -0.17;
    target_pose.position.y = -0.53;
    target_pose.position.z = 1.0;
    bool success;
    success = arm_move_group_interface.setApproximateJointValueTarget(target_pose, "link_grasp_center");
    ROS_INFO("IK Planner %s\n", success ? "SUCCEED" : "FAILED");
    arm_move_group_interface.getJointValueTarget(arm_joint_values);
    for (std::size_t i = 0; i < arm_joint_names.size(); ++i)
    {
      ROS_INFO("#2.1 Joint %s: %f", arm_joint_names[i].c_str(), arm_joint_values[i]);
    }
    for (std::size_t i = 0; i < gripper_joint_names.size(); ++i)
    {
      ROS_INFO("#2.2 Joint %s: %f", gripper_joint_names[i].c_str(), gripper_joint_values[i]);
    }

    arm_move_group_interface.move();

    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}