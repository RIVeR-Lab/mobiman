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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>
#include <math.h>

#define GOAL_THRESHOLD_ 0.05

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class moveit_demo
{
private:
  ros::NodeHandle nh_;
  ros::Publisher vis_pub_;
  moveit::planning_interface::MoveGroupInterface arm_move_group_interface_;
  moveit::planning_interface::MoveGroupInterface whole_body_move_group_interface_;
  moveit::core::RobotModelPtr kinematic_model_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> whole_body_joint_names_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

public:
  moveit_demo(ros::NodeHandle n) : nh_(n), arm_move_group_interface_("stretch_arm"), whole_body_move_group_interface_("stretch_whole_body"),
                                   tf2_(buffer_)
  {

    // init marker publisher
    this->vis_pub_ = this->nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    this->kinematic_model_ = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(this->kinematic_model_));
    kinematic_state->setToDefaultValues();

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Model frame: %s", this->kinematic_model_->getModelFrame().c_str());
    ROS_INFO("Arm Planning frame: %s", this->arm_move_group_interface_.getPlanningFrame().c_str());
    ROS_INFO("whole_body Planning frame: %s", this->whole_body_move_group_interface_.getPlanningFrame().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO("Available Planning Groups:");
    std::copy(this->arm_move_group_interface_.getJointModelGroupNames().begin(),
              this->arm_move_group_interface_.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO_STREAM("link_grasp_center pose: " << this->arm_move_group_interface_.getCurrentPose("link_grasp_center") << '\n');

    const moveit::core::JointModelGroup *arm_joint_model_group = this->kinematic_model_->getJointModelGroup("stretch_arm");
    const moveit::core::JointModelGroup *whole_body_joint_model_group = this->kinematic_model_->getJointModelGroup("stretch_whole_body");

    arm_joint_names_ = arm_joint_model_group->getVariableNames();
    whole_body_joint_names_ = whole_body_joint_model_group->getVariableNames();

    std::vector<double> arm_joint_values;
    std::vector<double> whole_body_joint_values;
    kinematic_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_values);
    kinematic_state->copyJointGroupPositions(whole_body_joint_model_group, whole_body_joint_values);
    for (std::size_t i = 0; i < this->arm_joint_names_.size(); ++i)
    {
      ROS_INFO("#1.1 Joint %s: %f", this->arm_joint_names_[i].c_str(), arm_joint_values[i]);
    }
    for (std::size_t i = 0; i < this->whole_body_joint_names_.size(); ++i)
    {
      ROS_INFO("#1.2 Joint %s: %f", this->whole_body_joint_names_[i].c_str(), whole_body_joint_values[i]);
    }

    arm_move_group_interface_.setPoseReferenceFrame("map");
    arm_move_group_interface_.setEndEffectorLink("link_grasp_center");
    arm_move_group_interface_.setNumPlanningAttempts(10);
    arm_move_group_interface_.setMaxVelocityScalingFactor(0.25);
    arm_move_group_interface_.setMaxAccelerationScalingFactor(0.25);
    whole_body_move_group_interface_.setPoseReferenceFrame("map");
    whole_body_move_group_interface_.setEndEffectorLink("link_grasp_center");
    whole_body_move_group_interface_.setNumPlanningAttempts(10);
    ROS_INFO("Pose Reference Frame is: %s", whole_body_move_group_interface_.getPoseReferenceFrame().c_str());
  };

  /**
 * @brief move_base module to stretch robot, moving to x y with orientation of theta
 * 
 * @param x the x coordinate
 * @param y the y coordinate
 * @param theta the orientation
 */
  void move_base(float x, float y, float theta)
  {
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, theta);
    myQuaternion.normalize();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.x = myQuaternion.getX();
    goal.target_pose.pose.orientation.y = myQuaternion.getY();
    goal.target_pose.pose.orientation.z = myQuaternion.getZ();
    goal.target_pose.pose.orientation.w = myQuaternion.getW();

    ROS_INFO("Sending goal: \nX: %f\nY: %f\nX: %f\nY: %f\nZ: %f\nOmega: %f\n", x, y, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved");
    else
      ROS_INFO("The base failed to move");
  }

  /**
 * @brief publish rviz marker at the target position
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 */
  void publish_marker(double x, double y, double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.ns = "visual_markers";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = GOAL_THRESHOLD_ * 2;
    marker.scale.y = GOAL_THRESHOLD_ * 2;
    marker.scale.z = GOAL_THRESHOLD_ * 2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Publish the marker
    while (this->vis_pub_.getNumSubscribers() < 1)
    {
      ROS_ERROR("Please create a subscriber to the marker");
      sleep(1);
    }
    this->vis_pub_.publish(marker);
    ROS_INFO("marker published");
  }
  /**
 * @brief the main function for navi to a goal pose
 *        this is a callback function listening at the goal topic
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 */
  void nav_to_goal(const geometry_msgs::Point &point)
  {
    ROS_INFO("goal received");
    std::vector<double> arm_joint_values;
    std::vector<double> whole_body_joint_values;

    geometry_msgs::Pose target_pose;
    target_pose.position.x = point.x;
    target_pose.position.y = point.y;
    target_pose.position.z = point.z;

    // set the target market in rviz
    this->publish_marker(target_pose.position.x, target_pose.position.y, target_pose.position.z);

    ros::Rate loop_rate(0.1);
    while (true)
    {
      geometry_msgs::PoseStamped gripper_pose = arm_move_group_interface_.getCurrentPose("link_grasp_center");
      // transform to map frame
      geometry_msgs::PoseStamped transformed_gripper_pose = buffer_.transform(gripper_pose, "map", ros::Duration(5.0));

      double error = sqrt(pow(transformed_gripper_pose.pose.position.x - point.x, 2) + pow(transformed_gripper_pose.pose.position.y - point.y, 2) + pow(transformed_gripper_pose.pose.position.z - point.z, 2));
      if (error < GOAL_THRESHOLD_)
      {
        ROS_INFO("Goal Pose achieved within threshold!");
        break;
      }
      ROS_INFO_STREAM(transformed_gripper_pose);
      ROS_INFO("Goal Pose not achieved! %f Replanning!", error);
      bool success;
      success = this->whole_body_move_group_interface_.setJointValueTarget(target_pose, "link_grasp_center");
      ROS_INFO("Whole Body IK Planner %s\n", success ? "SUCCEED" : "FAILED");
      this->whole_body_move_group_interface_.getJointValueTarget(whole_body_joint_values);

      for (std::size_t i = 0; i < this->whole_body_joint_names_.size(); ++i)
      {
        ROS_INFO("#2.1 Joint %s: %f", this->whole_body_joint_names_[i].c_str(), whole_body_joint_values[i]);
      }

      double x_axis = whole_body_joint_values[0];
      double y_axis = whole_body_joint_values[1];
      double theta = whole_body_joint_values[2];

      move_base(x_axis, y_axis, theta);
      ROS_INFO("base finish moving");

      this->arm_move_group_interface_.getJointValueTarget(arm_joint_values);

      arm_joint_values[0] = whole_body_joint_values[3];
      arm_joint_values[1] = whole_body_joint_values[4];
      arm_joint_values[2] = whole_body_joint_values[5];
      arm_joint_values[3] = whole_body_joint_values[6];
      arm_joint_values[4] = whole_body_joint_values[7];
      arm_joint_values[5] = whole_body_joint_values[8];

      for (std::size_t i = 0; i < this->arm_joint_names_.size(); ++i)
      {
        ROS_INFO("#3.1 Joint %s: %f", this->arm_joint_names_[i].c_str(), arm_joint_values[i]);
      }
      success = this->arm_move_group_interface_.setJointValueTarget(arm_joint_values);
      ROS_INFO("Arm Joint Values Set %s\n", success ? "SUCCEED" : "FAILED");
      this->arm_move_group_interface_.move();
      ROS_INFO("arm moving");

      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  moveit_demo demo(node_handle);

  ROS_INFO("Listen to gripper_goal_pose");
  // listen to the input pose
  ros::Subscriber sub = node_handle.subscribe("gripper_goal_pose", 1, &moveit_demo::nav_to_goal, &demo);

  ros::waitForShutdown();
  return 0;
}