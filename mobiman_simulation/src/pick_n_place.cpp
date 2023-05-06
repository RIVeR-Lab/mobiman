// LAST UPDATE: 2023.05.06
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

#include <math.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define GOAL_THRESHOLD_ 0.05

const double tau = 2 * M_PI;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class moveit_demo
{
  private:
    ros::NodeHandle nh_;
    ros::Publisher vis_pub_;
    std::string mp_group_name_arm_ = "arm";
    std::string mp_group_name_wholeBody = "whole_body";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface arm_move_group_interface_;
    moveit::planning_interface::MoveGroupInterface whole_body_move_group_interface_;
    moveit::core::RobotModelPtr kinematic_model_;
    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> whole_body_joint_names_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;

  public:
    moveit_demo(ros::NodeHandle n) : nh_(n), 
                                     arm_move_group_interface_(mp_group_name_arm_), 
                                     whole_body_move_group_interface_(mp_group_name_wholeBody),
                                     tf2_(buffer_)
    {
      // init marker publisher
      this->vis_pub_ = this->nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    
      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      this->kinematic_model_ = robot_model_loader.getModel();
      moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(this->kinematic_model_));
      kinematic_state->setToDefaultValues();

      // We can print the name of the reference frame for this robot.
      ROS_INFO("[moveit_demo::moveit_demo] Model frame: %s", this->kinematic_model_->getModelFrame().c_str());
      ROS_INFO("[moveit_demo::moveit_demo] Arm Planning frame: %s", this->arm_move_group_interface_.getPlanningFrame().c_str());
      ROS_INFO("[moveit_demo::moveit_demo] Whole_body Planning frame: %s", this->whole_body_move_group_interface_.getPlanningFrame().c_str());

      // We can get a list of all the groups in the robot:
      ROS_INFO("[moveit_demo::moveit_demo] Available Planning Groups:");
      std::copy(this->arm_move_group_interface_.getJointModelGroupNames().begin(),
                this->arm_move_group_interface_.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

      ROS_INFO_STREAM("[moveit_demo::moveit_demo] link_grasp_center pose: " << this->arm_move_group_interface_.getCurrentPose("link_grasp_center") << '\n');

      const moveit::core::JointModelGroup *arm_joint_model_group = this->kinematic_model_->getJointModelGroup(mp_group_name_arm_);
      const moveit::core::JointModelGroup *whole_body_joint_model_group = this->kinematic_model_->getJointModelGroup(mp_group_name_wholeBody);

      arm_joint_names_ = arm_joint_model_group->getVariableNames();
      whole_body_joint_names_ = whole_body_joint_model_group->getVariableNames();

      std::vector<double> arm_joint_values;
      std::vector<double> whole_body_joint_values;
      kinematic_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_values);
      kinematic_state->copyJointGroupPositions(whole_body_joint_model_group, whole_body_joint_values);
      for (std::size_t i = 0; i < this->arm_joint_names_.size(); ++i)
      {
        ROS_INFO("[moveit_demo::moveit_demo] Arm Joint %s: %f", this->arm_joint_names_[i].c_str(), arm_joint_values[i]);
      }
      for (std::size_t i = 0; i < this->whole_body_joint_names_.size(); ++i)
      {
        ROS_INFO("[moveit_demo::moveit_demo] WB Joint %s: %f", this->whole_body_joint_names_[i].c_str(), whole_body_joint_values[i]);
      }

      arm_move_group_interface_.setPoseReferenceFrame("world");
      arm_move_group_interface_.setEndEffectorLink("link_grasp_center");
      arm_move_group_interface_.setNumPlanningAttempts(10);
      arm_move_group_interface_.setMaxVelocityScalingFactor(0.25);
      arm_move_group_interface_.setMaxAccelerationScalingFactor(0.25);
      arm_move_group_interface_.setPlanningTime(60);
      whole_body_move_group_interface_.setPoseReferenceFrame("map");
      whole_body_move_group_interface_.setEndEffectorLink("link_grasp_center");
      whole_body_move_group_interface_.setNumPlanningAttempts(10);
      ROS_INFO("Pose Reference Frame is: %s", whole_body_move_group_interface_.getPoseReferenceFrame().c_str());
    };

    void addCollisionObjects()
    {
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      collision_objects.resize(2);

      // Add the first table where the cube will originally be kept.
      collision_objects[0].id = "table1";
      collision_objects[0].header.frame_id = "base_link";

      // Define the primitive and its dimensions.
      collision_objects[0].primitives.resize(1);
      collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
      collision_objects[0].primitives[0].dimensions.resize(3);
      collision_objects[0].primitives[0].dimensions[0] = 0.2;
      collision_objects[0].primitives[0].dimensions[1] = 0.4;
      collision_objects[0].primitives[0].dimensions[2] = 0.4;

      // Define the pose of the table.
      collision_objects[0].primitive_poses.resize(1);
      collision_objects[0].primitive_poses[0].position.x = 0.187;
      collision_objects[0].primitive_poses[0].position.y = -0.146;
      collision_objects[0].primitive_poses[0].position.z = 0.2;
      collision_objects[0].primitive_poses[0].orientation.w = 1.0;

      collision_objects[0].operation = collision_objects[0].ADD;

      /*
      // BEGIN_SUB_TUTORIAL table2
      // Add the second table where we will be placing the cube.
      collision_objects[1].id = "table2";
      collision_objects[1].header.frame_id = "base_link";

      // Define the primitive and its dimensions.
      collision_objects[1].primitives.resize(1);
      collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
      collision_objects[1].primitives[0].dimensions.resize(3);
      collision_objects[1].primitives[0].dimensions[0] = 0.4;
      collision_objects[1].primitives[0].dimensions[1] = 0.2;
      collision_objects[1].primitives[0].dimensions[2] = 0.4;

      // Define the pose of the table.
      collision_objects[1].primitive_poses.resize(1);
      collision_objects[1].primitive_poses[0].position.x = 0;
      collision_objects[1].primitive_poses[0].position.y = 0.5;
      collision_objects[1].primitive_poses[0].position.z = 0.2;
      collision_objects[1].primitive_poses[0].orientation.w = 1.0;
      // END_SUB_TUTORIAL

      collision_objects[1].operation = collision_objects[1].ADD;
      */      

      // Define the object that we will be manipulating
      collision_objects[1].header.frame_id = "base_link";
      collision_objects[1].id = "object";

      // Define the primitive and its dimensions.
      collision_objects[1].primitives.resize(1);
      collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
      collision_objects[1].primitives[0].dimensions.resize(3);
      collision_objects[1].primitives[0].dimensions[0] = 0.02;
      collision_objects[1].primitives[0].dimensions[1] = 0.02;
      collision_objects[1].primitives[0].dimensions[2] = 0.2;

      // Define the pose of the object.
      collision_objects[1].primitive_poses.resize(1);
      collision_objects[1].primitive_poses[0].position.x = 0.187;
      collision_objects[1].primitive_poses[0].position.y = -0.146;
      collision_objects[1].primitive_poses[0].position.z = 0.5;
      collision_objects[1].primitive_poses[0].orientation.w = 1.0;

      collision_objects[1].operation = collision_objects[1].ADD;
      

      planning_scene_interface_.applyCollisionObjects(collision_objects);
    }

    /*
    void openGripper()
    {
      const moveit::core::JointModelGroup *gripper_joint_model_group = this->kinematic_model_->getJointModelGroup("stretch_gripper");
      moveit::planning_interface::MoveGroupInterface gripper_interface("stretch_gripper");
      std::vector<std::string> gripper_joint_names_ = gripper_joint_model_group->getVariableNames();
      std::vector<double> gripper_joint_values;
      gripper_interface.getJointValueTarget(gripper_joint_values);

      for (std::size_t i = 0; i < gripper_joint_names_.size(); ++i)
      {
        ROS_INFO("Gripper Joint %s: %f", gripper_joint_names_[i].c_str(), gripper_joint_values[i]);
      }

      gripper_joint_values[0] = 0.25;
      gripper_joint_values[1] = 0.25;
      gripper_interface.setJointValueTarget(gripper_joint_values);
      gripper_interface.move();
    }
    */

    void attachObject()
    {
      moveit::planning_interface::MoveGroupInterface gripper_interface("gripper");
      gripper_interface.attachObject("object");
    }

    void detachObject()
    {
      moveit::planning_interface::MoveGroupInterface gripper_interface("gripper");
      gripper_interface.detachObject("object");
    }

    void pick()
    {
      std::vector<moveit_msgs::Grasp> grasps;
      grasps.resize(1);

      grasps[0].grasp_pose.header.frame_id = "base_link";
      tf2::Quaternion orientation;
      grasps[0].grasp_pose.pose.orientation.x = 0.0;
      grasps[0].grasp_pose.pose.orientation.y = 0.0;
      grasps[0].grasp_pose.pose.orientation.z = 0.0;
      grasps[0].grasp_pose.pose.orientation.w = 1.0;

      grasps[0].grasp_pose.pose.position.x = 0.187;
      grasps[0].grasp_pose.pose.position.y = -0.146;
      grasps[0].grasp_pose.pose.position.z = 0.50;

      arm_move_group_interface_.setPlanningTime(45.0);
      arm_move_group_interface_.setSupportSurfaceName("table1");

      arm_move_group_interface_.setGoalTolerance(1);
      arm_move_group_interface_.setGoalPositionTolerance(1);
      arm_move_group_interface_.setGoalOrientationTolerance(1);
      ROS_INFO("Position Goal Tolerance %f\n", arm_move_group_interface_.getGoalPositionTolerance());
      ROS_INFO("Orientation Goal Tolerance %f\n", arm_move_group_interface_.getGoalOrientationTolerance());

      // Call pick to pick up the object using the grasps given
      arm_move_group_interface_.pick("object", grasps);
    }

    void fake_pick()
    {
      arm_move_group_interface_.setPoseReferenceFrame("base_link");

      std::vector<double> arm_joint_values;
      arm_move_group_interface_.getJointValueTarget(arm_joint_values);
      arm_joint_values[0] = 0.484;
      arm_joint_values[5] = 0.5 * M_PI;

      bool success = this->arm_move_group_interface_.setJointValueTarget(arm_joint_values);
      ROS_INFO("Fake Pick IK Planner %s\n", success ? "SUCCEED" : "FAILED");
      if (success)
      {
        arm_move_group_interface_.move();
      }
    }

    void another_pick()
    {
      geometry_msgs::Pose target_pose;
      target_pose.position.x = 0.187;
      target_pose.position.y = -0.146;
      target_pose.position.z = 0.50;
      target_pose.orientation.w = 1.0;
      this->arm_move_group_interface_.setPoseReferenceFrame("base_link");
      bool success = this->arm_move_group_interface_.setApproximateJointValueTarget(target_pose, "link_grasp_center");
      ROS_INFO("Arm Planner %s\n", success ? "SUCCEED" : "FAILED");
      this->arm_move_group_interface_.move();
    }

    void another_place()
    {
      geometry_msgs::Pose target_pose;
      target_pose.position.x = 0.50;
      target_pose.position.y = -0.146;
      target_pose.position.z = 0.52;
      target_pose.orientation.w = 1.0;
      this->arm_move_group_interface_.setPoseReferenceFrame("base_link");
      bool success = this->arm_move_group_interface_.setApproximateJointValueTarget(target_pose, "link_grasp_center");
      ROS_INFO("Arm Planner %s\n", success ? "SUCCEED" : "FAILED");
      this->arm_move_group_interface_.move();
    }
};

int main(int argc, char **argv)
{
  std::cout << "[pick_n_place::main] START" << std::endl;

  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle("~");
  
  ros::AsyncSpinner spinner(4);
  spinner.start();

  moveit_demo demo(node_handle);

  demo.addCollisionObjects();

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  //demo.openGripper();
  //demo.another_pick();
  //demo.attachObject();
  //demo.another_place();
  //demo.detachObject();
  //demo.another_pick();

  std::cout << "[pick_n_place::main] END" << std::endl;

  ros::waitForShutdown();
  return 0;
}