// LAST UPDATE: 2023.05.11
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

// MoveIt
/*
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  std::cout << "[test_moveit_planner::main] START" << std::endl;

  ros::init(argc, argv, "test_moveit_planner");
  ros::NodeHandle node_handle("~");
  
  //ros::AsyncSpinner spinner(4);
  //spinner.start();

  // Robot Model
  robot_model_loader::RobotModelLoaderPtr robotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));

  // Robot Model
  moveit::core::RobotModelPtr robotModelPtr = robotModelLoaderPtr->getModel();

  // Robot State
  moveit::core::RobotStatePtr robotStatePtr(new moveit::core::RobotState(robotModelPtr));
  //robotStatePtr_->setToDefaultValues();

  // Planning Scene
  planning_scene_monitor::PlanningSceneMonitorPtr psmPtr(new planning_scene_monitor::PlanningSceneMonitor(robotModelLoaderPtr));

  // Listen for planning scene messages on topic /XXX and apply them to the internal planning scene accordingly
  psmPtr->startSceneMonitor();

  // Listens to changes of world geometry, collision objects, and (optionally) octomaps
  psmPtr->startWorldGeometryMonitor();
  //psmPtr->startWorldGeometryMonitor("/pick_n_place/collision_object");
  
  // Listen to joint state updates as well as changes in attached collision objects and update the internal planning scene accordingly
  psmPtr->startStateMonitor();

  //Planning Pipeline
  planning_pipeline::PlanningPipelinePtr planningPipelinePtr(new planning_pipeline::PlanningPipeline(robotModelPtr, node_handle, "/move_group/planning_plugin", "request_adapters"));
  //planning_pipeline::PlanningPipelinePtr planningPipelinePtr(new planning_pipeline::PlanningPipeline(robotModelPtr));

  // MotionPlanRequest
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // Set the ee goal pose
  geometry_msgs::PoseStamped ee_pose;
  ee_pose.header.frame_id = "world";
  ee_pose.pose.position.x = 0.2;
  ee_pose.pose.position.y = 0.0;
  ee_pose.pose.position.z = 1.0;
  ee_pose.pose.orientation.w = 1.0;

  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  std::string group_name = "panda_arm";
  std::string ee_frame = "panda_hand_tcp";
  std::string base_frame = "panda_link0";

  req.group_name = group_name;
  req.planner_id = "RRTstar";
  req.allowed_planning_time = 60;
  req.num_planning_attempts = 10;
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(ee_frame, ee_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  std::cout << "[MoveitInterface::moveitPlanTrajectory] BEFORE generatePlan" << std::endl;
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psmPtr);
    
    // Now, call the pipeline and check whether planning was successful.
    planningPipelinePtr->generatePlan(lscene, req, res);
  }
  std::cout << "[MoveitInterface::moveitPlanTrajectory] AFTER generatePlan" << std::endl;

  // Now, call the pipeline and check whether planning was successful.
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("[MoveitInterface::moveitPlan] Could not compute plan successfully");
    return false;
  }

  // Set Visualization of the trajectory
  // Moveit Visual Tools
  moveit_visual_tools::MoveItVisualTools visual_tools;
  visual_tools.setBaseFrame(base_frame);
  visual_tools.deleteAllMarkers();

  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(response.trajectory);

  // Publish Trajectory
  ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/test_moveit_planner/display_planned_path", 10, true);
  const moveit::core::JointModelGroup* jointModelGroupPtr = robotModelPtr->getJointModelGroup(group_name);

  display_pub.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), jointModelGroupPtr);

  /*
  while(ros::ok())
  {
    ros::spinOnce();
  }
  */

  std::cout << "[test_moveit_planner::main] END" << std::endl;

  ros::waitForShutdown();
  return 0;
}