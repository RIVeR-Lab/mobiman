// LAST UPDATE: 2023.05.16
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

//#include <math.h>

//#include <ros/ros.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <visualization_msgs/Marker.h>

/*
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
*/

//#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
//#include <moveit/planning_interface/planning_interface.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/PlanningScene.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  std::cout << "[test_moveit_planner_mobiman::main] START" << std::endl;

  ros::init(argc, argv, "test_moveit_planner_mobiman");
  ros::NodeHandle node_handle("/move_group/planning_pipelines/ompl");

  // User Inputs
  std::string group_name = "base";
  //std::string base_frame = "virtual_world_link";
  //std::string ee_frame = "ur5_tool0";
  std::string ee_frame = "base_link";
  std::string planner_id = "RRTstar";

  /// Robot Model
  robot_model_loader::RobotModelLoaderPtr robotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));

  /// Planning Scene
  planning_scene_monitor::PlanningSceneMonitorPtr psmPtr(new planning_scene_monitor::PlanningSceneMonitor(robotModelLoaderPtr));

  // Listen for planning scene messages on topic /XXX and apply them to the internal planning scene accordingly
  psmPtr->startSceneMonitor();

  // Listens to changes of world geometry, collision objects, and (optionally) octomaps
  //psmPtr->startWorldGeometryMonitor();
  psmPtr->startWorldGeometryMonitor("/collision_object");
  
  // Listen to joint state updates as well as changes in attached collision objects and update the internal planning scene accordingly
  psmPtr->startStateMonitor();

  /// Robot Model
  moveit::core::RobotModelPtr robotModelPtr = robotModelLoaderPtr->getModel();

  /// Robot State
  moveit::core::RobotStatePtr robotStatePtr(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psmPtr)->getCurrentState()));
  //moveit::core::RobotStatePtr robotStatePtr(new moveit::core::RobotState(robotModelPtr));
  //robotStatePtr_->setToDefaultValues();

  /// Joint Model Group
  const moveit::core::JointModelGroup* joint_model_group = robotStatePtr->getJointModelGroup("base");

  /// Planning Pipeline
  planning_pipeline::PlanningPipelinePtr planningPipelinePtr(new planning_pipeline::PlanningPipeline(robotModelPtr, node_handle, "planning_plugin", "request_adapters"));

  /// MotionPlanRequest
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  /// Set goal

  // Set the ee goal pose
  geometry_msgs::PoseStamped ee_pose;
  ee_pose.header.frame_id = "world";
  ee_pose.pose.position.x = -0.2;
  ee_pose.pose.position.y = 0.2;
  ee_pose.pose.position.z = 1.0;
  ee_pose.pose.orientation.w = 1.0;
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(ee_frame, ee_pose, tolerance_pose, tolerance_angle);

  // Set joint space goal
  moveit::core::RobotState goal_state(*robotStatePtr);
  std::vector<double> joint_values = {3.0, 1.0, 3.14};
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  /// Set Motion Plan Request 
  bool suc = false;
  req.group_name = group_name;
  req.planner_id = planner_id;
  req.allowed_planning_time = 60;
  req.num_planning_attempts = 10;
  
  req.goal_constraints.clear();
  //req.goal_constraints.push_back(pose_goal);
  req.goal_constraints.push_back(joint_goal);

  std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] BEFORE generatePlan" << std::endl;
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psmPtr);
    
    // Now, call the pipeline and check whether planning was successful.
    suc = planningPipelinePtr->generatePlan(lscene, req, res);
  }
  std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] AFTER generatePlan" << std::endl;

  // Now, call the pipeline and check whether planning was successful.
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] Could not compute plan successfully" << std::endl;
    return false;
  }

  // Set Visualization of the trajectory
  // Moveit Visual Tools
  //moveit_visual_tools::MoveItVisualTools visual_tools;
  //visual_tools.setBaseFrame(base_frame);
  //visual_tools.deleteAllMarkers();

  //moveit_msgs::DisplayTrajectory display_trajectory;
  //moveit_msgs::MotionPlanResponse response;
  //res.getMessage(response);

  //display_trajectory.trajectory_start = response.trajectory_start;
  //display_trajectory.trajectory.clear();
  //display_trajectory.trajectory.push_back(response.trajectory);

  // Publish Trajectory
  //ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/test_moveit_planner_mobiman/display_planned_path", 10, true);
  //const moveit::core::JointModelGroup* jointModelGroupPtr = robotModelPtr->getJointModelGroup(group_name);

  //display_pub.publish(display_trajectory);
  //visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), jointModelGroupPtr);

  int ctr = 1;
  while(ros::ok() && !suc)
  {
    std::cout << "[MoveitInterface::moveitPlanTrajectory] PLANNING AGAIN ... ctr: " << ctr << std::endl;

    ros::spinOnce();

    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world representation while planning
    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(psmPtr);
      
      // Now, call the pipeline and check whether planning was successful.
      suc = planningPipelinePtr->generatePlan(lscene, req, res);
    }
    
    // Now, call the pipeline and check whether planning was successful.
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("[MoveitInterface::moveitPlan] Could not compute plan successfully");
      return false;
    }
    ctr++;
  }

  std::cout << "[test_moveit_planner_mobiman::main] END" << std::endl;

  ros::waitForShutdown();
  return 0;
}