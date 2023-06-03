// LAST UPDATE: 2023.05.27
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

// --EXTERNAL LIBRARIES--
//#include <math.h>

//#include <ros/ros.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/*
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>

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

// --CUSTOM LIBRARIES--
#include <mobiman_simulation/common_utility.h>

using namespace std;

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void transformPose(tf::TransformListener& tflistener,
                   string& frame_from,
                   string& frame_to,
                   geometry_msgs::Pose& p_from,
                   geometry_msgs::Pose& p_to)
{
  tf::Pose p_from_tf;
  geometry_msgs::Pose p_from_msg = p_from;
  tf::poseMsgToTF(p_from_msg, p_from_tf);
  tf::Stamped<tf::Pose> p_from_stamped_tf(p_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Pose> p_to_stamped_tf;
  geometry_msgs::PoseStamped p_to_stamped_msg;

  try
  {
    tflistener.transformPose(frame_to, p_from_stamped_tf, p_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("[test_moveit_planner_mobiman::transformPose] Couldn't get transform!");
    ROS_ERROR("%s",ex.what());

    while(1);
    cout << "[test_moveit_planner_mobiman::transformPose] DEBUG INF" << endl;
    //ros::Duration(1.0).sleep();
  }

  tf::poseStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
  p_to = p_to_stamped_msg.pose;
}

int main(int argc, char **argv)
{
  std::cout << "[test_moveit_planner_mobiman::main] START" << std::endl;

  // Initialize Node
  ros::init(argc, argv, "test_moveit_planner_mobiman");

  tf::TransformListener tflistener;

  // Initialize Node Handle for moveit config
  ros::NodeHandle nh("/move_group/planning_pipelines/ompl");

  // Initialize Node Handle for parameters
  ros::NodeHandle pnh("~");

  // Initialize and set parameters
  string group_name, ee_frame, goal_frame, world_frame, ee_goal_type, planner_id, planning_framework;
  double allowed_planning_time, position_tolerance, orientation_tolerance, joint_tolerance;
  int num_planning_attempts;
  vector<double> ee_goal;
  pnh.param<string>("/group_name", group_name, "");
  pnh.param<string>("/ee_frame", ee_frame, "");
  pnh.param<string>("/goal_frame", goal_frame, "");
  pnh.param<string>("/world_frame", world_frame, "");
  pnh.param<string>("/ee_goal_type", ee_goal_type, "");
  pnh.getParam("/ee_goal", ee_goal);
  pnh.param<double>("/position_tolerance", position_tolerance, 0.0);
  pnh.param<double>("/orientation_tolerance", orientation_tolerance, 0.0);
  pnh.param<double>("/joint_tolerance", joint_tolerance, 0.0);
  pnh.param<string>("/planner_id", planner_id, "");
  pnh.param<double>("/allowed_planning_time", allowed_planning_time, 0.0);
  pnh.param<int>("/num_planning_attempts", num_planning_attempts, 0);
  pnh.param<string>("/planning_framework", planning_framework, "");

  cout << "[test_moveit_planner_mobiman::main] group_name: " << group_name << std::endl;
  cout << "[test_moveit_planner_mobiman::main] ee_frame: " << ee_frame << std::endl;
  cout << "[test_moveit_planner_mobiman::main] goal_frame: " << goal_frame << std::endl;
  cout << "[test_moveit_planner_mobiman::main] world_frame: " << world_frame << std::endl;
  cout << "[test_moveit_planner_mobiman::main] ee_goal_type: " << ee_goal_type << std::endl;
  cout << "[test_moveit_planner_mobiman::main] ee_goal: " << std::endl;
  print(ee_goal);
  cout << "[test_moveit_planner_mobiman::main] position_tolerance: " << position_tolerance << std::endl;
  cout << "[test_moveit_planner_mobiman::main] orientation_tolerance: " << orientation_tolerance << std::endl;
  cout << "[test_moveit_planner_mobiman::main] joint_tolerance: " << joint_tolerance << std::endl;
  cout << "[test_moveit_planner_mobiman::main] planner_id: " << planner_id << std::endl;
  cout << "[test_moveit_planner_mobiman::main] allowed_planning_time: " << allowed_planning_time << std::endl;
  cout << "[test_moveit_planner_mobiman::main] num_planning_attempts: " << num_planning_attempts << std::endl;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /// Robot Model Loader
  robot_model_loader::RobotModelLoaderPtr robotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));

  /// Robot Model
  moveit::core::RobotModelPtr robotModelPtr = robotModelLoaderPtr->getModel();

  /// Robot State
  //moveit::core::RobotStatePtr robotStatePtr(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psmPtr)->getCurrentState()));
  moveit::core::RobotStatePtr robotStatePtr(new moveit::core::RobotState(robotModelPtr));
  //robotStatePtr_->setToDefaultValues();

  /// Joint Model Group
  const moveit::core::JointModelGroup* jointModelGroupPtr = robotStatePtr->getJointModelGroup(group_name);

  /// Collision objects
  /*
  std::vector<moveit_msgs::CollisionObject> moveit_collision_objects;

  moveit_collision_objects.resize(1);
  moveit_collision_objects[0].id = "table1";
  
  moveit_collision_objects[0].primitives.resize(1);
  moveit_collision_objects[0].primitives[0].type = moveit_collision_objects[0].primitives[0].BOX;
  moveit_collision_objects[0].primitives[0].dimensions.resize(3);
  moveit_collision_objects[0].primitives[0].dimensions[0] = 0.2;
  moveit_collision_objects[0].primitives[0].dimensions[1] = 1.4;
  moveit_collision_objects[0].primitives[0].dimensions[2] = 0.4;

  moveit_collision_objects[0].pose.position.x = 1.0;
  moveit_collision_objects[0].pose.position.y = 0.0;
  moveit_collision_objects[0].pose.position.z = 0.2;
  moveit_collision_objects[0].pose.orientation.x = 0.0;
  moveit_collision_objects[0].pose.orientation.y = 0.0;
  moveit_collision_objects[0].pose.orientation.z = 0.0;
  moveit_collision_objects[0].pose.orientation.w = 1.0;

  moveit_collision_objects[0].operation = moveit_collision_objects[0].ADD;

  moveit_collision_objects[0].header.frame_id = "base_link";
  moveit_collision_objects[0].header.stamp = ros::Time::now();
  moveit_collision_objects[0].header.seq++;
  */

  if (planning_framework == "motion_plan_request")
  {
    std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] START motion_plan_request" << std::endl;

    /// Planning Scene Monitor
    planning_scene_monitor::PlanningSceneMonitorPtr psmPtr(new planning_scene_monitor::PlanningSceneMonitor(robotModelLoaderPtr));

    // Listen for planning scene messages on topic /XXX and apply them to the internal planning scene accordingly
    psmPtr->startSceneMonitor();

    // Listens to changes of world geometry, collision objects, and (optionally) octomaps
    psmPtr->startWorldGeometryMonitor("/collision_object");
    
    // Listen to joint state updates as well as changes in attached collision objects and update the internal planning scene accordingly
    psmPtr->startStateMonitor();

    /// Planning Pipeline
    planning_pipeline::PlanningPipelinePtr planningPipelinePtr(new planning_pipeline::PlanningPipeline(robotModelPtr, nh, "planning_plugin", "request_adapters")); 

    /// MotionPlanRequest
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    /// Set goal
    moveit_msgs::Constraints goal_constraint;

    if (ee_goal_type == "pose")
    {
      geometry_msgs::Pose ee_pose_wrt_goal_frame;

      // Set the ee goal pose
      geometry_msgs::Pose obj_pose_wrt_world;
      obj_pose_wrt_world.position.x = ee_goal[0];
      obj_pose_wrt_world.position.y = ee_goal[1];
      obj_pose_wrt_world.position.z = ee_goal[2];
      obj_pose_wrt_world.orientation.x = ee_goal[3];
      obj_pose_wrt_world.orientation.y = ee_goal[4];
      obj_pose_wrt_world.orientation.z = ee_goal[5];
      obj_pose_wrt_world.orientation.w = ee_goal[6];

      transformPose(tflistener, world_frame, goal_frame, obj_pose_wrt_world, ee_pose_wrt_goal_frame);

      geometry_msgs::PoseStamped ee_poseStamped_wrt_goal_frame;
      ee_poseStamped_wrt_goal_frame.header.frame_id = goal_frame;
      ee_poseStamped_wrt_goal_frame.pose = ee_pose_wrt_goal_frame;

      //std::vector<double> tolerance_pose(3, 0.1);
      //std::vector<double> tolerance_angle(3, 0.1);
      goal_constraint = kinematic_constraints::constructGoalConstraints(ee_frame, ee_poseStamped_wrt_goal_frame, position_tolerance, orientation_tolerance);
    }
    else if (ee_goal_type == "joint")
    {
      // Set joint space goal
      moveit::core::RobotState goal_state(*robotStatePtr);
      std::vector<double> joint_values = ee_goal;
      goal_state.setJointGroupPositions(jointModelGroupPtr, joint_values);
      goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, jointModelGroupPtr);
    }

    // Set Motion Plan Request 
    bool success = false;
    req.group_name = group_name;
    req.planner_id = planner_id;
    req.allowed_planning_time = allowed_planning_time;
    req.num_planning_attempts = num_planning_attempts;
    
    req.goal_constraints.clear();
    req.goal_constraints.push_back(goal_constraint);

    std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] BEFORE generatePlan" << std::endl;
    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world representation while planning
    {
      //planning_scene_monitor::LockedPlanningSceneRW lscene(psmPtr);
      psmPtr->lockSceneRead();
      planning_scene::PlanningScenePtr psPtr = psmPtr->getPlanningScene();
      psmPtr->unlockSceneRead();

      vector<moveit_msgs::CollisionObject> vco;
      //lscene->getCollisionObjectMsgs(vco);
      psPtr->getCollisionObjectMsgs(vco);

      std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] vco.size(): " << vco.size() << std::endl;
      for (size_t i = 0; i < vco.size(); i++)
      {
        cout << vco[i].id << endl;
      }

      int ctr = 0;
      while (vco.size() <= 0)
      {
        //planning_scene_monitor::LockedPlanningSceneRW lscene_tmp(psmPtr);
        psmPtr->lockSceneRead();
        psPtr = psmPtr->getPlanningScene();
        psmPtr->unlockSceneRead();

        //lscene->getCollisionObjectMsgs(vco);
        psPtr->getCollisionObjectMsgs(vco);

        ctr++;
      }

      planning_scene_monitor::LockedPlanningSceneRO lscene(psmPtr);

      if (vco.size() <= 0)
      {
        std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] DEBUG INF" << std::endl;
        while(1);
      }
      std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] BEFORE generatePlan: " << ctr << std::endl;
      
      // Now, call the pipeline and check whether planning was successful.
      success = planningPipelinePtr->generatePlan(lscene, req, res);
    }
    std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] AFTER generatePlan" << std::endl;

    // Now, call the pipeline and check whether planning was successful.
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] Could not compute plan successfully" << std::endl;
      return false;
    }

    int ctr = 0;
    while(ros::ok() && !success)
    {
      //std::cout << "[MoveitInterface::moveitPlanTrajectory] PLANNING AGAIN ... ctr: " << ctr << std::endl;

      //ros::spinOnce();

      // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world representation while planning
      {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psmPtr);
        
        // Now, call the pipeline and check whether planning was successful.
        success = planningPipelinePtr->generatePlan(lscene, req, res);
      }
      
      // Now, call the pipeline and check whether planning was successful.
      if (res.error_code_.val != res.error_code_.SUCCESS)
      {
        ROS_ERROR("[MoveitInterface::moveitPlan] Could not compute plan successfully");
        return false;
      }
      ctr++;
    }
    std::cout << "[test_moveit_planner_mobiman::main] ctr: " << ctr << std::endl;

    ros::waitForShutdown();
  }

  else if (planning_framework == "move_group_interface")
  {
    std::cout << "[test_moveit_planner_mobiman::test_moveit_planner_mobiman] START motion_plan_request" << std::endl;

    /// Move Group Interface
    moveit::planning_interface::MoveGroupInterface moveGroupInterface(group_name);

    /// Planning Scene Interface
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

    /// Move Group Interface Plan
    moveit::planning_interface::MoveGroupInterface::Plan mgiPlan;

    jointModelGroupPtr = moveGroupInterface.getCurrentState()->getJointModelGroup(group_name);

    // Add collision objects
    //planningSceneInterface.addCollisionObjects(moveit_collision_objects);

    //moveGroupInterface.setStartStateToCurrentState();
    moveGroupInterface.setStartState(*moveGroupInterface.getCurrentState());
    
    if (ee_goal_type == "pose")
    {
      geometry_msgs::Pose ee_pose_wrt_goal_frame;

      // Set the ee goal pose
      geometry_msgs::Pose obj_pose_wrt_world;
      obj_pose_wrt_world.position.x = ee_goal[0];
      obj_pose_wrt_world.position.y = ee_goal[1];
      obj_pose_wrt_world.position.z = ee_goal[2];
      obj_pose_wrt_world.orientation.x = ee_goal[3];
      obj_pose_wrt_world.orientation.y = ee_goal[4];
      obj_pose_wrt_world.orientation.z = ee_goal[5];
      obj_pose_wrt_world.orientation.w = ee_goal[6];

      cout << "x: " << obj_pose_wrt_world.position.x << endl;
      cout << "y: " << obj_pose_wrt_world.position.y << endl;
      cout << "z: " << obj_pose_wrt_world.position.z << endl;
      cout << "quad x: " << obj_pose_wrt_world.orientation.x << endl;
      cout << "quad y: " << obj_pose_wrt_world.orientation.y << endl;
      cout << "quad z: " << obj_pose_wrt_world.orientation.z << endl;
      cout << "quad w: " << obj_pose_wrt_world.orientation.w << endl << endl;

      transformPose(tflistener, world_frame, goal_frame, obj_pose_wrt_world, ee_pose_wrt_goal_frame);

      geometry_msgs::PoseStamped ee_poseStamped_wrt_goal_frame;
      ee_poseStamped_wrt_goal_frame.header.frame_id = goal_frame;
      ee_poseStamped_wrt_goal_frame.pose = ee_pose_wrt_goal_frame;

      cout << "x: " << ee_pose_wrt_goal_frame.position.x << endl;
      cout << "y: " << ee_pose_wrt_goal_frame.position.y << endl;
      cout << "z: " << ee_pose_wrt_goal_frame.position.z << endl;
      cout << "quad x: " << ee_pose_wrt_goal_frame.orientation.x << endl;
      cout << "quad y: " << ee_pose_wrt_goal_frame.orientation.y << endl;
      cout << "quad z: " << ee_pose_wrt_goal_frame.orientation.z << endl;
      cout << "quad w: " << ee_pose_wrt_goal_frame.orientation.w << endl;

      moveGroupInterface.setPoseTarget(ee_poseStamped_wrt_goal_frame, ee_frame);
      moveGroupInterface.setGoalPositionTolerance(position_tolerance);
      moveGroupInterface.setGoalOrientationTolerance(orientation_tolerance);
    }
    else if (ee_goal_type == "joint")
    {
      moveGroupInterface.setJointValueTarget(ee_goal);
      moveGroupInterface.setGoalJointTolerance(joint_tolerance);
    }
    
    moveGroupInterface.setPlannerId(planner_id);
    moveGroupInterface.setPlanningTime(allowed_planning_time);
    moveGroupInterface.setNumPlanningAttempts(num_planning_attempts);
    moveGroupInterface.allowReplanning(true);
    
    auto params = moveGroupInterface.getPlannerParams(planner_id, group_name);

    // Get an iterator pointing to the first element in the map
    auto it = params.begin();
  
    // Iterate through the map and print the elements
    while (it != params.end())
    {
      std::cout << "Key: " << it->first << ", Value: " << it->second << std::endl;
      ++it;
    }

    /*
    for (auto i = params.begin(); i != params.end(); i++)
    {
      cout << "[test_moveit_planner_mobiman::main] params: " << *i << std::endl;
    }
    */

    bool success = (moveGroupInterface.plan(mgiPlan) == moveit::core::MoveItErrorCode::SUCCESS);

    while(!success){}

    auto plan_joint_names = mgiPlan.trajectory_.joint_trajectory.joint_names;
    auto plan_joint_traj_points = mgiPlan.trajectory_.joint_trajectory.points;
    cout << "[test_moveit_planner_mobiman::main] plan_joint_names size: " << plan_joint_names.size() << std::endl;
    cout << "[test_moveit_planner_mobiman::main] plan_joint_traj_points size: " << plan_joint_traj_points.size() << std::endl;
    for (size_t i = 0; i < plan_joint_names.size(); i++)
    {
      cout << plan_joint_names[i] << endl;
    }

    for (size_t i = 0; i < plan_joint_traj_points.size(); i++)
    {
      cout << plan_joint_traj_points[i].time_from_start << endl;
    }
        

    std::cout << "[test_moveit_planner_mobiman::main] success: " << success << std::endl;

    ros::waitForShutdown();
  }
  
  std::cout << "[test_moveit_planner_mobiman::main] END" << std::endl;

  return 0;
}