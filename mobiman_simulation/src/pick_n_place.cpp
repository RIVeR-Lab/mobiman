// LAST UPDATE: 2023.05.10
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

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//namespace rvt = rviz_visual_tools;

class MoveitInterface
{
  private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;

    ros::Publisher vis_pub_;
    ros::Publisher display_pub_;
    ros::Publisher collision_object_pub_;

    std::string base_frame_ = "base_link";
    std::string ee_frame_ = "ur5_tool0";
    std::vector<std::string> base_joint_names_;
    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> whole_body_joint_names_;

    std::string mp_group_name_base_ = "base";
    std::string mp_group_name_arm_ = "arm";
    std::string mp_group_name_wholeBody_ = "whole_body";

    moveit_visual_tools::MoveItVisualTools visual_tools_;

    planning_scene_monitor::PlanningSceneMonitorPtr psmPtr_;
    planning_pipeline::PlanningPipelinePtr planningPipelinePtr_;
    moveit::core::RobotModelPtr robotModelPtr_;
    moveit::core::RobotStatePtr robotStatePtr_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface base_move_group_interface_;
    moveit::planning_interface::MoveGroupInterface arm_move_group_interface_;
    moveit::planning_interface::MoveGroupInterface whole_body_move_group_interface_;

    moveit_msgs::DisplayTrajectory display_trajectory_;

    std::vector<moveit_msgs::CollisionObject> collision_objects_;

  public:
    MoveitInterface(ros::NodeHandle n) : nh_(n), 
                                     base_move_group_interface_(mp_group_name_base_), 
                                     arm_move_group_interface_(mp_group_name_arm_), 
                                     whole_body_move_group_interface_(mp_group_name_wholeBody_),
                                     tf2_(buffer_)
    {
      std::cout << "[MoveitInterface::MoveitInterface] START" << std::endl;

      std::cout << "[MoveitInterface::MoveitInterface] START vis_pub_" << std::endl;
      // Marker Publisher
      vis_pub_ = nh_.advertise<visualization_msgs::Marker>("moveit_marker", 10);
      display_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/mobiman/display_planned_path", 10, true);
      collision_object_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10, true);
    
      std::cout << "[MoveitInterface::MoveitInterface] START visual_tools_" << std::endl;
      // Moveit Visual Tools
      visual_tools_.setBaseFrame(base_frame_);
      visual_tools_.deleteAllMarkers();
      
      std::cout << "[MoveitInterface::MoveitInterface] START robotModelLoaderPtr" << std::endl;
      // Robot Model Loader
      robot_model_loader::RobotModelLoaderPtr robotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
      
      std::cout << "[MoveitInterface::MoveitInterface] START robotModelPtr_" << std::endl;
      // Robot Model
      robotModelPtr_ = robotModelLoaderPtr->getModel();

      std::cout << "[MoveitInterface::MoveitInterface] START robotStatePtr_" << std::endl;
      // Robot State
      moveit::core::RobotStatePtr robotStatePtr(new moveit::core::RobotState(robotModelPtr_));
      robotStatePtr_ = robotStatePtr;
      //robotStatePtr_->setToDefaultValues();
      
      std::cout << "[MoveitInterface::MoveitInterface] START psmPtr" << std::endl;
      // Planning Scene
      planning_scene_monitor::PlanningSceneMonitorPtr psmPtr(new planning_scene_monitor::PlanningSceneMonitor(robotModelLoaderPtr));
      psmPtr_ = psmPtr;

      std::cout << "[MoveitInterface::MoveitInterface] START startSceneMonitor" << std::endl;
      // listen for planning scene messages on topic /XXX and apply them to the internal planning scene accordingly
      psmPtr_->startSceneMonitor();

      std::cout << "[MoveitInterface::MoveitInterface] START startWorldGeometryMonitor" << std::endl;
      // listens to changes of world geometry, collision objects, and (optionally) octomaps
      psmPtr_->startWorldGeometryMonitor("/pick_n_place/collision_object");
      
      std::cout << "[MoveitInterface::MoveitInterface] START startStateMonitor" << std::endl;
      // listen to joint state updates as well as changes in attached collision objects and update the internal planning scene accordingly
      psmPtr_->startStateMonitor();

      std::cout << "[MoveitInterface::MoveitInterface] START planningPipelinePtr_" << std::endl;
      //Planning Pipeline
      planning_pipeline::PlanningPipelinePtr planningPipelinePtr(new planning_pipeline::PlanningPipeline(robotModelPtr_, nh_, "/move_group/planning_plugin", "request_adapters"));
      //planning_pipeline::PlanningPipelinePtr planningPipelinePtr(new planning_pipeline::PlanningPipeline(robotModelPtr_));
      planningPipelinePtr_ = planningPipelinePtr;

      //auto plannerId = whole_body_move_group_interface_.getPlannerId();
      std::cout << "[MoveitInterface::MoveitInterface] BEFORE plannerId: " << whole_body_move_group_interface_.getPlannerId() << std::endl;
      whole_body_move_group_interface_.setPlannerId("hey");
      std::cout << "[MoveitInterface::MoveitInterface] AFTER plannerId: " << whole_body_move_group_interface_.getPlannerId() << std::endl;

      /*
      std::string planner_plugin_name;
      nh_.getParam("/move_group/planning_plugin", planner_plugin_name);

      std::cout << "[MoveitInterface::MoveitInterface] BEFORE planner_plugin_name: " << planner_plugin_name << std::endl;
      planner_plugin_name = planningPipelinePtr->getPlannerPluginName();
      std::cout << "[MoveitInterface::MoveitInterface] AFTER planner_plugin_name: " << planner_plugin_name << std::endl;

      std::vector<std::string> apns = planningPipelinePtr->getAdapterPluginNames();
      std::cout << "[MoveitInterface::MoveitInterface] apns size: " << apns.size() << std::endl;
      for (auto apn=apns.begin(); apn != apns.end(); apn++)
      {
        std::cout << *apn << std::endl;
      }

      std::vector<std::string> algos;
      planningPipelinePtr_->getPlannerManager()->getPlanningAlgorithms(algos);
      std::cout << "[MoveitInterface::MoveitInterface] algos size: " << algos.size() << std::endl;
      for (auto algo=algos.begin(); algo != algos.end(); algo++)
      {
        std::cout << *algo << std::endl;
      }

      auto pcs = planningPipelinePtr_->getPlannerManager()->getPlannerConfigurations();
      std::cout << "[MoveitInterface::MoveitInterface] pcs size: " << pcs.size() << std::endl;
      for (auto pc=pcs.begin(); pc != pcs.end(); pc++)
      {
        std::cout << pc->first << std::endl;
        std::cout << pc->second.name << std::endl;
      }

      boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
      planning_interface::PlannerManagerPtr planner_instance;

      //if (!nh_.getParam("planning_plugin", planner_plugin_name))
      //  ROS_FATAL_STREAM("[MoveitInterface::MoveitInterface] Could not find planner plugin name");
      
      try
      {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
      }
      catch (pluginlib::PluginlibException& ex)
      {
        ROS_FATAL_STREAM("[MoveitInterface::MoveitInterface] Exception while creating planning plugin loader " << ex.what());
      }
      
      try
      {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robotModelPtr_, nh_.getNamespace()))
          ROS_FATAL_STREAM("[MoveitInterface::MoveitInterface] Could not initialize planner instance");
        ROS_INFO_STREAM("[MoveitInterface::MoveitInterface] Using planning interface '" << planner_instance->getDescription() << "'");

        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        
        std::cout << "classes size: " << classes.size() << std::endl;
        for (const auto& cls : classes)
          ss << cls << " ";
        std::cout << "Exception while loading planner '" << planner_plugin_name << "': " << std::endl << "Available plugins: " << ss.str() << std::endl;
      }
      catch (pluginlib::PluginlibException& ex)
      {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        
        std::cout << "classes size: " << classes.size() << std::endl;
        for (const auto& cls : classes)
          ss << cls << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                            << "Available plugins: " << ss.str());
      }
      */

      /*
      //std::cout << "[MoveitInterface::MoveitInterface] DEBUG INF" << std::endl;
      //while(1);

      // We can print the name of the reference frame for this robot.
      std::cout << "[MoveitInterface::MoveitInterface] Model frame: " << robotModelPtr_->getModelFrame().c_str() << std::endl;

      std::cout << "[MoveitInterface::MoveitInterface] Base Planning frame: " << base_move_group_interface_.getPlanningFrame().c_str() << std::endl;
      std::cout << "[MoveitInterface::MoveitInterface] Arm Planning frame: " << arm_move_group_interface_.getPlanningFrame().c_str() << std::endl;
      std::cout << "[MoveitInterface::MoveitInterface] Whole_body Planning frame: " << whole_body_move_group_interface_.getPlanningFrame().c_str() << std::endl;

      // We can get a list of all the groups in the robot:
      std::cout << "[MoveitInterface::MoveitInterface] Available WB Planning Groups:" << std::endl;
      std::copy(whole_body_move_group_interface_.getJointModelGroupNames().begin(),
                whole_body_move_group_interface_.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
      std::cout << "" << std::endl;

      std::cout << "[MoveitInterface::MoveitInterface] base_frame_ pose: " << std::endl;
      std::cout << whole_body_move_group_interface_.getCurrentPose(base_frame_) << std::endl;

      std::cout << "[MoveitInterface::MoveitInterface] ee_frame_ pose: " << std::endl;
      std::cout << whole_body_move_group_interface_.getCurrentPose(ee_frame_) << std::endl;

      const moveit::core::JointModelGroup* jointModelGroupPtrBase = robotModelPtr_->getJointModelGroup(mp_group_name_base_);
      const moveit::core::JointModelGroup* jointModelGroupPtrArm = robotModelPtr_->getJointModelGroup(mp_group_name_arm_);
      const moveit::core::JointModelGroup* jointModelGroupPtrWholeBody = robotModelPtr_->getJointModelGroup(mp_group_name_wholeBody_);

      base_joint_names_ = jointModelGroupPtrBase->getVariableNames();
      arm_joint_names_ = jointModelGroupPtrArm->getVariableNames();
      whole_body_joint_names_ = jointModelGroupPtrWholeBody->getVariableNames();

      std::vector<double> base_joint_values;
      std::vector<double> arm_joint_values;
      std::vector<double> whole_body_joint_values;

      robotStatePtr_->copyJointGroupPositions(jointModelGroupPtrBase, base_joint_values);
      robotStatePtr_->copyJointGroupPositions(jointModelGroupPtrArm, arm_joint_values);
      robotStatePtr_->copyJointGroupPositions(jointModelGroupPtrWholeBody, whole_body_joint_values);

      for (std::size_t i = 0; i < this->base_joint_names_.size(); ++i)
      {
        std::cout << "[MoveitInterface::MoveitInterface] Base Joint " << this->base_joint_names_[i].c_str() << ": " << base_joint_values[i] << std::endl;
      }
      std::cout << "" << std::endl;

      for (std::size_t i = 0; i < this->arm_joint_names_.size(); ++i)
      {
        std::cout << "[MoveitInterface::MoveitInterface] Arm Joint " << this->arm_joint_names_[i].c_str() << ": " << arm_joint_values[i] << std::endl;
      }
      std::cout << "" << std::endl;

      for (std::size_t i = 0; i < this->whole_body_joint_names_.size(); ++i)
      {
        std::cout << "[MoveitInterface::MoveitInterface] WB Joint " << this->whole_body_joint_names_[i].c_str() << ": " << whole_body_joint_values[i] << std::endl;
      }
      std::cout << "" << std::endl;

      base_move_group_interface_.setPoseReferenceFrame("virtual_world_link");
      base_move_group_interface_.setEndEffectorLink(base_frame_);
      base_move_group_interface_.setNumPlanningAttempts(10);

      arm_move_group_interface_.setPoseReferenceFrame("virtual_world_link");
      arm_move_group_interface_.setEndEffectorLink(ee_frame_);
      arm_move_group_interface_.setNumPlanningAttempts(10);
      arm_move_group_interface_.setMaxVelocityScalingFactor(0.25);
      arm_move_group_interface_.setMaxAccelerationScalingFactor(0.25);
      arm_move_group_interface_.setPlanningTime(60);

      whole_body_move_group_interface_.setPoseReferenceFrame("virtual_world_link");
      whole_body_move_group_interface_.setEndEffectorLink(ee_frame_);
      whole_body_move_group_interface_.setNumPlanningAttempts(10);
      whole_body_move_group_interface_.setPlannerId("RRTstar");
      
      std::cout << "[MoveitInterface::MoveitInterface] base getPoseReferenceFrame: " << base_move_group_interface_.getPoseReferenceFrame().c_str() << std::endl;
      std::cout << "[MoveitInterface::MoveitInterface] arm getPoseReferenceFrame: " << arm_move_group_interface_.getPoseReferenceFrame().c_str() << std::endl;
      std::cout << "[MoveitInterface::MoveitInterface] whole_body getPoseReferenceFrame: " << whole_body_move_group_interface_.getPoseReferenceFrame().c_str() << std::endl;
      */

      std::cout << "[MoveitInterface::MoveitInterface] END" << std::endl;
    };

    ~MoveitInterface(){} 
    
    bool moveitPlanTrajectory()
    {
      std::cout << "[MoveitInterface::moveitPlanTrajectory] START" << std::endl;

      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "virtual_world_link";
      pose.pose.position.x = 3.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 1.0;
      pose.pose.orientation.w = 1.0;

      std::vector<double> tolerance_pose(3, 0.01);
      std::vector<double> tolerance_angle(3, 0.01);

      // mp_group_name_base_
      // mp_group_name_arm_
      // mp_group_name_wholeBody_
      req.group_name = mp_group_name_wholeBody_;
      req.planner_id = "RRTstar";
      req.allowed_planning_time = 60;
      req.num_planning_attempts = 1000;
      //req.workspace_parameters.max_corner = {};
      //req.workspace_parameters.min_corner = {};
      moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(ee_frame_, pose, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal);

      std::cout << "[MoveitInterface::moveitPlanTrajectory] BEFORE generatePlan" << std::endl;
      // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
      // representation while planning
      {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psmPtr_);
        
        // Now, call the pipeline and check whether planning was successful.
        planningPipelinePtr_->generatePlan(lscene, req, res);
      }
      /* Now, call the pipeline and check whether planning was successful. */
      /* Check that the planning was successful */
      if (res.error_code_.val != res.error_code_.SUCCESS)
      {
        ROS_ERROR("[MoveitInterface::moveitPlan] Could not compute plan successfully");

        std::cout << "[MoveitInterface::moveitPlan] DEBUG INF" << std::endl;
        while(1);
        return false;
      }
      std::cout << "[MoveitInterface::moveitPlanTrajectory] AFTER generatePlan" << std::endl;

      /* Visualize the trajectory */
      std::cout << "[MoveitInterface::moveitPlanTrajectory] Visualizing the trajectory" << std::endl;
      moveit_msgs::MotionPlanResponse response;
      res.getMessage(response);

      //const moveit::core::JointModelGroup* jointModelGroupPtrBase = robotModelPtr_->getJointModelGroup(mp_group_name_base_);
      //const moveit::core::JointModelGroup* jointModelGroupPtrArm = robotModelPtr_->getJointModelGroup(mp_group_name_arm_);
      //const moveit::core::JointModelGroup* jointModelGroupPtrWholeBody = robotModelPtr_->getJointModelGroup(mp_group_name_wholeBody);

      display_trajectory_.trajectory_start = response.trajectory_start;
      display_trajectory_.trajectory.clear();
      display_trajectory_.trajectory.push_back(response.trajectory);
      
      std::cout << "[MoveitInterface::moveitPlanTrajectory] END" << std::endl;

      return true;
    }

    void publishTrajectory()
    {
      const moveit::core::JointModelGroup* jointModelGroupPtrWholeBody = robotModelPtr_->getJointModelGroup(mp_group_name_wholeBody_);

      display_pub_.publish(display_trajectory_);
      visual_tools_.publishTrajectoryLine(display_trajectory_.trajectory.back(), jointModelGroupPtrWholeBody);
    }

    void addCollisionObjects()
    {
      std::cout << "[MoveitInterface::addCollisionObjects] START" << std::endl;

      collision_objects_.clear();
      collision_objects_.resize(2);

      // Add the first table where the cube will originally be kept.
      collision_objects_[0].id = "table1";
      collision_objects_[0].header.frame_id = "virtual_world_link";

      // Define the primitive and its dimensions.
      collision_objects_[0].primitives.resize(1);
      collision_objects_[0].primitives[0].type = collision_objects_[0].primitives[0].BOX;
      collision_objects_[0].primitives[0].dimensions.resize(3);
      collision_objects_[0].primitives[0].dimensions[0] = 0.2;
      collision_objects_[0].primitives[0].dimensions[1] = 1.4;
      collision_objects_[0].primitives[0].dimensions[2] = 0.4;

      // Define the pose of the table.
      collision_objects_[0].primitive_poses.resize(1);
      collision_objects_[0].primitive_poses[0].position.x = 1.0;
      collision_objects_[0].primitive_poses[0].position.y = 0.0;
      collision_objects_[0].primitive_poses[0].position.z = 0.2;
      collision_objects_[0].primitive_poses[0].orientation.x = 0.0;
      collision_objects_[0].primitive_poses[0].orientation.y = 0.0;
      collision_objects_[0].primitive_poses[0].orientation.z = 0.0;
      collision_objects_[0].primitive_poses[0].orientation.w = 1.0;

      collision_objects_[0].operation = collision_objects_[0].ADD;

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
      collision_objects_[1].header.frame_id = "virtual_world_link";
      collision_objects_[1].id = "object";

      // Define the primitive and its dimensions.
      collision_objects_[1].primitives.resize(1);
      collision_objects_[1].primitives[0].type = collision_objects_[1].primitives[0].BOX;
      collision_objects_[1].primitives[0].dimensions.resize(3);
      collision_objects_[1].primitives[0].dimensions[0] = 0.02;
      collision_objects_[1].primitives[0].dimensions[1] = 0.02;
      collision_objects_[1].primitives[0].dimensions[2] = 0.2;

      // Define the pose of the object.
      collision_objects_[1].primitive_poses.resize(1);
      collision_objects_[1].primitive_poses[0].position.x = 1.0;
      collision_objects_[1].primitive_poses[0].position.y = 0.0;
      collision_objects_[1].primitive_poses[0].position.z = 0.5;
      collision_objects_[1].primitive_poses[0].orientation.x = 0.0;
      collision_objects_[1].primitive_poses[0].orientation.y = 0.0;
      collision_objects_[1].primitive_poses[0].orientation.z = 0.0;
      collision_objects_[1].primitive_poses[0].orientation.w = 1.0;

      collision_objects_[1].operation = collision_objects_[1].ADD;

      planning_scene_interface_.applyCollisionObjects(collision_objects_);
      publishCollisionObjects();

      std::cout << "[MoveitInterface::addCollisionObjects] END" << std::endl;
    }

    void publishCollisionObjects()
    {
      //std::cout << "[MoveitInterface::publishCollisionObjects] START" << std::endl;

      for (size_t i = 0; i < collision_objects_.size(); i++)
      {
        collision_objects_[i].header.stamp = ros::Time::now();
        collision_object_pub_.publish(collision_objects_[i]);
      }

      //std::cout << "[MoveitInterface::publishCollisionObjects] END" << std::endl;
    }

    /*
    void openGripper()
    {is:issue is:open joint_values;
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

    /*
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
      std::cout << "[pick_n_place::pick] Position Goal Tolerance: " << arm_move_group_interface_.getGoalPositionTolerance() << std::endl;
      std::cout << "[pick_n_place::pick] Orientation Goal Tolerance: " << arm_move_group_interface_.getGoalOrientationTolerance() << std::endl;

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
      if (success)
      {
        std::cout << "[pick_n_place::fake_pick] Fake Pick IK Planner: SUCCEED" << std::endl;
      }
      else
      {
        std::cout << "[pick_n_place::fake_pick] Fake Pick IK Planner: FAILED" << std::endl;
      }
      
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
      if (success)
      {
        std::cout << "[pick_n_place::another_pick] Arm Planner: SUCCEED" << std::endl;
      }
      else
      {
        std::cout << "[pick_n_place::another_pick] Arm Planner: FAILED" << std::endl;
      }
      this->arm_move_group_interface_.move();
    }

    void another_place()
    {
      geometry_msgs::Pose target_pose;
      target_pose.position.x = 1.0;
      target_pose.position.y = 0.0;
      target_pose.position.z = 0.5;
      target_pose.orientation.w = 1.0;
      this->arm_move_group_interface_.setPoseReferenceFrame("base_link");
      bool success = this->arm_move_group_interface_.setApproximateJointValueTarget(target_pose, "link_grasp_center");
      if (success)
      {
        std::cout << "[pick_n_place::another_place] Arm Planner: SUCCEED" << std::endl;
      }
      else
      {
        std::cout << "[pick_n_place::another_place] Arm Planner: FAILED" << std::endl;
      }
      this->arm_move_group_interface_.move();
    }
    */
};

int main(int argc, char **argv)
{
  std::cout << "[pick_n_place::main] START" << std::endl;

  ros::init(argc, argv, "pick_n_place");
  ros::NodeHandle node_handle("~");
  
  ros::AsyncSpinner spinner(4);
  spinner.start();

  MoveitInterface demo(node_handle);
  demo.addCollisionObjects();

  // Wait a bit for ROS things to initialize
  ros::WallDuration(2.0).sleep();

  ///*
  std::cout << "[pick_n_place::main] START moveitPlanTrajectory" << std::endl;
  demo.moveitPlanTrajectory();
  std::cout << "[pick_n_place::main] END moveitPlanTrajectory" << std::endl;

  demo.publishTrajectory();
  std::cout << "[pick_n_place::main] END publishTrajectory" << std::endl;
  //*/

  while(ros::ok())
  {
    //demo.publishCollisionObjects();
    ros::spinOnce();
  }


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