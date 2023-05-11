### Description

Overview of your issue here.

### Your environment
* ROS Distro: Noetic
* OS Version: Ubuntu 20.04
* Source or Binary build? Both
* Binary release version: MoveIt 1.0 (latest)
* Source branch: origin/master

### Steps to reproduce

1. I created a MoveIt Config package using [MoveIt Setup Assistant](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html) and my robot's xacro.

2. I launched a Gazebo simulation with the robot's model and controllers.

3. I launched the `move.group.launch`. I tested to plan trajectories with different available algorithms using the RViz MotionPlanning interface.

Up to this point, everything worked well as expected.

4. I followed the [Motion Planning Pipeline](https://ros-planning.github.io/moveit_tutorials/doc/motion_planning_pipeline/motion_planning_pipeline_tutorial.html) tutorial to write a function (coding contents given below) that generates a trajectory with a desired planning algorithm (e.g. RRTstar).

Create a Planning Pipeline object:
`planning_pipeline::PlanningPipelinePtr planningPipelinePtr(new planning_pipeline::PlanningPipeline(robotModelPtr_, nh_, "ompl_interface/OMPLPlanner", "request_adapters"));`

Create a motion planning request and result objects:
```
planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;

```

Create a target pose with tolerances:
```
geometry_msgs::PoseStamped pose;
pose.header.frame_id = "virtual_world_link";
pose.pose.position.x = 3.0;
pose.pose.position.y = 0.0;
pose.pose.position.z = 1.0;
pose.pose.orientation.w = 1.0;

std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);
```

Set values for the request object, in which I set the desired planner:
```
req.group_name = "arm";
req.planner_id = "RRTstar";
req.allowed_planning_time = 60;
req.num_planning_attempts = 10;
moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(ee_frame_, pose, tolerance_pose, tolerance_angle);
req.goal_constraints.push_back(pose_goal);
```

Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world representation while planning:
```
{
    planning_scene_monitor::LockedPlanningSceneRO lscene(psmPtr_);
        
    // Now, call the pipeline and check whether planning was successful.
    planningPipelinePtr_->generatePlan(lscene, req, res);
}
```

Now, call the pipeline and check whether planning was successful:
if (res.error_code_.val != res.error_code_.SUCCESS)
{
    ROS_ERROR("[MoveitInterface::moveitPlan] Could not compute plan successfully");
    return false;
}

5. Launched 

### Expected behaviour
I was expecting to generate trajectories using the specified (e.g. RRTstar) algorithm.

### Actual behaviour
However, it generates 

### Backtrace or Console output


