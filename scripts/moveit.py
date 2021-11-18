#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import rospy
import moveit_commander


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_planner", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "stretch_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0.9967
joint_goal[1] = 0.13
joint_goal[2] = 0.13
joint_goal[3] = 0.13
joint_goal[4] = 0.13
joint_goal[5] = 4.0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

rospy.sleep(3)

joint_goal[0] = 0.1
joint_goal[1] = 0.0
joint_goal[2] = 0.0
joint_goal[3] = 0.0
joint_goal[4] = 0.0
joint_goal[5] = 0.0

move_group.go(joint_goal, wait=True)
move_group.stop()