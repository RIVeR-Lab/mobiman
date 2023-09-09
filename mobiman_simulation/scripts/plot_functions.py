#!/usr/bin/env python3

'''
LAST UPDATE: 2023.09.08

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:
[1] 

NUA TODO:
'''

import os
import math
import json
import rospkg
import rospy
import numpy as np
import matplotlib.pyplot as plt

def plot_func(   
        data_x, data_y,
        limit_min=None, limit_max=None, 
        data_label='', data_color='r',
        label_x='', label_y='', title='', 
        save_path=''):
    
    fig = plt.figure()
    plt.plot(data_x, data_y, color=data_color, label=data_label)

    if limit_min is not None:
        plt.axhline(limit_min, color='k', linewidth=2, linestyle="--")

    if limit_max is not None:
        plt.axhline(limit_max, color='k', linewidth=2, linestyle="--")

    plt.xlabel(label_x)
    plt.ylabel(label_y)
    plt.title(title)
    plt.legend()
    plt.savefig(save_path)
    plt.close()

def reward_func(x_min, x_max, y_min, y_max, x_query):
    reward = 0
    if x_query < x_min:
        reward = y_max
    elif x_query > x_max:
        reward = y_min
    else:
        reward = (y_min - y_max) * (x_query - x_min) / (x_max - x_min) + y_max
    return reward

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    print("[plot_functions::__main__] START")

    rospy.init_node('plot_functions', anonymous=True, log_level=rospy.WARN)

    ## Get variables
    rospack = rospkg.RosPack()
    mobiman_path = rospack.get_path('mobiman_simulation') + "/"
    save_path_rel = rospy.get_param('save_path_rel', "")

    save_path = mobiman_path + save_path_rel
    isExist = os.path.exists(save_path)
    if not isExist:
        os.makedirs(save_path)

    goal_distance_threshold = rospy.get_param("goal_distance_threshold", 0.0)
    goal_range_min = rospy.get_param("goal_range_min", 0.0)
    goal_range_max_x = rospy.get_param('goal_range_max_x', 0.0)
    goal_range_max_y = rospy.get_param('goal_range_max_y', 0.0)
    goal_range_max_z = rospy.get_param('goal_range_max_z', 0.0)
    self_collision_range_min = rospy.get_param('self_collision_range_min', 0.0)
    self_collision_range_max = rospy.get_param('self_collision_range_max', 0.0)
    ext_collision_range_min = rospy.get_param('ext_collision_range_min', 0.0)
    ext_collision_range_max = rospy.get_param('ext_collision_range_max', 0.0)
    rollover_pitch_threshold = rospy.get_param('rollover_pitch_threshold', 0.0)
    rollover_roll_threshold = rospy.get_param('rollover_roll_threshold', 0.0)

    reward_terminal_goal = rospy.get_param('reward_terminal_goal', 0.0)
    reward_terminal_collision = rospy.get_param('reward_terminal_collision', 0.0)
    reward_terminal_roll = rospy.get_param('reward_terminal_roll', 0.0)
    reward_terminal_max_step = rospy.get_param('reward_terminal_max_step', 0.0)

    reward_step_goal = rospy.get_param('reward_step_goal', 0.0)
    reward_step_target = rospy.get_param('reward_step_target', 0.0)
    reward_step_mode0 = rospy.get_param('reward_step_mode0', 0.0)
    reward_step_mode1 = rospy.get_param('reward_step_mode1', 0.0)
    reward_step_mode2 = rospy.get_param('reward_step_mode2', 0.0)
    reward_step_mpc = rospy.get_param('reward_step_mpc', 0.0)

    alpha_step_goal_base = rospy.get_param('alpha_step_goal_base', 0.0)
    alpha_step_goal_ee = rospy.get_param('alpha_step_goal_ee', 0.0)
    alpha_step_target = rospy.get_param('alpha_step_target', 0.0)
    alpha_step_mode = rospy.get_param('alpha_step_mode', 0.0)
    alpha_step_mpc = rospy.get_param('alpha_step_mpc', 0.0)

    print("[plot_functions::__main__] save_path: " + str(save_path))

    print("[plot_functions::__main__] reward_terminal_goal: " + str(reward_terminal_goal))
    print("[plot_functions::__main__] reward_terminal_collision: " + str(reward_terminal_collision))
    print("[plot_functions::__main__] reward_terminal_roll: " + str(reward_terminal_roll))
    print("[plot_functions::__main__] reward_terminal_max_step: " + str(reward_terminal_max_step))
    print("[plot_functions::__main__] reward_step_goal: " + str(reward_step_goal))
    print("[plot_functions::__main__] reward_step_target: " + str(reward_step_target))
    print("[plot_functions::__main__] reward_step_mode0: " + str(reward_step_mode0))
    print("[plot_functions::__main__] reward_step_mode1: " + str(reward_step_mode1))
    print("[plot_functions::__main__] reward_step_mode2: " + str(reward_step_mode2))
    print("[plot_functions::__main__] reward_step_mpc: " + str(reward_step_mpc))
    print("[plot_functions::__main__] alpha_step_goal_base: " + str(alpha_step_goal_base))
    print("[plot_functions::__main__] alpha_step_goal_ee: " + str(alpha_step_goal_ee))
    print("[plot_functions::__main__] alpha_step_target: " + str(alpha_step_target))
    print("[plot_functions::__main__] alpha_step_mode: " + str(alpha_step_mode))
    print("[plot_functions::__main__] alpha_step_mpc: " + str(alpha_step_mpc))


    n_data = 101

    dist_min = 0.0
    dist_max = 5.0
    
    reward_min = 0.0
    reward_max = float(reward_step_goal) # type: ignore
    
    dist = np.linspace(-2*dist_max, 2*dist_max, n_data)
    
    reward = np.zeros(n_data)
    for i in range(n_data):
        reward[i] = reward_func(dist_min, dist_max, reward_min, reward_max, dist[i])


    # Step Reward: base to goal
    current_base_distance2goal = np.linspace(-2*dist_max, 2*dist_max, n_data)
    reward_step_goal_base_val = np.zeros(n_data)
    for i in range(n_data):
        reward_step_goal_base_val[i] = reward_func(0, goal_range_max_x, 0, reward_step_goal, current_base_distance2goal)
    reward_step_goal_base = alpha_step_goal_base * reward_step_goal_base_val # type: ignore

    plot_func(dist, reward, save_path=save_path+'reward_step_goal_base.png')
    #plot_func(dist, reward, save_path=save_path+'reward_step_goal_base.png')
    
    print("[plot_functions::__main__] END")