#!/usr/bin/env python3

'''
LAST UPDATE: 2023.09.20

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

    action_time_horizon = rospy.get_param("action_time_horizon", 0.0)

    goal_range_min_x = rospy.get_param("goal_range_min_x", 0.0)
    goal_range_max_x = rospy.get_param('goal_range_max_x', 0.0)
    goal_range_min_y = rospy.get_param('goal_range_min_y', 0.0)
    goal_range_max_y = rospy.get_param('goal_range_max_y', 0.0)
    goal_range_min_z = rospy.get_param('goal_range_min_z', 0.0)
    goal_range_max_z = rospy.get_param('goal_range_max_z', 0.0)

    reward_terminal_goal = rospy.get_param('reward_terminal_goal', 0.0)
    reward_terminal_collision = rospy.get_param('reward_terminal_collision', 0.0)
    reward_terminal_roll = rospy.get_param('reward_terminal_roll', 0.0)
    reward_terminal_max_step = rospy.get_param('reward_terminal_max_step', 0.0)

    reward_step_goal = rospy.get_param('reward_step_goal', 0.0)
    reward_step_target = rospy.get_param('reward_step_target', 0.0)
    reward_step_mode0 = rospy.get_param('reward_step_mode0', 0.0)
    reward_step_mode1 = rospy.get_param('reward_step_mode1', 0.0)
    reward_step_mode2 = rospy.get_param('reward_step_mode2', 0.0)
    reward_step_mpc_exit = rospy.get_param('reward_step_mpc_exit', 0.0)
    reward_step_time_horizon_min = rospy.get_param('reward_step_time_horizon_min', 0.0)
    reward_step_time_horizon_max = rospy.get_param('reward_step_time_horizon_max', 0.0)

    alpha_step_goal_base = rospy.get_param('alpha_step_goal_base', 0.0)
    alpha_step_goal_ee = rospy.get_param('alpha_step_goal_ee', 0.0)
    alpha_step_target = rospy.get_param('alpha_step_target', 0.0)
    alpha_step_mode = rospy.get_param('alpha_step_mode', 0.0)
    alpha_step_mpc_exit = rospy.get_param('alpha_step_mpc_exit', 0.0)

    print("[plot_functions::__main__] save_path: " + str(save_path))

    print("[plot_functions::__main__] action_time_horizon: " + str(action_time_horizon))

    print("[plot_functions::__main__] goal_range_min_x: " + str(goal_range_min_x))
    print("[plot_functions::__main__] goal_range_max_x: " + str(goal_range_max_x))
    print("[plot_functions::__main__] goal_range_min_y: " + str(goal_range_min_y))
    print("[plot_functions::__main__] goal_range_max_y: " + str(goal_range_max_y))
    print("[plot_functions::__main__] goal_range_min_z: " + str(goal_range_min_z))
    print("[plot_functions::__main__] goal_range_max_z: " + str(goal_range_max_z))
    
    print("[plot_functions::__main__] reward_terminal_goal: " + str(reward_terminal_goal))
    print("[plot_functions::__main__] reward_terminal_collision: " + str(reward_terminal_collision))
    print("[plot_functions::__main__] reward_terminal_roll: " + str(reward_terminal_roll))
    print("[plot_functions::__main__] reward_terminal_max_step: " + str(reward_terminal_max_step))
    print("[plot_functions::__main__] reward_step_goal: " + str(reward_step_goal))
    print("[plot_functions::__main__] reward_step_target: " + str(reward_step_target))
    print("[plot_functions::__main__] reward_step_mode0: " + str(reward_step_mode0))
    print("[plot_functions::__main__] reward_step_mode1: " + str(reward_step_mode1))
    print("[plot_functions::__main__] reward_step_mode2: " + str(reward_step_mode2))
    print("[plot_functions::__main__] reward_step_mpc_exit: " + str(reward_step_mpc_exit))
    print("[plot_functions::__main__] reward_step_time_horizon_min: " + str(reward_step_time_horizon_min))
    print("[plot_functions::__main__] reward_step_time_horizon_max: " + str(reward_step_time_horizon_max))

    print("[plot_functions::__main__] alpha_step_goal_base: " + str(alpha_step_goal_base))
    print("[plot_functions::__main__] alpha_step_goal_ee: " + str(alpha_step_goal_ee))
    print("[plot_functions::__main__] alpha_step_target: " + str(alpha_step_target))
    print("[plot_functions::__main__] alpha_step_mode: " + str(alpha_step_mode))
    print("[plot_functions::__main__] alpha_step_mpc_exit: " + str(alpha_step_mpc_exit))


    n_data = 101

    # Step Reward 1: base to goal
    dist_min = 0.0
    dist_max = 5.0
    current_base_distance2goal = np.linspace(dist_min, 2*dist_max, n_data)
    reward_step_goal_base = np.zeros(n_data)
    for i in range(n_data):
        reward_step_goal_base[i] = reward_func(0, goal_range_max_x, 0, reward_step_goal, current_base_distance2goal[i])
    weighted_reward_step_goal_base = alpha_step_goal_base * reward_step_goal_base # type: ignore
    plot_func(current_base_distance2goal, reward_step_goal_base, save_path=save_path+'reward_step_goal_base.png')

    # Step Reward 2: ee to goal
    current_ee_distance2goal = np.linspace(dist_min, 2*dist_max, n_data)
    reward_step_goal_ee = np.zeros(n_data)
    for i in range(n_data):
        reward_step_goal_ee[i] = reward_func(0, goal_range_max_x, 0, reward_step_goal, current_ee_distance2goal[i])
    weighted_reward_step_goal_ee = alpha_step_goal_ee * reward_step_goal_ee # type: ignore
    plot_func(current_ee_distance2goal, reward_step_goal_base, save_path=save_path+'reward_step_goal_ee.png')

    # Step Reward 3: ee to target
    '''
    current_ee_distance2target_oridistance2target_yaw = np.linspace(dist_min, dist_max + math.pi, n_data)
    current_ee_oridistance2target_yaw = np.linspace(0, math.pi, n_data)
    current_ee_oridistance2target_quat = np.linspace(0, 1.0, n_data)
    reward_step_target_mode0 = np.zeros(n_data)
    reward_step_target_mode12 = np.zeros(n_data)
    for i in range(n_data):
        reward_step_target_mode0[i] = reward_func(0, goal_range_max_x + math.pi, 0, reward_step_target, current_ee_distance2target + current_ee_oridistance2target_yaw) # type: ignore
        reward_step_target_mode12[i] = reward_func(0, goal_range_max_x + 1.0, 0, reward_step_target, current_ee_distance2target + current_ee_oridistance2target_quat) # type: ignore
    weighted_reward_step_target_mode0 = alpha_step_target * reward_step_target_mode0 # type: ignore
    weighted_reward_step_target_mode12 = alpha_step_target * reward_step_target_mode12 # type: ignore
    plot_func(current_ee_distance2target + current_ee_oridistance2target_yaw, reward_step_target_mode0, save_path=save_path+'reward_step_target_mode0.png')
    plot_func(current_ee_distance2target + current_ee_oridistance2target_quat, weighted_reward_step_target_mode12, save_path=save_path+'weighted_reward_step_target_mode12.png')
    '''

    # Step Reward 4: model mode
    reward_step_mode0_vec = reward_step_mode0 * np.ones(n_data) # type: ignore
    reward_step_mode1_vec = reward_step_mode1 * np.ones(n_data) # type: ignore
    reward_step_mode2_vec = reward_step_mode2 * np.ones(n_data) # type: ignore
    weighted_reward_step_mode0 = alpha_step_mode * reward_step_mode0_vec # type: ignore
    weighted_reward_step_mode1 = alpha_step_mode * reward_step_mode1_vec # type: ignore
    weighted_reward_step_mode2 = alpha_step_mode * reward_step_mode2_vec # type: ignore

    # Step Reward 5: mpc result
    dt_action = np.linspace(0, 2*action_time_horizon, n_data) # type: ignore
    reward_step_mpc_exit_vec = reward_step_mpc_exit * np.ones(n_data) # type: ignore
    reward_step_mpc_target_vec = reward_step_target * np.ones(n_data) # type: ignore
    reward_step_mpc_time_horizon = np.zeros(n_data)
    coeff = reward_step_time_horizon_max / reward_step_time_horizon_min # type: ignore
    for i in range(n_data):
        reward_step_mpc_time_horizon[i] = reward_func(0, action_time_horizon + coeff * action_time_horizon, reward_step_time_horizon_min, reward_step_time_horizon_max, dt_action[i])
    weighted_reward_step_mpc_time_horizon = alpha_step_mpc_exit * reward_step_mpc_time_horizon # type: ignore
    plot_func(dt_action, reward_step_mpc_time_horizon, save_path=save_path+'reward_step_mpc_time_horizon.png')


    print("[plot_functions::__main__] END")