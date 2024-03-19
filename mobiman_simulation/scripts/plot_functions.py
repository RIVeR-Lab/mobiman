#!/usr/bin/env python3

'''
LAST UPDATE: 2024.03.14

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
import scipy.stats as stats

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
    plt.grid()
    plt.legend()
    plt.savefig(save_path)
    plt.close()

def plot_func_multi(   
        data_x_multi, data_y_multi,
        data_label_multi, data_color='r',
        label_x='', label_y='', title='', 
        save_path=''):
    
    print("len data_x_multi: " + str(len(data_x_multi)))
    print("len data_y_multi: " + str(len(data_y_multi)))
    print("len data_label_multi: " + str(len(data_label_multi)))

    fig = plt.figure()
    for i, dx in enumerate(data_x_multi):
        plt.plot(dx, data_y_multi[i], label=data_label_multi[i])

    plt.xlabel(label_x)
    plt.ylabel(label_y)
    plt.title(title)
    plt.grid()
    plt.legend()
    plt.savefig(save_path)
    plt.close()

def plot_bar(   
        data_x, data_y,
        data_label='', data_color='r',
        label_x='', label_y='', title='', 
        save_path=''):
    
    fig = plt.figure()
    plt.bar(data_x, data_y, color=data_color, label=data_label)

    plt.xlabel(label_x)
    plt.ylabel(label_y)
    plt.title(title)
    plt.grid()
    plt.legend()
    plt.savefig(save_path)
    plt.close()

'''
DESCRIPTION: TODO...
'''
def linear_function(x_min, x_max, y_min, y_max, query_x, slope_sign=1):
    if x_min <= query_x <= x_max:
        slope = slope_sign * (y_max - y_min) / (x_max - x_min)
        y_intercept = y_max - slope * x_min
        return slope * query_x + y_intercept
    else:
        if query_x < x_min:
            if slope_sign < 0:
                return y_max
            else:
                return y_min
        else:
            if slope_sign < 0:
                return y_min
            else:
                return y_max

'''
DESCRIPTION: TODO...
'''     
def exponential_function(x, gamma, exp_factor=0):
    alpha = 1
    if exp_factor > 0:
        alpha = 1 / (np.exp(exp_factor))
    return alpha * np.exp(gamma * x) # type: ignore


'''
DESCRIPTION: TODO...
'''     
def sigmoid_function(x, gamma):
    return (1 / (1 + np.exp(-gamma * x))) # type: ignore

'''
DESCRIPTION: TODO...
'''
def gaussian_function(x, sigma):
    """ Return the scaled Gaussian with standard deviation sigma. """
    gaussian = np.exp(- (x / sigma)**2)
    scaled_result = 2 * gaussian - 1
    return scaled_result

'''
DESCRIPTION: TODO...
'''
def reward_step_target2goal_diff_func(curr_target2goal, prev_target2goal, reward_step_target2goal, reward_step_target2goal_mu, reward_step_target2goal_sigma):
    diff_target2goal = prev_target2goal - curr_target2goal
    reward_step_target2goal = reward_step_target2goal * gaussian_function(diff_target2goal-reward_step_target2goal_mu, reward_step_target2goal_sigma)
    return reward_step_target2goal

'''
DESCRIPTION: TODO...
'''
def reward_step_target2goal_curr_func(curr_target2goal, reward_step_target2goal, reward_step_target2goal_mu, reward_step_target2goal_sigma):
    reward_step_target2goal = reward_step_target2goal * gaussian_function(curr_target2goal-reward_step_target2goal_mu, reward_step_target2goal_sigma)
    return reward_step_target2goal

'''
DESCRIPTION: TODO...
'''
def reward_step_time_horizon_func(dt_action, action_time_horizon, reward_step_time_horizon_max, reward_step_time_horizon_min):
    reward_step_mpc_time_horizon = 0
    if dt_action <= action_time_horizon:
        reward_step_mpc_time_horizon = linear_function(0.0, action_time_horizon, 
                                                       0.0, reward_step_time_horizon_max, 
                                                       dt_action, slope_sign=-1) # type: ignore
    elif dt_action <= 2*action_time_horizon: # type: ignore
        reward_step_mpc_time_horizon = linear_function(action_time_horizon, 2*action_time_horizon,  # type: ignore
                                                       reward_step_time_horizon_min, 0.0, 
                                                       dt_action, slope_sign=-1) # type: ignore
    else:
        if  dt_action > 2*action_time_horizon: # type: ignore
            reward_step_mpc_time_horizon = reward_step_time_horizon_min
        else:
            reward_step_mpc_time_horizon = reward_step_time_horizon_max
    return reward_step_mpc_time_horizon

'''
DESCRIPTION: TODO...
'''
def example_reward_shaping_v0(save_path):

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

    reward_step_target2goal = rospy.get_param('reward_step_target2goal', 0.0)
    reward_step_target2goal_mu_regular = rospy.get_param('reward_step_target2goal_mu_regular', 0.0)
    reward_step_target2goal_sigma_regular = rospy.get_param('reward_step_target2goal_sigma_regular', 0.0)
    reward_step_target2goal_mu_last_step = rospy.get_param('reward_step_target2goal_mu_last_step', 0.0)
    reward_step_target2goal_sigma_last_step = rospy.get_param('reward_step_target2goal_sigma_last_step', 0.0)
    reward_step_mode0 = rospy.get_param('reward_step_mode0', 0.0)
    reward_step_mode1 = rospy.get_param('reward_step_mode1', 0.0)
    reward_step_mode2 = rospy.get_param('reward_step_mode2', 0.0)
    reward_step_mpc_exit = rospy.get_param('reward_step_mpc_exit', 0.0)
    reward_step_time_horizon_min = rospy.get_param('reward_step_time_horizon_min', 0.0)
    reward_step_time_horizon_max = rospy.get_param('reward_step_time_horizon_max', 0.0)

    alpha_step_goal_base = rospy.get_param('alpha_step_goal_base', 0.0)
    alpha_step_goal_ee = rospy.get_param('alpha_step_goal_ee', 0.0)
    alpha_step_target_pos = rospy.get_param('alpha_step_target_pos', 0.0)
    alpha_step_target_ori = rospy.get_param('alpha_step_target_ori', 0.0)
    alpha_step_mode = rospy.get_param('alpha_step_mode', 0.0)
    alpha_step_mpc_result = rospy.get_param('alpha_step_mpc_result', 0.0)

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
    print("[plot_functions::__main__] reward_step_target2goal: " + str(reward_step_target2goal))
    print("[plot_functions::__main__] reward_step_target2goal_mu_regular: " + str(reward_step_target2goal_mu_regular))
    print("[plot_functions::__main__] reward_step_target2goal_sigma_regular: " + str(reward_step_target2goal_sigma_regular))
    print("[plot_functions::__main__] reward_step_target2goal_mu_last_step: " + str(reward_step_target2goal_mu_last_step))
    print("[plot_functions::__main__] reward_step_target2goal_sigma_last_step: " + str(reward_step_target2goal_sigma_last_step))
    print("[plot_functions::__main__] reward_step_mode0: " + str(reward_step_mode0))
    print("[plot_functions::__main__] reward_step_mode1: " + str(reward_step_mode1))
    print("[plot_functions::__main__] reward_step_mode2: " + str(reward_step_mode2))
    print("[plot_functions::__main__] reward_step_mpc_exit: " + str(reward_step_mpc_exit))
    print("[plot_functions::__main__] reward_step_time_horizon_min: " + str(reward_step_time_horizon_min))
    print("[plot_functions::__main__] reward_step_time_horizon_max: " + str(reward_step_time_horizon_max))

    print("[plot_functions::__main__] alpha_step_goal_base: " + str(alpha_step_goal_base))
    print("[plot_functions::__main__] alpha_step_goal_ee: " + str(alpha_step_goal_ee))
    print("[plot_functions::__main__] alpha_step_target_pos: " + str(alpha_step_target_pos))
    print("[plot_functions::__main__] alpha_step_target_ori: " + str(alpha_step_target_ori))
    print("[plot_functions::__main__] alpha_step_mode: " + str(alpha_step_mode))
    print("[plot_functions::__main__] alpha_step_mpc_result: " + str(alpha_step_mpc_result))

    n_data = 101
    dist_min = 0.0
    dist_max = 10.0

    x_data = np.arange(n_data)

    ### ACTIVE:
    extra_tag = "active_"

    # Step Reward 1: target to goal (considers both "previous vs. current" and "current target to goal")
    curr_target2goal = np.zeros(n_data)
    prev_target2goal = 10 * np.ones(n_data)
    diff_target2goal = 10 * np.linspace(-1.0, 1.0, n_data) # type: ignore

    reward_step_target2goal_regular = np.zeros(n_data)
    reward_step_target2goal_last_step = np.zeros(n_data)

    for i in range(n_data):
        curr_target2goal[i] = prev_target2goal[i] - diff_target2goal[i]

        reward_step_target2goal_regular[i] = reward_step_target2goal_diff_func(curr_target2goal[i], prev_target2goal[i], reward_step_target2goal, reward_step_target2goal_mu_regular, reward_step_target2goal_sigma_regular)
        reward_step_target2goal_last_step[i] = reward_step_target2goal_curr_func(curr_target2goal[i], reward_step_target2goal, reward_step_target2goal_mu_last_step, reward_step_target2goal_sigma_last_step)

    title = "Step Reward 1: target to goal \n (considers both \"previous vs. current\" and \"current target to goal\") \n diff_target2goal range = [" + str(diff_target2goal[0]) + ", " + str(diff_target2goal[-1]) + "], prev_target = " + str(prev_target2goal[0]) # type: ignore
    plot_func(curr_target2goal, reward_step_target2goal_regular, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_regular", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_regular_wrt_curr.png')
    plot_func(diff_target2goal, reward_step_target2goal_regular, 
              label_x="diff_target2goal [m]", label_y="reward_step_target2goal_regular", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_regular_wrt_diff.png')
    plot_func(curr_target2goal, reward_step_target2goal_last_step, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_last_step", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_last_step_wrt_curr.png')
    plot_func(diff_target2goal, reward_step_target2goal_last_step, 
              label_x="diff_target2goal [m]", label_y="reward_step_target2goal_last_step", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_last_step_wrt_diff.png')
    
    # Step Reward 2: model mode
    reward_step_mode0_vec = reward_step_mode0 * np.ones(n_data) # type: ignore
    reward_step_mode1_vec = reward_step_mode1 * np.ones(n_data) # type: ignore
    reward_step_mode2_vec = reward_step_mode2 * np.ones(n_data) # type: ignore
    weighted_reward_step_mode0 = alpha_step_mode * reward_step_mode0_vec # type: ignore
    weighted_reward_step_mode1 = alpha_step_mode * reward_step_mode1_vec # type: ignore
    weighted_reward_step_mode2 = alpha_step_mode * reward_step_mode2_vec # type: ignore

    title = "Step Reward 2: model mode"
    plot_bar(["mode0\n"+str(reward_step_mode0), "mode1\n"+str(reward_step_mode1), "mode2\n"+str(reward_step_mode2)], [reward_step_mode0, reward_step_mode1, reward_step_mode2],
             label_x="Model modes", label_y="reward_step_mode", 
             title=title, save_path=save_path+extra_tag+'reward_step_mode.png')

    # Step Reward 3: mpc result : time horizon
    dt_action = np.linspace(0, 3*action_time_horizon, n_data) # type: ignore
    reward_step_time_horizon = np.zeros(n_data) # type: ignore
    for i in range(n_data):
        reward_step_time_horizon[i] = reward_step_time_horizon_func(dt_action[i], action_time_horizon, reward_step_time_horizon_max, reward_step_time_horizon_min)
    weighted_reward_step_mpc_time_horizon = alpha_step_mpc_result * reward_step_time_horizon # type: ignore

    title = "Step Reward 3: mpc result : time horizon"
    plot_func(dt_action, reward_step_time_horizon, 
              label_x="dt_action [s]", label_y="reward_step_time_horizon", 
              title=title, save_path=save_path+extra_tag+'reward_step_time_horizon.png')

def example_reward_shaping_v1(save_path):

    print("[plot_functions::example_reward_shaping_v1] START")

    print("[plot_functions::example_reward_shaping_v1] save_path: " + str(save_path))

    alpha = 1
    gamma = np.linspace(-5.0, -2.0, 4)
    n_data = 100
    
    reward_target_multi = []
    diff_target_multi = []
    label_y_multi = []
    for i in range(gamma.size):
        diff_target = np.linspace(0.0, 1.0, n_data)
        reward_target = np.zeros(n_data)
        for j in range(n_data):
            reward_target[j] = alpha * exponential_function(diff_target[j], gamma[i])
        
        diff_target_multi.append(diff_target)
        reward_target_multi.append(reward_target)
        label_y_multi.append('gamma: ' + str(gamma[i]))

    plot_func_multi(   
        diff_target_multi, reward_target_multi,
        data_label_multi=label_y_multi,
        label_x='diff_T_R1', label_y='reward_step_target', title='Target Step Reward Function\n(alpha = ' + str(alpha) + ')', 
        save_path=save_path+'reward_step_target_gamma.png')

    print("[plot_functions::example_reward_shaping_v1] END")

def example_reward_shaping_v2(save_path):

    print("[plot_functions::example_reward_shaping_v2] START")

    print("[plot_functions::example_reward_shaping_v2] save_path: " + str(save_path))

    alpha = -1
    gamma = np.linspace(2.0, 5.0, 4)
    n_data = 100

    reward_target_multi = []
    diff_target_multi = []
    label_y_multi = []
    for i in range(gamma.size):
        diff_target = np.linspace(0.0, 1.0, n_data)
        reward_target = np.zeros(n_data)
        for j in range(n_data):
            reward_target[j] = alpha * exponential_function(diff_target[j], gamma[i], exp_factor=gamma[i])
        
        diff_target_multi.append(diff_target)
        reward_target_multi.append(reward_target)
        label_y_multi.append('gamma: ' + str(gamma[i]))

    plot_func_multi(   
        diff_target_multi, reward_target_multi,
        data_label_multi=label_y_multi,
        label_x='diff_G_R1', label_y='penalty_step_target', title='Target Step Penalty Function\n(alpha = ' + str(alpha) + ')', 
        save_path=save_path+'reward_step_penalty_gamma.png')

    print("[plot_functions::example_reward_shaping_v2] END")

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

    example_reward_shaping_v1(save_path)
    example_reward_shaping_v2(save_path)

    print("[plot_functions::__main__] END")