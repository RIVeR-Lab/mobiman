#!/usr/bin/env python3

'''
LAST UPDATE: 2023.09.28

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
def sigmoid_function(x, gamma):
    return (1 / (1 + np.exp(-gamma*x)))

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

    reward_step_target2goal = rospy.get_param('reward_step_target2goal', 0.0)
    reward_step_target2goal_threshold = rospy.get_param('reward_step_target2goal_threshold', 0.0)
    reward_step_target2goal_scale_beta = rospy.get_param('reward_step_target2goal_scale_beta', 0.0)
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
    print("[plot_functions::__main__] reward_step_target2goal: " + str(reward_step_target2goal))
    print("[plot_functions::__main__] reward_step_target2goal_threshold: " + str(reward_step_target2goal_threshold))
    print("[plot_functions::__main__] reward_step_target2goal_scale_beta: " + str(reward_step_target2goal_scale_beta))
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
    print("[plot_functions::__main__] alpha_step_mpc_exit: " + str(alpha_step_mpc_exit))

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
    
    scale_data = np.zeros(n_data)
    reward_step_target2goal_data = np.zeros(n_data)
    reward_step_target2goal_data_exp_curr = np.zeros(n_data)
    reward_step_target2goal_scaled_data = np.zeros(n_data)
    reward_step_target2goal_data_analytical = np.zeros(n_data)
    reward_step_target2goal_data_sigmoid_diff = np.zeros(n_data)
    reward_step_target2goal_data_sigmoid_curr = np.zeros(n_data)
    reward_step_target2goal_data_sigmoid_scaled = np.zeros(n_data)
    reward_step_target2goal_data_sigmoid_scaled_exp = np.zeros(n_data)

    for i in range(n_data):
        curr_target2goal[i] = prev_target2goal[i] - diff_target2goal[i]

        if (curr_target2goal[i] <= 2*reward_step_target2goal_threshold): # type: ignore
            reward_step_target2goal_data[i] = linear_function(0, 2*reward_step_target2goal_threshold, 0, reward_step_target2goal, curr_target2goal[i], -1) # type: ignore
        else:
            reward_step_target2goal_data[i] = linear_function(2*reward_step_target2goal_threshold, 4*reward_step_target2goal_threshold, -reward_step_target2goal, 0.0, curr_target2goal[i], -1) # type: ignore

        reward_step_target2goal_data_sigmoid_diff[i] = 2 * reward_step_target2goal * sigmoid_function(diff_target2goal[i], reward_step_target2goal_threshold) - reward_step_target2goal # type: ignore

        reward_step_target2goal_data_exp_curr[i] = np.exp(-curr_target2goal[i])
        #reward_step_target2goal_data_exp_curr[i] = 0.5 * reward_step_target2goal * np.exp(-curr_target2goal[i])

        #scale2 = reward_step_target2goal_data_exp_curr[i] / 

        scale = 1
        scale2 = 1
        diff_target2goal_abs = abs(diff_target2goal[i])
        if diff_target2goal_abs < 2 * reward_step_target2goal_threshold: # type: ignore
            scale = (diff_target2goal_abs + reward_step_target2goal_scale_beta *reward_step_target2goal_threshold) / (3*reward_step_target2goal_threshold) # type: ignore
            scale2 = np.exp(-curr_target2goal[i])

        scale_data[i] = scale
        reward_step_target2goal_scaled_data[i] = scale * pow(-reward_step_target2goal_data[i], 3) / pow(reward_step_target2goal, 2) # type: ignore
        reward_step_target2goal_data_sigmoid_scaled[i] = scale * reward_step_target2goal_data_sigmoid_diff[i] # type: ignore
        reward_step_target2goal_data_sigmoid_scaled_exp[i] = scale2 * reward_step_target2goal_data_sigmoid_diff[i] # type: ignore

        if curr_target2goal[i] > 2*reward_step_target2goal_threshold: # type: ignore
            reward_step_target2goal_data_analytical[i] = scale * pow(-reward_step_target2goal, 3) / pow(reward_step_target2goal, 2) # type: ignore
        else:
            reward_step_target2goal_data_analytical[i] = scale * pow(-reward_step_target2goal * (curr_target2goal[i] - reward_step_target2goal_threshold) / reward_step_target2goal_threshold, 3) / pow(reward_step_target2goal, 2) # type: ignore

    #plot_func(prev_target2goal, reward_step_target2goal_data, save_path=save_path+extra_tag+'reward_step_target2goal_wrt_prev_target2goal.png')
    title = "Step Reward 1: target to goal \n (considers both \"previous vs. current\" and \"current target to goal\") \n diff_target2goal range = [" + str(diff_target2goal[0]) + ", " + str(diff_target2goal[-1]) + "], prev_target = " + str(prev_target2goal[0]) # type: ignore
    plot_func(curr_target2goal, reward_step_target2goal_data, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_data", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data.png')
    plot_func(curr_target2goal, reward_step_target2goal_data_exp_curr, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_data_exp_curr", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_exp_curr.png')
    plot_func(diff_target2goal, reward_step_target2goal_data_sigmoid_diff, 
              label_x="diff_target2goal [m]", label_y="reward_step_target2goal_data_sigmoid_diff", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_sigmoid_diff.png')
    plot_func(diff_target2goal, reward_step_target2goal_scaled_data, 
              label_x="diff_target2goal [m]", label_y="reward_step_target2goal_scaled_data", 
              title="", save_path=save_path+extra_tag+'reward_step_target2goal_scaled_data_wrt_diff.png')
    plot_func(curr_target2goal, reward_step_target2goal_scaled_data, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_scaled_data", 
              title="", save_path=save_path+extra_tag+'reward_step_target2goal_scaled_data_wrt_curr.png')
    plot_func(diff_target2goal, scale_data, 
              label_x="diff_target2goal [m]", label_y="scale_data", 
              title=title, save_path=save_path+extra_tag+'scale_data_wrt_diff.png')
    plot_func(curr_target2goal, scale_data, 
              label_x="curr_target2goal [m]", label_y="scale_data", 
              title=title, save_path=save_path+extra_tag+'scale_data_wrt_curr.png')
    plot_func(diff_target2goal, reward_step_target2goal_data_analytical, 
              label_x="diff_target2goal [m]", label_y="reward_step_target2goal_data_analytical", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_analytical_wrt_diff.png')
    plot_func(curr_target2goal, reward_step_target2goal_data_analytical, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_data_analytical", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_analytical_wrt_curr.png')
    plot_func(diff_target2goal, reward_step_target2goal_data_sigmoid_scaled, 
              label_x="diff_target2goal [m]", label_y="reward_step_target2goal_data_sigmoid_scaled", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_sigmoid_scaled_wrt_diff.png')
    plot_func(curr_target2goal, reward_step_target2goal_data_sigmoid_scaled, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_data_sigmoid_scaled", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_sigmoid_scaled_wrt_curr.png')
    
    plot_func(diff_target2goal, reward_step_target2goal_data_sigmoid_scaled_exp, 
              label_x="diff_target2goal [m]", label_y="reward_step_target2goal_data_sigmoid_scaled_exp", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_sigmoid_scaled_exp_wrt_diff.png')
    plot_func(curr_target2goal, reward_step_target2goal_data_sigmoid_scaled_exp, 
              label_x="curr_target2goal [m]", label_y="reward_step_target2goal_data_sigmoid_scaled", 
              title=title, save_path=save_path+extra_tag+'reward_step_target2goal_data_sigmoid_scaled_exp_wrt_curr.png')
    
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
    reward_step_mpc_time_horizon = np.zeros(n_data) # type: ignore
    reward_step_mpc_time_horizon_plus = np.zeros(int(0.5*n_data)+1) # type: ignore
    reward_step_mpc_time_horizon_minus = np.zeros(int(0.5*n_data)) # type: ignore
    coeff = reward_step_time_horizon_max / reward_step_time_horizon_min # type: ignore
    for i in range(n_data):
        if dt_action[i] <= action_time_horizon:
            reward_step_mpc_time_horizon[i] = linear_function(0, action_time_horizon, 0.0, reward_step_time_horizon_max, dt_action[i], -1)
        elif dt_action[i] <= 2*action_time_horizon: # type: ignore
            reward_step_mpc_time_horizon[i] = linear_function(action_time_horizon, 2*action_time_horizon, reward_step_time_horizon_min, 0.0, dt_action[i], -1) # type: ignore
        else:
            if  dt_action[i] > 2*action_time_horizon: # type: ignore
                reward_step_mpc_time_horizon[i] = reward_step_time_horizon_min
            else:
                reward_step_mpc_time_horizon[i] = reward_step_time_horizon_max
    weighted_reward_step_mpc_time_horizon = alpha_step_mpc_exit * reward_step_mpc_time_horizon # type: ignore

    title = "Step Reward 3: mpc result : time horizon"
    plot_func(dt_action, reward_step_mpc_time_horizon, 
              label_x="dt_action [s]", label_y="reward_step_mpc_time_horizon", 
              title=title, save_path=save_path+extra_tag+'reward_step_mpc_time_horizon.png')

    ### OTHER IDEAS:
    extra_tag = "other_"

    # Step Reward 1: base to goal
    current_base_distance2goal = np.linspace(dist_min, 2*dist_max, n_data)
    reward_step_goal_base = np.zeros(n_data)
    for i in range(n_data):
        reward_step_goal_base[i] = linear_function(0, goal_range_max_x, 0, reward_step_target2goal, current_base_distance2goal[i], -1)
    weighted_reward_step_goal_base = alpha_step_goal_base * reward_step_goal_base # type: ignore
    plot_func(current_base_distance2goal, reward_step_goal_base, save_path=save_path+extra_tag+'reward_step_goal_base.png')

    # Step Reward 2: ee to goal
    current_ee_distance2goal = np.linspace(dist_min, 2*dist_max, n_data)
    reward_step_goal_ee = np.zeros(n_data)
    for i in range(n_data):
        reward_step_goal_ee[i] = linear_function(0, goal_range_max_x, 0, reward_step_target2goal, current_ee_distance2goal[i], -1)
    weighted_reward_step_goal_ee = alpha_step_goal_ee * reward_step_goal_ee # type: ignore
    plot_func(current_ee_distance2goal, reward_step_goal_base, save_path=save_path+extra_tag+'reward_step_goal_ee.png')

    # Step Reward 3: ee to target
    current_ee_distance2target_pos = np.linspace(dist_min, 2*dist_max, n_data)
    current_ee_distance2target_ori = np.linspace(0.0, 1.0, n_data)
    reward_step_target_pos = np.zeros(n_data)
    reward_step_target_ori = np.zeros(n_data)
    for i in range(n_data):
        #reward_step_target_pos = reward_func(0, goal_range_max_x + math.pi, 0, reward_step_target, current_ee_distance2target + current_ee_oridistance2target_yaw) # type: ignore
        #reward_step_target_mode12[i] = reward_func(0, goal_range_max_x + 1.0, 0, reward_step_target, current_ee_distance2target + current_ee_oridistance2target_quat) # type: ignore
        reward_step_target_pos[i] = linear_function(0, goal_range_max_x, 0, reward_step_target2goal, current_ee_distance2target_pos[i], -1) # type: ignore
        reward_step_target_ori[i] = linear_function(0, 1.0, 0, reward_step_target2goal, current_ee_distance2target_ori[i], -1) # type: ignore
    weighted_reward_step_target_pos = alpha_step_target_pos * reward_step_target_pos # type: ignore
    weighted_reward_step_target_ori = alpha_step_target_ori * reward_step_target_ori # type: ignore
    plot_func(current_ee_distance2target_pos, reward_step_target_pos, save_path=save_path+extra_tag+'reward_step_target_pos.png')
    plot_func(current_ee_distance2target_ori, reward_step_target_ori, save_path=save_path+extra_tag+'reward_step_target_ori.png')

    # Step Reward 4: model mode
    reward_step_mode0_vec = reward_step_mode0 * np.ones(n_data) # type: ignore
    reward_step_mode1_vec = reward_step_mode1 * np.ones(n_data) # type: ignore
    reward_step_mode2_vec = reward_step_mode2 * np.ones(n_data) # type: ignore
    weighted_reward_step_mode0 = alpha_step_mode * reward_step_mode0_vec # type: ignore
    weighted_reward_step_mode1 = alpha_step_mode * reward_step_mode1_vec # type: ignore
    weighted_reward_step_mode2 = alpha_step_mode * reward_step_mode2_vec # type: ignore

    # Step Reward 5: mpc time horizon
    dt_action = np.linspace(0, 3*action_time_horizon, n_data) # type: ignore
    reward_step_mpc_time_horizon = np.zeros(n_data) # type: ignore
    reward_step_mpc_time_horizon_plus = np.zeros(int(0.5*n_data)+1) # type: ignore
    reward_step_mpc_time_horizon_minus = np.zeros(int(0.5*n_data)) # type: ignore
    coeff = reward_step_time_horizon_max / reward_step_time_horizon_min # type: ignore
    for i in range(n_data):
        if dt_action[i] <= action_time_horizon:
            reward_step_mpc_time_horizon[i] = linear_function(0, action_time_horizon, 0.0, reward_step_time_horizon_max, dt_action[i], -1)
        elif dt_action[i] <= 2*action_time_horizon: # type: ignore
            reward_step_mpc_time_horizon[i] = linear_function(action_time_horizon, 2*action_time_horizon, reward_step_time_horizon_min, 0.0, dt_action[i], -1) # type: ignore
        else:
            if  dt_action[i] > 2*action_time_horizon: # type: ignore
                reward_step_mpc_time_horizon[i] = reward_step_time_horizon_min
            else:
                reward_step_mpc_time_horizon[i] = reward_step_time_horizon_max
    weighted_reward_step_mpc_time_horizon = alpha_step_mpc_exit * reward_step_mpc_time_horizon # type: ignore
    plot_func(dt_action, reward_step_mpc_time_horizon, save_path=save_path+extra_tag+'reward_step_mpc_time_horizon.png')

    print("[plot_functions::__main__] END")