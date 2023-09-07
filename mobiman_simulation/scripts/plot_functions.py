#!/usr/bin/env python3

'''
LAST UPDATE: 2023.09.07

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

def reward_func(self, x_min, x_max, y_min, y_max, x_query):
        reward = 0
        if x_min <= x_query <= x_max:
            reward = (y_min - y_max) * (x_query - x_min) / (x_max - x_min)
        return reward

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    print("[plot_functions::__main__] START")

    rospy.init_node('plot_functions', anonymous=True, log_level=rospy.WARN)

    '''
    ## Get variables
    rospack = rospkg.RosPack()
    mobiman_path = rospack.get_path('mobiman_simulation') + "/"
    
    option = rospy.get_param('option', 0)
    ocs2_log_path_rel = rospy.get_param('ocs2_log_path_rel', "")
    ocs2_log_name = rospy.get_param('ocs2_log_name', "")
    n_round_digit = rospy.get_param('n_round_digit', 2)
    save_prefix = rospy.get_param('save_prefix', "")

    ocs2_log_path = mobiman_path + ocs2_log_path_rel
    '''

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

    #plot_func()
    
    print("[plot_functions::__main__] END")