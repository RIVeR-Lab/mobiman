#!/usr/bin/env python3

'''
LAST UPDATE: 2023.08.04

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:
[1] 

NUA TODO:
'''

import math
import json
import rospkg
import rospy
import numpy as np
import matplotlib.pyplot as plt

def plot_log(   
        data_x, data1_y, data2_y=None, 
        limit_min=None, limit_max=None, 
        label_data1='', label_data2='',
        color1='r', color2='b', 
        label_x='', label_y='', title='', 
        fig_num=0, save_path='plot_ocs2_log.png'):
    
    plt.figure(fig_num)
    plt.plot(data_x, data1_y, color=color1, label=label_data1)

    if  data2_y is not None:
        plt.plot(data_x, data2_y, color=color2, label=label_data2)

    if limit_min is not None:
        plt.axhline(limit_min, color='k', linewidth=2, linestyle="--")

    if limit_min is not None:
        plt.axhline(limit_min, color='k', linewidth=2, linestyle="--")

    plt.xlabel(label_x)
    plt.ylabel(label_y)
    plt.title(title)
    plt.legend()
    plt.savefig(save_path)

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    print("[plot_ocs2_log::__main__] START")

    rospy.init_node('plot_ocs2_log', anonymous=True, log_level=rospy.WARN)

    ## Get variables
    rospack = rospkg.RosPack()
    mobiman_path = rospack.get_path('mobiman_simulation') + "/"
    option = rospy.get_param('option', 0)
    ocs2_log_path = rospy.get_param('ocs2_log_path', "")
    ocs2_log_name = rospy.get_param('ocs2_log_name', "")
    ocs2_log_folder_name = rospy.get_param('ocs2_log_folder_name', "")
    n_round_digit = rospy.get_param('n_round_digit', 2)

    # Open JSON file
    f = open(mobiman_path + ocs2_log_path + ocs2_log_name)

    # Get JSON object as a dictionary
    data = json.load(f)

    # Close JSON file
    f.close()
    
    # Print JSON keys
    print("[plot_ocs2_log::__main__] keys: " + str(data.keys()))

    ## Extract data
    dt = round(data['dt'], int(n_round_digit)) # type: ignore
    time_start = round(data['time_start'], int(n_round_digit)) # type: ignore
    time_end = round(data['time_end'], n_round_digit) # type: ignore
    state_pos_data = data['state_position']
    state_velo_base_data = data['state_velocity_base']
    input_data = data['command']

    # Set data length
    data_size = len(input_data)
    if len(state_pos_data) != len(input_data):
        print("[plot_ocs2_log::__main__] PROBLEMATICO 1!")
        print("[plot_ocs2_log::__main__] state_pos_data len: " + str(len(state_pos_data)))
        print("[plot_ocs2_log::__main__] input_data len: " + str(len(input_data)))
        data_size = min(len(state_pos_data), len(input_data))
        state_pos_data = state_pos_data[:data_size]
        input_data = input_data[:data_size]

    # Set dt
    _dt = round((time_end - time_start) / data_size, n_round_digit) # type: ignore

    # Set state, input size
    state_size = len(state_pos_data[0])
    state_velo_base_size = len(state_velo_base_data[0])
    input_size = len(input_data[0])

    # Set offsets
    state_arm_offset = 3
    input_arm_offset = 2

    # Print data
    print("[plot_ocs2_log::__main__] time_start: " + str(time_start))
    print("[plot_ocs2_log::__main__] time_end: " + str(time_end))
    print("[plot_ocs2_log::__main__] dt: " + str(dt))
    print("[plot_ocs2_log::__main__] data_size: " + str(data_size))
    print("[plot_ocs2_log::__main__] state_size: " + str(state_size))
    print("[plot_ocs2_log::__main__] input_size: " + str(input_size))
    print("[plot_ocs2_log::__main__] state_arm_offset: " + str(state_arm_offset))
    print("[plot_ocs2_log::__main__] input_arm_offset: " + str(input_arm_offset))

    # Set time vector
    time = np.arange(time_start, time_end, dt)
    #print("[plot_ocs2_log::__main__] time: " + str(time))

    # Set joint states and commands wrt the time vector
    state_velo_base = np.zeros((state_velo_base_size, data_size))
    for i, single_state in enumerate(state_velo_base_data):
        for j, ss in enumerate(single_state):
            state_velo_base[j][i] = ss

    state_arm_size = state_size-state_arm_offset
    state_pos_arm = np.zeros((state_arm_size, data_size))
    for i, single_state in enumerate(state_pos_data):
        for j, ss in enumerate(single_state[state_arm_offset:]):
            state_pos_arm[j][i] = ss

    input = np.zeros((input_size, data_size))
    for i, single_command in enumerate(input_data):
        for j, sc in enumerate(single_command):
            input[j][i] = sc
            
    if state_velo_base.shape[0] + state_pos_arm.shape[0] != input.shape[0]:
        print("[plot_ocs2_log::__main__] PROBLEMATICO 2!")
        print("[plot_ocs2_log::__main__] state_velo_base.shape: " + str(state_velo_base.shape))
        print("[plot_ocs2_log::__main__] state_pos_arm.shape: " + str(state_pos_arm.shape))
        print("[plot_ocs2_log::__main__] input.shape: " + str(input.shape))

    state = np.vstack([state_velo_base, state_pos_arm])

    if state.shape != input.shape:
        print("[plot_ocs2_log::__main__] PROBLEMATICO 3!")
        print("[plot_ocs2_log::__main__] state.shape: " + str(state.shape))
        print("[plot_ocs2_log::__main__] input.shape: " + str(input.shape))

    ### Plots
    ## Base states (position or velocity) - input commands
    plot_log(
        time, state[0], input[0],
        label_data1='State', label_data2='Command',
        label_x="Time", label_y='Lateral velocity [m/s]',
        fig_num=1,
        save_path=mobiman_path + ocs2_log_path + 'plot_base_veloX.png')
    
    ## Arm states - commands
    for i in range(state_arm_size):
        joint_offset = i
        idx = input_arm_offset + joint_offset

        state[idx] *= 180 / math.pi
        input[idx] *= 180 / math.pi

        plot_log(
            time, state[idx], input[idx],
            label_data1='State', label_data2='Command',
            label_x="Time", label_y='Arm joint angle [deg]',
            fig_num=2+i,
            save_path=mobiman_path + ocs2_log_path + 'plot_arm_joint' + str(joint_offset) + '.png')

    print("[plot_ocs2_log::__main__] END")