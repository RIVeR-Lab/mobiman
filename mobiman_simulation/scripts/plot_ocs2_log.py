#!/usr/bin/env python3

'''
LAST UPDATE: 2023.08.07

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

def add_log_data(log_path, log_name, data_name, data_value):
    
    logfile = log_path + log_name + ".json"
    new_data = {data_name: data_value}

    with open(logfile,'r+') as file:
        # First we load existing data into a dict.
        file_data = json.load(file)

        # Join new_data with file_data inside emp_details
        file_data.update(new_data)

        # Sets file's current position at offset.
        file.seek(0)

        # convert back to json.
        json.dump(file_data, file, indent = 4)

def plot_log(   
        data_x, data1_y, data2_y=None, 
        limit_min=None, limit_max=None, 
        label_data1='', label_data2='',
        color1='r', color2='b', 
        label_x='', label_y='', title='', 
        fig_num=0, save_path='plot_ocs2_log.png'):
    
    fig = plt.figure()
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
    plt.close()

def extract_and_plot_log(log_path, log_name, save_prefix='',save_plot_flag=True):
    
    # Open JSON file
    f = open(log_path + log_name + ".json")

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

    state_arm_offset = None
    if 'state_arm_offset' not in data.keys():
        add_log_data(log_path, log_name, "state_arm_offset", 3)
        state_arm_offset = 3
    else:
        state_arm_offset = data['state_arm_offset']

    input_arm_offset = None
    if 'input_arm_offset' not in data.keys():
        add_log_data(log_path, log_name, "input_arm_offset", 2)
        input_arm_offset = 2
    else:
        input_arm_offset = data['input_arm_offset']
    
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

    # Print data
    print("[plot_ocs2_log::__main__] log_path: " + str(log_path))
    print("[plot_ocs2_log::__main__] log_name: " + str(log_name))
    print("[plot_ocs2_log::__main__] time_start: " + str(time_start))
    print("[plot_ocs2_log::__main__] time_end: " + str(time_end))
    print("[plot_ocs2_log::__main__] state_arm_offset: " + str(state_arm_offset))
    print("[plot_ocs2_log::__main__] input_arm_offset: " + str(input_arm_offset))
    print("[plot_ocs2_log::__main__] dt: " + str(dt))
    print("[plot_ocs2_log::__main__] _dt: " + str(_dt))
    print("[plot_ocs2_log::__main__] data_size: " + str(data_size))
    print("[plot_ocs2_log::__main__] state_size: " + str(state_size))
    print("[plot_ocs2_log::__main__] input_size: " + str(input_size))
    
    # Set time vector
    time = np.linspace(time_start, time_start + dt*data_size, data_size)
    #print("[plot_ocs2_log::__main__] time: " + str(time))

    if len(time) != data_size:
        print("[plot_ocs2_log::__main__] PROBLEMATICO 2!")
        print("[plot_ocs2_log::__main__] time len: " + str(len(time)))
        print("[plot_ocs2_log::__main__] data_size: " + str(data_size))

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

    state = np.vstack([state_velo_base, state_pos_arm])

    if state_velo_base.shape[0] + state_pos_arm.shape[0] != input.shape[0]:
        print("[plot_ocs2_log::__main__] PROBLEMATICO 3!")
        print("[plot_ocs2_log::__main__] state_velo_base.shape: " + str(state_velo_base.shape))
        print("[plot_ocs2_log::__main__] state_pos_arm.shape: " + str(state_pos_arm.shape))
        print("[plot_ocs2_log::__main__] input.shape: " + str(input.shape))

    if state.shape != input.shape:
        print("[plot_ocs2_log::__main__] PROBLEMATICO 4!")
        print("[plot_ocs2_log::__main__] state.shape: " + str(state.shape))
        print("[plot_ocs2_log::__main__] input.shape: " + str(input.shape))

    if (save_plot_flag):
        ### Plots

        _save_prefix = save_prefix
        if save_prefix:
            _save_prefix = save_prefix + '_'

        # Create new folder for plots if not exist
        save_path = log_path + _save_prefix + log_name + "_plots/"
        isExist = os.path.exists(save_path)
        if not isExist:
            os.makedirs(save_path)

        ## Base states (velocity) - input commands
        plot_log(
            time, state[0], input[0],
            label_data1='State', label_data2='Command',
            label_x="Time", label_y='Lateral velocity [m/s]',
            fig_num=1,
            save_path=save_path + 'base_veloX.png')
        
        plot_log(
            time, state[1], input[1],
            label_data1='State', label_data2='Command',
            label_x="Time", label_y='Angular velocity [rad/s]',
            fig_num=2,
            save_path=save_path + 'base_veloYaw.png')
        
        ## Arm states (position) - input commands
        for i in range(state_arm_size):
            idx = input_arm_offset + i
            
            state[idx] *= 180 / math.pi
            input[idx] *= 180 / math.pi

            plot_log(
                time, state[idx], input[idx],
                label_data1='State', label_data2='Command',
                label_x="Time", label_y='Arm joint angle [deg]',
                fig_num=3+i,
                save_path=save_path + 'arm_joint' + str(i) + '.png')
            
    return time, state, input, input_arm_offset

def extract_and_plot_log_in_folder(log_path, save_prefix=''):
    
    # Get log files in the folder
    file_list = os.listdir(log_path)
    log_list = []
    for file in file_list:
        if file.endswith(".json"):
            log_list.append(file)
    log_list.sort()
    print(log_list)

    # Initialize variables
    time_concat = np.empty((0,0))
    state_concat = np.empty((0,0))
    input_concat = np.empty((0,0))

    for k, log_name in enumerate(log_list):
        
        log_name_trimmed = log_name
        log_name_trimmed = log_name_trimmed.replace('.json', '')
        time_tmp, state_tmp, input_tmp, input_arm_offset = extract_and_plot_log(log_path, log_name_trimmed)
        
        if k == 0:
            time_concat = time_tmp
            state_concat = state_tmp
            input_concat = input_tmp
        else:
            time_concat = np.concatenate((time_concat, time_tmp), axis=0)
            state_concat = np.hstack([state_concat, state_tmp])
            input_concat = np.hstack([input_concat, input_tmp])
        
        print("--------------------------------------------")

    print("[plot_ocs2_log::__main__] time_concat.shape: " + str(time_concat.shape))
    print("[plot_ocs2_log::__main__] state_concat.shape: " + str(state_concat.shape))
    print("[plot_ocs2_log::__main__] input_concat.shape: " + str(input_concat.shape))
    
    ### Plots

    if save_prefix:
            save_prefix += '_'

    ## Base states (velocity) - input commands
    plot_log(
        time_concat, state_concat[0], input_concat[0],
        label_data1='State', label_data2='Command',
        label_x="Time", label_y='Lateral velocity [m/s]',
        fig_num=1,
        save_path=log_path + save_prefix + 'base_veloX.png')
    
    plot_log(
        time_concat, state_concat[1], input_concat[1],
        label_data1='State', label_data2='Command',
        label_x="Time", label_y='Angular velocity [rad/s]',
        fig_num=2,
        save_path=log_path + save_prefix + 'base_veloYaw.png')
    
    ## Arm states (position) - commands
    for i in range(state_concat.shape[0] - input_arm_offset): # type: ignore
        idx = input_arm_offset + i # type: ignore

        plot_log(
            time_concat, state_concat[idx], input_concat[idx],
            label_data1='State', label_data2='Command',
            label_x="Time", label_y='Arm joint angle [deg]',
            fig_num=3+i,
            save_path=log_path + save_prefix + 'arm_joint' + str(i) + '.png') # type: ignore

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
    ocs2_log_path_rel = rospy.get_param('ocs2_log_path_rel', "")
    ocs2_log_name = rospy.get_param('ocs2_log_name', "")
    n_round_digit = rospy.get_param('n_round_digit', 2)
    save_prefix = rospy.get_param('save_prefix', "")

    ocs2_log_path = mobiman_path + ocs2_log_path_rel

    if option == 0:
        extract_and_plot_log(ocs2_log_path, ocs2_log_name, save_prefix=save_prefix, save_plot_flag=True) # type: ignore
        
    if option == 1:
        extract_and_plot_log_in_folder(ocs2_log_path, save_prefix=save_prefix) # type: ignore
    
    print("[plot_ocs2_log::__main__] END")