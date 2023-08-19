#!/usr/bin/env python3

'''
LAST UPDATE: 2023.08.16

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

def plot_log_state(   
        data1_x, data1_y, 
        data2_x, data2_y,
        state_index=0, time_index=0,
        limit_min=None, limit_max=None, 
        label_data1='', label_data2='',
        color1='r', color2='g',
        label_x='', label_y='', title='', 
        save_path='plot_ocs2_log_state.png'):
    
    fig = plt.figure()
    plt.plot(data1_x, data1_y[state_index], color=color1, label=label_data1)

    if time_index >= 0:
        data2_x[time_index] = np.reshape(data2_x[time_index], data2_y[time_index][state_index].shape)
        plt.plot(data2_x[time_index], data2_y[time_index][state_index], color=color2, label=label_data2)
    else:
        for i, dat in enumerate(data2_y):
            data2_x[i] = np.reshape(data2_x[i], dat[state_index].shape)
            if i == 0:
                plt.plot(data2_x[i], dat[state_index], color=color2, label=label_data2)
            else:
                plt.plot(data2_x[i], dat[state_index], color=color2)

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

def plot_log_input(   
        data_x=None, data1_y=None, data2_y=None, 
        data3_x=None, data3_y=None,
        input_index=0, state_index=0, time_index=-1,
        limit_min=None, limit_max=None, 
        label_data1='', label_data2='', label_data3='',
        color1='r', color2='b', color3='g',
        label_x='', label_y='', title='', 
        fig_num=0, save_path='plot_ocs2_log.png'):
    
    fig = plt.figure()
    if  data1_y is not None:
        plt.plot(data_x, data1_y[input_index], color=color1, label=label_data1)

    if  data2_y is not None:
        plt.plot(data_x, data2_y[input_index], color=color2, label=label_data2)

    if  (data3_x is not None) and (data3_y is not None) and state_index >= 0:
        if time_index >= 0:
            data3_x[time_index] = np.reshape(data3_x[time_index], data3_y[time_index][state_index].shape)
            plt.plot(data3_x[time_index], data3_y[time_index][state_index], color=color3, label=label_data3)
        else:
            for i, dat in enumerate(data3_y):
                data3_x[i] = np.reshape(data3_x[i], dat[state_index].shape)
                if i == 0:
                    plt.plot(data3_x[i], dat[state_index], color=color3, label=label_data3)
                else:
                    plt.plot(data3_x[i], dat[state_index], color=color3)

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

def extract_and_plot_log(log_path, log_name, mpc_ts_idx=0, save_prefix='',save_plot_flag=True):
    
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
    model_mode = data['model_mode']
    state_pos_data = data['state_position']
    state_velo_base_data = data['state_velocity_base']
    input_data = data['command']
    mpc_time_trajectory_data = data['mpc_time_trajectory']
    mpc_state_trajectory_data = data['mpc_state_trajectory']
    mpc_input_trajectory_data = data['mpc_input_trajectory']

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
        state_velo_base_data = state_velo_base_data[:data_size]
        input_data = input_data[:data_size]

    mpc_time_trajectory_data_size = len(mpc_time_trajectory_data)
    mpc_state_trajectory_data_size = len(mpc_state_trajectory_data)
    mpc_input_trajectory_data_size = len(mpc_input_trajectory_data)

    mpc_time_trajectory_sample_size = len(mpc_time_trajectory_data[0])
    mpc_state_trajectory_sample_size = len(mpc_state_trajectory_data[0])
    mpc_input_trajectory_sample_size = len(mpc_input_trajectory_data[0])

    mpc_state_trajectory_size = len(mpc_state_trajectory_data[0][0])
    mpc_input_trajectory_size = len(mpc_input_trajectory_data[0][0])

    # Set dt
    _dt = round((time_end - time_start) / data_size, n_round_digit) # type: ignore

    # Set state, input size
    state_pos_data_size = len(state_pos_data[0])
    state_velo_base_data_size = len(state_velo_base_data[0])
    input_data_size = len(input_data[0])

    # Print data
    print("[plot_ocs2_log::__main__] log_path: " + str(log_path))
    print("[plot_ocs2_log::__main__] log_name: " + str(log_name))
    print("[plot_ocs2_log::__main__] time_start: " + str(time_start))
    print("[plot_ocs2_log::__main__] time_end: " + str(time_end))
    print("[plot_ocs2_log::__main__] state_arm_offset: " + str(state_arm_offset))
    print("[plot_ocs2_log::__main__] input_arm_offset: " + str(input_arm_offset))
    print("[plot_ocs2_log::__main__] model_mode: " + str(model_mode))
    print("[plot_ocs2_log::__main__] dt: " + str(dt))
    print("[plot_ocs2_log::__main__] _dt: " + str(_dt))
    print("[plot_ocs2_log::__main__] data_size: " + str(data_size))
    print("[plot_ocs2_log::__main__] state_pos_data_size: " + str(state_pos_data_size))
    print("[plot_ocs2_log::__main__] state_velo_base_data_size: " + str(state_velo_base_data_size))
    print("[plot_ocs2_log::__main__] input_data_size: " + str(input_data_size))
    print("[plot_ocs2_log::__main__] mpc_time_trajectory_data_size: " + str(mpc_time_trajectory_data_size))
    print("[plot_ocs2_log::__main__] mpc_time_trajectory_sample_size: " + str(mpc_time_trajectory_sample_size))
    print("[plot_ocs2_log::__main__] mpc_state_trajectory_data_size: " + str(mpc_state_trajectory_data_size))
    print("[plot_ocs2_log::__main__] mpc_state_trajectory_sample_size: " + str(mpc_state_trajectory_sample_size))
    print("[plot_ocs2_log::__main__] mpc_state_trajectory_size: " + str(mpc_state_trajectory_size))
    print("[plot_ocs2_log::__main__] mpc_input_trajectory_data_size: " + str(mpc_input_trajectory_data_size))
    print("[plot_ocs2_log::__main__] mpc_input_trajectory_sample_size: " + str(mpc_input_trajectory_sample_size))
    print("[plot_ocs2_log::__main__] mpc_input_trajectory_size: " + str(mpc_input_trajectory_size))
    
    # Set time vector
    time = np.linspace(time_start, time_start + dt*data_size, data_size)
    #print("[plot_ocs2_log::__main__] time: " + str(time))

    if len(time) != data_size:
        print("[plot_ocs2_log::__main__] PROBLEMATICO 2!")
        print("[plot_ocs2_log::__main__] time len: " + str(len(time)))
        print("[plot_ocs2_log::__main__] data_size: " + str(data_size))

    # Set joint states and commands wrt the time vector
    state_pos_base = np.zeros((state_arm_offset, data_size))
    for i, single_state in enumerate(state_pos_data):
        for j in range(state_arm_offset):
            if j == 2:
                state_pos_base[j][i] = single_state[j] * 180 / math.pi
            else:
                state_pos_base[j][i] = single_state[j]

    state_velo_base = np.zeros((state_velo_base_data_size, data_size))
    for i, single_state in enumerate(state_velo_base_data):
        for j, ss in enumerate(single_state):
            state_velo_base[j][i] = ss

    state_arm_size = state_pos_data_size-state_arm_offset
    state_pos_arm = np.zeros((state_arm_size, data_size))
    for i, single_state in enumerate(state_pos_data):
        for j, ss in enumerate(single_state[state_arm_offset:]):
            state_pos_arm[j][i] = ss * 180 / math.pi

    input_size = state_velo_base_data_size + state_arm_size
    ioff = 0
    if model_mode == 1:
        ioff = input_arm_offset
    input = np.zeros((input_size, data_size))
    for i, single_input in enumerate(input_data):
        for j, si in enumerate(single_input):
            input[ioff+j][i] = si * 180 / math.pi

    state = np.vstack([state_velo_base, state_pos_arm])

    print("[plot_ocs2_log::__main__] state_pos_base.shape: " + str(state_pos_base.shape))
    print("[plot_ocs2_log::__main__] state_velo_base.shape: " + str(state_velo_base.shape))
    print("[plot_ocs2_log::__main__] state_pos_arm.shape: " + str(state_pos_arm.shape))
    print("[plot_ocs2_log::__main__] state.shape: " + str(state.shape))
    print("[plot_ocs2_log::__main__] input.shape: " + str(input.shape))

    # Set MPC time
    mpc_time = []
    for i, traj_sample in enumerate(mpc_time_trajectory_data):
        mpc_time_trajectory_sample_size = len(traj_sample)
        mpc_time_sample_traj = np.zeros((1, mpc_time_trajectory_sample_size))
        for j, tv in enumerate(traj_sample):
            mpc_time_sample_traj[0,j] = tv
        mpc_time.append(mpc_time_sample_traj)

    # Set MPC state
    mpc_state = []
    for i, traj_sample in enumerate(mpc_state_trajectory_data):
        mpc_state_trajectory_sample_size = len(traj_sample)
        mpc_state_sample_traj = np.zeros((state_pos_data_size, mpc_state_trajectory_sample_size))
        for j, traj in enumerate(traj_sample):
            for k, tv in enumerate(traj):
                idx = k
                if model_mode == 1:
                    idx = state_arm_offset + k
                if idx >= state_arm_offset or idx == 2:
                    mpc_state_sample_traj[idx,j] = tv * 180 / math.pi
                else:
                    mpc_state_sample_traj[idx,j] = tv
        mpc_state.append(mpc_state_sample_traj)

    # Set MPC input
    mpc_input = []
    for i, traj_sample in enumerate(mpc_input_trajectory_data):
        mpc_input_trajectory_sample_size = len(traj_sample)
        mpc_input_sample_traj = np.zeros((input_size, mpc_input_trajectory_sample_size))
        for j, traj in enumerate(traj_sample):
            for k, tv in enumerate(traj):
                idx = k
                if model_mode == 1:
                    idx = input_arm_offset + k
                if idx >= input_arm_offset:
                    mpc_input_sample_traj[idx,j] = tv * 180 / math.pi
                else:
                    mpc_input_sample_traj[idx,j] = tv
        mpc_input.append(mpc_input_sample_traj)

    print("[plot_ocs2_log::__main__] mpc_time len: " + str(len(mpc_time)))
    print("[plot_ocs2_log::__main__] mpc_time[0] shape: " + str(mpc_time[0].shape))
    print("[plot_ocs2_log::__main__] mpc_state len: " + str(len(mpc_state)))
    print("[plot_ocs2_log::__main__] mpc_state[0] shape: " + str(mpc_state[0].shape))
    print("[plot_ocs2_log::__main__] mpc_input len: " + str(len(mpc_input)))
    print("[plot_ocs2_log::__main__] mpc_input[0] shape: " + str(mpc_input[0].shape))

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

        ## Base states (position) - MPC state estimates
        plot_log_state(
            time, state_pos_base, mpc_time, mpc_state, 
            state_index=0, time_index=-1, 
            label_data1='State', label_data2='MPC state estimate',
            label_x="Time", label_y='Position x [m]',
            save_path=save_path + 'base_posX.png')
        
        plot_log_state(
            time, state_pos_base, mpc_time, mpc_state, 
            state_index=1, time_index=-1, 
            label_data1='State', label_data2='MPC state estimate',
            label_x="Time", label_y='Position y [m]',
            save_path=save_path + 'base_posY.png')
        
        plot_log_state(
            time, state_pos_base, mpc_time, mpc_state, 
            state_index=2, time_index=-1, 
            label_data1='State', label_data2='MPC state estimate',
            label_x="Time", label_y='Yaw angle [deg]',
            save_path=save_path + 'base_yaw.png')

        ## Base states (velocity) - input commands
        plot_log_input(
            time, state, input, mpc_time, mpc_input, 
            input_index=0, state_index=0, time_index=-1, 
            label_data1='State', label_data2='Command input', label_data3='MPC input',
            label_x="Time", label_y='Lateral velocity [m/s]',
            fig_num=1,
            save_path=save_path + 'base_veloX.png')

        plot_log_input(
            time, state, input, mpc_time, mpc_input, 
            input_index=1, state_index=1, time_index=-1,
            label_data1='State', label_data2='Command input', label_data3='MPC input',
            label_x="Time", label_y='Angular velocity [rad/s]',
            fig_num=2,
            save_path=save_path + 'base_veloYaw.png')
        
        ## Arm states (position) - input commands
        for i in range(state_arm_size):
            idxi = input_arm_offset + i
            idxs = state_arm_offset + i
            
            #state[idxi] *= 180 / math.pi
            #input[idxi] *= 180 / math.pi

            plot_log_input(
                time, state, input, mpc_time, mpc_state, 
                input_index=idxi, state_index=idxs, time_index=-1,
                label_data1='State', label_data2='Command input', label_data3='MPC state estimate',
                label_x="Time", label_y='Arm joint angle [deg]',
                fig_num=3+i,
                save_path=save_path + 'arm_joint' + str(i+1) + '_pos.png')
            
            plot_log_input(
                data3_x=mpc_time, data3_y=mpc_input, 
                state_index=idxi, time_index=-1,
                label_data3='MPC input',
                label_x="Time", label_y='Arm joint angular velocity [deg/sec]',
                fig_num=3+i,
                save_path=save_path + 'arm_joint' + str(i+1) + '_velo.png')
            
    return time, state, input, state_pos_base, mpc_time, mpc_state, mpc_input, dt, input_arm_offset, state_arm_offset

def extract_and_plot_log_in_folder(log_path, mpc_ts_idx=0, save_prefix='', save_plot_flag=True):
    
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
    state_pos_base_concat = np.empty((0,0))
    mpc_time_concat = []
    mpc_state_concat = []
    mpc_input_concat = []

    for k, log_name in enumerate(log_list):
        
        print("[plot_ocs2_log::__main__] k: " + str(k))
        print("[plot_ocs2_log::__main__] log_name: " + log_name)

        log_name_trimmed = log_name
        log_name_trimmed = log_name_trimmed.replace('.json', '')
        time_tmp, state_tmp, input_tmp, state_pos_base_tmp, mpc_time_tmp, mpc_state_tmp, mpc_input_tmp, dt, input_arm_offset, state_arm_offset = extract_and_plot_log(log_path, log_name_trimmed, mpc_ts_idx=mpc_ts_idx, save_plot_flag=save_plot_flag)
        
        if k == 0:
            time_concat = time_tmp
            state_concat = state_tmp
            input_concat = input_tmp
            state_pos_base_concat = state_pos_base_tmp
            mpc_time_concat = mpc_time_tmp
            mpc_state_concat = mpc_state_tmp
            mpc_input_concat = mpc_input_tmp
        else:
            if time_tmp[0] <= time_concat[-1]:
                time_diff = time_concat[-1] - time_tmp[0]
                time_tmp += time_diff + dt

            if mpc_time_tmp[0][0][0] <= mpc_time_concat[-1][0][0]:
                time_diff = mpc_time_concat[-1][0][0] - mpc_time_tmp[0][0][0]
                for tim in time_diff:
                    tim += time_diff + dt

            time_concat = np.concatenate((time_concat, time_tmp), axis=0)
            state_concat = np.hstack([state_concat, state_tmp])
            input_concat = np.hstack([input_concat, input_tmp])
            state_pos_base_concat = np.hstack([state_pos_base_concat, state_pos_base_tmp])
            mpc_time_concat += mpc_time_tmp
            mpc_state_concat += mpc_state_tmp
            mpc_input_concat += mpc_input_tmp
        
        print("--------------------------------------------")

    print("[plot_ocs2_log::__main__] time_concat.shape: " + str(time_concat.shape))
    print("[plot_ocs2_log::__main__] state_concat.shape: " + str(state_concat.shape))
    print("[plot_ocs2_log::__main__] input_concat.shape: " + str(input_concat.shape))
    print("[plot_ocs2_log::__main__] state_pos_base_concat.shape: " + str(state_pos_base_concat.shape))
    print("[plot_ocs2_log::__main__] mpc_time_concat len: " + str(len(mpc_time_concat)))
    print("[plot_ocs2_log::__main__] mpc_time_concat[0].shape: " + str(mpc_time_concat[0].shape))
    print("[plot_ocs2_log::__main__] mpc_state_concat len: " + str(len(mpc_state_concat)))
    print("[plot_ocs2_log::__main__] mpc_state_concat[0].shape: " + str(mpc_state_concat[0].shape))
    print("[plot_ocs2_log::__main__] mpc_input_concat len: " + str(len(mpc_input_concat)))
    print("[plot_ocs2_log::__main__] mpc_input_concat[0].shape: " + str(mpc_input_concat[0].shape))
    
    ### Plots
    if save_prefix:
            save_prefix += '_'

    ## Base states (position) - MPC state estimates
    plot_log_state(
        time_concat, state_pos_base_concat, mpc_time_concat, mpc_state_concat, 
        state_index=0, time_index=-1, 
        label_data1='State', label_data2='MPC state estimate',
        label_x="Time", label_y='Position x [m]',
        save_path=log_path + save_prefix + 'base_posX.png')
    
    plot_log_state(
        time_concat, state_pos_base_concat, mpc_time_concat, mpc_state_concat, 
        state_index=1, time_index=-1, 
        label_data1='State', label_data2='MPC state estimate',
        label_x="Time", label_y='Position y [m]',
        save_path=log_path + save_prefix + 'base_posY.png')
    
    plot_log_state(
        time_concat, state_pos_base_concat, mpc_time_concat, mpc_state_concat, 
        state_index=2, time_index=-1, 
        label_data1='State', label_data2='MPC state estimate',
        label_x="Time", label_y='Yaw angle [deg]',
        save_path=log_path + save_prefix + 'base_yaw.png')

    ## Base states (velocity) - input commands
    plot_log_input(
        time_concat, state_concat, input_concat, mpc_time_concat, mpc_input_concat, 
        input_index=0, state_index=0, time_index=-1, 
        label_data1='State', label_data2='Command input', label_data3='MPC input',
        label_x="Time", label_y='Lateral velocity [m/s]',
        save_path=log_path + save_prefix + 'base_veloX.png')

    plot_log_input(
        time_concat, state_concat, input_concat, mpc_time_concat, mpc_input_concat, 
        input_index=1, state_index=1, time_index=-1, 
        label_data1='State', label_data2='Command input', label_data3='MPC input',
        label_x="Time", label_y='Angular velocity [rad/s]',
        save_path=log_path + save_prefix + 'base_veloYaw.png')
    
    ## Arm states (position and velocity) - commands
    for i in range(state_concat.shape[0] - input_arm_offset): # type: ignore
        idxi = input_arm_offset + i # type: ignore
        idxs = state_arm_offset + i # type: ignore
        
        plot_log_input(
            time_concat, state_concat, input_concat, mpc_time_concat, mpc_state_concat, 
            input_index=idxi, state_index=idxs, time_index=-1, 
            label_data1='State', label_data2='Command input', label_data3='MPC state estimate',
            label_x="Time", label_y='Arm joint angle [deg]',
            save_path=log_path + save_prefix + 'arm_joint' + str(i+1) + '_pos.png') # type: ignore
        
        plot_log_input(
            data3_x=mpc_time_concat, data3_y=mpc_input_concat, 
            state_index=idxi, time_index=-1,
            label_data3='MPC input',
            label_x="Time", label_y='Arm joint angular velocity [deg/sec]',
            save_path=log_path + save_prefix + 'arm_joint' + str(i+1) + '_velo.png')

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
        extract_and_plot_log_in_folder(ocs2_log_path, save_prefix=save_prefix, save_plot_flag=False) # type: ignore
    
    print("[plot_ocs2_log::__main__] END")