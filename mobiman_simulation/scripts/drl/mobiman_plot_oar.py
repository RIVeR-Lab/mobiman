#!/usr/bin/python3

'''
LAST UPDATE: 2024.04.10

AUTHOR:	Sarvesh Prajapati (SP)
        Neset Unver Akmandor (NUA)	

E-MAIL: prajapati.s@northeastern.edu
        akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:

NUA TODO:
- 
'''

from cProfile import label
import os
import sys
import time
import csv
from turtle import color, st
from cv2 import mean
import math
from matplotlib.markers import MarkerStyle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
#from mobiman.mobiman_simulation.scripts.drl.mobiman_drl_training import print_array
from collections import deque
import rospy
import rospkg
from toolz import interleave
from matplotlib.ticker import PercentFormatter
import matplotlib.lines as mlines

#from stable_baselines3.common.results_plotter import ts2xy
#from stable_baselines3.common.monitor import load_results

'''
DESCRIPTION: NUA TODO: Update!
'''
class PlotMobiman(object):
    
    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def __init__(self,
                 mobiman_path:str,
                 data_folder:str,
                 data_names:list,
                 plot_title:str,
                 plot_window_timestep:int,
                 plot_window_episode:int,
                 save_flag:bool,
                 save_folder:str,
                 observation_index:int,
                 action_index:int):
        print("[mobiman_plot_oar::PlotMobiman::__init__] START")
        
        self.mobiman_path = mobiman_path
        self.data_folder = data_folder
        self.data_names = data_names
        self.plot_title = plot_title
        self.plot_window_timestep = plot_window_timestep
        self.plot_window_episode = plot_window_episode
        self.save_flag = save_flag
        self.save_folder = save_folder
        self.action_indexs = action_index
        self.observation_index = observation_index
        
        #self.df = None
        '''
        try:
            os.chdir(self.data_folder)
        except Exception as e:
            print("[mobiman_plot_oar::PlotMobiman::__init__] Unable to find the folder! Program will now terminate!")
            sys.exit(1)
        '''
        #self.create_dataset()

        print("[mobiman_plot_oar::PlotMobiman::__init__] END")

    '''
    DESCRIPTION: TODO...
    '''
    def read_data_n_row(self, file):
        with open(file, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            data = np.array(next(reader))
            i = 1
            for row in reader:
                i += 1
            return i
        
    '''
    DESCRIPTION: TODO...
    '''
    def read_data_n_col(self, file):
        with open(file, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            data = np.array(next(reader))
            return len(data)

    '''
    DESCRIPTION: TODO...
    '''
    def read_data(self, file):
        with open(file, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            data = np.array(next(reader))
            for row in reader:
                data_row = np.array(row)
                data = np.vstack((data, data_row))
            return data

    '''
    DESCRIPTION: TODO...
    '''
    def get_data_col(self, file, col_name):
        print("[mobiman_plot_oar::PlotMobiman::get_data_col] START")
        
        data = self.read_data(file)
        data_keys = list(data[0])

        print("[mobiman_plot_oar::PlotMobiman::get_data_col] data_keys len: " + str(len(data_keys)))
        print(data_keys)

        idx = data_keys.index(col_name)
        print("[mobiman_plot_oar::PlotMobiman::get_data_col] col_idx: " + str(idx))

        data_col = []
        for i, row_vals in enumerate(data[1:]):
            data_col.append(row_vals[idx])
        
        print("[mobiman_plot_oar::PlotMobiman::get_data_col] END")
        return data_col
    
    '''
    DESCRIPTION: TODO...
    '''
    def get_data_col_episode(self, file, col_name, dtype="float", ep_end_token="[]"):
        print("[mobiman_plot_oar::PlotMobiman::get_data_col_episode] START")
        
        data_col = self.get_data_col(file, col_name)
        
        data_ep = []
        data_ep_single = []
        for i, d in enumerate(data_col):
            #print("i -> " + str(i) + str(d))

            if d == ep_end_token:
                data_ep.append(data_ep_single)
                data_ep_single = []
            else:
                dval = d
                if dtype == "float":
                    dval = float(dval)
                data_ep_single.append(dval)

        print("[mobiman_plot_oar::PlotMobiman::get_data_col_episode] END")
        return data_ep
    
    '''
    DESCRIPTION: TODO...
    '''
    def generate_dataframe(self) -> pd.DataFrame:
        main_path = os.path.join(self.mobiman_path, self.data_folder)
        folders = sorted(os.listdir(main_path))[::-1]
        files = deque()
        folder_ = None
        for folder in folders:
            try:
                log_csv = pd.read_csv(main_path + '/' + folder + '/' + 'training_log.csv', names=['key', 'value'])
                folder_ = folder
                break
            except Exception as e:
                print(e)
        while True:
            n_robot = None
            try:
                for i in range(10):
                    print(log_csv.iloc[i]['key'])
                    if log_csv.iloc[i]['key'] == 'n_robot':
                        n_robot = log_csv.iloc[i]['value']
                log_csv.iloc[14]['value']
                file = []
                for i in range(int(n_robot)): # type: ignore
                    file.append(f'{main_path}/{folder}/oar_data_training_jackalJaco_{i}.csv')
            except Exception as e:
                print(e)
                # print(log_csv.head(10))
                # n_robot = log_csv.iloc[6]['value']
                # log_csv.iloc[14]['value']
                # file = []
                # for i in range(int(n_robot)):
                #     file.append(f'{main_path}/{folder}/oar_data_training_jackalJaco_{i}.csv')
            files.append(file)
            if pd.notna(log_csv.iloc[14]['value']):
                folder = log_csv.iloc[14]['value'].replace('/','')
                # print(folder)
                log_csv = pd.read_csv(main_path + '/' + folder + '/' + 'training_log.csv', names=['key', 'value'])
            else:
                break
        main_df = None
        while len(files) != 0:
            csvs = files.pop()
            dfs = []
            for csv in csvs:
                dfs.append(pd.read_csv(csv))
            result = pd.concat(dfs).sort_index(kind='merge').reset_index(drop=True) # type: ignore
            if not isinstance(main_df, pd.DataFrame):
                main_df = result.copy()
            else:
                main_df = pd.concat([main_df, result], ignore_index=True)
        return main_df # type: ignore

    '''
    DESCRIPTION: TODO...
    '''
    def split_df_by_nan(self, df, by_row : str = "episode_index"):
        sub_dfs = []
        start_idx = None
    
        for idx, row in df.iterrows():
            if pd.isna(row[by_row]):
                if start_idx is not None:
                    sub_dfs.append(df.iloc[start_idx:idx+1])
                    start_idx = None
            else:
                if start_idx is None:
                    start_idx = idx
    
        if start_idx is not None:
            sub_dfs.append(df.iloc[start_idx:])
    
        return sub_dfs

    '''
    DESCRIPTION: TODO...
    '''
    def generate_dataframe_episodic(self) -> pd.DataFrame:
        main_path = os.path.join(self.mobiman_path, self.data_folder)
        folders = sorted(os.listdir(main_path))[::-1]
        files = deque()
        folder_ = None
        for folder in folders:
            try:
                log_csv = pd.read_csv(main_path + '/' + folder + '/' + 'training_log.csv', names=['key', 'value'])
                folder_ = folder
                break
            except Exception as e:
                print(e)
        while True:
            try:
                for i in range(10):
                    print(log_csv.iloc[i]['key'])
                    if log_csv.iloc[i]['key'] == 'n_robot':
                        n_robot = log_csv.iloc[i]['value']
                        break
                for i in range(20):
                    if log_csv.iloc[i]['key'] == 'initial_training_path':
                        prev_check = log_csv.iloc[i]['value']
                        break
                print('log_csv:', )
                file = []
                for i in range(int(n_robot)):
                    file.append(f'{main_path}/{folder}/oar_data_training_jackalJaco_{i}.csv')
            except Exception as e:
                print(e)
            files.append(file)
            if pd.notna(prev_check):
                folder = prev_check.replace('/','')
                # print(folder)
                log_csv = pd.read_csv(main_path + '/' + folder + '/' + 'training_log.csv', names=['key', 'value'])
            else:
                break
        main_df = None
        while len(files) != 0:
            csvs = files.pop()
            dfs = []
            for csv in csvs:
                dfs.append(self.split_df_by_nan(pd.read_csv(csv)))
            result = pd.concat(interleave(dfs), ignore_index=True)
            if not isinstance(main_df, pd.DataFrame):
                main_df = result.copy()
            else:
                main_df = pd.concat([main_df, result], ignore_index=True)
        print(len(main_df)) # type: ignore
        return main_df # type: ignore

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def plot_action(self):
        df = self.generate_dataframe()
        print("[mobiman_plot_oar::PlotMobiman::plot_action] START")

        columns = ['a0', 'a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7']
        columns_ = ['model_mode','self_collision_flag','target_pos_x', 'target_pos_y',
                   'target_pos_z', 'target_pos_roll', 'target_pos_pitch', 'target_pos_yaw']
        new_df = df.copy()
        new_df['Action'] = new_df['action'].apply(lambda x : [float(a) for a in x[1:-1].replace('\n', '').split(' ') if a != ''])
        new_df[columns] = pd.DataFrame(new_df['Action'].tolist(), index=new_df.index)
        # self.df.dropna(inplace=True)
        
        print("[mobiman_plot_oar::PlotMobiman::plot_action] action_sequences: " + str(self.action_sequences)) # type: ignore
        
        if self.action_sequences == [-1]: # type: ignore
            for i in range(0,8):
                new_df[f'a{i}'].hist()
                plt.title(f'Action {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                # plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_action_{i}.png')
                # if self.plot_flag:
                plt.show()
                # plt.close()
                
        else:
            for i in self.action_sequences: # type: ignore
                new_df[f'a{i}'].hist()
                plt.title(f'Action {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                # plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_action_{i}.png')
                # if self.plot_flag:
                plt.show()
                # plt.close()

        print("[mobiman_plot_oar::PlotMobiman::plot_action] END")

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def plot_observations(self):
        print("[mobiman_plot_oar::PlotMobiman::plot_observations] END")
        df = self.generate_dataframe()
        columns = [f'o{a}' for a in range(0,25)]
        columns_ = columns
        new_df = df.copy()
        new_df['Observation'] = new_df['observation'].apply(lambda x : [float(a) for a in x[1:-1].replace('\n', '').split(',') if a != ''])
        new_df[columns] = pd.DataFrame(new_df['Observation'].tolist(), index=new_df.index)
        # self.df.dropna(inplace=True)
        
        print("[mobiman_plot_oar::PlotMobiman::plot_observations] observation_sequences: " + str(self.observation_sequences)) # type: ignore

        if self.observation_sequences == [-1]: # type: ignore
            for i in range(0,74):
                new_df[f'o{i}'].hist()
                plt.title(f'Observation {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                # plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_observation_{i}.png')
                # if self.plot_flag:
                plt.show()
                # plt.close()
                
        else:
            for i in self.observation_sequences: # type: ignore
                new_df[f'o{i}'].hist()
                plt.title(f'Observation {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                # plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_observation_{i}.png')
                # if self.plot_flag:
                plt.show()
                # plt.close()

        print("[mobiman_plot_oar::PlotMobiman::plot_observations] END") 
    
    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def plot_reward_episodic(self, color='red', legend='PPO', plot=True, trim=True, trim_len=-1):
        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] START")
        df = self.generate_dataframe_episodic()
        
        # clean_df['rewards'] = 
        nan_indices = df.index[df['reward'].isnull()].tolist()
        result = []
        start = 0
        for end in nan_indices:
            interval_sum = df['reward'][start:end].sum()
            result.append(interval_sum)
            start = end + 1

        # Check if there's any remaining data after the last NaN
        if start < len(df):
            interval_sum = df['reward'][start:].sum()
            result.append(interval_sum)

        clean_df = pd.DataFrame({'reward':result})
        # clean_df.plot()

        clean_data = clean_df.dropna()
        window_size = 200
        clean_data['Reward'] = clean_data['reward'].apply(lambda x: float(x))
        clean_data['Reward'] = clean_data['Reward'].round(4)
        print("MEAN: ",len(clean_data['Reward'].rolling(window=window_size).mean()))
        print("STD: ",len(clean_data['Reward'].rolling(window=window_size).std()))
        clean_data['cumulative_reward'] = clean_data['Reward'].rolling(window=window_size).mean()
        clean_data['std_reward'] = clean_data['Reward'].rolling(window=window_size).std()
        x = np.arange(len(clean_data))
        upper_bound = clean_data['cumulative_reward'] + clean_data['std_reward']
        lower_bound = clean_data['cumulative_reward'] - clean_data['std_reward']
        rewards = clean_data['cumulative_reward']
        if trim:
            x = x[:trim_len]
            upper_bound = upper_bound[:trim_len]
            lower_bound = lower_bound[:trim_len]
            rewards = rewards[:trim_len]

        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] clean_data.info: ")
        print(clean_data.info())
        
        plt.plot(x, rewards, color=color, label=legend)
        # plt.fill_between(x, clean_data['cumulative_reward'].iloc[:]-clean_data['std_reward'].iloc[:],
        #                  clean_data['cumulative_reward'].iloc[:]+clean_data['std_reward'].iloc[:], facecolor='blue', alpha=0.5)
        plt.fill_between(x, lower_bound, upper_bound, color=color, alpha=0.1) # type: ignore
        plt.title(f'Rolling average of reward, window size {window_size}')
        plt.xlabel("Episodes")
        plt.ylabel("Rewards")
        
        if plot:
            plt.legend()
            plt.show()

        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] END")

    '''
    DESCRIPTION: TODO...
    '''
    def print_array(self, arr):
        for i in range(len(arr)):
            print(str(i) + " -> " + str(arr[i]))

    '''
    DESCRIPTION: TODO...
    '''
    def print_log(self, log_path):

        with open(log_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0

            print("----------")
            print("[mobiman_plot_oar::PlotMobiman::print_training_log]")
            for row in csv_reader:
                print(str(line_count) + " -> " + str(row[0]) + ": " + str(row[1]))
                line_count += 1
            print("----------")

    '''
    DESCRIPTION: TODO...
    '''
    def get_param_value_from_log(self, log_path, param_name):

        with open(log_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                if row[0] == param_name:
                    return row[1]
    
    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def plot_test_hist(self):
        for data in self.data_names:
            csv_path = self.mobiman_path + data
            df = pd.read_csv(csv_path)
            specific_value = 'goal'  # Change this to your specific value
            events = []

            # Initialize a list to store the current event
            current_event = []
            prev_val = None
            # Iterate over the DataFrame
            for index, row in df.iterrows():
                # print(row['action'])
                if row['result'] == '[]':
                # if isinstance(row['result'], list):  # Check if it's an empty value
                    if current_event and prev_val == specific_value:
                        events.append(current_event.copy())  # Append a copy of the current event
                    current_event = []  # Reset the current event
                else:
                # else Exception as e:
                    current_event.append(row['action'])
                    prev_val = row['result']

            # Check if the last event matches the specific value
            if current_event and current_event[-1] == specific_value:
                events.append(current_event.copy())
            if len(events) == 0:
                return
            try:
                # print()
                print(events[-1][-1])
                if isinstance(eval(events[-1][-1]), int):
                    events = [eval(item) for sublist in events for item in sublist]
                    plt.hist(events, weights=np.ones(len(events)) / len(events))
                    plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
                    plt.show()
            except Exception as e:
                events = [item for sublist in events for item in sublist]
                action_hist = []
                for x in events:
                    actions = [float(a) for a in x[1:-1].replace('\n', '').split(' ') if a != '']
                    if actions[0] < 0.3:
                        action_hist.append(1)
                    elif 0.3 <actions[0] <= 0.6:
                        action_hist.append(2)
                    else:
                        action_hist.append(3)
                fig, ax = plt.subplots(figsize=(10, 8))
                _ = ax.hist(action_hist, weights=np.ones(len(action_hist)) / len(action_hist))
                ax.set_xticks(np.arange(1,4))
                plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
                plt.show()

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def get_testing_result(self, filename):

        with open(filename, newline='') as csvfile: # type: ignore
            reader = csv.reader(csvfile, delimiter=',')
            data = np.array(next(reader))
            
            data_keys = list(data)

            #print("[mobiman_plot_oar::PlotMobiman::get_testing_result] filename: " + str(filename))
            #print("[mobiman_plot_oar::PlotMobiman::get_testing_result] data_keys len: " + str(len(data_keys)))
            #print(data_keys)

            idx_result = data_keys.index("result")
            idx_testing_eval_index = data_keys.index("testing_eval_index")

            n_testing_eval = 5          ### NUA TODO: Read from the log file!
            result_eval_goal = np.zeros(n_testing_eval)
            result_eval_oob = np.zeros(n_testing_eval)
            result_eval_collision = np.zeros(n_testing_eval)
            result_eval_rollover = np.zeros(n_testing_eval)
            result_eval_max_step = np.zeros(n_testing_eval)

            result_total_goal = []
            result_total_oob = []
            result_total_collision = []
            result_total_rollover = []
            result_total_max_step = []

            eval_flag = False
            for row in reader:
                if row[idx_result] == "[]" and eval_flag:
                    result_total_goal.append(np.mean(result_eval_goal))
                    result_total_oob.append(np.mean(result_eval_oob))
                    result_total_collision.append(np.mean(result_eval_collision))
                    result_total_rollover.append(np.mean(result_eval_rollover))
                    result_total_max_step.append(np.mean(result_eval_max_step))
                    
                    result_eval_goal = np.zeros(n_testing_eval)
                    result_eval_oob = np.zeros(n_testing_eval)
                    result_eval_collision = np.zeros(n_testing_eval)
                    result_eval_rollover = np.zeros(n_testing_eval)
                    result_eval_max_step = np.zeros(n_testing_eval)
                    eval_flag = False
                
                else:
                    if row[idx_result] != "[]":
                        eval_idx = int(float(row[idx_testing_eval_index])) - 1 # type: ignore

                        if row[idx_result] == "goal":
                            result_eval_goal[eval_idx] += 1

                        elif row[idx_result] == "out_of_boundary":
                            result_eval_oob[eval_idx] += 1

                        elif row[idx_result] == "collision":
                            result_eval_collision[eval_idx] += 1

                        elif row[idx_result] == "rollover":
                            result_eval_rollover[eval_idx] += 1

                        elif row[idx_result] == "max_step" :
                            result_eval_max_step[eval_idx] += 1

                        if eval_idx == n_testing_eval-1:
                            eval_flag = True

        if self.save_flag:
            n_time_goal = np.mean(result_total_goal)
            n_time_out_of_boundary = np.mean(result_total_oob)
            n_time_collision = np.mean(result_total_collision)
            n_time_rollover = np.mean(result_total_rollover)
            n_time_max_step = np.mean(result_total_max_step)

            y = np.array([n_time_goal, n_time_out_of_boundary, n_time_collision, n_time_rollover, n_time_max_step])
            labels = ["Goal", "Out_of_boundary", "Collision", "Rollover", "Max_step"]
            colors = ['green', 'magenta', 'red', 'orange', 'blue']
            explode = [0.2, 0, 0, 0, 0]

            plt.figure()
            plt.pie(y, colors=colors, labels=labels, explode=explode, autopct='%1.0f%%', pctdistance=1.1, labeldistance=1.3)
            #plt.legend()

            plot_title = ""
            if self.plot_title:
                plot_title = self.plot_title
          
            plt.title(plot_title)
            #plt.show()

            #if self.save_folder:
            #    file_folder = filename.split("/")
            #    fn = file_folder[-1]
            #    save_path = self.save_folder + "plot_pie_" + fn + "_result.png"
            #else:

            file_folder = filename.split("/")
            fn = file_folder[-1]
            file_folder = '/'.join(file_folder[:-1])
            save_path = file_folder + "/plot_pie_" + fn + "_result.png"

            #print("[mobiman_plot_oar::PlotMobiman::get_testing_result] Saving to " + str(save_path) + "!")
            plt.savefig(save_path)
            plt.close()

            #print("[mobiman_plot_oar::PlotMobiman::get_testing_result] result_total_goal: " + str(result_total_goal))
            #print("[mobiman_plot_oar::PlotMobiman::get_testing_result] result_total_goal len: " + str(len(result_total_goal)))
            #print("[mobiman_plot_oar::PlotMobiman::get_testing_result] result_total_goal mean: " + str(np.mean(result_total_goal)))
            #print("")
            #print("")
            #print("")

            return result_total_goal, result_total_oob, result_total_collision, result_total_rollover, result_total_max_step

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def get_testing_result_arena(self, folder_names):

        plt.figure()
        data_x = ["20k", "40k", "60k", "80k", "100k"]

        for i, fn in enumerate(folder_names): # type: ignore
            #print("[mobiman_plot_oar::get_testing_result_arena] fn: " + str(fn))

            prefix = "oar_data"
            files = os.listdir(self.mobiman_path + fn)

            # Filter files that start with the given prefix
            oar_data_files = [file for file in files if file.startswith(prefix)]

            #print("[mobiman_plot_oar::get_testing_result_arena] filtered_files: " + str(oar_data_files))

            if "ocs2wb" in fn:
                goal_avg = np.ones(5)
                data_label = "ocs2wb"
            else:
                goal_avg = np.zeros(5)

            print("fn: " + str(fn))
            for oar_data in oar_data_files:
                result_total_goal, result_total_oob, result_total_collision, result_total_rollover, result_total_max_step = self.get_testing_result(self.mobiman_path + fn + oar_data) # type: ignore

                if "ocs2wb" in oar_data:
                    goal_avg *= np.mean(result_total_goal)
                else:
                    if "20000" in oar_data:
                        idx = 0
                    elif "40000" in oar_data:
                        idx = 1
                    elif "60000" in oar_data:
                        idx = 2
                    elif "80000" in oar_data:
                        idx = 3
                    elif "100000" in oar_data:
                        idx = 4
                    
                    goal_avg[idx] = np.mean(result_total_goal)

                    if "ppo" in oar_data:
                        if "rewMGT" in oar_data:
                            data_label = "re4mpc-PPO"
                        else:
                            data_label = "re4mpc-PPO w/o Target reward"
                    elif "sac" in oar_data:
                        if "rewMGT" in oar_data:
                            data_label = "re4mpc-SAC"
                        else:
                            data_label = "re4mpc-SAC w/o Target reward"
                    elif "dqn" in oar_data:
                        if "rewMGT" in oar_data:
                            data_label = "re4mpc-DQN"
                        else:
                            data_label = "re4mpc-DQN w/o Target reward"

                    #print("oar_data: " + str(oar_data))
                    #print("idx: " + str(idx))
            
            #print("goal_avg: " + str(goal_avg))        
            plt.plot(data_x, goal_avg, label=data_label, ls='-', marker='o')

        plt.title("Success Rate wrt. Training Time")
        plt.xlabel("Training Timesteps")
        plt.ylabel("Rate of Reaching Goal")
        plt.legend()
        plt.grid()
        #plt.show()

        if self.save_flag:
            if self.save_folder:
                save_path = self.save_folder + "testing_result_arena.png"
            else:
                file_folder = self.mobiman_path + "dataset/drl/arena/"
                save_path = file_folder + "testing_result_arena.png"

            plt.savefig(save_path)
        
        plt.close()

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def string_to_array(self, input_str, delim):

        input_str = input_str[1:-1]

        #print("input_str")
        #print(input_str)

        input_array_split = input_str.split(delim)

        #print("input_array_split")
        #print(input_array_split)

        cleaned_arr = []
        for s in input_array_split:
            if s != '':
                cleaned_arr.append(float(s))

        #print("cleaned_arr")
        #print(cleaned_arr)

        return cleaned_arr

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def get_testing_analysis_model_mode(self, filename):
        
        with open(filename, newline='') as csvfile: # type: ignore
            reader = csv.reader(csvfile, delimiter=',')
            data = np.array(next(reader))
            
            data_keys = list(data)

            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] filename: " + str(filename))
            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] data_keys len: " + str(len(data_keys)))
            print(data_keys)

            idx_result = data_keys.index("result")
            idx_action = data_keys.index("action")
            idx_robot_pos_wrt_world = data_keys.index("robot_pos_wrt_world")
            idx_goal_pos_wrt_world = data_keys.index("goal_pos_wrt_world")
            idx_obs_pos_wrt_world = data_keys.index("obs_pos_wrt_world")

            robot_pos_traj_x = []
            robot_pos_traj_y = []
            mode_color_traj = []
            mode_color_traj = []

            n_mode0 = 0
            n_mode1 = 0
            n_mode2 = 0
            n_mode0_goal_tmp = 0
            n_mode1_goal_tmp = 0
            n_mode2_goal_tmp = 0
            n_mode0_goal = 0
            n_mode1_goal = 0
            n_mode2_goal = 0

            end_flag = False
            c = 0

            plt.figure()
            fig, ax = plt.subplots(sharex=True, sharey=True)
            rect = patches.Rectangle((-1.0, 1.825), 2, 0.35, linewidth=5, edgecolor='k', facecolor='orange')
            ax.add_patch(rect)
            ax.annotate("CONVEYOR BELT", (-0.55, 1.9))

            for row in reader:
                if row[idx_result] != "[]":

                    #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] c: " + str(c))

                    action = self.string_to_array(row[idx_action], " ")

                    #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] action: ")
                    #print(action)

                    robot_pos_wrt_world = self.string_to_array(row[idx_robot_pos_wrt_world], ",")
                    robot_pos_traj_x.append(robot_pos_wrt_world[0]) # type: ignore
                    robot_pos_traj_y.append(robot_pos_wrt_world[1]) # type: ignore

                    #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] action: ")
                    #print(action)

                    #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] robot_pos_wrt_world: ")
                    #print(robot_pos_wrt_world)
                    #print("")

                    if action[0] <= 0.3: # type: ignore
                        mode_color_traj.append("green")
                        n_mode0 += 1
                        n_mode0_goal_tmp += 1

                    elif action[0] > 0.6: # type: ignore
                        mode_color_traj.append("magenta")
                        n_mode2 += 1
                        n_mode2_goal_tmp += 1
                    
                    else:
                        mode_color_traj.append("blue")
                        n_mode1 += 1
                        n_mode1_goal_tmp += 1

                    if row[idx_result] == "goal":
                        end_flag = True
                        n_mode0_goal += n_mode0_goal_tmp
                        n_mode1_goal += n_mode1_goal_tmp
                        n_mode2_goal += n_mode2_goal_tmp

                    elif row[idx_result] == "time_horizon":
                        end_flag = False

                    else:
                        end_flag = True
                        mode_color_traj_tmp = mode_color_traj
                        for i, mc in enumerate(mode_color_traj_tmp):
                            mode_color_traj[i] = "gray"

                if end_flag:
                    
                    for j, x in enumerate(robot_pos_traj_x): # type: ignore
                        if j+1 <= (len(robot_pos_traj_x)-1):
                            #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] j: " + str(j))
                            #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] j+1: " + str(j+1))
                            #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] len(robot_pos_traj_x)-1: " + str((len(robot_pos_traj_x)-1)))
                            x_pos = [robot_pos_traj_x[j], robot_pos_traj_x[j+1]]
                            y_pos = [robot_pos_traj_y[j], robot_pos_traj_y[j+1]]
                            plt.plot(x_pos, y_pos, color=mode_color_traj[j], ls='-', marker='o')
                        else:
                            plt.plot(robot_pos_traj_x[j], robot_pos_traj_y[j], color=mode_color_traj[j], ls='-', marker='o')
                    
                    robot_pos_traj_x = []
                    robot_pos_traj_y = []
                    mode_color_traj = []
                    n_mode0_goal_tmp = 0
                    n_mode1_goal_tmp = 0
                    n_mode2_goal_tmp = 0
                    end_flag = False

                c += 1

        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] n_mode0: " + str(n_mode0))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] n_mode1: " + str(n_mode1))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] n_mode2: " + str(n_mode2))

        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] n_mode0_goal: " + str(n_mode0_goal))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] n_mode1_goal: " + str(n_mode1_goal))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] n_mode2_goal: " + str(n_mode2_goal))

        tot = n_mode0 + n_mode1 + n_mode2
        tot_goal = n_mode0_goal + n_mode1_goal + n_mode2_goal

        perc_mode0 = round(100 * n_mode0 / tot, 2)
        perc_mode1 = round(100 * n_mode1 / tot, 2)
        perc_mode2 = round(100 * n_mode2 / tot, 2)

        perc_mode0_goal = round(100 * n_mode0_goal / tot_goal, 2)
        perc_mode1_goal = round(100 * n_mode1_goal / tot_goal, 2)
        perc_mode2_goal = round(100 * n_mode2_goal / tot_goal, 2)

        label_mode0 = mlines.Line2D([], [], color='green', marker='o', ls='-', label='Base: ' + str(perc_mode0_goal) + "%")
        label_mode1 = mlines.Line2D([], [], color='blue', marker='o', ls='-', label='Arm: ' + str(perc_mode1_goal) + "%")
        label_mode2 = mlines.Line2D([], [], color='magenta', marker='o', ls='-', label='Whole-body: ' + str(perc_mode2_goal) + "%")
        plt.legend(handles=[label_mode0, label_mode1, label_mode2], loc='lower left')

        if "ppo" in filename:
            if "rewMGT" in filename:
                data_label = "re4mpc-PPO"
            else:
                data_label = "re4mpc-PPO w/o Target reward"
        elif "sac" in filename:
            if "rewMGT" in filename:
                data_label = "re4mpc-SAC"
            else:
                data_label = "re4mpc-SAC w/o Target reward"
        elif "dqn" in filename:
            if "rewMGT" in filename:
                data_label = "re4mpc-DQN"
            else:
                data_label = "re4mpc-DQN w/o Target reward"

        plt.title("Model Mode Map: " + str(data_label))
        plt.xlabel("x-axis [m]")
        plt.ylabel("y_axis [m]")
        #plt.legend()
        plt.grid()
        #plt.show()

        if self.save_flag:
            if self.save_folder:
                save_path = self.save_folder + "/testing_model_mode.png"
            else:
                file_folder = filename.split("/")
                file_folder = '/'.join(file_folder[:-1])
                save_path = file_folder + "/testing_model_mode.png"

            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_model_mode] Saving to " + str(save_path) + "!")
            #print("DEBUG_INF")
            #while 1:
            #    continue

            plt.savefig(save_path)
        plt.close()

    '''
    DESCRIPTION: TODO...
    '''
    def get_euclidean_distance_2D(self, p1, p2={"x":0.0, "y":0.0}):
        return math.sqrt((p1["x"] - p2["x"])**2 + (p1["y"] - p2["y"])**2)

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def get_testing_analysis_target_type(self, filename):
        
        #### NUA TODO: LEFT HEREEEEE, GET HISTOGRAM OF HOW MANY TIMES TARGET MODE IS SELECTED, COMPARE WRT SUCCESS RATE. 
        # ADD OBSTACLES AND GOAL IN MODEL MODE PLOT. ALSO ENABLE MULTI PLOTS WRT MODEL MODES FOR BETTER VISUALIZATION

        with open(filename, newline='') as csvfile: # type: ignore
            reader = csv.reader(csvfile, delimiter=',')
            data = np.array(next(reader))
            
            data_keys = list(data)

            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] filename: " + str(filename))
            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] data_keys len: " + str(len(data_keys)))
            print(data_keys)

            idx_result = data_keys.index("result")
            idx_action = data_keys.index("action")
            idx_observation = data_keys.index("observation")
            idx_robot_pos_wrt_world = data_keys.index("robot_pos_wrt_world")
            #idx_goal_pos_wrt_world = data_keys.index("goal_pos_wrt_world")
            #idx_obs_pos_wrt_world = data_keys.index("obs_pos_wrt_world")

            mode_color_traj = []
            goal_dist_when_target = []

            n_mode0 = 0
            n_mode1 = 0
            n_mode2 = 0
            n_mode0_goal_tmp = 0
            n_mode1_goal_tmp = 0
            n_mode2_goal_tmp = 0
            n_mode0_goal = 0
            n_mode1_goal = 0
            n_mode2_goal = 0

            n_target = 0
            n_goal = 0
            n_target_tmp = 0
            n_goal_tmp = 0
            n_target_success = 0
            n_goal_success = 0

            plt.figure()
            fig, ax = plt.subplots(sharex=True, sharey=True)
            rect = patches.Rectangle((-1.0, 1.85), 2, 0.35, linewidth=5, edgecolor='k', facecolor='orange')
            ax.add_patch(rect)
            ax.annotate("CONVEYOR BELT", (-0.55, 1.9))

            for row in reader:
                if row[idx_result] != "[]":
                    #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] c: " + str(c))

                    action = self.string_to_array(row[idx_action], " ")
                    observation = self.string_to_array(row[idx_observation], ",")

                    robot_pos_wrt_world = self.string_to_array(row[idx_robot_pos_wrt_world], ",")
                    #robot_pos_traj_x.append(robot_pos_wrt_world[0]) # type: ignore
                    #robot_pos_traj_y.append(robot_pos_wrt_world[1]) # type: ignore

                    #print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] action: ")
                    #print(action)

                    ### MODEL MODE
                    if action[0] <= 0.3: # type: ignore
                        mode_color_traj.append("green")
                        n_mode0 += 1
                        n_mode0_goal_tmp += 1

                    elif action[0] > 0.6: # type: ignore
                        mode_color_traj.append("magenta")
                        n_mode2 += 1
                        n_mode2_goal_tmp += 1
                    
                    else:
                        mode_color_traj.append("blue")
                        n_mode1 += 1
                        n_mode1_goal_tmp += 1

                    ### TARGET TYPE
                    if action[1] <= 0.5: # type: ignore
                        n_target += 1
                        n_target_tmp += 1

                        plt.plot(robot_pos_wrt_world[0], robot_pos_wrt_world[1], color='b', ls='-', marker='o')

                        goal_dist = self.get_euclidean_distance_2D({"x":observation[0], "y":observation[1]})
                        goal_dist_when_target.append(goal_dist)
                    else:
                        n_goal += 1
                        n_goal_tmp += 1

                        plt.plot(robot_pos_wrt_world[0], robot_pos_wrt_world[1], color='r', ls='-', marker='o')

                    if row[idx_result] == "goal":
                        end_flag = True
                        n_mode0_goal += n_mode0_goal_tmp
                        n_mode1_goal += n_mode1_goal_tmp
                        n_mode2_goal += n_mode2_goal_tmp

                        n_target_success += n_target_tmp
                        n_goal_success += n_goal_tmp

                        mode_color_traj = []
                        n_mode0_goal_tmp = 0
                        n_mode1_goal_tmp = 0
                        n_mode2_goal_tmp = 0
                        n_target_tmp = 0
                        n_goal_tmp = 0

                    elif row[idx_result] == "time_horizon":
                        end_flag = False

                    else:
                        end_flag = True
                        mode_color_traj_tmp = mode_color_traj
                        for i, mc in enumerate(mode_color_traj_tmp):
                            mode_color_traj[i] = "gray"

                        mode_color_traj = []
                        n_mode0_goal_tmp = 0
                        n_mode1_goal_tmp = 0
                        n_mode2_goal_tmp = 0
                        n_target_tmp = 0
                        n_goal_tmp = 0

        tot = n_target + n_goal
        tot_success = n_target_success + n_goal_success

        perc_target = round(100 * n_target / tot, 2)
        perc_goal = round(100 * n_goal / tot, 2)

        perc_target_success = round(100 * n_target_success / tot_success, 2)
        perc_goal_success = round(100 * n_goal_success / tot_success, 2)

        label_target = mlines.Line2D([], [], color='blue', marker='o', ls='-', label='Sub-Goal: ' + str(perc_target_success) + "%")
        label_goal = mlines.Line2D([], [], color='red', marker='o', ls='-', label='Goal: ' + str(perc_goal_success) + "%")
        plt.legend(handles=[label_target, label_goal])

        if "ppo" in filename:
            if "rewMGT" in filename:
                data_label = "re4mpc-PPO"
            else:
                data_label = "re4mpc-PPO w/o Target reward"
        elif "sac" in filename:
            if "rewMGT" in filename:
                data_label = "re4mpc-SAC"
            else:
                data_label = "re4mpc-SAC w/o Target reward"
        elif "dqn" in filename:
            if "rewMGT" in filename:
                data_label = "re4mpc-DQN"
            else:
                data_label = "re4mpc-DQN w/o Target reward"

        plt.title("Target Type Map: " + str(data_label))
        plt.xlabel("x-axis [m]")
        plt.ylabel("y_axis [m]")
        #plt.legend()
        plt.grid()
        #plt.show()

        if self.save_flag:
            if self.save_folder:
                save_path = self.save_folder + "/testing_target_type.png"
            else:
                file_folder = filename.split("/")
                file_folder = '/'.join(file_folder[:-1])
                save_path = file_folder + "/testing_target_type.png"

            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] Saving to " + str(save_path) + "!")
            #print("DEBUG_INF")
            #while 1:
            #    continue

            plt.savefig(save_path)
        plt.close()

        '''
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_mode0: " + str(n_mode0))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_mode1: " + str(n_mode1))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_mode2: " + str(n_mode2))

        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_mode0_goal: " + str(n_mode0_goal))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_mode1_goal: " + str(n_mode1_goal))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_mode2_goal: " + str(n_mode2_goal))

        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_target: " + str(n_target))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_goal: " + str(n_goal))

        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_target_success: " + str(n_target_success))
        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] n_goal_success: " + str(n_goal_success))

        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type] goal_dist_when_target len: " + str(len(goal_dist_when_target)))
        '''

        return n_mode0, n_mode1, n_mode2, n_mode0_goal, n_mode1_goal, n_mode2_goal, n_target, n_goal, n_target_success, n_goal_success, goal_dist_when_target

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def get_testing_analysis_target_type_arena(self, data_names):

        n_bin = 50
        data_label = []
        data_target_perc = []
        data_target_perc_success = []
        #data_hist_goal_dist = []
        #data_bin_goal_dist = []

        plt.figure(0)

        for dn in data_names:
            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type_arena] dn: " + str(dn))
            n_mode0, n_mode1, n_mode2, n_mode0_goal, n_mode1_goal, n_mode2_goal, n_target, n_goal, n_target_success, n_goal_success, goal_dist_when_target = self.get_testing_analysis_target_type(self.mobiman_path + dn)

            target_perc = 100 * n_target / (n_target+n_goal)
            target_perc_success = 100 * n_target_success / (n_target_success+n_goal_success)

            data_target_perc.append(target_perc)
            data_target_perc_success.append(target_perc_success)

            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type_arena] target_perc: " + str(target_perc))
            print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type_arena] target_perc_success: " + str(target_perc_success))

            hist, bin_edges = np.histogram(goal_dist_when_target, n_bin)

            #data_hist_goal_dist.append(hist)
            #data_bin_goal_dist.append(bin_edges)

            if "ppo" in dn:
                if "rewMGT" in dn:
                    data_label.append("re4mpc-PPO")
                else:
                    data_label.append("re4mpc-PPO w/o Target reward")
            elif "sac" in dn:
                if "rewMGT" in dn:
                    data_label.append("re4mpc-SAC")
                else:
                    data_label.append("re4mpc-SAC w/o Target reward")
            elif "dqn" in dn:
                if "rewMGT" in dn:
                    data_label.append("re4mpc-DQN")
                else:
                    data_label.append("re4mpc-DQN w/o Target reward")
            
            plt.hist(bin_edges[:-1], bin_edges, weights=hist, alpha=0.3, label=data_label[-1]) # type: ignore

        print("[mobiman_plot_oar::PlotMobiman::get_testing_analysis_target_type_arena] data_label len: " + str(len(data_label)))
        print(data_label)

        #print("data_hist_goal_dist  len: " + str(len(data_hist_goal_dist)))
        #print(data_hist_goal_dist[0])

        #print("data_bin_goal_dist len: " + str(len(data_bin_goal_dist)))
        #print(data_bin_goal_dist[0])

        #for i, dlab in enumerate(data_label):
        #    plt.hist(data_hist_goal_dist[i], 'auto', alpha=0.5, label=dlab)

        plt.title("Histogram of goal distance when Target Mode")
        plt.xlabel("Distance to Goal")
        plt.ylabel("Target Mode Count")
        plt.legend()
        plt.grid()
        plt.show()

        fig, ax = plt.subplots(1)
        y_pos = np.arange(len(data_label))

        ax.barh(y_pos, data_target_perc, align='center')
        ax.set_yticks(y_pos, labels=data_label)
        ax.invert_yaxis()  # labels read top-to-bottom
        ax.set_xlabel('Percentage of Target Mode when the result is success')
        ax.set_title('Target Reward Effect on Target Mode Percentage')

        #plt.title("Histogram of goal distance when Target Mode")
        #plt.xlabel("Distance to Goal")
        #plt.ylabel("Target Mode Count")
        plt.legend()
        plt.grid()
        plt.show()

'''
DESCRIPTION: NUA TODO: Update!
'''
if __name__ == '__main__':
    print("[mobiman_plot_oar::__main__] START")

    rospy.init_node('mobiman_plot_oar')

    rospack = rospkg.RosPack()
    mobiman_path = rospack.get_path('mobiman_simulation') + "/"
    
    data_folder = rospy.get_param('data_folder', "")
    data_names = rospy.get_param('data_names', [])

    plot_title = rospy.get_param('plot_title')
    plot_window_timestep = rospy.get_param('plot_window_timestep')
    plot_window_episode = rospy.get_param('plot_window_episode')
    
    save_flag = rospy.get_param('save_flag') 
    save_folder = rospy.get_param('save_folder') 

    plot_result_pie_flag = rospy.get_param('plot_result_pie_flag')
    plot_result_arena_flag = rospy.get_param('plot_result_arena_flag')
    plot_analysis_model_mode_flag = rospy.get_param('plot_analysis_model_mode_flag')
    plot_analysis_target_type_flag = rospy.get_param('plot_analysis_target_type_flag')

    plot_reward_flag = rospy.get_param('plot_reward_flag')
    plot_action_hist_flag = rospy.get_param('plot_action_hist_flag')
    plot_observation_hist_flag = rospy.get_param('plot_observation_hist_flag')

    ### NUA TODO: Use the same histogram function to plot both training and testing data!!!
    plot_test_hist_flag = rospy.get_param('plot_observation_hist_flag')

    observation_index = rospy.get_param('observation_index')
    action_index = rospy.get_param('action_index')
       
    print("[mobiman_plot_oar::__main__] mobiman_path: " + str(mobiman_path))
    print("[mobiman_plot_oar::__main__] data_folder: " + str(data_folder))
    print("[mobiman_plot_oar::__main__] data_names: " + str(data_names))
    print("[mobiman_plot_oar::__main__] plot_title: " + str(plot_title))
    print("[mobiman_plot_oar::__main__] plot_window_timestep: " + str(plot_window_timestep))
    print("[mobiman_plot_oar::__main__] plot_window_episode: " + str(plot_window_episode))
    print("[mobiman_plot_oar::__main__] save_flag: " + str(save_flag))
    print("[mobiman_plot_oar::__main__] save_folder: " + str(save_folder))
    print("[mobiman_plot_oar::__main__] plot_result_pie_flag: " + str(plot_result_pie_flag))
    print("[mobiman_plot_oar::__main__] plot_result_arena_flag: " + str(plot_result_arena_flag))
    print("[mobiman_plot_oar::__main__] plot_analysis_model_mode_flag: " + str(plot_analysis_model_mode_flag))
    print("[mobiman_plot_oar::__main__] plot_analysis_target_type_flag: " + str(plot_analysis_target_type_flag))
    print("[mobiman_plot_oar::__main__] plot_reward_flag: " + str(plot_reward_flag))
    print("[mobiman_plot_oar::__main__] plot_action_hist_flag: " + str(plot_action_hist_flag))
    print("[mobiman_plot_oar::__main__] plot_observation_hist_flag: " + str(plot_observation_hist_flag))
    print("[mobiman_plot_oar::__main__] plot_test_hist_flag: " + str(plot_test_hist_flag))
    print("[mobiman_plot_oar::__main__] observation_index: " + str(observation_index))
    print("[mobiman_plot_oar::__main__] action_index: " + str(action_index))

    colors=['r', 'g', 'b', 'm', 'c', 'y']

    plot_mobiman = PlotMobiman(mobiman_path=mobiman_path,
                               data_folder=data_folder, # type: ignore
                               data_names=data_names, # type: ignore
                               plot_title=plot_title, # type: ignore
                               plot_window_timestep=plot_window_timestep, # type: ignore
                               plot_window_episode=plot_window_episode, # type: ignore
                               save_flag=save_flag, # type: ignore
                               save_folder=save_folder, # type: ignore               
                               observation_index=observation_index, # type: ignore
                               action_index=action_index) # type: ignore

    if plot_result_pie_flag:
        for dn in data_names: # type: ignore
            file_path = plot_mobiman.mobiman_path + dn
            print("[mobiman_plot_oar::__main__] data_name: " + dn)
            plot_mobiman.get_testing_result(file_path)

    if plot_result_arena_flag:
        plot_mobiman.get_testing_result_arena(data_folder)

    if plot_analysis_model_mode_flag:
        for dn in data_names: # type: ignore
            file_path = plot_mobiman.mobiman_path + dn
            print("[mobiman_plot_oar::__main__] data_name: " + dn)
            plot_mobiman.get_testing_analysis_model_mode(file_path)

    if plot_analysis_target_type_flag:
        for dn in data_names: # type: ignore
            file_path = plot_mobiman.mobiman_path + dn
            print("[mobiman_plot_oar::__main__] data_name: " + dn)
            plot_mobiman.get_testing_analysis_target_type(file_path)
        
        #plot_mobiman.get_testing_analysis_target_type_arena(data_names)

    if plot_reward_flag:
        for i, data in enumerate(data_folder): # type: ignore
            plot_mobiman = PlotMobiman(mobiman_path=mobiman_path,
                                       data_folder=data, # type: ignore
                                       data_names=data_names, # type: ignore
                                       plot_title=plot_title, # type: ignore
                                       plot_window_timestep=plot_window_timestep, # type: ignore
                                       plot_window_episode=plot_window_episode, # type: ignore
                                       save_flag=save_flag, # type: ignore
                                       save_folder=save_folder, # type: ignore               
                                       observation_index=observation_index, # type: ignore
                                       action_index=action_index) # type: ignore
            plot_mobiman.plot_reward_episodic(plot=False, color=colors[i], legend=data.split('/')[-1]) # type: ignore
        plt.legend(loc='upper left')
        plt.show()

    if plot_action_hist_flag:
        plot_mobiman.plot_action()
    
    if plot_observation_hist_flag:
        plot_mobiman.plot_observations()   

    if plot_test_hist_flag:
        plot_mobiman = PlotMobiman(mobiman_path=mobiman_path,
                        data_folder="", # type: ignore
                        data_names=data_names, # type: ignore
                        plot_title=plot_title, # type: ignore
                        plot_window_timestep=plot_window_timestep, # type: ignore
                        plot_window_episode=plot_window_episode, # type: ignore
                        save_flag=save_flag, # type: ignore
                        save_folder=save_folder, # type: ignore               
                        observation_index=observation_index, # type: ignore
                        action_index=action_index) # type: ignore
        plot_mobiman.plot_test_hist()
    
    print("[mobiman_plot_oar::__main__] END")
