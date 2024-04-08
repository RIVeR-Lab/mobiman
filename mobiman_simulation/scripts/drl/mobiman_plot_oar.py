#!/usr/bin/python3

'''
LAST UPDATE: 2024.04.08

AUTHOR:	Sarvesh Prajapati (SP)
        Neset Unver Akmandor (NUA)	

E-MAIL: prajapati.s@northeastern.edu
        akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:

NUA TODO:
- 
'''

import os
import sys
import time
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
#from mobiman.mobiman_simulation.scripts.drl.mobiman_drl_training import print_array
from collections import deque
import rospy
import rospkg
from toolz import interleave
from matplotlib.ticker import PercentFormatter
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
    DESCRIPTION: TODO...
    '''
    def plot_rewards(self):
        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] START")
        df = self.generate_dataframe()
        clean_data = df[df['reward'].notna()]

        window_size = 100
        clean_data['Reward'] = clean_data['reward'].apply(lambda x: float(x))
        clean_data['Reward'] = clean_data['Reward'].round(4)
        clean_data['cumulative_reward'] = clean_data['Reward'].rolling(window=window_size).mean()
        clean_data['reward'].to_clipboard()
        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] clean_data.info: ")
        print(clean_data.info())
        
        plt.plot(clean_data['cumulative_reward'].iloc[:])
        plt.title(f'Rolling average of reward, window size {window_size}')
        plt.xlabel("Steps")
        plt.ylabel("Rewards")
        plt.show()

        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] END")    
    
    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def plot_rewards_episodic(self, color='red', legend='PPO', plot=True, trim=True, trim_len=-1):
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
    def plot_result(self, file, title=""):
        data_result_ep = self.get_data_col_episode(file, "result", "str")
        
        n_time_goal = 0
        n_time_out_of_boundary = 0
        n_time_collision = 0
        n_time_rollover = 0
        n_time_max_step = 0

        for res_ep in data_result_ep:
            if "goal" in res_ep:
                n_time_goal += 1

            elif "out_of_boundary" in res_ep:
                n_time_out_of_boundary += 1

            elif "collision" in res_ep:
                n_time_collision += 1

            elif "rollover" in res_ep:
                n_time_rollover += 1

            elif "max_step" in res_ep:
                n_time_max_step += 1

        #print("[mobiman_plot_oar::PlotMobiman::plot_result] n_time_goal: " + str(n_time_goal))
        #print("[mobiman_plot_oar::PlotMobiman::plot_result] n_time_out_of_boundary: " + str(n_time_out_of_boundary))
        #print("[mobiman_plot_oar::PlotMobiman::plot_result] n_time_collision: " + str(n_time_collision))
        #print("[mobiman_plot_oar::PlotMobiman::plot_result] n_time_rollover: " + str(n_time_rollover))
        #print("[mobiman_plot_oar::PlotMobiman::plot_result] n_time_max_step: " + str(n_time_max_step))

        y = np.array([n_time_goal, n_time_out_of_boundary, n_time_collision, n_time_rollover, n_time_max_step])
        labels = ["Goal", "Out_of_boundary", "Collision", "Rollover", "Max_step"]
        colors = ['green', 'magenta', 'red', 'orange', 'blue']
        explode = [0.2, 0, 0, 0, 0]

        plt.figure()
        plt.pie(y, colors=colors, labels=labels, explode=explode, autopct='%1.0f%%', pctdistance=1.1, labeldistance=1.3)
        #plt.legend()

        if self.plot_title:
            plot_title = self.plot_title + title
        else:
            plot_title = title
        plt.title(plot_title)
        #plt.show()

        if self.save_flag:
            if self.save_folder:
                file_folder = file.split("/")
                filename = file_folder[-1]
                plt.savefig(self.save_folder + filename + "_result.png")
            else:
                file_folder = file.split("/")
                filename = file_folder[-1]
                file_folder = '/'.join(file_folder[:-1])
                save_path = file_folder + "/plot_pie_" + filename + "_result.png"
            
            print("[mobiman_plot_oar::PlotMobiman::plot_result] Saving to " + str(save_path) + "!")
            plt.savefig(save_path)

    '''
    DESCRIPTION: TODO...
    '''
    def plot_reward_episode(self, file, title=""):
        print("[mobiman_plot_oar::PlotMobiman::plot_reward_episode] START")
        
        data_reward_ep = self.get_data_col_episode(file, "reward", ep_end_token="")

        n_data = len(data_reward_ep)
        ep_index = range(n_data)
        #ep_total = np.linspace(0, n_data, n_data) # type: ignore
        rew_ep_total = []
        for i, rew_ep in enumerate(data_reward_ep):
            #print(str(i) + " -> " + str(len(rew_ep)))
            rew_ep_total.append(sum(rew_ep))

        file_folder = file.split("/")
        filename = file_folder[-1]
        file_folder = '/'.join(file_folder[:-1])
        save_path = file_folder + "/" + filename + "_reward_episode.png"

        print("[mobiman_plot_oar::PlotMobiman::plot_reward_episode] save_path: " + str(save_path))

        self.plot_func( # type: ignore
            ep_index, rew_ep_total,
            data_label='', data_color='k',
            label_x='ep_index', label_y='rew_ep_total', title='Reward (Episode)', 
            save_path=save_path)

        print("[mobiman_plot_oar::PlotMobiman::plot_reward_episode] END")

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

            print("[mobiman_plot_oar::PlotMobiman::get_data_col] data_keys len: " + str(len(data_keys)))
            print(data_keys)

            idx_result = data_keys.index("result")
            idx_testing_index = data_keys.index("testing_index")
            idx_testing_eval_index = data_keys.index("testing_eval_index")

            n_testing_eval = 5          ### NUA TODO: Read from the log file!
            result_eval_goal = np.zeros(n_testing_eval)
            result_eval_oob = np.zeros(n_testing_eval)
            result_eval_collision = np.zeros(n_testing_eval)
            result_eval_rollover = np.zeros(n_testing_eval)
            result_eval_max_step = np.zeros(n_testing_eval)

            n_ep = 0
            for row in reader:
                eval_idx = row[idx_testing_eval_index] - 1

                if row[idx_result] == "goal":
                    n_time_goal += 1
                    n_ep += 1

                elif row[idx_result] == "out_of_boundary":
                    n_time_out_of_boundary += 1
                    n_ep += 1

                elif row[idx_result] == "collision":
                    n_time_collision += 1
                    n_ep += 1

                elif row[idx_result] == "rollover":
                    n_time_rollover += 1
                    n_ep += 1

                elif row[idx_result] == "max_step" :
                    n_time_max_step += 1
                    n_ep += 1

                
                result_eval[eval_idx] = np.array(row)
                data = np.vstack((data, data_row))

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    def get_testing_result_arena(self, folder_names):

        for i, fn in enumerate(folder_names): # type: ignore
            print("[mobiman_plot_oar::get_testing_result_arena] fn: " + str(fn))

            prefix = "oar_data"
            files = os.listdir(self.mobiman_path + fn)

            # Filter files that start with the given prefix
            oar_data_files = [file for file in files if file.startswith(prefix)]

            print("[mobiman_plot_oar::get_testing_result_arena] filtered_files: " + str(oar_data_files))

            for oar_data in oar_data_files:
                self.get_testing_result(self.mobiman_path + fn + "/" + oar_data)

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

    plot_result_flag = rospy.get_param('plot_result_flag')
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
    print("[mobiman_plot_oar::__main__] plot_result_flag: " + str(plot_result_flag))
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

    if plot_result_flag:

        plot_mobiman.get_testing_result_arena(data_folder)

        for dn in data_names: # type: ignore
            file_path = plot_mobiman.mobiman_path + dn
            # n_row = plot_mobiman.read_data_n_row(file_path)
            # n_col = plot_mobiman.read_data_n_col(file_path)

            print("[mobiman_plot_oar::__main__] data_name: " + dn)
            # print("[mobiman_plot_oar::__main__] n_row: " + str(n_row))
            # print("[mobiman_plot_oar::__main__] n_col: " + str(n_col))

            #plot_mobiman.plot_result(file_path)

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
            plot_mobiman.plot_rewards_episodic(plot=False, color=colors[i], legend=data.split('/')[-1]) # type: ignore
        plt.legend(loc='upper left')
        plt.show()

        '''
        for dn in data_names: # type: ignore
            file_path = plot_mobiman.mobiman_path + dn
            n_row = plot_mobiman.read_data_n_row(file_path)
            n_col = plot_mobiman.read_data_n_col(file_path)

            print("[mobiman_plot_oar::__main__] data_name: " + dn)
            print("[mobiman_plot_oar::__main__] n_row: " + str(n_row))
            print("[mobiman_plot_oar::__main__] n_col: " + str(n_col))

            plot_mobiman.plot_reward_episode(file_path, title="")
            #plot_mobiman.plot_reward()
        '''

    if plot_action_hist_flag:
        plot_mobiman.plot_action()
    
    if plot_observation_hist_flag:
        plot_mobiman.plot_observations()   

    if plot_test_hist_flag:
    # for dn in data_names:
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
