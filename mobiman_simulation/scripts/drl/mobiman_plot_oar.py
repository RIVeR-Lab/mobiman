#!/usr/bin/python3

'''
LAST UPDATE: 2024.02.24

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
                 plot_path:str,
                 plot_flag:bool,
                 plot_title:str,
                 plot_window_timestep:int,
                 plot_window_episode:int,
                 save_flag:bool,
                 data_folder:str,
                 data_names=list,
                 observation_sequence=int,
                 action_sequence=int):
        print("[mobiman_plot_oar::PlotMobiman::__init__] START")
        
        self.mobiman_path = mobiman_path
        self.plot_path = plot_path
        self.plot_flag = plot_flag
        self.plot_title = plot_title
        self.plot_window_timestep = plot_window_timestep
        self.plot_window_episode = plot_window_episode
        self.save_flag = save_flag
        self.data_folder = data_folder
        self.data_names = data_names
        self.action_sequences = action_sequence
        self.observation_sequences = observation_sequence
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
    def get_data_col(self, file, col_name, dtype="float"):
        data = self.read_data(file)
        data_keys = list(data[0])
        print(len(data_keys))
        print(data_keys)

        idx = data_keys.index(col_name)
        print(idx)

        data_col = []
        for i, row_vals in enumerate(data[1:]):
            if dtype == "str":
                data_col.append(row_vals[idx])
            else:    
                data_col.append(float(row_vals[idx]))
        
        return data_col
    
    '''
    DESCRIPTION: TODO...
    '''
    def get_data_col_episode(self, file, col_name, dtype="float", ep_end_token="[]"):

        data_col = self.get_data_col(file, col_name, dtype)
        
        data_ep = []
        data_ep_single = []
        for d in data_col:
            #print(d)
            if d == ep_end_token:
                data_ep.append(data_ep_single)
                data_ep_single = []
            else:
                data_ep_single.append(d)

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
            n_robot = log_csv.iloc[6]['value']
            log_csv.iloc[14]['value']
            file = []
            for i in range(int(n_robot)):
                file.append(f'{main_path}/{folder}/oar_data_training_jackalJaco_{i}.csv')
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
            result = pd.concat(dfs).sort_index(kind='merge').reset_index(drop=True)
            if not isinstance(main_df, pd.DataFrame):
                main_df = result.copy()
            else:
                main_df = pd.concat([main_df, result], ignore_index=True)
        return main_df

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
            n_robot = log_csv.iloc[6]['value']
            log_csv.iloc[14]['value']
            file = []
            for i in range(int(n_robot)):
                file.append(f'{main_path}/{folder}/oar_data_training_jackalJaco_{i}.csv')
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
            counter = 0
            dfs_len = [len(a) for a in dfs]
            dfs_idx = [0 for _ in dfs]
            while True:
                # print(dfs_len, dfs_idx)
                if dfs_len == dfs_idx:
                    break
                if dfs_len[counter] <= dfs_idx[counter]:
                    counter = (counter + 1) % len(dfs)
                    continue
                # result = pd.concat(dfs).sort_index(kind='merge').reset_index(drop=True)
                if not isinstance(main_df, pd.DataFrame):
                    main_df = pd.DataFrame(columns=list(dfs[counter].columns))
                    # main_df = main_df.append(dfs[counter].iloc[dfs_idx[counter]], ignore_index=True)
                # else:
                main_df = pd.concat([main_df, dfs[counter].iloc[dfs_idx[counter]]], ignore_index=True)
                    # main_df = main_df.append(dfs[counter].iloc[dfs_idx[counter]], ignore_index=True)
                dfs_idx[counter] += 1
                if pd.isna(dfs[counter]['episode_index'].iloc[dfs_idx[counter] - 1]):
                    counter = (counter + 1) % len(dfs)
        print('maindf: ', main_df)
        return main_df


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
        
        print("[mobiman_plot_oar::PlotMobiman::plot_action] action_sequences: " + str(self.action_sequences))
        
        if self.action_sequences == [-1]:
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
            for i in self.action_sequences:
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
        
        print("[mobiman_plot_oar::PlotMobiman::plot_observations] observation_sequences: " + str(self.observation_sequences))

        if self.observation_sequences == [-1]:
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
            for i in self.observation_sequences:
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
            if self.plot_path:
                save_path = plot_path
                plt.savefig(save_path)
            else:
                file_folder = file.split("/")
                filename = file_folder[-1]
                file_folder = '/'.join(file_folder[:-1])
                save_path = file_folder + "/" + filename + "_result.png"
            
            print("[mobiman_plot_oar::PlotMobiman::plot_result] Saving to " + str(save_path) + "!")
            plt.savefig(save_path)

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
    '''
    def create_dataset(self):
        print("[mobiman_plot_oar::PlotMobiman::create_dataset] START")
        
        data_paths = []
        self.df = pd.read_csv(os.path.join(os.getcwd(), 'oar_data.csv'))
        if self.plot_path == "":
            self.plot_path = os.path.join(os.getcwd(), 'plots')
        if not os.path.exists(self.plot_path):
            os.makedirs(self.plot_path)
        data_paths.append(os.path.join(os.getcwd(), 'oar_data.csv'))
        self.df = self.df[0:0]
        
        if continue_initial == True:
            while not pd.isnull(pd.read_csv('training_log.csv')['training'].iloc[13]):
                prev_data = pd.read_csv('training_log.csv')['training'].iloc[13]
                try:
                    os.chdir(f'../{prev_data}')
                    data_paths.append(os.path.join(os.getcwd(), 'oar_data.csv'))
                except Exception as e:
                    print(f"[-] {prev_data} not found!")
                    break
        
        data_paths = data_paths[::-1]
        if self.continue_initial_count == 0:
            self.continue_initial_count = len(data_paths)
        
        print("[mobiman_plot_oar::PlotMobiman::create_dataset] data_paths:")
        print(data_paths)
        
        for data in data_paths:
            if self.continue_initial_count > 0:
                self.continue_initial_count -= 1
                new_data = pd.read_csv(data)
                new_data.index += len(self.df) - 1
                print(len(new_data))
                self.df = pd.concat([self.df, new_data])
        
        print("[mobiman_plot_oar::PlotMobiman::create_dataset] df.head: " + str(self.df.head()))
        print("[mobiman_plot_oar::PlotMobiman::create_dataset] df.info: " + str(self.df.info()))
        print("[mobiman_plot_oar::PlotMobiman::create_dataset] df.tail: " + str(self.df.tail()))
        print("[mobiman_plot_oar::PlotMobiman::create_dataset] END")
    '''
    
    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    '''
    def plot_rewards(self):
        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] START")
        
        clean_data = self.df.dropna()
        window_size = self.window_episodic
        clean_data['Reward'] = clean_data['Reward'].apply(lambda x: float(x))
        clean_data['Reward'] = clean_data['Reward'].round(4)
        clean_data['cumulative_reward'] = clean_data['Reward'].rolling(window=window_size).mean()
        
        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] clean_data.info: ")
        print(clean_data.info())
        
        plt.plot(clean_data['cumulative_reward'].iloc[:])
        plt.title(f'Rolling average of reward, window size {window_size}')
        plt.xlabel("Steps")
        plt.ylabel("Rewards")
        plt.show()

        print("[mobiman_plot_oar::PlotMobiman::plot_rewards] END")
    '''

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    '''
    def plot_episodic_reward(self):
        print("[mobiman_plot_oar::PlotMobiman::plot_episodic_reward] START")

        episodic_reward = []
        prev_counter = 0
        window_size = self.window_episodic
        for i in range(len(self.df)):
            if self.df['Reward'].iloc[i] == '[]':
                episodic_reward.append(self.df['Reward'].iloc[prev_counter:i].apply(lambda x: float(x)).sum())
                prev_counter = i + 1
        episodic_df = pd.DataFrame(np.asarray(episodic_reward).T, columns=['col'])
        plt.plot(episodic_df.rolling(window=window_size).mean())
        plt.title(f'Rolling average of Episodic reward, window size {window_size}')
        plt.xlabel('Episodes')
        plt.ylabel('Reward')
        plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_episodic.png')
        if self.plot_flag:
            plt.show()
        plt.close()

        print("[mobiman_plot_oar::PlotMobiman::plot_episodic_reward] END")
    '''

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    '''
    def plot_action(self):
        print("[mobiman_plot_oar::PlotMobiman::plot_action] START")

        columns = ['a0', 'a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7']
        columns_ = ['model_mode','self_collision_flag','target_pos_x', 'target_pos_y',
                   'target_pos_z', 'target_pos_roll', 'target_pos_pitch', 'target_pos_yaw']
        new_df = self.df.copy()
        new_df['Action'] = new_df['Action'].apply(lambda x : [float(a) for a in x[1:-1].replace('\n', '').split(' ') if a != ''])
        new_df[columns] = pd.DataFrame(new_df['Action'].tolist(), index=new_df.index)
        self.df.dropna(inplace=True)
        
        print("[mobiman_plot_oar::PlotMobiman::plot_action] action_sequences: " + str(self.action_sequences))
        
        if self.action_sequences == [-1]:
            for i in range(0,8):
                new_df[f'a{i}'].hist()
                plt.title(f'Action {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_action_{i}.png')
                if self.plot_flag:
                    plt.show()
                plt.close()
                
        else:
            for i in self.action_sequences:
                new_df[f'a{i}'].hist()
                plt.title(f'Action {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_action_{i}.png')
                if self.plot_flag:
                    plt.show()
                plt.close()

        print("[mobiman_plot_oar::PlotMobiman::plot_action] END")
    '''

    '''
    DESCRIPTION: NUA TODO: Update!
    '''
    '''
    def plot_observations(self):
        print("[mobiman_plot_oar::PlotMobiman::plot_observations] END")

        columns = [f'o{a}' for a in range(0,74)]
        columns_ = columns
        new_df = self.df.copy()
        new_df['Observation'] = new_df['Observation'].apply(lambda x : [float(a) for a in x[1:-1].replace('\n', '').split(',') if a != ''])
        new_df[columns] = pd.DataFrame(new_df['Observation'].tolist(), index=new_df.index)
        self.df.dropna(inplace=True)
        
        print("[mobiman_plot_oar::PlotMobiman::plot_observations] observation_sequences: " + str(self.observation_sequences))

        if self.observation_sequences == [-1]:
            for i in range(0,74):
                new_df[f'o{i}'].hist()
                plt.title(f'Observation {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_observation_{i}.png')
                if self.plot_flag:
                    plt.show()
                plt.close()
                
        else:
            for i in self.observation_sequences:
                new_df[f'o{i}'].hist()
                plt.title(f'Observation {columns_[i]} Histogram')
                plt.xlabel(f'{columns_[i]}')
                plt.ylabel('Count')
                plt.savefig(f'{self.plot_path}/{self.data_folder.split("/")[-2]}_observation_{i}.png')
                if self.plot_flag:
                    plt.show()
                plt.close()

        print("[mobiman_plot_oar::PlotMobiman::plot_observations] END")
    '''

'''
DESCRIPTION: NUA TODO: Update!
'''
if __name__ == '__main__':
    print("[mobiman_plot_oar::__main__] START")

    rospy.init_node('mobiman_plot_oar')

    rospack = rospkg.RosPack()
    mobiman_path = rospack.get_path('mobiman_simulation') + "/"
    
    plot_path = rospy.get_param('plot_path')
    plot_flag = rospy.get_param('plot_flag')
    plot_title = rospy.get_param('plot_title')
    plot_window_timestep = rospy.get_param('plot_window_timestep')
    plot_window_episode = rospy.get_param('plot_window_episode')
    observation_sequence = rospy.get_param('observation_sequence')
    action_sequence = rospy.get_param('action_sequence')
    save_flag = rospy.get_param('save_flag')    
    
    data_folder = rospy.get_param('data_folder')
    try:
        data_names = rospy.get_param('data_names')
    except Exception as e:
        data_names = []
    
    plot_rewards = rospy.get_param('plot_rewards')

    print("[mobiman_plot_oar::__main__] mobiman_path: " + str(mobiman_path))
    print("[mobiman_plot_oar::__main__] plot_path: " + str(plot_path))
    print("[mobiman_plot_oar::__main__] plot_flag: " + str(plot_flag))
    print("[mobiman_plot_oar::__main__] plot_title: " + str(plot_title))
    print("[mobiman_plot_oar::__main__] plot_window_timestep: " + str(plot_window_timestep))
    print("[mobiman_plot_oar::__main__] plot_window_episode: " + str(plot_window_episode))
    print("[mobiman_plot_oar::__main__] save_flag: " + str(save_flag))
    print("[mobiman_plot_oar::__main__] data_folder: " + str(data_folder))
    print("[mobiman_plot_oar::__main__] data_names: " + str(data_names))

    plot_mobiman = PlotMobiman(mobiman_path=mobiman_path,
                               plot_path=plot_path, # type: ignore
                               plot_flag=plot_flag, # type: ignore
                               plot_title=plot_title, # type: ignore
                               plot_window_timestep=plot_window_timestep, # type: ignore
                               plot_window_episode=plot_window_episode, # type: ignore
                               save_flag=save_flag, # type: ignore
                               data_folder=data_folder, # type: ignore
                               data_names=data_names,
                               observation_sequence=observation_sequence,
                               action_sequence=action_sequence) # type: ignore


    if plot_rewards:
        plot_mobiman.plot_rewards()
        plot_mobiman.plot_action()
        plot_mobiman.plot_observations()
    else:
        for dn in data_names: # type: ignore
            file_path = plot_mobiman.mobiman_path + dn
            n_row = plot_mobiman.read_data_n_row(file_path)
            n_col = plot_mobiman.read_data_n_col(file_path)

            print("[mobiman_plot_oar::__main__] data_name: " + dn)
            print("[mobiman_plot_oar::__main__] n_row: " + str(n_row))
            print("[mobiman_plot_oar::__main__] n_col: " + str(n_col))

            #data_result = plot_mobiman.get_data_col(file_path, "result", "str")
            #plot_mobiman.print_array(data_result)

            #data_result_ep = plot_mobiman.get_data_col_episode(file_path, "result", "str")
            #plot_mobiman.print_array(data_result_ep)

            #get_param_value_from_log(log_path, param_name)

            plot_mobiman.plot_result(file_path, title="")

    '''
    plot_path = rospy.get_param('data_path')
    observation_sequences = [int(a) for a in str(rospy.get_param('observation_sequences')).split(',')]
    action_sequences = [int(a) for a in str(rospy.get_param('action_sequences')).split(',')]
    continue_initial = rospy.get_param('continue_initial')
    continue_initial_count = rospy.get_param('continue_initial_count')
    
    plot_mobiman = PlotMobiman(data_path,plot_flag, plot_path, observation_sequences, action_sequences, continue_initial, continue_initial_count, window_episodic) # type: ignore
    plot_mobiman.plot_episodic_reward()
    plot_mobiman.plot_action()
    plot_mobiman.plot_observations()
    '''

    print("[mobiman_plot_oar::__main__] END")