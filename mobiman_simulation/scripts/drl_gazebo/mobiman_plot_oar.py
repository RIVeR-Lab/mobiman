#!/usr/bin/python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import rospy
import numpy as np
import time


class PlotMobiman(object):
    def __init__(self, data_folder:str, plot_flag:str, plot_path:str, observation_sequences:list, action_sequences:list, continue_initial: bool, continue_initial_count: int, window_episodic: int):
        self.data_folder = data_folder
        self.plot_flag = plot_flag
        self.plot_path = plot_path
        self.observation_sequences = observation_sequences
        self.action_sequences = action_sequences
        self.continue_initial = continue_initial
        self.continue_initial_count = continue_initial_count
        self.window_episodic = window_episodic
        self.df = None
        try:
            os.chdir(self.data_folder)
        except Exception as e:
            print("[+] Unable to find the folder! Program will now terminate!")
            sys.exit(1)
        self.create_dataset()

    def create_dataset(self):
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
        print(data_paths)
        for data in data_paths:
            if self.continue_initial_count > 0:
                self.continue_initial_count -= 1
                new_data = pd.read_csv(data)
                new_data.index += len(self.df) - 1
                print(len(new_data))
                self.df = pd.concat([self.df, new_data])
        # print(self.df.head())
        # print(self.df.info())
        # print(self.df.tail())
        

    def plot_rewards(self):
        clean_data = self.df.dropna()
        window_size = self.window_episodic
        clean_data['Reward'] = clean_data['Reward'].apply(lambda x: float(x))
        clean_data['Reward'] = clean_data['Reward'].round(4)
        clean_data['cumulative_reward'] = clean_data['Reward'].rolling(window=window_size).mean()
        print(clean_data.info())
        plt.plot(clean_data['cumulative_reward'].iloc[:])
        plt.title(f'Rolling average of reward, window size {window_size}')
        plt.xlabel("Steps")
        plt.ylabel("Rewards")
        plt.show()
    

    def plot_episodic_reward(self):
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


    def plot_action(self):
        columns = ['a0', 'a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7']
        columns_ = ['model_mode','self_collision_flag','target_pos_x', 'target_pos_y',
                   'target_pos_z', 'target_pos_roll', 'target_pos_pitch', 'target_pos_yaw']
        new_df = self.df.copy()
        new_df['Action'] = new_df['Action'].apply(lambda x : [float(a) for a in x[1:-1].replace('\n', '').split(' ') if a != ''])
        new_df[columns] = pd.DataFrame(new_df['Action'].tolist(), index=new_df.index)
        self.df.dropna(inplace=True)
        print(self.action_sequences, '****')
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
    
    def plot_observations(self):
        columns = [f'o{a}' for a in range(0,74)]
        columns_ = columns
        new_df = self.df.copy()
        new_df['Observation'] = new_df['Observation'].apply(lambda x : [float(a) for a in x[1:-1].replace('\n', '').split(',') if a != ''])
        new_df[columns] = pd.DataFrame(new_df['Observation'].tolist(), index=new_df.index)
        self.df.dropna(inplace=True)
        print(self.observation_sequences, '****')
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


if __name__ == '__main__':
    rospy.init_node('mobiman_drl_training_plot')
    print("[+] Plotting node created!")
    data_path = rospy.get_param('data_path')
    observation_sequences = [int(a) for a in str(rospy.get_param('observation_sequences')).split(',')]
    action_sequences = [int(a) for a in str(rospy.get_param('action_sequences')).split(',')]
    continue_initial = rospy.get_param('continue_initial')
    continue_initial_count = rospy.get_param('continue_initial_count')
    plot_flag = rospy.get_param('plot_flag')
    plot_path = rospy.get_param('plot_path')
    window_episodic = rospy.get_param('window_episodic')
    plot_mobiman = PlotMobiman(data_path,plot_flag, plot_path, observation_sequences, action_sequences, continue_initial, continue_initial_count, window_episodic)
    plot_mobiman.plot_episodic_reward()
    plot_mobiman.plot_action()
    plot_mobiman.plot_observations()