#!/usr/bin/env python3

'''
LAST UPDATE: 2023.06.30

AUTHOR: Neset Unver Akmandor (NUA)
        Eric Dusel (ED)

E-MAIL: akmandor.n@northeastern.edu
        dusel.e@northeastern.edu

DESCRIPTION: TODO...

NUA TODO:
'''

import rospy
import rospkg
import csv
import numpy as np

'''
DESCRIPTION: TODO...
'''
def write_data(file, data):
    file_status = open(file, 'a')
    with file_status:
        write = csv.writer(file_status)
        write.writerows(data)
        print("mobiman_drl_config::write_data -> Data is written in " + str(file))

'''
DESCRIPTION: TODO...
'''
def read_data(file):
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
def get_training_param(initial_training_path, param_name) -> str:

    log_file = initial_training_path + 'training_log.csv'
    with open(log_file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0] == param_name:
                return row[1]

'''
NUA TODO: This config class seems unnecessary. Include all parameters into the main config file called by the launch file.
DESCRIPTION: TODO...
'''
class Config():

    def __init__(self, data_folder_path="", odom={}, goal={}):        

        print("[mobiman_drl_config::Config::__init__] START")
        print("[mobiman_drl_config::Config::__init__] data_folder_path: " + str(data_folder_path))

        rospack = rospkg.RosPack()
        mobiman_path = rospack.get_path('mobiman_simulation') + "/"

        ## General
        self.mode = rospy.get_param('mode', "")

        print("[mobiman_drl_config::Config::__init__] mode: " + str(self.mode))

        if self.mode == "training":
            
            print("[mobiman_drl_config::Config::__init__] START training")

            self.world_frame_name = rospy.get_param('world_frame_name', "")
            self.max_episode_steps = rospy.get_param('max_episode_steps', 0)
            self.training_timesteps = rospy.get_param('training_timesteps', 0)

            ## Sensors
            self.laser_size_downsampled = rospy.get_param('laser_size_downsampled', 0)
            self.laser_normalize_flag = rospy.get_param('laser_normalize_flag', False)
            self.laser_error_threshold = rospy.get_param('laser_error_threshold', 0.0)

            ## Robots
            '''
            self.velocity_control_msg = rospy.get_param('robot_velo_control_msg', "")
            self.velocity_control_data_path = rospy.get_param('trajectory_data_path', "")
            velocity_control_data_str = read_data(mobiman_path + self.velocity_control_data_path + "velocity_control_data.csv")
            self.velocity_control_data = np.zeros(velocity_control_data_str.shape)

            for i, row in enumerate(velocity_control_data_str):
                for j, val in enumerate(row):
                    self.velocity_control_data[i][j] = float(val)

            self.min_lateral_speed = min(self.velocity_control_data[:,0])               # [m/s]
            self.max_lateral_speed = max(self.velocity_control_data[:,0])               # [m/s]
            self.init_lateral_speed = self.velocity_control_data[0,0]                   # [m/s]

            self.min_angular_speed = min(self.velocity_control_data[:,1])               # [rad/s]
            self.max_angular_speed = max(self.velocity_control_data[:,1])               # [rad/s]
            self.init_angular_speed = self.velocity_control_data[0,1]                   # [rad/s]
            '''

            ## Algorithm
            self.observation_space_type = rospy.get_param('observation_space_type', "")

            self.goal_close_threshold = rospy.get_param("goal_close_threshold", 0.0)
            self.obs_min_range = rospy.get_param('obs_min_range', 0.0)

            self.n_actions = 3
            self.n_observations = 333

            self.n_obs_stack = rospy.get_param("n_obs_stack", 0.0)
            self.n_skip_obs_stack = rospy.get_param("n_skip_obs_stack", 0.0)

            self.cnn_obs_shape = (1,-1)
            self.fc_obs_shape = (-1, )

            self.occupancy_image_width = rospy.get_param('occupancy_image_width',0)
            self.occupancy_image_height = rospy.get_param('occupancy_image_height',0)
            
            # Rewards
            self.reward_terminal_success = rospy.get_param('reward_terminal_success', 0.0)
            self.reward_step_scale = rospy.get_param('reward_step_scale', 0.0)
            self.penalty_terminal_fail = rospy.get_param('penalty_terminal_fail', 0.0)
            self.penalty_cumulative_step = rospy.get_param('penalty_cumulative_step', 0.0)
            #self.reward_terminal_mintime = rospy.get_param('reward_terminal_mintime', 0.0)
            
            if data_folder_path:

                ## Write all parameters
                training_log_file = data_folder_path + "training_log.csv"

                training_log_data = []
                training_log_data.append(["mode", self.mode])
                training_log_data.append(["world_frame_name", self.world_frame_name])
                training_log_data.append(["max_episode_steps", self.max_episode_steps])
                training_log_data.append(["training_timesteps", self.training_timesteps])
                training_log_data.append(["occupancy_image_width", self.occupancy_image_width])
                training_log_data.append(["occupancy_image_height", self.occupancy_image_height])
                training_log_data.append(["observation_space_type", self.observation_space_type])
                training_log_data.append(["goal_close_threshold", self.goal_close_threshold])
                training_log_data.append(["obs_min_range", self.obs_min_range])
                training_log_data.append(["n_actions", self.n_actions])
                training_log_data.append(["n_observations", self.n_observations])
                training_log_data.append(["n_obs_stack", self.n_obs_stack])
                training_log_data.append(["n_skip_obs_stack", self.n_skip_obs_stack])
                training_log_data.append(["cnn_obs_shape", self.cnn_obs_shape])
                training_log_data.append(["fc_obs_shape", self.fc_obs_shape])
                training_log_data.append(["reward_terminal_success", self.reward_terminal_success])
                training_log_data.append(["reward_step_scale", self.reward_step_scale])
                training_log_data.append(["penalty_terminal_fail", self.penalty_terminal_fail])
                training_log_data.append(["penalty_cumulative_step", self.penalty_cumulative_step])
                #training_log_data.append(["reward_terminal_mintime", self.reward_terminal_mintime])

                write_data(training_log_file, training_log_data)

        ### NUA TODO: OUT OF DATE! UPDATE!
        elif self.mode == "testing":

            print("[mobiman_drl_config::Config::__init__] DEBUG INF testing")
            while 1:
                continue

            self.initial_training_path = mobiman_path + rospy.get_param('initial_training_path', "")
            self.max_testing_episodes = rospy.get_param('max_testing_episodes', "")

            self.world_frame_name = get_training_param(self.initial_training_path, "world_frame_name")
            self.max_episode_steps = int(get_training_param(self.initial_training_path, "max_episode_steps"))
            self.training_timesteps = int(get_training_param(self.initial_training_path, "training_timesteps"))

            ## Sensors
            self.laser_size_downsampled = int(get_training_param(self.initial_training_path, "laser_size_downsampled"))
            self.laser_error_threshold = float(get_training_param(self.initial_training_path, "laser_error_threshold"))

            if get_training_param(self.initial_training_path, "laser_normalize_flag") == "False":
                self.laser_normalize_flag = False
            else:
                self.laser_normalize_flag = True

            ## Robots
            self.velocity_control_msg = rospy.get_param('robot_velo_control_msg', "")
            self.velocity_control_data_path = get_training_param(self.initial_training_path, "velocity_control_data_path")
            velocity_control_data_str = read_data(mobiman_path + self.velocity_control_data_path + "velocity_control_data.csv")
            self.velocity_control_data = np.zeros(velocity_control_data_str.shape)

            for i, row in enumerate(velocity_control_data_str):
                for j, val in enumerate(row):
                    self.velocity_control_data[i][j] = float(val)

            self.min_lateral_speed = min(self.velocity_control_data[:,0])               # [m/s]
            self.max_lateral_speed = max(self.velocity_control_data[:,0])               # [m/s]
            self.init_lateral_speed = self.velocity_control_data[0,0]                   # [m/s]

            self.min_angular_speed = min(self.velocity_control_data[:,1])               # [rad/s]
            self.max_angular_speed = max(self.velocity_control_data[:,1])               # [rad/s]
            self.init_angular_speed = self.velocity_control_data[0,1]                   # [rad/s]

            ## Algorithm
            self.observation_space_type = get_training_param(self.initial_training_path, "observation_space_type")

            self.goal_close_threshold = float(get_training_param(self.initial_training_path, "goal_close_threshold"))
            self.obs_min_range = float(get_training_param(self.initial_training_path, "obs_min_range"))

            self.n_actions = len(self.velocity_control_data)
            self.n_observations = self.n_actions

            self.n_obs_stack = int(get_training_param(self.initial_training_path, "n_obs_stack"))
            self.n_skip_obs_stack = int(get_training_param(self.initial_training_path, "n_skip_obs_stack"))

            self.cnn_obs_shape = (1,-1)
            self.fc_obs_shape = (-1, )

            # Waypoints
            if self.observation_space_type == "mobiman_WP_FC" or \
                self.observation_space_type == "laser_WP_1DCNN_FC":
 
                self.n_wp = int(get_training_param(self.initial_training_path, "n_wp"))
                self.look_ahead = float(get_training_param(self.initial_training_path, "look_ahead"))
                self.wp_reached_dist = float(get_training_param(self.initial_training_path, "wp_reached_dist"))
                self.wp_global_dist = float(get_training_param(self.initial_training_path, "wp_global_dist"))
                self.wp_dynamic = int(get_training_param(self.initial_training_path, "wp_dynamic"))

            # Rewards
            self.reward_terminal_success = float(get_training_param(self.initial_training_path, "reward_terminal_success"))
            self.reward_step_scale = float(get_training_param(self.initial_training_path, "reward_step_scale"))
            self.penalty_terminal_fail = float(get_training_param(self.initial_training_path, "penalty_terminal_fail"))
            self.penalty_cumulative_step = float(get_training_param(self.initial_training_path, "penalty_cumulative_step"))
            #self.reward_terminal_mintime = float(get_training_param(self.initial_training_path, "reward_terminal_mintime"))
                
            '''
            if data_folder_path:

                ## Write all parameters
                testing_log_file = data_folder_path + "testing_input_log.csv"

                testing_log_data = []
                testing_log_data.append(["mode", self.mode])
                testing_log_data.append(["initial_training_path", self.initial_training_path])
                
                write_data(testing_log_file, testing_log_data)
            '''

        print("[mobiman_drl_config::Config::__init__] world_frame_name: " + str(self.world_frame_name))
        print("[mobiman_drl_config::Config::__init__] max_episode_steps: " + str(self.max_episode_steps))
        print("[mobiman_drl_config::Config::__init__] training_timesteps: " + str(self.training_timesteps))
        #print("[mobiman_drl_config::Config::__init__] laser_size_downsampled: " + str(self.laser_size_downsampled))
        #print("[mobiman_drl_config::Config::__init__] laser_normalize_flag: " + str(self.laser_normalize_flag))
        #print("[mobiman_drl_config::Config::__init__] laser_error_threshold: " + str(self.laser_error_threshold))
        #print("[mobiman_drl_config::Config::__init__] velocity_control_data_path: " + str(self.velocity_control_data_path))
        print("[mobiman_drl_config::Config::__init__] observation_space_type: " + str(self.observation_space_type))
        print("[mobiman_drl_config::Config::__init__] goal_close_threshold: " + str(self.goal_close_threshold))
        print("[mobiman_drl_config::Config::__init__] obs_min_range: " + str(self.obs_min_range))
        print("[mobiman_drl_config::Config::__init__] n_actions: " + str(self.n_actions))
        print("[mobiman_drl_config::Config::__init__] n_observations: " + str(self.n_observations))
        print("[mobiman_drl_config::Config::__init__] n_obs_stack: " + str(self.n_obs_stack))
        print("[mobiman_drl_config::Config::__init__] n_skip_obs_stack: " + str(self.n_skip_obs_stack))
        print("[mobiman_drl_config::Config::__init__] reward_terminal_success: " + str(self.reward_terminal_success))
        print("[mobiman_drl_config::Config::__init__] reward_step_scale: " + str(self.reward_step_scale))
        print("[mobiman_drl_config::Config::__init__] penalty_terminal_fail: " + str(self.penalty_terminal_fail))
        print("[mobiman_drl_config::Config::__init__] penalty_cumulative_step: " + str(self.penalty_cumulative_step))

        if odom:
            self.x = odom["x"]
            self.y = odom["y"]
            self.th = odom["theta"]
            self.v = odom["u"]
            self.omega = odom["omega"]

        if goal:
            self.goalX = goal["x"]
            self.goalY = goal["y"]

        '''
        print("--------------")
        print("Config::__init__ -> x: " + str(odom["x"]))
        print("Config::__init__ -> y: " + str(odom["y"]))
        print("Config::__init__ -> theta: " + str(odom["theta"]))
        print("Config::__init__ -> u: " + str(odom["u"]))
        print("Config::__init__ -> omega: " + str(odom["omega"]))
        print("--------------")
        '''

    '''
    NUA TODO: 
    '''
    def set_odom(self, odom):

        self.x = odom["x"]
        self.y = odom["y"]
        self.theta = odom["theta"]
        self.v = odom["u"]
        self.omega = odom["omega"]

    '''
    NUA TODO: 
    '''
    def set_goal(self, goal):

        self.goalX = goal["x"]
        self.goalY = goal["y"]

    '''
    NUA TODO: 
    '''
    def set_laser_data(self, laser_scan):

        self.laser_frame_id = laser_scan.header.frame_id
        self.laser_angle_min = laser_scan.angle_min                   # [rad]
        self.laser_angle_max = laser_scan.angle_max                   # [rad]
        self.laser_angle_increment = laser_scan.angle_increment       # [rad]
        self.laser_range_min = laser_scan.range_min                   # [m]
        self.laser_range_max = laser_scan.range_max                   # [m]
        self.laser_time_increment = laser_scan.time_increment
        self.laser_scan_time = laser_scan.scan_time
        self.laser_n_range = len(laser_scan.ranges)
        self.laser_downsample_scale = 1
        if 0 < self.laser_size_downsampled < len(laser_scan.ranges):
            self.laser_downsample_scale = int(len(laser_scan.ranges) / self.laser_size_downsampled)
            self.laser_n_range = self.laser_size_downsampled
            self.laser_angle_increment = (self.laser_angle_max - self.laser_angle_min) / self.laser_size_downsampled
