#!/usr/bin/env python3

'''
LAST UPDATE: 2023.08.23

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

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
        print("[mobiman_drl_config::write_data] Data is written in " + str(file))

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
    return ""

'''
DESCRIPTION: TODO...
'''
class Config():

    def __init__(self, data_folder_path=""):        

        print("[mobiman_drl_config::Config::__init__] START")
        print("[mobiman_drl_config::Config::__init__] data_folder_path: " + str(data_folder_path))

        ## General
        self.ros_pkg_name = rospy.get_param('ros_pkg_name', "")
        self.mode = rospy.get_param('mode', "")
        self.world_name = rospy.get_param('world_name', "")
        self.world_frame_name = rospy.get_param('world_frame_name', "")
        self.robot_frame_name = rospy.get_param('robot_frame_name', "")
        self.arm_joint_names = rospy.get_param('arm_joint_names', "")
        self.n_armstate = len(self.arm_joint_names) # type: ignore
        self.ee_frame_name = rospy.get_param('ee_frame_name', "")
        self.goal_frame_name = rospy.get_param('goal_frame_name', "")
        self.imu_msg_name = rospy.get_param('imu_msg_name', "")
        self.arm_state_msg_name = rospy.get_param('arm_state_msg_name', "")
        self.target_msg_name = rospy.get_param('target_msg_name', "")
        self.occgrid_msg_name = rospy.get_param('occgrid_msg_name', "")
        self.selfcoldistance_msg_name = rospy.get_param('selfcoldistance_msg_name', "")
        self.extcoldistance_base_msg_name = rospy.get_param('extcoldistance_base_msg_name', "")
        self.extcoldistance_arm_msg_name = rospy.get_param('extcoldistance_arm_msg_name', "")
        self.pointsonrobot_msg_name = rospy.get_param('pointsonrobot_msg_name', "")
        self.goal_status_msg_name = rospy.get_param('goal_status_msg_name', "")

        self.data_folder_path = data_folder_path

        rospack = rospkg.RosPack()
        self.ros_pkg_path = rospack.get_path(self.ros_pkg_name) + "/"

        if self.mode == "training":

            print("[mobiman_drl_config::Config::__init__] START training")

            self.training_log_name = rospy.get_param('training_log_name', "")
            self.max_episode_steps = rospy.get_param('max_episode_steps', 0)
            self.training_timesteps = rospy.get_param('training_timesteps', 0)

            ## Sensors
            self.occgrid_normalize_flag = rospy.get_param('occgrid_normalize_flag', False)
            self.occgrid_occ_min = rospy.get_param('occgrid_occ_min', False)
            self.occgrid_occ_max = rospy.get_param('occgrid_occ_max', False)

            ## Algorithm
            self.observation_space_type = rospy.get_param('observation_space_type', "")

            self.action_time_horizon = rospy.get_param("action_time_horizon", 0.0)
            self.action_type = rospy.get_param("action_type", 0)
            self.n_action_model = rospy.get_param("n_action_model", 0.0)
            self.n_action_constraint = rospy.get_param("n_action_constraint", 0.0)
            self.n_action_target = rospy.get_param("n_action_target", 0.0)

            self.ablation_mode = rospy.get_param("ablation_mode", 0.0)
            self.last_step_distance_threshold = rospy.get_param("last_step_distance_threshold", 0.0)
            self.goal_distance_pos_threshold = rospy.get_param("goal_distance_pos_threshold", 0.0)
            self.goal_distance_ori_threshold_yaw = rospy.get_param("goal_distance_ori_threshold_yaw", 0.0)
            self.goal_distance_ori_threshold_quat = rospy.get_param("goal_distance_ori_threshold_quat", 0.0)
            self.goal_range_min_x = rospy.get_param("goal_range_min_x", 0.0)
            self.goal_range_max_x = rospy.get_param('goal_range_max_x', 0.0)
            self.goal_range_min_y = rospy.get_param('goal_range_min_y', 0.0)
            self.goal_range_max_y = rospy.get_param('goal_range_max_y', 0.0)
            self.goal_range_min_z = rospy.get_param('goal_range_min_z', 0.0)
            self.goal_range_max_z = rospy.get_param('goal_range_max_z', 0.0)
            self.self_collision_range_min = rospy.get_param('self_collision_range_min', 0.0)
            self.self_collision_range_max = rospy.get_param('self_collision_range_max', 0.0)
            self.ext_collision_range_base_min = rospy.get_param('ext_collision_range_base_min', 0.0)
            self.ext_collision_range_arm_min = rospy.get_param('ext_collision_range_arm_min', 0.0)
            self.ext_collision_range_max = rospy.get_param('ext_collision_range_max', 0.0)
            self.rollover_pitch_threshold = rospy.get_param('rollover_pitch_threshold', 0.0)
            self.rollover_roll_threshold = rospy.get_param('rollover_roll_threshold', 0.0)
            
            self.n_obs_stack = rospy.get_param("n_obs_stack", 0.0)
            self.n_skip_obs_stack = rospy.get_param("n_skip_obs_stack", 0.0)

            self.fc_obs_shape = (-1, )
            self.cnn_obs_shape = (1,-1)

            if self.action_type == 0:
                self.n_action = self.n_action_model + pow(2, self.n_action_constraint) + self.n_action_target # type: ignore
            else:
                self.n_action = rospy.get_param("n_action", 0.0)

            # Rewards
            self.reward_terminal_goal = rospy.get_param('reward_terminal_goal', 0.0)
            self.reward_terminal_collision = rospy.get_param('reward_terminal_collision', 0.0)
            self.reward_terminal_roll = rospy.get_param('reward_terminal_roll', 0.0)
            self.reward_terminal_max_step = rospy.get_param('reward_terminal_max_step', 0.0)

            self.reward_step_goal = rospy.get_param('reward_step_goal', 0.0)
            self.reward_step_target = rospy.get_param('reward_step_target', 0.0)
            self.reward_step_mode0 = rospy.get_param('reward_step_mode0', 0.0)
            self.reward_step_mode1 = rospy.get_param('reward_step_mode1', 0.0)
            self.reward_step_mode2 = rospy.get_param('reward_step_mode2', 0.0)
            self.reward_step_mpc_exit = rospy.get_param('reward_step_mpc_exit', 0.0)

            self.alpha_step_goal_base = rospy.get_param('alpha_step_goal_base', 0.0)
            self.alpha_step_goal_ee = rospy.get_param('alpha_step_goal_ee', 0.0)
            self.alpha_step_target = rospy.get_param('alpha_step_target', 0.0)
            self.alpha_step_mode = rospy.get_param('alpha_step_mode', 0.0)
            self.alpha_step_mpc_exit = rospy.get_param('alpha_step_mpc_exit', 0.0)
            
            if data_folder_path:

                ## Write all parameters
                training_log_file = data_folder_path + self.training_log_name + ".csv" # type: ignore

                training_log_data = []
                training_log_data.append(["mode", self.mode])
                training_log_data.append(["world_name", self.world_name])
                training_log_data.append(["world_frame_name", self.world_frame_name])
                training_log_data.append(["robot_frame_name", self.robot_frame_name])
                for i, jname in enumerate(self.arm_joint_names): # type: ignore
                    training_log_data.append(["arm_joint_names_" + str(i), jname])
                training_log_data.append(["n_armstate", self.n_armstate])
                training_log_data.append(["ee_frame_name", self.ee_frame_name])
                training_log_data.append(["goal_frame_name", self.goal_frame_name])
                training_log_data.append(["imu_msg_name", self.imu_msg_name])
                training_log_data.append(["arm_state_msg_name", self.arm_state_msg_name])
                training_log_data.append(["target_msg_name", self.target_msg_name])
                training_log_data.append(["occgrid_msg_name", self.occgrid_msg_name])
                training_log_data.append(["selfcoldistance_msg_name", self.selfcoldistance_msg_name])
                training_log_data.append(["extcoldistance_base_msg_name", self.extcoldistance_base_msg_name])
                training_log_data.append(["extcoldistance_arm_msg_name", self.extcoldistance_arm_msg_name])
                training_log_data.append(["pointsonrobot_msg_name", self.pointsonrobot_msg_name])
                training_log_data.append(["goal_status_msg_name", self.goal_status_msg_name])
                training_log_data.append(["max_episode_steps", self.max_episode_steps])
                training_log_data.append(["training_timesteps", self.training_timesteps])
                training_log_data.append(["occgrid_normalize_flag", self.occgrid_normalize_flag])
                training_log_data.append(["occgrid_occ_min", self.occgrid_occ_min])
                training_log_data.append(["occgrid_occ_max", self.occgrid_occ_max])
                training_log_data.append(["observation_space_type", self.observation_space_type])
                training_log_data.append(["action_time_horizon", self.action_time_horizon])
                training_log_data.append(["action_type", self.action_type])
                training_log_data.append(["n_action_model", self.n_action_model])
                training_log_data.append(["n_action_constraint", self.n_action_constraint])
                training_log_data.append(["n_action_target", self.n_action_target])
                training_log_data.append(["ablation_mode", self.ablation_mode])
                training_log_data.append(["last_step_distance_threshold", self.last_step_distance_threshold])
                training_log_data.append(["goal_distance_pos_threshold", self.goal_distance_pos_threshold])
                training_log_data.append(["goal_distance_ori_threshold_yaw", self.goal_distance_ori_threshold_yaw])
                training_log_data.append(["goal_distance_ori_threshold_quat", self.goal_distance_ori_threshold_quat])
                training_log_data.append(["goal_range_min_x", self.goal_range_min_x])
                training_log_data.append(["goal_range_max_x", self.goal_range_max_x])
                training_log_data.append(["goal_range_min_y", self.goal_range_min_y])
                training_log_data.append(["goal_range_max_y", self.goal_range_max_y])
                training_log_data.append(["goal_range_min_z", self.goal_range_min_z])
                training_log_data.append(["goal_range_max_z", self.goal_range_max_z])
                training_log_data.append(["self_collision_range_min", self.self_collision_range_min])
                training_log_data.append(["self_collision_range_max", self.self_collision_range_max])
                training_log_data.append(["ext_collision_range_base_min", self.ext_collision_range_base_min])
                training_log_data.append(["ext_collision_range_arm_min", self.ext_collision_range_arm_min])
                training_log_data.append(["ext_collision_range_max", self.ext_collision_range_max])
                training_log_data.append(["rollover_pitch_threshold", self.rollover_pitch_threshold])
                training_log_data.append(["rollover_roll_threshold", self.rollover_roll_threshold])
                training_log_data.append(["n_obs_stack", self.n_obs_stack])
                training_log_data.append(["n_skip_obs_stack", self.n_skip_obs_stack]) 
                training_log_data.append(["fc_obs_shape", self.fc_obs_shape])
                training_log_data.append(["cnn_obs_shape", self.cnn_obs_shape])
                training_log_data.append(["n_action", self.n_action])
                training_log_data.append(["reward_terminal_goal", self.reward_terminal_goal])
                training_log_data.append(["reward_terminal_collision", self.reward_terminal_collision])
                training_log_data.append(["reward_terminal_roll", self.reward_terminal_roll])
                training_log_data.append(["reward_terminal_max_step", self.reward_terminal_max_step])
                training_log_data.append(["reward_step_goal", self.reward_step_goal])
                training_log_data.append(["reward_step_target", self.reward_step_target])
                training_log_data.append(["reward_step_mode0", self.reward_step_mode0])
                training_log_data.append(["reward_step_mode1", self.reward_step_mode1])
                training_log_data.append(["reward_step_mode2", self.reward_step_mode2])
                training_log_data.append(["reward_step_mpc_exit", self.reward_step_mpc_exit])
                training_log_data.append(["alpha_step_goal_base", self.alpha_step_goal_base])
                training_log_data.append(["alpha_step_goal_ee", self.alpha_step_goal_ee])
                training_log_data.append(["alpha_step_target", self.alpha_step_target])
                training_log_data.append(["alpha_step_mode", self.alpha_step_mode])
                training_log_data.append(["alpha_step_mpc_exit", self.alpha_step_mpc_exit])

                write_data(training_log_file, training_log_data)

        ### NUA TODO: OUT OF DATE! UPDATE!
        '''
        elif self.mode == "testing":

            print("[mobiman_drl_config::Config::__init__] DEBUG INF testing")
            while 1:
                continue

            self.initial_training_path = self.ros_pkg_path + rospy.get_param('initial_training_path', "")
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
            velocity_control_data_str = read_data(self.ros_pkg_path + self.velocity_control_data_path + "velocity_control_data.csv")
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

            self.goal_range_min = float(get_training_param(self.initial_training_path, "goal_range_min"))
            self.collision_range_min = float(get_training_param(self.initial_training_path, "collision_range_min"))

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
                
            if data_folder_path:

                ## Write all parameters
                testing_log_file = data_folder_path + "testing_input_log.csv"

                testing_log_data = []
                testing_log_data.append(["mode", self.mode])
                testing_log_data.append(["initial_training_path", self.initial_training_path])
                
                write_data(testing_log_file, testing_log_data)
        '''

        print("[mobiman_drl_config::Config::__init__] mode: " + str(self.mode))
        print("[mobiman_drl_config::Config::__init__] world_name: " + str(self.world_name))
        print("[mobiman_drl_config::Config::__init__] world_frame_name: " + str(self.world_frame_name))
        print("[mobiman_drl_config::Config::__init__] n_armstate: " + str(self.n_armstate))
        print("[mobiman_drl_config::Config::__init__] imu_msg_name: " + str(self.imu_msg_name))
        print("[mobiman_drl_config::Config::__init__] arm_state_msg_name: " + str(self.arm_state_msg_name))
        print("[mobiman_drl_config::Config::__init__] target_msg_name: " + str(self.target_msg_name))
        print("[mobiman_drl_config::Config::__init__] occgrid_msg_name: " + str(self.occgrid_msg_name))
        print("[mobiman_drl_config::Config::__init__] selfcoldistance_msg_name: " + str(self.selfcoldistance_msg_name))
        print("[mobiman_drl_config::Config::__init__] extcoldistance_base_msg_name: " + str(self.extcoldistance_base_msg_name))
        print("[mobiman_drl_config::Config::__init__] extcoldistance_arm_msg_name: " + str(self.extcoldistance_arm_msg_name))
        print("[mobiman_drl_config::Config::__init__] pointsonrobot_msg_name: " + str(self.pointsonrobot_msg_name))
        print("[mobiman_drl_config::Config::__init__] goal_status_msg_name: " + str(self.goal_status_msg_name))
        print("[mobiman_drl_config::Config::__init__] goal_frame_name: " + str(self.goal_frame_name))
        print("[mobiman_drl_config::Config::__init__] max_episode_steps: " + str(self.max_episode_steps))
        print("[mobiman_drl_config::Config::__init__] training_timesteps: " + str(self.training_timesteps))
        print("[mobiman_drl_config::Config::__init__] occgrid_normalize_flag: " + str(self.occgrid_normalize_flag))
        print("[mobiman_drl_config::Config::__init__] occgrid_occ_min: " + str(self.occgrid_occ_min))
        print("[mobiman_drl_config::Config::__init__] occgrid_occ_max: " + str(self.occgrid_occ_max))
        print("[mobiman_drl_config::Config::__init__] observation_space_type: " + str(self.observation_space_type))
        print("[mobiman_drl_config::Config::__init__] action_time_horizon: " + str(self.action_time_horizon))
        print("[mobiman_drl_config::Config::__init__] action_type: " + str(self.action_type))
        print("[mobiman_drl_config::Config::__init__] n_action_model: " + str(self.n_action_model))
        print("[mobiman_drl_config::Config::__init__] n_action_constraint: " + str(self.n_action_constraint))
        print("[mobiman_drl_config::Config::__init__] n_action_target: " + str(self.n_action_target))
        print("[mobiman_drl_config::Config::__init__] ablation_mode: " + str(self.ablation_mode))
        print("[mobiman_drl_config::Config::__init__] last_step_distance_threshold: " + str(self.last_step_distance_threshold))
        print("[mobiman_drl_config::Config::__init__] goal_distance_pos_threshold: " + str(self.goal_distance_pos_threshold))
        print("[mobiman_drl_config::Config::__init__] goal_distance_ori_threshold_yaw: " + str(self.goal_distance_ori_threshold_yaw))
        print("[mobiman_drl_config::Config::__init__] goal_distance_ori_threshold_quat: " + str(self.goal_distance_ori_threshold_quat))
        print("[mobiman_drl_config::Config::__init__] goal_range_min_x: " + str(self.goal_range_min_x))
        print("[mobiman_drl_config::Config::__init__] goal_range_max_x: " + str(self.goal_range_max_x))
        print("[mobiman_drl_config::Config::__init__] goal_range_min_y: " + str(self.goal_range_min_y))
        print("[mobiman_drl_config::Config::__init__] goal_range_max_y: " + str(self.goal_range_max_y))
        print("[mobiman_drl_config::Config::__init__] goal_range_min_z: " + str(self.goal_range_min_z))
        print("[mobiman_drl_config::Config::__init__] goal_range_max_z: " + str(self.goal_range_max_z))
        print("[mobiman_drl_config::Config::__init__] self_collision_range_min: " + str(self.self_collision_range_min))
        print("[mobiman_drl_config::Config::__init__] self_collision_range_max: " + str(self.self_collision_range_max))
        print("[mobiman_drl_config::Config::__init__] ext_collision_range_base_min: " + str(self.ext_collision_range_base_min))
        print("[mobiman_drl_config::Config::__init__] ext_collision_range_arm_min: " + str(self.ext_collision_range_arm_min))
        print("[mobiman_drl_config::Config::__init__] ext_collision_range_max: " + str(self.ext_collision_range_max))
        print("[mobiman_drl_config::Config::__init__] rollover_pitch_threshold: " + str(self.rollover_pitch_threshold))
        print("[mobiman_drl_config::Config::__init__] rollover_roll_threshold: " + str(self.rollover_roll_threshold))
        print("[mobiman_drl_config::Config::__init__] n_obs_stack: " + str(self.n_obs_stack))
        print("[mobiman_drl_config::Config::__init__] n_skip_obs_stack: " + str(self.n_skip_obs_stack))
        print("[mobiman_drl_config::Config::__init__] fc_obs_shape: " + str(self.fc_obs_shape))
        print("[mobiman_drl_config::Config::__init__] cnn_obs_shape: " + str(self.cnn_obs_shape))
        print("[mobiman_drl_config::Config::__init__] n_action: " + str(self.n_action))
        print("[mobiman_drl_config::Config::__init__] reward_terminal_goal: " + str(self.reward_terminal_goal))
        print("[mobiman_drl_config::Config::__init__] reward_terminal_collision: " + str(self.reward_terminal_collision))
        print("[mobiman_drl_config::Config::__init__] reward_terminal_roll: " + str(self.reward_terminal_roll))
        print("[mobiman_drl_config::Config::__init__] reward_terminal_max_step: " + str(self.reward_terminal_max_step))
        print("[mobiman_drl_config::Config::__init__] reward_step_goal: " + str(self.reward_step_goal))
        print("[mobiman_drl_config::Config::__init__] reward_step_target: " + str(self.reward_step_target))
        print("[mobiman_drl_config::Config::__init__] reward_step_mode0: " + str(self.reward_step_mode0))
        print("[mobiman_drl_config::Config::__init__] reward_step_mode1: " + str(self.reward_step_mode1))
        print("[mobiman_drl_config::Config::__init__] reward_step_mode2: " + str(self.reward_step_mode2))
        print("[mobiman_drl_config::Config::__init__] reward_step_mpc_exit: " + str(self.reward_step_mpc_exit))
        print("[mobiman_drl_config::Config::__init__] alpha_step_goal_base: " + str(self.alpha_step_goal_base))
        print("[mobiman_drl_config::Config::__init__] alpha_step_goal_ee: " + str(self.alpha_step_goal_ee))
        print("[mobiman_drl_config::Config::__init__] alpha_step_target: " + str(self.alpha_step_target))
        print("[mobiman_drl_config::Config::__init__] alpha_step_mode: " + str(self.alpha_step_mode))
        print("[mobiman_drl_config::Config::__init__] alpha_step_mpc_exit: " + str(self.alpha_step_mpc_exit))

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
    def set_observation_shape(self, obs_shape):
        self.observation_shape = obs_shape
        training_log_data = []
        training_log_data.append(["observation_shape", self.observation_shape])
        training_log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        write_data(training_log_file, training_log_data)
        print("[mobiman_drl_config::Config::set_observation_shape] observation_shape: " + str(self.observation_shape))

    '''
    NUA TODO: 
    '''
    def set_action_shape(self, act_shape):
        self.action_shape = act_shape
        training_log_data = []
        training_log_data.append(["action_shape", self.action_shape])
        training_log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        write_data(training_log_file, training_log_data)
        print("[mobiman_drl_config::Config::set_action_shape] action_shape: " + str(self.action_shape))

    '''
    NUA TODO: 
    '''
    def set_laserscan_config(self, laserscan_msg):
        self.laser_frame_id = laserscan_msg.header.frame_id
        self.laser_angle_min = laserscan_msg.angle_min                   # [rad]
        self.laser_angle_max = laserscan_msg.angle_max                   # [rad]
        self.laser_angle_increment = laserscan_msg.angle_increment       # [rad]
        self.laser_range_min = laserscan_msg.range_min                   # [m]
        self.laser_range_max = laserscan_msg.range_max                   # [m]
        self.laser_time_increment = laserscan_msg.time_increment
        self.laser_scan_time = laserscan_msg.scan_time
        self.laser_n_range = len(laserscan_msg.ranges)
        self.laser_downsample_scale = 1
        '''
        if 0 < self.laser_size_downsampled < len(laserscan_msg.ranges):
            self.laser_downsample_scale = int(len(laserscan_msg.ranges) / self.laser_size_downsampled)
            self.laser_n_range = self.laser_size_downsampled
            self.laser_angle_increment = (self.laser_angle_max - self.laser_angle_min) / self.laser_size_downsampled
        '''

    '''
    NUA TODO: 
    '''
    def set_occgrid_config(self, occgrid_msg):
        self.occgrid_data_size = len(occgrid_msg.data)
        self.occgrid_width = occgrid_msg.info.width
        self.occgrid_height = occgrid_msg.info.height
        self.occgrid_resolution = round(occgrid_msg.info.resolution, 2)

        training_log_data = []
        training_log_data.append(["occgrid_data_size", self.occgrid_data_size])
        training_log_data.append(["occgrid_width", self.occgrid_width])
        training_log_data.append(["occgrid_height", self.occgrid_height])
        training_log_data.append(["occgrid_resolution", self.occgrid_resolution])
        training_log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        write_data(training_log_file, training_log_data)

        print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_data_size: " + str(self.occgrid_data_size))
        print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_width: " + str(self.occgrid_width))
        print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_height: " + str(self.occgrid_height))
        print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_resolution: " + str(self.occgrid_resolution))

    '''
    NUA TODO: 
    '''
    def set_selfcoldistance_config(self, n_selfcoldistance):
        self.n_selfcoldistance = n_selfcoldistance
        training_log_data = []
        training_log_data.append(["n_selfcoldistance", self.n_selfcoldistance])
        training_log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        write_data(training_log_file, training_log_data)

        print("[mobiman_drl_config::Config::set_selfcoldistance_config] n_selfcoldistance: " + str(self.n_selfcoldistance))

    '''
    NUA TODO: 
    '''
    def set_extcoldistance_base_config(self, n_extcoldistance_base):
        self.n_extcoldistance_base = n_extcoldistance_base
        training_log_data = []
        training_log_data.append(["n_extcoldistance_base", self.n_extcoldistance_base])
        training_log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        write_data(training_log_file, training_log_data)

        print("[mobiman_drl_config::Config::set_extcoldistance_base_config] n_extcoldistance_base: " + str(self.n_extcoldistance_base))

    '''
    NUA TODO: 
    '''
    def set_extcoldistance_arm_config(self, n_extcoldistance_arm):
        self.n_extcoldistance_arm = n_extcoldistance_arm
        training_log_data = []
        training_log_data.append(["n_extcoldistance_arm", self.n_extcoldistance_arm])
        training_log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        write_data(training_log_file, training_log_data)

        print("[mobiman_drl_config::Config::set_selfcoldistance_config] n_extcoldistance_arm: " + str(self.n_extcoldistance_arm))

    
